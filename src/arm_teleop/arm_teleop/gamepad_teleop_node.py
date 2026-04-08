"""Gamepad (PDP Xbox) teleoperation of RoArm-M2-S.

Reads the Linux joystick device via `pygame.joystick`, integrates stick
velocities into joint target positions at 50 Hz, clamps to URDF joint limits,
and publishes `/joint_command` (sensor_msgs/JointState) to be consumed by the
`arm_driver` node.

Control mapping (PDP Wired Controller for Xbox, XInput via Linux `xpad`):

    Left stick X      -> base yaw        (joint 0, base_link_to_link1)
    Left stick Y      -> shoulder pitch  (joint 1, link1_to_link2)
    Right stick X     -> elbow pitch     (joint 2, link2_to_link3)
    Right stick Y     -> wrist / hand    (joint 3, link3_to_gripper_link)
    LB (button)       -> gripper close
    RB (button)       -> gripper open
    LT (analog axis)  -> close speed multiplier
    RT (analog axis)  -> open  speed multiplier
    Start button      -> go home pose
    Back/Select btn   -> emergency stop (zero all velocities, hold pose)

pygame axis/button indices are NOT a standard across all controllers, so on
startup the node prints every axis/button id it finds and uses parameters
(default values below) to decide which ids drive which joint. Override via ROS
parameters if the defaults don't match your gamepad — see `jstest /dev/input/js0`.
"""

from __future__ import annotations

import os
import time

import pygame
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from arm_driver.roarm_protocol import URDF_JOINT_NAMES, clamp_urdf


# Default axis / button mapping for a standard XInput Xbox pad on Linux xpad.
# These may differ on a PDP controller — override via ROS parameters if needed.
DEFAULT_AXIS_LEFT_X = 0
DEFAULT_AXIS_LEFT_Y = 1
DEFAULT_AXIS_RIGHT_X = 3
DEFAULT_AXIS_RIGHT_Y = 4
DEFAULT_AXIS_LT = 2
DEFAULT_AXIS_RT = 5
DEFAULT_BUTTON_LB = 4
DEFAULT_BUTTON_RB = 5
DEFAULT_BUTTON_START = 7
DEFAULT_BUTTON_SELECT = 6

# Initial home pose (radians, URDF convention): all zero + gripper half-open
HOME_POSE = [0.0, 0.0, 1.5708, 0.75]

# Stick dead-zone — values below this magnitude are treated as 0
DEAD_ZONE = 0.15

# Velocity gains: how many rad/sec each axis can drive its joint at full deflection
MAX_VEL_BASE = 1.2      # rad/s
MAX_VEL_SHOULDER = 0.9  # rad/s
MAX_VEL_ELBOW = 1.0     # rad/s
MAX_VEL_HAND = 0.8      # rad/s (wrist / also used as direct if you disable gripper buttons)

# Gripper: buttons + triggers drive the gripper independently of the right-Y axis.
GRIPPER_BASE_SPEED = 1.0   # rad/s when only LB/RB pressed, no trigger
GRIPPER_TRIGGER_SCALE = 1.5  # extra multiplier at full LT/RT pull

CONTROL_HZ = 50


def deadzone(v: float, dz: float = DEAD_ZONE) -> float:
    return 0.0 if abs(v) < dz else v


class GamepadTeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("gamepad_teleop")

        # Parameters
        self.declare_parameter("device_index", 0)
        self.declare_parameter("axis_left_x", DEFAULT_AXIS_LEFT_X)
        self.declare_parameter("axis_left_y", DEFAULT_AXIS_LEFT_Y)
        self.declare_parameter("axis_right_x", DEFAULT_AXIS_RIGHT_X)
        self.declare_parameter("axis_right_y", DEFAULT_AXIS_RIGHT_Y)
        self.declare_parameter("axis_lt", DEFAULT_AXIS_LT)
        self.declare_parameter("axis_rt", DEFAULT_AXIS_RT)
        self.declare_parameter("button_lb", DEFAULT_BUTTON_LB)
        self.declare_parameter("button_rb", DEFAULT_BUTTON_RB)
        self.declare_parameter("button_start", DEFAULT_BUTTON_START)
        self.declare_parameter("button_select", DEFAULT_BUTTON_SELECT)

        p = self.get_parameter
        self._ax = (
            p("axis_left_x").value,
            p("axis_left_y").value,
            p("axis_right_x").value,
            p("axis_right_y").value,
            p("axis_lt").value,
            p("axis_rt").value,
        )
        self._bt = (
            p("button_lb").value,
            p("button_rb").value,
            p("button_start").value,
            p("button_select").value,
        )

        # Init pygame joystick
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")  # headless
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No joystick found. Did you forward /dev/input/js0 via usbipd?")
            raise RuntimeError("no joystick")

        dev_idx = p("device_index").value
        self.js = pygame.joystick.Joystick(dev_idx)
        self.js.init()
        self.get_logger().info(
            f"Opened joystick '{self.js.get_name()}' "
            f"(axes={self.js.get_numaxes()}, buttons={self.js.get_numbuttons()}, "
            f"hats={self.js.get_numhats()})"
        )

        # Publisher
        self.cmd_pub = self.create_publisher(JointState, "/joint_command", 10)

        # Target state (URDF convention): [base, shoulder, elbow, hand]
        self._target = list(HOME_POSE)
        self._estop = False
        self._last_tick = time.time()
        self._last_pub_target = list(self._target)

        # First publish so arm_driver gets an initial pose
        self._publish_target()

        # Control loop timer
        self.create_timer(1.0 / CONTROL_HZ, self._tick)

        self.get_logger().info(
            "Teleop ready. Left stick=base/shoulder, Right stick=elbow/hand, "
            "LB/RB=gripper, LT/RT=grip speed, Start=home, Back=emergency stop."
        )

    # ----------------------------------------------------------------
    def _tick(self) -> None:
        pygame.event.pump()
        now = time.time()
        dt = max(0.001, now - self._last_tick)
        self._last_tick = now

        try:
            lx = deadzone(self.js.get_axis(self._ax[0]))
            ly = deadzone(self.js.get_axis(self._ax[1]))
            rx = deadzone(self.js.get_axis(self._ax[2]))
            ry = deadzone(self.js.get_axis(self._ax[3]))
            # Triggers on Linux XInput usually rest at -1.0 (unpressed) to +1.0 (fully pulled).
            # Remap to [0, 1].
            lt_raw = self.js.get_axis(self._ax[4])
            rt_raw = self.js.get_axis(self._ax[5])
            lt = max(0.0, (lt_raw + 1.0) * 0.5)
            rt = max(0.0, (rt_raw + 1.0) * 0.5)

            lb = bool(self.js.get_button(self._bt[0]))
            rb = bool(self.js.get_button(self._bt[1]))
            btn_start = bool(self.js.get_button(self._bt[2]))
            btn_back = bool(self.js.get_button(self._bt[3]))
        except pygame.error as e:
            self.get_logger().warn(f"pygame read error: {e}")
            return
        except IndexError:
            self.get_logger().warn_once(
                "Axis/button index out of range — check your gamepad mapping parameters."
            ) if hasattr(self.get_logger(), "warn_once") else self.get_logger().warn(
                "Axis/button index out of range — check your gamepad mapping parameters."
            )
            return

        # Emergency stop (press Back/Select): freeze target and ignore further sticks
        if btn_back:
            if not self._estop:
                self.get_logger().warn("EMERGENCY STOP (Back button)")
            self._estop = True
            return
        # Start button re-enables and sends home pose
        if btn_start:
            if self._estop:
                self.get_logger().info("Re-enabled (Start button), going home")
            self._estop = False
            self._target = list(HOME_POSE)
            self._publish_target()
            return

        if self._estop:
            return

        # Integrate stick velocities into target joint positions
        # Note: stick Y axes are inverted on pygame (up = -1)
        self._target[0] += lx * MAX_VEL_BASE * dt
        self._target[1] += (-ly) * MAX_VEL_SHOULDER * dt
        self._target[2] += rx * MAX_VEL_ELBOW * dt
        self._target[3] += (-ry) * MAX_VEL_HAND * dt

        # Gripper LB/RB with LT/RT speed scaling
        grip_speed = GRIPPER_BASE_SPEED
        if lb:  # close
            self._target[3] -= grip_speed * (1.0 + GRIPPER_TRIGGER_SCALE * lt) * dt
        if rb:  # open
            self._target[3] += grip_speed * (1.0 + GRIPPER_TRIGGER_SCALE * rt) * dt

        # Clamp to joint limits
        self._target = clamp_urdf(self._target)

        # Only publish if something actually changed (avoids flooding at rest)
        if self._target != self._last_pub_target:
            self._publish_target()

    # ----------------------------------------------------------------
    def _publish_target(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(URDF_JOINT_NAMES)
        msg.position = list(self._target)
        self.cmd_pub.publish(msg)
        self._last_pub_target = list(self._target)

    # ----------------------------------------------------------------
    def destroy_node(self) -> bool:
        try:
            pygame.joystick.quit()
            pygame.quit()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = GamepadTeleopNode()
    except Exception as e:
        print(f"[gamepad_teleop_node] startup failed: {e}")
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

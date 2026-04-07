"""
Skill Executor Node — Executes TaskSpec by dispatching skills sequentially.

Subscribes to /task_spec, executes each step by calling the appropriate
ROS 2 action/service, and reports progress on /anima_status.

This is Layer 3 of the ANIMA architecture.
"""

import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


# Known locations in the simulation world (hardcoded for v1)
# These would come from a semantic map in production
KNOWN_LOCATIONS = {
    "table": {"x": 1.0, "y": 0.5, "z": 0.0, "yaw": 0.0},
    "storage_box": {"x": -0.5, "y": -1.0, "z": 0.0, "yaw": 1.57},
    "center": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
    "door": {"x": -2.0, "y": 0.0, "z": 0.0, "yaw": 3.14},
}


class SkillExecutorNode(Node):
    """Executes skills from TaskSpec sequentially."""

    def __init__(self):
        super().__init__('skill_executor')

        # Subscribe to task specs from ANIMA core
        self.task_sub = self.create_subscription(
            String, '/task_spec', self.on_task_spec, 10
        )

        # Status publisher
        self.status_pub = self.create_publisher(String, '/anima_status', 10)

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Skill dispatch table
        self.skill_handlers = {
            'detect_object': self.skill_detect,
            'navigate_to': self.skill_navigate,
            'pick_object': self.skill_pick,
            'place_object': self.skill_place,
            'say': self.skill_say,
        }

        self.get_logger().info('Skill Executor Node started.')

    def on_task_spec(self, msg: String):
        """Execute a TaskSpec step by step."""
        try:
            spec = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid TaskSpec JSON: {e}')
            return

        self.get_logger().info(f'Executing task: {spec.get("description", "unknown")}')

        for i, step in enumerate(spec.get('steps', [])):
            skill = step.get('skill')
            params = step.get('params', {})

            self.publish_status(
                f'Step {i+1}/{len(spec["steps"])}: {skill}({params})'
            )

            handler = self.skill_handlers.get(skill)
            if handler is None:
                self.publish_status(f'ERROR: Unknown skill "{skill}"')
                return

            success = handler(params)
            if not success:
                self.publish_status(f'FAILED at step {i+1}: {skill}')
                return

        self.publish_status(f'Task completed: {spec.get("description", "")}')

    def skill_detect(self, params: dict) -> bool:
        """Detect objects (placeholder — uses Gazebo ground truth in v1)."""
        target = params.get('target', 'unknown')
        self.get_logger().info(f'[DETECT] Looking for: {target}')
        # TODO: Query Gazebo model positions or run perception pipeline
        self.publish_status(f'Detected: {target}')
        return True

    def skill_navigate(self, params: dict) -> bool:
        """Navigate to a named location using Nav2."""
        target = params.get('target', 'center')
        location = KNOWN_LOCATIONS.get(target)

        if location is None:
            self.get_logger().error(f'Unknown location: {target}')
            return False

        self.get_logger().info(f'[NAV] Navigating to {target}: {location}')

        # Wait for Nav2 action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Nav2 not available. Simulating navigation.')
            self.publish_status(f'[SIM] Navigated to {target}')
            return True

        # Send navigation goal
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = location['x']
        goal.pose.pose.position.y = location['y']

        # Convert yaw to quaternion (simplified: only z rotation)
        import math
        yaw = location.get('yaw', 0.0)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f'Navigation goal rejected: {target}')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)

        self.publish_status(f'Arrived at {target}')
        return True

    def skill_pick(self, params: dict) -> bool:
        """Pick up an object (placeholder for MoveIt2 integration)."""
        target = params.get('target', 'unknown')
        self.get_logger().info(f'[PICK] Picking up: {target}')
        # TODO: Call MoveIt2 pick action
        # 1. Move arm to pre-grasp pose above object
        # 2. Lower arm to grasp pose
        # 3. Close gripper
        # 4. Lift arm
        self.publish_status(f'Picked up: {target}')
        return True

    def skill_place(self, params: dict) -> bool:
        """Place an object (placeholder for MoveIt2 integration)."""
        target = params.get('target', 'unknown')
        destination = params.get('destination', 'unknown')
        self.get_logger().info(f'[PLACE] Placing {target} in {destination}')
        # TODO: Call MoveIt2 place action
        # 1. Move arm to pre-place pose above destination
        # 2. Lower arm
        # 3. Open gripper
        # 4. Retract arm
        self.publish_status(f'Placed {target} in {destination}')
        return True

    def skill_say(self, params: dict) -> bool:
        """Report status to user."""
        text = params.get('text', '')
        self.get_logger().info(f'[SAY] {text}')
        self.publish_status(f'Robot says: {text}')
        return True

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SkillExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

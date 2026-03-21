"""Collect demonstration data via vision teleoperation in simulation.

Usage:
    python scripts/collect_demos.py --num-episodes 50 --dataset-name so100_pick_place
"""

import argparse

import cv2
import numpy as np

from src.pose_estimation.holistic_tracker import HolisticTracker
from src.retargeting.combined import CombinedRetarget
from src.simulation.arm_env import ArmEnvConfig, ArmSimEnv
from src.teleoperation.data_recorder import DataRecorder, DataRecorderConfig


def main() -> None:
    parser = argparse.ArgumentParser(description="Collect teleoperation demonstrations")
    parser.add_argument("--num-episodes", type=int, default=50)
    parser.add_argument("--dataset-name", type=str, default="so100_pick_place")
    parser.add_argument("--max-steps", type=int, default=200, help="Max steps per episode")
    parser.add_argument("--camera", type=int, default=0)
    args = parser.parse_args()

    # Initialize components
    tracker = HolisticTracker()
    retarget = CombinedRetarget()
    env = ArmSimEnv(ArmEnvConfig(render_mode="rgb_array"))
    recorder = DataRecorder(DataRecorderConfig(dataset_name=args.dataset_name))

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print("ERROR: Cannot open camera")
        return

    print(f"Collecting {args.num_episodes} episodes for '{args.dataset_name}'")
    print("Controls: SPACE=start/end episode, 'q'=quit and save, 's'=skip episode")

    episode_active = False
    step_count = 0

    try:
        while recorder.num_episodes < args.num_episodes:
            ret, frame = cap.read()
            if not ret:
                break

            frame = cv2.flip(frame, 1)
            holistic_pose = tracker.detect(frame)
            action = retarget.retarget(holistic_pose)
            env.set_joint_positions(action.arm_joints)
            sim_image = env.render_image()

            # Record if episode is active
            if episode_active:
                recorder.record_frame(
                    joint_positions=env.get_joint_positions(),
                    joint_actions=action.arm_joints,
                    camera_image=frame,
                    sim_image=sim_image,
                )
                step_count += 1

                if step_count >= args.max_steps:
                    recorder.end_episode()
                    episode_active = False
                    env.reset()
                    step_count = 0

            # Display
            sim_bgr = cv2.cvtColor(cv2.resize(sim_image, (640, 480)), cv2.COLOR_RGB2BGR)
            display = np.hstack([frame, sim_bgr])
            status = (
                f"EP {recorder.num_episodes}/{args.num_episodes} | "
                f"{'REC' if episode_active else 'READY'} | "
                f"Steps: {step_count}"
            )
            cv2.putText(display, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("Data Collection", display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord(" "):
                if episode_active:
                    recorder.end_episode()
                    episode_active = False
                    env.reset()
                    step_count = 0
                else:
                    recorder.start_episode()
                    episode_active = True
                    step_count = 0
            elif key == ord("s"):
                if episode_active:
                    episode_active = False
                    env.reset()
                    step_count = 0
                    print("Episode skipped")
            elif key == ord("q"):
                break

    finally:
        if episode_active:
            recorder.end_episode()

        save_path = recorder.save()
        cap.release()
        cv2.destroyAllWindows()
        tracker.close()
        env.close()
        print(f"\nDataset saved to: {save_path}")


if __name__ == "__main__":
    main()

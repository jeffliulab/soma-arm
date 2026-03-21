"""MediaPipe Holistic demo: webcam -> body + hand landmarks overlay.

Usage:
    python scripts/demo_mediapipe.py

Press 'q' to quit.
"""

import time

import cv2
import mediapipe as mp

mp_holistic = mp.solutions.holistic
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles


def main() -> None:
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("ERROR: Cannot open webcam")
        return

    print("Starting MediaPipe Holistic demo... Press 'q' to quit.")

    with mp_holistic.Holistic(
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
        model_complexity=1,
    ) as holistic:
        fps_start = time.time()
        frame_count = 0

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            # Flip for mirror view, convert BGR -> RGB
            frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            rgb.flags.writeable = False

            results = holistic.process(rgb)

            rgb.flags.writeable = True
            annotated = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

            # Draw pose landmarks (body)
            if results.pose_landmarks:
                mp_drawing.draw_landmarks(
                    annotated,
                    results.pose_landmarks,
                    mp_holistic.POSE_CONNECTIONS,
                    landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style(),
                )

            # Draw left hand
            if results.left_hand_landmarks:
                mp_drawing.draw_landmarks(
                    annotated,
                    results.left_hand_landmarks,
                    mp_holistic.HAND_CONNECTIONS,
                    mp_drawing_styles.get_default_hand_landmarks_style(),
                    mp_drawing_styles.get_default_hand_connections_style(),
                )

            # Draw right hand
            if results.right_hand_landmarks:
                mp_drawing.draw_landmarks(
                    annotated,
                    results.right_hand_landmarks,
                    mp_holistic.HAND_CONNECTIONS,
                    mp_drawing_styles.get_default_hand_landmarks_style(),
                    mp_drawing_styles.get_default_hand_connections_style(),
                )

            # FPS calculation
            frame_count += 1
            elapsed = time.time() - fps_start
            if elapsed > 0:
                fps = frame_count / elapsed
                cv2.putText(
                    annotated,
                    f"FPS: {fps:.1f}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 255, 0),
                    2,
                )

            # Show key landmark info
            if results.pose_landmarks:
                lm = results.pose_landmarks.landmark
                # Right wrist (landmark 16)
                rw = lm[16]
                cv2.putText(
                    annotated,
                    f"R Wrist: ({rw.x:.2f}, {rw.y:.2f}, {rw.z:.2f})",
                    (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 0),
                    1,
                )
                # Right elbow (landmark 14)
                re = lm[14]
                cv2.putText(
                    annotated,
                    f"R Elbow: ({re.x:.2f}, {re.y:.2f}, {re.z:.2f})",
                    (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 0),
                    1,
                )
                # Right shoulder (landmark 12)
                rs = lm[12]
                cv2.putText(
                    annotated,
                    f"R Shoulder: ({rs.x:.2f}, {rs.y:.2f}, {rs.z:.2f})",
                    (10, 115),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 0),
                    1,
                )

            cv2.imshow("SmartRobotArm - MediaPipe Holistic", annotated)

            if cv2.waitKey(5) & 0xFF == ord("q"):
                break

    cap.release()
    cv2.destroyAllWindows()
    print(f"Average FPS: {frame_count / elapsed:.1f}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import mediapipe as mp
import math

class HandGestureTeleop(Node):
    def __init__(self):
        super().__init__('hand_gesture_teleop')
        # Publisher to /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/wheel_controller/cmd_vel_unstamped', 10)
        # Mediapipe setup
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
        self.mp_draw = mp.solutions.drawing_utils
        
        # Open webcam
        self.cap = cv2.VideoCapture(0)
        # Timer to run at ~30 Hz
        self.timer = self.create_timer(0.03, self.process_frame)

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        
        frame = cv2.flip(frame, 1)  # Mirror image
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.hands.process(frame_rgb)

        gesture = "none"

        if result.multi_hand_landmarks:
            hand_landmarks = result.multi_hand_landmarks[0]
            self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

            # Extract landmarks for fingers
            finger_tips_ids = [4, 8, 12, 16, 20]  # Thumb, Index, Middle, Ring, Pinky
            finger_mcp_ids = [2, 5, 9, 13, 17]

            fingers_open = []
            for tip, mcp in zip(finger_tips_ids, finger_mcp_ids):
                if hand_landmarks.landmark[tip].y < hand_landmarks.landmark[mcp].y:
                    fingers_open.append(1)
                else:
                    fingers_open.append(0)

            # Determine gesture
            if all(f == 1 for f in fingers_open[1:]):  # Ignore thumb
                gesture = "forward"
            elif all(f == 0 for f in fingers_open[1:]):
                gesture = "stop"
            elif fingers_open[1] == 1 and fingers_open[2:] == [0, 0, 0]:
                gesture = "left"
            elif fingers_open[1] == 1 and fingers_open[2] == 1 and fingers_open[3:] == [0, 0]:
                gesture = "right"

        # Map gesture to Twist
        twist = Twist()
        if gesture == "forward":
            twist.linear.x = 0.2
            twist.angular.z = 0.0
        elif gesture == "stop":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        elif gesture == "left":
            twist.linear.x = 0.0
            twist.angular.z = 0.5
        elif gesture == "right":
            twist.linear.x = 0.0
            twist.angular.z = -0.5
        self.cmd_vel_pub.publish(twist)

        # Display webcam feed
        cv2.putText(frame, f"Gesture: {gesture}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Hand Gesture Teleop", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = HandGestureTeleop()
    rclpy.spin(node)
    node.cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

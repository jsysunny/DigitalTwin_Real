#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

# === 글로벌 상태 변수 ===
hsv_center = [25, 150, 150]  # 초기 HSV 값 (노란색 대략)
auto_adjust = True           # True면 클릭 기반 자동조정, False면 트랙바 우선


def adjust_hsv_range(base_hsv, delta_h=10, delta_s=80, delta_v=80):
    h, s, v = base_hsv
    lower = (max(h - delta_h, 0), max(s - delta_s, 0), max(v - delta_v, 0))
    upper = (min(h + delta_h, 179), min(s + delta_s, 255), min(v + delta_v, 255))
    return lower, upper


def mouse_callback(event, x, y, flags, param):
    global hsv_center, auto_adjust
    if event == cv2.EVENT_LBUTTONDOWN:
        pixel = param[y, x]
        hsv_pixel = cv2.cvtColor(np.uint8([[pixel]]), cv2.COLOR_BGR2HSV)
        hsv_center = hsv_pixel[0][0].tolist()
        auto_adjust = True
        print(f"[CLICK] HSV Center updated to: {hsv_center}")


def create_trackbar_window():
    cv2.namedWindow("HSV Adjust")
    cv2.createTrackbar("H Lower", "HSV Adjust", 18, 179, lambda x: None)
    cv2.createTrackbar("H Upper", "HSV Adjust", 35, 179, lambda x: None)
    cv2.createTrackbar("S Lower", "HSV Adjust", 70, 255, lambda x: None)
    cv2.createTrackbar("S Upper", "HSV Adjust", 255, 255, lambda x: None)
    cv2.createTrackbar("V Lower", "HSV Adjust", 100, 255, lambda x: None)
    cv2.createTrackbar("V Upper", "HSV Adjust", 255, 255, lambda x: None)


class LaneFollower(Node):
    def __init__(self):
        super().__init__('lane_follower')

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.screenshot_count = 0

        # PID parameters
        self.kp = 0.006
        self.kd = 0.01
        self.ki = 0.0
        self.prev_error = 0.0
        self.integral = 0.0

        # Control limits
        self.max_angular_z = 1.0
        self.min_speed = 0.03
        self.max_speed = 0.08

        # Screenshot save directory
        self.save_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'picture')
        os.makedirs(self.save_dir, exist_ok=True)

        create_trackbar_window()

    def image_callback(self, msg):
        global auto_adjust, hsv_center

        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        height, width = frame.shape[:2]

        # BEV 변환
        src = np.float32([[0, height], [width, height], [width, int(height * 0.55)], [0, int(height * 0.55)]])
        dst = np.float32([[0, height], [width, height], [width, 0], [0, 0]])
        matrix = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(frame, matrix, (width, height))

        # HSV 변환 및 마스킹
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)

        if auto_adjust:
            lower, upper = adjust_hsv_range(hsv_center)
        else:
            h_l = cv2.getTrackbarPos("H Lower", "HSV Adjust")
            h_u = cv2.getTrackbarPos("H Upper", "HSV Adjust")
            s_l = cv2.getTrackbarPos("S Lower", "HSV Adjust")
            s_u = cv2.getTrackbarPos("S Upper", "HSV Adjust")
            v_l = cv2.getTrackbarPos("V Lower", "HSV Adjust")
            v_u = cv2.getTrackbarPos("V Upper", "HSV Adjust")
            lower, upper = (h_l, s_l, v_l), (h_u, s_u, v_u)

        yellow_mask = cv2.inRange(hsv, lower, upper)

        # 노이즈 제거
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 히스토그램 기반 중심 계산
        histogram = np.sum(mask[mask.shape[0]//2:, :], axis=0)
        midpoint = histogram.shape[0] // 2
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint
        lane_center = (leftx_base + rightx_base) // 2
        frame_center = width // 2

        # PID 제어
        error = frame_center - lane_center
        self.integral += error
        derivative = error - self.prev_error
        angular_z = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        linear_x = self.max_speed if abs(error) < 50 else self.min_speed

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = np.clip(angular_z, -self.max_angular_z, self.max_angular_z)
        self.publisher_.publish(twist)

        # 시각화
        debug_frame = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        debug_frame[yellow_mask > 0] = [0, 255, 255]
        cv2.line(debug_frame, (lane_center, height), (lane_center, height - 50), (255, 0, 0), 3)
        cv2.line(debug_frame, (frame_center, height), (frame_center, height - 50), (0, 255, 0), 3)

        cv2.imshow("Lane Detection", debug_frame)
        cv2.setMouseCallback("Lane Detection", mouse_callback, warped)

        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            filename = os.path.join(self.save_dir, f'screenshot_{self.screenshot_count:03d}.png')
            cv2.imwrite(filename, debug_frame)
            print(f"Screenshot saved: {filename}")
            self.screenshot_count += 1
        elif key == ord('m'):
            auto_adjust = not auto_adjust
            print(f"[MODE SWITCH] Auto adjust: {auto_adjust}")


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

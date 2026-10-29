import threading

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node


class StarCenterPublisher(Node):
    def __init__(self):
        super().__init__("publisher")

        # 发布五角星中心点和画面中心点，坐标单位都是图像像素。
        self.star_center_publisher_ = self.create_publisher(PointStamped, "/star/center", 10)
        self.image_center_publisher_ = self.create_publisher(PointStamped, "/image/center", 10)

        # 定时器回调中做图形检测，采集图像放到单独线程中。
        self.timer = self.create_timer(0.0167, self.star_detect_callback)

        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error("无法打开摄像头")
            raise RuntimeError("摄像头初始化失败")

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
        self.cap.set(cv2.CAP_PROP_FPS, 120)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, 120)
        self.cap.set(cv2.CAP_PROP_GAIN, 120)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # latest_frame 保存采集线程读到的最新一帧。
        self.latest_frame = None
        self.lock = threading.Lock()
        self.running = True

        self.grab_thread = threading.Thread(target=self.grab_loop, daemon=True)
        self.grab_thread.start()

        self.get_logger().info("发布节点启动")

    def grab_loop(self):
        # 采集线程只负责读摄像头，并保留最新一帧，减少画面延迟。
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.latest_frame = frame

    def star_detect_callback(self):
        # 从采集线程取出最新画面，避免在处理图像时阻塞摄像头采集。
        with self.lock:
            frame = self.latest_frame.copy() if self.latest_frame is not None else None

        if frame is None:
            return

        frame_height, frame_width = frame.shape[:2]
        image_center_x = frame_width // 2
        image_center_y = frame_height // 2
        self.publish_image_center(image_center_x, image_center_y)

        # 1. 图像预处理：黑色五角星变成白色前景。
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (3, 3), 0)
        binary = cv2.adaptiveThreshold(
            blur,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            81,
            19,
        )

        kernel = np.ones((3, 3), np.uint8)
        morphed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=1)
        morphed = cv2.morphologyEx(morphed, cv2.MORPH_OPEN, kernel, iterations=1)

        # 2. 使用 RETR_CCOMP 找内外两层轮廓。
        cnts, hierarchy = cv2.findContours(
            morphed,
            cv2.RETR_CCOMP,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        outer = None
        inner = None
        outer_center = None
        inner_center = None

        # 3. 筛选五角星外轮廓，再从它的子轮廓里找内五角星。
        if hierarchy is not None:
            hierarchy = hierarchy[0]
            outer_candidates = []

            for i, contour in enumerate(cnts):
                _, _, child_id, parent_id = hierarchy[i]
                if parent_id != -1 or child_id == -1:
                    continue

                area = cv2.contourArea(contour)
                if not (5000 < area < 50000):
                    continue

                peri = cv2.arcLength(contour, True)
                if peri == 0:
                    continue

                approx = cv2.approxPolyDP(contour, 0.023 * peri, True)
                if not (8 <= len(approx) <= 12):
                    continue

                hull = cv2.convexHull(approx)
                hull_approx = cv2.approxPolyDP(hull, 0.023 * peri, True)
                if len(hull_approx) != 5:
                    continue

                hull_area = cv2.contourArea(hull)
                if hull_area == 0:
                    continue

                solidity = area / hull_area
                if not (0.40 < solidity < 0.55):
                    continue

                outer_candidates.append((area, contour, child_id))

            outer_candidates.sort(key=lambda item: item[0], reverse=True)

            for outer_area, outer_contour, child_id in outer_candidates:
                current_child = child_id
                while current_child != -1:
                    child_contour = cnts[current_child]
                    child_area = cv2.contourArea(child_contour)

                    if 3000 < child_area < outer_area:
                        area_ratio = outer_area / child_area
                        if 2.0 < area_ratio < 5.0:
                            outer = outer_contour
                            inner = child_contour
                            break

                    current_child = hierarchy[current_child][0]

                if inner is not None:
                    break

                if outer is None:
                    outer = outer_contour

        # 4. 计算中心点，检测成功后发布 PointStamped。
        if outer is not None:
            cv2.drawContours(frame, [outer], -1, (0, 255, 0), 2)
            moment = cv2.moments(outer)
            if moment["m00"] != 0:
                outer_center = (
                    int(moment["m10"] / moment["m00"]),
                    int(moment["m01"] / moment["m00"]),
                )

        if inner is not None:
            cv2.drawContours(frame, [inner], -1, (255, 0, 0), 2)
            moment = cv2.moments(inner)
            if moment["m00"] != 0:
                inner_center = (
                    int(moment["m10"] / moment["m00"]),
                    int(moment["m01"] / moment["m00"]),
                )

        if outer_center is not None:
            if inner_center is not None:
                center_x = (outer_center[0] + inner_center[0]) // 2
                center_y = (outer_center[1] + inner_center[1]) // 2
            else:
                center_x, center_y = outer_center

            cv2.circle(frame, (center_x, center_y), 3, (0, 0, 255), -1)
            self.publish_center(center_x, center_y)

            self.get_logger().info(
                "Star center: ({}, {})".format(center_x, center_y),
                throttle_duration_sec=0.5,
            )

        # 画面中心点：用红色十字标记，并在终端打印坐标。
        cross_len = 20
        cv2.line(
            frame,
            (image_center_x - cross_len, image_center_y),
            (image_center_x + cross_len, image_center_y),
            (0, 0, 255),
            2,
        )
        cv2.line(
            frame,
            (image_center_x, image_center_y - cross_len),
            (image_center_x, image_center_y + cross_len),
            (0, 0, 255),
            2,
        )

        cv2.imshow("Frame", frame)
        # cv2.imshow("Morphed", morphed)
        cv2.waitKey(1)

    def publish_center(self, center_x, center_y):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.point.x = float(center_x)
        msg.point.y = float(center_y)
        msg.point.z = 0.0
        self.star_center_publisher_.publish(msg)

    def publish_image_center(self, center_x, center_y):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.point.x = float(center_x)
        msg.point.y = float(center_y)
        msg.point.z = 0.0
        self.image_center_publisher_.publish(msg)

    def destroy_node(self):
        self.running = False
        if hasattr(self, "grab_thread"):
            self.grab_thread.join(timeout=1.0)

        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()

        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = StarCenterPublisher()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

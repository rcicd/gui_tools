#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import CompressedImage
from rclpy.node import Node
from sensor_msgs.srv import SetCameraInfo
from PIL import Image
import cv_bridge
import numpy as np
import cv2
from calibration_gui import GUI
import threading


class IntrinsicCalibrationNode(Node):
    def __init__(self, node_name: str, gui: GUI):
        super().__init__(node_name)
        self.declare_parameter("vehicle", "")
        self.declare_parameter("assets_dir", "")
        self.assets_dir = self.get_parameter("assets_dir").get_parameter_value().string_value
        self.veh_name = self.get_parameter("vehicle").get_parameter_value().string_value
        self.image_subscriber = self.create_subscription(
            CompressedImage,
            f"/{self.veh_name}/image/compressed",
            self.image_callback,
            1
        )
        self.intrinsic_publisher = self.create_client(SetCameraInfo, f"/{self.veh_name}/set_camera_info")
        self.objp = np.zeros((5 * 7, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:7, 0:5].T.reshape(-1, 2)
        self.obj_points = []
        self.img_points = []
        self.bridge = cv_bridge.CvBridge()
        self.gui = gui
        self.gui.assets_dir = self.assets_dir
        self.gui.update_placeholder()
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # self.counter = 0
        self.calibration_table = np.zeros((3, 3, 3))
        self.big_calibration_table = np.zeros((3,))
        self.image_size = None
        self.is_first = True

    # @staticmethod
    def determine_size(self, corners: np.ndarray):
        left_up = corners[28, 0, :]
        right_up = corners[0, 0, :]
        left_down = corners[34, 0, :]
        width = np.linalg.norm(left_up - right_up)
        height = np.linalg.norm(left_up - left_down)
        if width > 240 and height > 320:
            return 1
        if 90 < width < 130 < height < 170:
            return -1
        return 0

    # @staticmethod
    def determine_rotation(self, corners):
        left_up = corners[28, 0, :]
        right_up = corners[0, 0, :]
        dx = right_up[0] - left_up[0]
        dy = right_up[1] - left_up[1]
        theta_tg = dy / dx
        if np.abs(theta_tg) < np.tan(np.pi / 12):
            return 1
        if theta_tg > np.tan(np.pi / 12):
            return 0
        return 2

    # @staticmethod
    def determine_region(self, corners, image_size):
        left_up = corners[28, 0, :]
        right_up = corners[0, 0, :]
        left_down = corners[34, 0, :]
        right_down = corners[6, 0, :]
        middle = (left_up + right_up + left_down + right_down) / 4
        # self.get_logger().info(f"Middle: {middle}")
        if middle[0] < 5 * image_size[1] / 12:
            horizontal_region = 0
        elif middle[0] > 7 * image_size[1] / 12:
            horizontal_region = 2
        else:
            horizontal_region = 1
        if middle[1] < 5 * image_size[0] / 12:
            vertical_region = 0
        elif middle[1] > 7 * image_size[0] / 12:
            vertical_region = 2
        else:
            vertical_region = 1
        return vertical_region, horizontal_region

    def update_points(self, corners):
        self.obj_points.append(self.objp)
        self.img_points.append(corners)

    def image_callback(self, msg: CompressedImage):
        raw_image = self.bridge.compressed_imgmsg_to_cv2(msg)
        image = cv2.cvtColor(self.bridge.compressed_imgmsg_to_cv2(msg), cv2.COLOR_BGR2GRAY)
        if self.is_first:
            self.is_first = False
            self.image_size = image.shape
        ret, corners = cv2.findChessboardCorners(image, (7, 5), None)
        if ret:
            corners2 = cv2.cornerSubPix(image, corners, (11, 11), (-1, -1), self.criteria)
            chessboard_size = self.determine_size(corners2)
            self.gui.update_counter(chessboard_size)
            if chessboard_size == 1:
                rotation = self.determine_rotation(corners2)
                if self.big_calibration_table[rotation] == 0:
                    self.big_calibration_table[rotation] = 1
                    self.update_points(corners2)
                    self.gui.update_big_table(self.big_calibration_table)
                    if self.calibration_table.sum() + self.big_calibration_table.sum() >= 27:
                        self.gui.enable_button()
            elif chessboard_size == -1:
                rotation = self.determine_rotation(corners2)
                row, column = self.determine_region(corners2, raw_image.shape)
                if self.calibration_table[rotation, row, column] == 0:
                    self.calibration_table[rotation, row, column] = 1
                    self.update_points(corners2)
                    self.gui.update_tables(self.calibration_table)
                    if self.calibration_table.sum() + self.big_calibration_table.sum() >= 27:
                        self.gui.enable_button()
            cv2.drawChessboardCorners(raw_image, (7, 5), corners2, ret)
        pil_image = Image.fromarray(cv2.cvtColor(raw_image, cv2.COLOR_BGR2RGB))
        self.gui.update_image(pil_image)

    def set_intrinsics(self):
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points,
            self.img_points,
            self.image_size,
            None,
            None
        )
        request = SetCameraInfo.Request()
        request.camera_info.height = self.image_size[0]
        request.camera_info.width = self.image_size[1]
        request.camera_info.distortion_model = "plumb_bob"
        request.camera_info.d = dist.reshape((5,))
        request.camera_info.k = mtx.reshape((9,))
        self.result = self.intrinsic_publisher.call_async(request)
        self.get_logger().info(f"Mtx: \n{mtx.reshape((9,))}\n Dist: \n{dist}\n")
        # rclpy.spin_until_future_complete(self, result)
        # rate = self.create_rate(1)
        # rate.sleep()

    def validate_service(self):
        self.get_logger().info(f"Intrinsics set status: {self.result.done()}")


def start_node(_gui: GUI):
    node = IntrinsicCalibrationNode("intrinsic_calibration_node", _gui)
    _gui.node = node
    rclpy.spin(node)
    print("Node has been shut down")


if __name__ == "__main__":
    rclpy.init()
    gui = GUI()
    ros2_thread = threading.Thread(target=start_node, args=[gui])
    ros2_thread.start()
    gui.start()
    rclpy.shutdown()
    ros2_thread.join()

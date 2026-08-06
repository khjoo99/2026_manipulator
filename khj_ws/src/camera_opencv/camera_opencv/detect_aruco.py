import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_broadcaster import TransformBroadcaster


def euler_to_quaternion(roll, pitch, yaw):
    """Roll, pitch, yaw를 quaternion x, y, z, w로 변환한다."""

    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


def rotation_matrix_to_quaternion(rotation_matrix):
    """
    3×3 회전행렬을 quaternion x, y, z, w로 변환한다.

    cv2.Rodrigues()로 생성한 회전행렬을
    ROS TransformStamped에 넣기 위해 사용한다.
    """

    matrix = np.asarray(
        rotation_matrix,
        dtype=np.float64,
    )

    trace = (
        matrix[0, 0]
        + matrix[1, 1]
        + matrix[2, 2]
    )

    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0

        qw = 0.25 * scale
        qx = (matrix[2, 1] - matrix[1, 2]) / scale
        qy = (matrix[0, 2] - matrix[2, 0]) / scale
        qz = (matrix[1, 0] - matrix[0, 1]) / scale

    elif (
        matrix[0, 0] > matrix[1, 1]
        and matrix[0, 0] > matrix[2, 2]
    ):
        scale = math.sqrt(
            1.0
            + matrix[0, 0]
            - matrix[1, 1]
            - matrix[2, 2]
        ) * 2.0

        qw = (matrix[2, 1] - matrix[1, 2]) / scale
        qx = 0.25 * scale
        qy = (matrix[0, 1] + matrix[1, 0]) / scale
        qz = (matrix[0, 2] + matrix[2, 0]) / scale

    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(
            1.0
            + matrix[1, 1]
            - matrix[0, 0]
            - matrix[2, 2]
        ) * 2.0

        qw = (matrix[0, 2] - matrix[2, 0]) / scale
        qx = (matrix[0, 1] + matrix[1, 0]) / scale
        qy = 0.25 * scale
        qz = (matrix[1, 2] + matrix[2, 1]) / scale

    else:
        scale = math.sqrt(
            1.0
            + matrix[2, 2]
            - matrix[0, 0]
            - matrix[1, 1]
        ) * 2.0

        qw = (matrix[1, 0] - matrix[0, 1]) / scale
        qx = (matrix[0, 2] + matrix[2, 0]) / scale
        qy = (matrix[1, 2] + matrix[2, 1]) / scale
        qz = 0.25 * scale

    quaternion = np.array(
        [qx, qy, qz, qw],
        dtype=np.float64,
    )

    norm = np.linalg.norm(quaternion)

    if norm > 0.0:
        quaternion /= norm

    return tuple(quaternion)


class ArucoTfPublisher(Node):
    """Gazebo 카메라에서 ArUco를 검출하고 상자 상단 TF를 발행한다."""

    IMAGE_TOPIC = "/gripper_camera/image_raw"
    CAMERA_INFO_TOPIC = "/gripper_camera/camera_info"

    # tf2에 연결되어 있는 카메라 링크
    CAMERA_LINK_FRAME = "camera_link"

    # OpenCV 좌표계를 표현할 광학 좌표계
    OPTICAL_FRAME = "gripper_camera_optical_frame"

    # ArUco 상자 위에 붙어 있는 마커
    TARGET_MARKER_ID = 0

    # 저장소의 ArUco 마커 실제 크기: 4 cm
    MARKER_LENGTH_M = 0.04

    # 이전에 Gazebo 카메라 센서를 camera_link 앞으로
    # 4 cm 이동시켰다면 0.04를 사용한다.
    CAMERA_OFFSET_X = 0.04
    CAMERA_OFFSET_Y = 0.0
    CAMERA_OFFSET_Z = 0.0

    def __init__(self):
        super().__init__("aruco_tf_publisher")

        self.bridge = CvBridge()

        self.camera_matrix = None
        self.distortion_coefficients = None

        self.last_log_time = 0.0
        self.last_camera_warning_time = 0.0

        # Dynamic TF
        self.tf_broadcaster = TransformBroadcaster(self)

        self.image_subscription = self.create_subscription(
            Image,
            self.IMAGE_TOPIC,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            self.CAMERA_INFO_TOPIC,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )

        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )

        # OpenCV 버전에 따라 생성 방법이 다를 수 있어 둘 다 대응
        if hasattr(cv2.aruco, "DetectorParameters"):
            self.detector_parameters = (
                cv2.aruco.DetectorParameters()
            )
        else:
            self.detector_parameters = (
                cv2.aruco.DetectorParameters_create()
            )

        # 최신 API가 있으면 ArucoDetector 사용
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(
                self.dictionary,
                self.detector_parameters,
            )
        else:
            self.detector = None

        cv2.namedWindow(
            "Aruco Detection",
            cv2.WINDOW_NORMAL,
        )

        self.get_logger().info(
            "ArUco TF 노드를 시작합니다."
        )
        self.get_logger().info(
            f"Image topic: {self.IMAGE_TOPIC}"
        )
        self.get_logger().info(
            f"CameraInfo topic: {self.CAMERA_INFO_TOPIC}"
        )
        self.get_logger().info(
            f"Target marker ID: {self.TARGET_MARKER_ID}"
        )


    def camera_info_callback(self, msg: CameraInfo):
        if len(msg.k) != 9:
            return

        self.camera_matrix = np.array(
            msg.k,
            dtype=np.float64,
        ).reshape(3, 3)

        self.distortion_coefficients = np.array(
            msg.d,
            dtype=np.float64,
        ).reshape(-1, 1)

        # 실제 Gazebo 카메라 프레임 이름 사용
        self.camera_frame_id = msg.header.frame_id

        self.get_logger().info(
            f"CameraInfo 수신 완료: frame={self.camera_frame_id}, "
            f"fx={self.camera_matrix[0, 0]:.2f}, "
            f"fy={self.camera_matrix[1, 1]:.2f}"
        )

    def detect_markers(self, gray_image):
        """설치된 OpenCV 버전에 맞춰 마커를 검출한다."""

        if self.detector is not None:
            return self.detector.detectMarkers(
                gray_image
            )

        return cv2.aruco.detectMarkers(
            gray_image,
            self.dictionary,
            parameters=self.detector_parameters,
        )

    def publish_marker_tf(
        self,
        marker_id,
        rvec,
        tvec,
        timestamp,
    ):
        """rvec와 tvec를 이용해 상자 상단 dynamic TF를 발행한다."""

        # Rodrigues 회전벡터 → 3×3 회전행렬
        rotation_matrix, _ = cv2.Rodrigues(
            rvec
        )

        qx, qy, qz, qw = (
            rotation_matrix_to_quaternion(
                rotation_matrix
            )
        )

        transform = TransformStamped()

        transform.header.stamp = timestamp
        transform.header.frame_id = (
            self.OPTICAL_FRAME
        )

        transform.child_frame_id = (
            f"aruco_box_top_{marker_id}"
        )

        transform.transform.translation.x = float(
            tvec[0]
        )
        transform.transform.translation.y = float(
            tvec[1]
        )
        transform.transform.translation.z = float(
            tvec[2]
        )

        transform.transform.rotation.x = float(qx)
        transform.transform.rotation.y = float(qy)
        transform.transform.rotation.z = float(qz)
        transform.transform.rotation.w = float(qw)

        self.tf_broadcaster.sendTransform(
            transform
        )

    def image_callback(self, msg):
        """카메라 이미지에서 ArUco를 찾고 TF와 OpenCV 화면을 출력한다."""

        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )

        except Exception as error:
            self.get_logger().error(
                f"이미지 변환 오류: {error}"
            )
            return

        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY,
        )

        corners, ids, rejected = (
            self.detect_markers(gray)
        )

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(
                frame,
                corners,
                ids,
            )

        # CameraInfo가 아직 안 들어온 경우에는
        # 마커 테두리까지만 그리고 자세 계산은 하지 않는다.
        if (
            self.camera_matrix is None
            or self.distortion_coefficients is None
            or self.camera_frame_id is None
            ):

            cv2.putText(
                frame,
                "Waiting for camera_info",
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        elif ids is not None:
            flat_ids = ids.flatten()

            for index, marker_id_value in enumerate(
                flat_ids
            ):
                marker_id = int(marker_id_value)

                if marker_id != self.TARGET_MARKER_ID:
                    continue

                selected_corners = [
                    corners[index]
                ]

                # 마커 위치와 자세 계산
                rvecs, tvecs, _ = (
                    cv2.aruco.estimatePoseSingleMarkers(
                        selected_corners,
                        self.MARKER_LENGTH_M,
                        self.camera_matrix,
                        self.distortion_coefficients,
                    )
                )

                rvec = rvecs[0].reshape(3)
                tvec = tvecs[0].reshape(3)

                # 영상에 좌표축 표시
                cv2.drawFrameAxes(
                    frame,
                    self.camera_matrix,
                    self.distortion_coefficients,
                    rvec.reshape(3, 1),
                    tvec.reshape(3, 1),
                    self.MARKER_LENGTH_M * 0.75,
                    2,
                )

                # TF 발행
                self.publish_marker_tf(
                    marker_id,
                    rvec,
                    tvec,
                    msg.header.stamp,
                )

                distance = float(
                    np.linalg.norm(tvec)
                )

                marker_points = (
                    corners[index]
                    .reshape(4, 2)
                    .astype(int)
                )

                text_x = int(marker_points[0][0])
                text_y = max(
                    int(marker_points[0][1]) - 15,
                    25,
                )

                position_text = (
                    f"ID:{marker_id} "
                    f"X:{tvec[0]:.3f} "
                    f"Y:{tvec[1]:.3f} "
                    f"Z:{tvec[2]:.3f} m"
                )

                cv2.putText(
                    frame,
                    position_text,
                    (text_x, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

                cv2.putText(
                    frame,
                    f"Distance: {distance:.3f} m",
                    (15, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

                current_time = time.monotonic()

                if (
                    current_time
                    - self.last_log_time
                    >= 0.5
                ):
                    self.last_log_time = current_time

                    rotation_matrix, _ = (
                        cv2.Rodrigues(rvec)
                    )

                    qx, qy, qz, qw = (
                        rotation_matrix_to_quaternion(
                            rotation_matrix
                        )
                    )

                    self.get_logger().info(
                        f"ArUco ID={marker_id} | "
                        f"tvec: "
                        f"x={tvec[0]:.4f}, "
                        f"y={tvec[1]:.4f}, "
                        f"z={tvec[2]:.4f} m | "
                        f"quaternion: "
                        f"x={qx:.4f}, "
                        f"y={qy:.4f}, "
                        f"z={qz:.4f}, "
                        f"w={qw:.4f}"
                    )

        cv2.imshow(
            "Aruco Detection",
            frame,
        )

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args)

    node = ArucoTfPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("ArUco TF 노드를 종료합니다.")

    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
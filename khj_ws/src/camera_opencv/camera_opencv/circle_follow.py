import random

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CircleFollow(Node):

    def __init__(self):
        super().__init__("circle_follow")

        # DDS 이미지 publisher
        self.image_pub = self.create_publisher(
            Image,
            "/camera/circle_image",
            qos_profile_sensor_data,
        )

        self.bridge = CvBridge()

        # 영상 크기
        self.width = 640
        self.height = 480

        # 카메라 열기
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            raise RuntimeError("카메라를 열 수 없습니다.")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        cv2.namedWindow(
            "Circle Follow",
            cv2.WINDOW_NORMAL,
        )

        # 화면 안의 고정된 위치 10개
        self.fixed_positions = [
            (100, 100),
            (320, 80),
            (540, 100),
            (550, 240),
            (540, 400),
            (320, 400),
            (100, 400),
            (90, 240),
            (220, 240),
            (420, 240),
        ]

        # 시작할 때 10개 위치의 방문 순서를 무작위로 섞음
        self.target_positions = self.fixed_positions.copy()
        random.shuffle(self.target_positions)

        # 시작 위치
        self.circle_position = np.array(
            self.target_positions[0],
            dtype=np.float32,
        )

        # 현재 이동할 목표 번호
        self.target_index = 1

        # 원 이동 속도
        self.move_speed = 10.0

        # 이동 경로 저장
        self.path_points = [
            (
                int(self.circle_position[0]),
                int(self.circle_position[1]),
            )
        ]

        # 10개 위치를 모두 방문한 후 기다리는 프레임
        self.complete_count = 0

        # 약 1초 대기
        self.reset_wait_frames = 30

        # 30 FPS
        self.timer = self.create_timer(
            1.0 / 30.0,
            self.timer_callback,
        )

        self.get_logger().info(
            "/camera/circle_image 토픽으로 이미지를 발행합니다."
        )

    def reset_circle(self):
        """
        기존 경로를 모두 지우고
        새로운 무작위 순서로 다시 시작한다.
        """

        random.shuffle(self.target_positions)

        self.circle_position = np.array(
            self.target_positions[0],
            dtype=np.float32,
        )

        self.target_index = 1

        self.path_points = [
            (
                int(self.circle_position[0]),
                int(self.circle_position[1]),
            )
        ]

        self.complete_count = 0

        self.get_logger().info(
            "10개 위치 방문 완료: 경로를 지우고 다시 시작합니다."
        )

    def move_circle(self):
        """
        현재 원의 위치를 다음 목표 위치 방향으로 이동시킨다.
        """

        # 10개 위치를 모두 방문한 경우
        if self.target_index >= len(self.target_positions):
            self.complete_count += 1

            # 약 1초 동안 완성된 도형을 보여준 뒤 초기화
            if self.complete_count >= self.reset_wait_frames:
                self.reset_circle()

            return

        # 다음 목표 위치
        target = np.array(
            self.target_positions[self.target_index],
            dtype=np.float32,
        )

        # 현재 위치에서 목표 위치까지의 방향
        direction = target - self.circle_position

        # 현재 위치와 목표 위치 사이의 거리
        distance = np.linalg.norm(direction)

        # 목표 지점 가까이에 도착한 경우
        if distance <= self.move_speed:
            self.circle_position = target
            self.target_index += 1

            self.get_logger().info(
                f"목표 위치 도착: "
                f"{self.target_index}/"
                f"{len(self.target_positions)}"
            )

        else:
            # 방향 벡터를 길이가 1인 단위 벡터로 변경
            unit_direction = direction / distance

            # 일정한 속도로 원 이동
            self.circle_position += (
                unit_direction * self.move_speed
            )

        # 현재 위치를 경로에 저장
        current_point = (
            int(self.circle_position[0]),
            int(self.circle_position[1]),
        )

        self.path_points.append(current_point)

    def draw_objects(self, frame):
        """
        카메라 영상에 목표 위치, 경로, 움직이는 원을 그린다.
        """

        # 고정된 10개 위치 표시
        for index, position in enumerate(self.target_positions):
            # 이미 방문한 위치는 초록색
            if index < self.target_index:
                point_color = (0, 255, 0)

            # 아직 방문하지 않은 위치는 노란색
            else:
                point_color = (0, 255, 255)

            cv2.circle(
                frame,
                position,
                6,
                point_color,
                -1,
                cv2.LINE_AA,
            )

            # 방문 순서 번호 표시
            cv2.putText(
                frame,
                str(index + 1),
                (position[0] + 8, position[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                point_color,
                1,
                cv2.LINE_AA,
            )

        # 원이 지나간 이동 경로 그리기
        if len(self.path_points) >= 2:
            points = np.array(
                self.path_points,
                dtype=np.int32,
            )

            points = points.reshape((-1, 1, 2))

            cv2.polylines(
                frame,
                [points],
                False,
                (255, 0, 0),
                3,
                cv2.LINE_AA,
            )

        # 현재 움직이는 원 위치
        center = (
            int(self.circle_position[0]),
            int(self.circle_position[1]),
        )

        # 움직이는 빨간색 원
        cv2.circle(
            frame,
            center,
            15,
            (0, 0, 255),
            -1,
            cv2.LINE_AA,
        )

        # 진행 상황 표시
        completed = min(
            self.target_index,
            len(self.target_positions),
        )

        cv2.putText(
            frame,
            f"Position: {completed}/10",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

    def timer_callback(self):
        """
        카메라 이미지를 읽고 원과 경로를 그린 뒤
        DDS로 발행하고 imshow로 출력한다.
        """

        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warning(
                "카메라 이미지를 읽지 못했습니다."
            )
            return

        # 영상 크기를 640 × 480으로 고정
        frame = cv2.resize(
            frame,
            (self.width, self.height),
        )

        # 원 위치 이동
        self.move_circle()

        # 영상에 원과 경로 그리기
        self.draw_objects(frame)

        # OpenCV 이미지 → ROS Image 메시지
        image_msg = self.bridge.cv2_to_imgmsg(
            frame,
            encoding="bgr8",
        )

        image_msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        image_msg.header.frame_id = "camera_link"

        # DDS 토픽 발행
        self.image_pub.publish(image_msg)

        # 현재 화면에 직접 출력
        cv2.imshow(
            "Circle Follow",
            frame,
        )

        key = cv2.waitKey(1) & 0xFF

        # OpenCV 창을 클릭한 후 q를 누르면 종료
        if key == ord("q"):
            raise KeyboardInterrupt

    def close(self):
        """
        카메라와 OpenCV 창을 종료한다.
        """

        if self.cap is not None:
            self.cap.release()

        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = CircleFollow()
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("circle_follow 노드를 종료합니다.")

    except RuntimeError as error:
        print(f"실행 오류: {error}")

    finally:
        if node is not None:
            node.close()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
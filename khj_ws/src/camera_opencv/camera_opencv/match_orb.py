import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class MatchOrb(Node):

    def __init__(self):
        super().__init__("match_orb")

        # camera_pub.py에서 발행하는 카메라 영상 구독
        self.sub = self.create_subscription(
            Image,
            "camera/image_raw",
            self.image_callback,
            10,
        )

        self.bridge = CvBridge()

        # a34.py와 동일한 ORB 설정
        self.orb = cv2.ORB_create(nfeatures=1000)

        # ORB descriptor는 binary descriptor이므로
        # Hamming 거리 사용
        self.bf = cv2.BFMatcher_create(
            cv2.NORM_HAMMING,
            crossCheck=True,
        )

        # 기준 물체 영상
        self.src1 = None
        self.img1 = None
        self.kp1 = None
        self.des1 = None

        # 기준 물체를 촬영할 중앙 영역 크기
        self.roi_width = 320
        self.roi_height = 240

        cv2.namedWindow("match_orb")

        self.get_logger().info(
            "물체를 중앙 사각형 안에 넣고 Space 키를 누르세요."
        )
        self.get_logger().info(
            "Space: 기준 사진 촬영 / r: 초기화 / q: 종료"
        )

    def image_callback(self, msg):
        try:
            # ROS Image → OpenCV BGR 영상
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )

        except Exception as error:
            self.get_logger().error(
                f"영상 변환 실패: {error}"
            )
            return

        display = frame.copy()

        height, width = frame.shape[:2]

        # 화면 중앙에 물체 촬영 영역 생성
        x1 = (width - self.roi_width) // 2
        y1 = (height - self.roi_height) // 2
        x2 = x1 + self.roi_width
        y2 = y1 + self.roi_height

        # 기준 사진이 없는 경우
        if self.des1 is None:
            cv2.rectangle(
                display,
                (x1, y1),
                (x2, y2),
                (0, 255, 255),
                2,
            )

            cv2.putText(
                display,
                "Put object in box and press SPACE",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )

        # 기준 사진이 촬영된 경우 ORB 매칭 수행
        else:
            display = self.detect_object(frame)

        cv2.putText(
            display,
            "SPACE: capture  r: reset  q: quit",
            (20, height - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
        )

        cv2.imshow("match_orb", display)

        key = cv2.waitKey(1) & 0xFF

        # Space: 중앙 사각형 영역을 기준 물체로 촬영
        if key == 32:
            self.capture_reference(
                frame,
                x1,
                y1,
                x2,
                y2,
            )

        # r: 기준 물체 초기화
        elif key == ord("r"):
            self.reset_reference()

        # q: 종료
        elif key == ord("q"):
            raise KeyboardInterrupt

    def capture_reference(
        self,
        frame,
        x1,
        y1,
        x2,
        y2,
    ):
        """중앙 사각형 영역을 기준 물체 사진으로 저장한다."""

        # a34.py의 src1에 해당
        self.src1 = frame[y1:y2, x1:x2].copy()

        if self.src1.size == 0:
            self.get_logger().warning(
                "기준 사진 촬영 영역이 비어 있습니다."
            )
            return

        # 컬러 영상 → 흑백 영상
        self.img1 = cv2.cvtColor(
            self.src1,
            cv2.COLOR_BGR2GRAY,
        )

        # a34.py의 기준 사진 ORB keypoint와 descriptor 생성
        self.kp1, self.des1 = self.orb.detectAndCompute(
            self.img1,
            None,
        )

        if self.des1 is None or len(self.kp1) < 10:
            count = 0 if self.kp1 is None else len(self.kp1)

            self.get_logger().warning(
                f"ORB 특징점이 부족합니다: {count}개"
            )
            self.get_logger().warning(
                "글자나 무늬가 많은 물체를 사용하세요."
            )

            self.src1 = None
            self.img1 = None
            self.kp1 = None
            self.des1 = None
            return

        # 촬영한 기준 사진 저장
        cv2.imwrite(
            "orb_reference.jpg",
            self.src1,
        )

        # 기준 물체의 ORB keypoint 표시
        keypoint_image = cv2.drawKeypoints(
            self.src1,
            self.kp1,
            None,
            flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
        )

        cv2.imshow(
            "ORB reference",
            keypoint_image,
        )

        self.get_logger().info(
            f"기준 물체 촬영 완료: keypoints={len(self.kp1)}"
        )

        self.get_logger().info(
            f"descriptor 크기: {self.des1.shape}"
        )

    def detect_object(self, frame):
        """a34.py의 src2를 실시간 카메라 영상으로 대체한다."""

        # a34.py의 src2에 해당
        src2 = frame.copy()

        # 카메라 현재 영상을 흑백으로 변환
        img2 = cv2.cvtColor(
            src2,
            cv2.COLOR_BGR2GRAY,
        )

        # 현재 카메라 영상의 ORB keypoint와 descriptor
        kp2, des2 = self.orb.detectAndCompute(
            img2,
            None,
        )

        if des2 is None or kp2 is None:
            cv2.putText(
                src2,
                "No ORB descriptors",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )
            return src2

        # a34.py 3번:
        # 기준 물체와 카메라 영상 descriptor 매칭
        matches = self.bf.match(
            self.des1,
            des2,
        )

        if len(matches) == 0:
            return src2

        # a34.py 4번:
        # descriptor 거리 기준 오름차순 정렬
        matches = sorted(
            matches,
            key=lambda match: match.distance,
        )

        min_dist = matches[0].distance

        # a34.py에서는 distance < 5 * minDist 사용
        #
        # minDist가 0에 가까울 경우 good match가 전부 사라지는
        # 문제를 막기 위해 최소 임계값 30을 추가
        distance_threshold = max(
            5 * min_dist,
            30.0,
        )

        good_matches = list(
            filter(
                lambda match:
                match.distance < distance_threshold,
                matches,
            )
        )

        cv2.putText(
            src2,
            f"matches: {len(matches)}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            src2,
            f"good matches: {len(good_matches)}",
            (20, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
        )

        # a34.py와 동일하게 good match가 5개 미만이면
        # Homography를 계산하지 않는다.
        if len(good_matches) < 5:
            cv2.putText(
                src2,
                "OBJECT NOT FOUND",
                (20, 105),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 0, 255),
                2,
            )
            return src2

        # a34.py 5번:
        # 기준 사진과 카메라 영상의 매칭 좌표 생성
        src1_pts = np.float32(
            [
                self.kp1[match.queryIdx].pt
                for match in good_matches
            ]
        )

        src2_pts = np.float32(
            [
                kp2[match.trainIdx].pt
                for match in good_matches
            ]
        )

        # Homography 계산
        H, mask = cv2.findHomography(
            src1_pts,
            src2_pts,
            cv2.RANSAC,
            3.0,
        )

        if H is None or mask is None:
            cv2.putText(
                src2,
                "Homography failed",
                (20, 105),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )
            return src2

        mask_matches = mask.ravel().tolist()
        inlier_count = int(np.count_nonzero(mask))

        # Homography 내부 매칭점이 너무 적으면
        # 잘못된 검출로 판단
        if inlier_count < 5:
            cv2.putText(
                src2,
                f"Too few inliers: {inlier_count}",
                (20, 105),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )
            return src2

        # a34.py 6번:
        # 기준 사진의 네 모서리 좌표
        h, w = self.img1.shape

        pts = np.float32(
            [
                [0, 0],
                [0, h - 1],
                [w - 1, h - 1],
                [w - 1, 0],
            ]
        ).reshape(-1, 1, 2)

        # 기준 물체 모서리를 현재 카메라 영상 좌표로 변환
        pts2 = cv2.perspectiveTransform(
            pts,
            H,
        )

        # 검출된 물체 외곽선 표시
        src2 = cv2.polylines(
            src2,
            [np.int32(pts2)],
            True,
            (255, 0, 0),
            3,
        )

        cv2.putText(
            src2,
            "OBJECT DETECTED",
            (20, 105),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 0),
            2,
        )

        cv2.putText(
            src2,
            f"inliers: {inlier_count}",
            (20, 135),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2,
        )

        # a34.py와 동일하게 RANSAC 내부 매칭점만
        # 초록색 선으로 표시
        draw_params = dict(
            matchColor=(0, 255, 0),
            singlePointColor=None,
            matchesMask=mask_matches,
            flags=2,
        )

        dst2 = cv2.drawMatches(
            self.src1,
            self.kp1,
            src2,
            kp2,
            good_matches,
            None,
            **draw_params,
        )

        cv2.imshow(
            "ORB matches",
            dst2,
        )

        return src2

    def reset_reference(self):
        """기준 물체를 지우고 다시 촬영한다."""

        self.src1 = None
        self.img1 = None
        self.kp1 = None
        self.des1 = None

        try:
            cv2.destroyWindow("ORB reference")
        except cv2.error:
            pass

        try:
            cv2.destroyWindow("ORB matches")
        except cv2.error:
            pass

        self.get_logger().info(
            "기준 물체를 초기화했습니다."
        )
        self.get_logger().info(
            "새 물체를 중앙 사각형에 넣고 Space를 누르세요."
        )


def main(args=None):
    rclpy.init(args=args)

    node = MatchOrb()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("키보드 인터럽트")

    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
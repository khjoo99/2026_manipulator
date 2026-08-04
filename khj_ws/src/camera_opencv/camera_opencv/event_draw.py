import color
import cv2
import numpy as np


WIDTH = 640
HEIGHT = 480
FPS = 30

WINDOW_NAME = "camera canvas"


def on_mouse(event, x, y, flags, param):
    """
    카메라 영상 위에 표시할 overlay에 마우스로 그림을 그린다.
    """

    overlay, option = param

    # 왼쪽 마우스 버튼을 누른 순간
    if event == cv2.EVENT_LBUTTONDOWN:
        on_mouse.old_x = x
        on_mouse.old_y = y

        current_color = list(color.COLORS.values())[option[0]]

        cv2.circle(
            overlay,
            (x, y),
            3,
            current_color,
            -1,
            cv2.LINE_AA,
        )

        print(f"Mouse clicked at ({x}, {y})")

    # 왼쪽 버튼을 누른 상태로 이동할 때
    elif (
        event == cv2.EVENT_MOUSEMOVE
        and flags & cv2.EVENT_FLAG_LBUTTON
    ):
        current_color = list(color.COLORS.values())[option[0]]

        cv2.line(
            overlay,
            (on_mouse.old_x, on_mouse.old_y),
            (x, y),
            current_color,
            3,
            cv2.LINE_AA,
        )

        on_mouse.old_x = x
        on_mouse.old_y = y

        print(f"Mouse dragged to ({x}, {y})")


def main():
    # GStreamer 카메라 파이프라인
    pipeline = (
        "v4l2src device=/dev/video0 ! "
        f"image/jpeg,width={WIDTH},height={HEIGHT},"
        f"framerate={FPS}/1 ! "
        "jpegdec ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink max-buffers=1 drop=true sync=false"
    )

    cap = cv2.VideoCapture(
        pipeline,
        cv2.CAP_GSTREAMER,
    )

    if not cap.isOpened():
        print("카메라를 열 수 없습니다: /dev/video0")
        return

    print("카메라가 정상적으로 열렸습니다.")

    cv2.namedWindow(
        WINDOW_NAME,
        cv2.WINDOW_NORMAL,
    )

    cv2.resizeWindow(
        WINDOW_NAME,
        WIDTH,
        HEIGHT,
    )

    # 그림을 영구적으로 저장할 투명한 검은색 이미지
    overlay = np.zeros(
        (HEIGHT, WIDTH, 3),
        dtype=np.uint8,
    )

    # 현재 색상 번호
    option = [0]

    cv2.setMouseCallback(
        WINDOW_NAME,
        on_mouse,
        (overlay, option),
    )

    color_names = list(color.COLORS.keys())

    print(f"현재 색상: {color_names[option[0]]}")
    print("스페이스바: 색상 변경")
    print("c: 그림 지우기")
    print("q: 종료")

    while True:
        ret, frame = cap.read()

        if not ret:
            print("카메라 프레임을 읽지 못했습니다.")
            continue

        # 카메라 프레임의 크기가 다를 때만 크기 조절
        if frame.shape[1] != WIDTH or frame.shape[0] != HEIGHT:
            frame = cv2.resize(
                frame,
                (WIDTH, HEIGHT),
            )

        # 카메라 영상과 그림 overlay 합성
        output = cv2.add(
            frame,
            overlay,
        )

        # 현재 색상 표시
        current_name = color_names[option[0]]
        current_color = list(color.COLORS.values())[option[0]]

        cv2.rectangle(
            output,
            (10, 10),
            (260, 55),
            (0, 0, 0),
            -1,
        )

        cv2.putText(
            output,
            f"Color: {current_name}",
            (20, 42),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            current_color,
            2,
            cv2.LINE_AA,
        )

        cv2.imshow(
            WINDOW_NAME,
            output,
        )

        key = cv2.waitKey(1) & 0xFF

        # q: 종료
        if key == ord("q"):
            break

        # 스페이스바: 다음 색상
        elif key == ord(" "):
            option[0] += 1

            if option[0] >= len(color.COLORS):
                option[0] = 0

            print(
                f"현재 색상: "
                f"{color_names[option[0]]}"
            )

        # c: 그린 내용 전체 삭제
        elif key == ord("c"):
            overlay.fill(0)
            print("그림을 모두 지웠습니다.")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
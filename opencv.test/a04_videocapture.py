from pathlib import Path
import cv2

def main():
    file_path = Path(__file__).parent
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    
    #MJPG설정
    fourrcc = cv2.VideoWriter_fourcc(*'MJPG')
    cap.set(cv2.CAP_PROP_FOURCC, fourrcc)
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return
    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다.")
            break
        cv2.imshow("Video Capture", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
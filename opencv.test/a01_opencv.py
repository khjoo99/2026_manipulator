# python3 a01_opencv.py
import cv2
import numpy as np
from pathlib import Path

def main():
    file_path = Path(__file__).parent
    print("안녕하세요")
    print(cv2.__version__)
    black_img = np.zeros((300, 300, 1), dtype=np.uint8)
    cv2.imshow("black", black_img)
    # img = cv2.imread("data/robot.jpg")  # 상대 경로
    # img = cv2.imread(str(file_path / "data/robot.jpg"))  # 절대 경로
    # cv2.imshow("robot", img)
    cv2.waitKey()  # 블럭 함수


if __name__ == "__main__":
    main()
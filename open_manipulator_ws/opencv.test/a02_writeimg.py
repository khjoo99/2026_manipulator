# python3 a01_opencv.py
import cv2
import numpy as np
from pathlib import Path

def main():
    file_path = Path(__file__).parent
    img = cv2.imread("data/robot.jpg")  # 상대 경로
    img = cv2.imread(str(file_path / "data/robot.jpg"), cv2.IMREAD_GRAYSCALE)  # 절대 경로
    print(type(img), img.shape, img.dtype)
    img = img.reshape(500, 2000)
    x = img.shape[1]  # 열
    y = img.shape[0]  # 행
    cv2.imshow("robot", img)
    
    cv2.imwrite(str(file_path / "data/robot_gray.jpg"), img)  # 절대 경로
    imgwrite_op = [cv2.IMWRITE_JPEG_QUALITY, 10]
    cv2.imwrite(str(file_path / "data/robot_gray_10.jpg"), img, imgwrite_op)  # 절대 경로
    cv2.imwrite(str(file_path / "data/robot_gray.bmp"), img)
    cv2.waitKey()  # 블럭 함수


if __name__ == "__main__":
    main()
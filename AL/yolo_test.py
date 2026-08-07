# depth + yolo 가상 환경에서 실행

import cv2
from ultralytics import YOLO


def main():
    model = YOLO("yolo26n.pt")  # load a pretrained YOLO26n model
    results = model("/home/khjoo/2026_manipulator/AL/data/dog.jpg")
    annotated = results[0].plot()  # type: ignore
    cv2.imshow("result", annotated)
    cv2.waitKey()


if __name__ == "__main__":
    main()
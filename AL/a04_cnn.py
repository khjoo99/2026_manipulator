from pathlib import Path

import color
import cv2
import numpy as np


def main():
    data_file_path = Path(__file__).parent / "data"

    with open(data_file_path / "coco90-2017.names", "r") as f:
        class_names = [line.strip() for line in f.readlines()]
    print(class_names)

    image_name = ["dog.jpg", "person.jpg", "horses.jpg", "eagle.jpg"]
    src = cv2.imread(str(data_file_path / image_name[0]))  # type: ignore

    cv2.imshow("img", src)
    cv2.waitKey()


if __name__ == "__main__":
    main()
from pathlib import Path

import color
import cv2
import numpy as np


def main():
    file_path = Path(__file__).parent
    img = np.full((500, 500, 3), 255, dtype=np.uint8)
    x1, x2 = 100, 400
    y1, y2 = 100, 400
    cv2.rectangle(img, (x1, y1), (x2, y2), color.RED, 3)
    cv2.imshow("canvas", img)
    cv2.waitKey()  # 블럭 함수
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
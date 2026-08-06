from pathlib import Path

import cv2
import numpy as np


def findLocalMaxima(src):
    kernel = cv2.getStructuringElement(shape=cv2.MORPH_RECT, ksize=(11, 11))
    dilate = cv2.dilate(src, kernel)
    localMax = src == dilate

    erode = cv2.erode(src, kernel)
    localMax2 = src > erode
    localMax &= localMax2
    points = np.argwhere(localMax == True)
    points[:, [0, 1]] = points[:, [1, 0]]
    return points


def main():
    file_path = Path(__file__).parent
    file_path = str(file_path / "data/chessboard2.jpg")
    img: np.ndarray = cv2.imread(file_path, cv2.IMREAD_GRAYSCALE)  # type: ignore
    res = cv2.preCornerDetect(img, ksize=3)
    ret, res2 = cv2.threshold(np.abs(res), 0.1, 0, cv2.THRESH_TOZERO)
    corners = findLocalMaxima(res2)
    for x, y in corners:
        cv2.circle(img, (x, y), 5, 255, 2)
    cv2.imshow("corner", res)
    cv2.imshow("img", img)
    cv2.waitKey()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
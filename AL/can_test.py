import os

import cv2
from inference_sdk import InferenceHTTPClient


# ============================================
# 설정
# ============================================

MODEL_ID = "can-or-can-not-pwbv4/2"

IMAGE_PATH = "/home/khjoo/2026_manipulator/AL/data/can.jpg"


# ============================================
# API KEY
# ============================================

api_key = os.environ.get("ROBOFLOW_API_KEY")

if not api_key:
    raise RuntimeError(
        "ROBOFLOW_API_KEY가 없습니다.\n"
        "export ROBOFLOW_API_KEY='YOUR_API_KEY' 를 실행하세요."
    )


# ============================================
# 이미지 직접 읽기
# ============================================

print("이미지 경로:")
print(IMAGE_PATH)

print("\n파일 존재 여부:")
print(os.path.isfile(IMAGE_PATH))


frame = cv2.imread(IMAGE_PATH)

if frame is None:
    raise RuntimeError(
        f"OpenCV가 이미지를 읽지 못했습니다: {IMAGE_PATH}"
    )


print("\n이미지 읽기 성공")
print("shape:", frame.shape)
print("dtype:", frame.dtype)


# ============================================
# Roboflow
# ============================================

client = InferenceHTTPClient(
    api_url="https://serverless.roboflow.com",
    api_key=api_key,
)


print("\n모델:")
print(MODEL_ID)

print("\n추론 시작...")


# ★ 경로를 보내는 게 아니라
# cv2로 읽은 numpy 이미지를 직접 보냄
result = client.infer(
    frame,
    model_id=MODEL_ID,
)


# ============================================
# 결과
# ============================================

print("\n==============================")
print("전체 결과")
print("==============================")

print(result)


print("\n==============================")
print("검출 결과")
print("==============================")


predictions = result.get("predictions", [])


if len(predictions) == 0:

    print("검출된 캔이 없습니다.")

else:

    for prediction in predictions:

        class_name = prediction["class"]
        confidence = prediction["confidence"]

        print(
            f"{class_name} : "
            f"{confidence:.2f}"
        )
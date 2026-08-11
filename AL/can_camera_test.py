import os
import cv2
import numpy as np
import pyrealsense2 as rs

from concurrent.futures import ThreadPoolExecutor

from inference import get_model
import supervision as sv


# =========================================================
# Model
# =========================================================

# CAN_MODEL_ID = "can-or-can-not-pwbv4/2"
CAN_MODEL_ID = "can-a8pgu/2"

PAPER_MODEL_ID = (
    "siddhants-workspace-3y7tn/"
    "crumpled-paper-detection-neac2-2-rfdetr-seg-small-t1"
)

PLASTIC_MODEL_ID = "plastic-bottle-classification-9u5cn/1"


CAN_THRESHOLD = 0.60
PAPER_THRESHOLD = 0.92
PLASTIC_THRESHOLD = 0.60


# =========================================================
# API KEY
# =========================================================

api_key = os.environ.get("ROBOFLOW_API_KEY")

if not api_key:
    raise RuntimeError(
        "ROBOFLOW_API_KEY가 없습니다.\n"
        "export ROBOFLOW_API_KEY='YOUR_API_KEY'"
    )


# =========================================================
# AI Model Loading
# =========================================================

print("====================================")
print("로컬 모델 로딩 시작")
print("====================================")

print()
print("[1/3] 캔 모델 로딩 중...")

can_model = get_model(
    model_id=CAN_MODEL_ID,
    api_key=api_key,
)

print("캔 모델 로딩 완료")


print()
print("[2/3] 구겨진 종이 모델 로딩 중...")

paper_model = get_model(
    model_id=PAPER_MODEL_ID,
    api_key=api_key,
)

print("구겨진 종이 모델 로딩 완료")


print()
print("[3/3] 플라스틱 모델 로딩 중...")

plastic_model = get_model(
    model_id=PLASTIC_MODEL_ID,
    api_key=api_key,
)

print("플라스틱 모델 로딩 완료")


print()
print("모든 모델 로딩 완료")
print("====================================")


# =========================================================
# RealSense D455
# =========================================================

pipeline = rs.pipeline()
config = rs.config()


# Depth
config.enable_stream(
    rs.stream.depth,
    640,
    480,
    rs.format.z16,
    30
)


# Color
config.enable_stream(
    rs.stream.color,
    640,
    480,
    rs.format.bgr8,
    30
)


print()
print("RealSense 시작 중...")


profile = pipeline.start(config)


# =========================================================
# Depth Scale
# =========================================================

depth_sensor = (
    profile
    .get_device()
    .first_depth_sensor()
)

depth_scale = (
    depth_sensor
    .get_depth_scale()
)


print(
    f"Depth Scale : {depth_scale}"
)


# =========================================================
# Depth → Color Alignment
# =========================================================

align = rs.align(
    rs.stream.color
)


print("RealSense Color + Depth 시작 완료")


# =========================================================
# Roboflow Result 변환
# =========================================================

def convert_result(
    result,
    threshold
):

    detections = (
        sv.Detections.from_inference(
            result
        )
    )


    predictions = []


    if len(detections) == 0:

        return predictions


    class_names = (
        detections.data.get(
            "class_name"
        )
    )


    for i in range(
        len(detections)
    ):

        # Confidence
        if (
            detections.confidence
            is not None
        ):

            confidence = float(
                detections.confidence[i]
            )

        else:

            confidence = 0.0


        # Threshold 이하 제거
        if confidence < threshold:

            continue


        # Bounding Box
        x1, y1, x2, y2 = (
            detections.xyxy[i]
        )


        # Class
        if class_names is not None:

            class_name = str(
                class_names[i]
            )

        else:

            class_name = str(
                detections.class_id[i]
            )


        # Bounding Box 중심점
        center_x = int(
            (x1 + x2) / 2
        )

        center_y = int(
            (y1 + y2) / 2
        )


        predictions.append(
            {
                "x1": int(x1),
                "y1": int(y1),
                "x2": int(x2),
                "y2": int(y2),

                "center_x": center_x,
                "center_y": center_y,

                "confidence": confidence,
                "class": class_name,

                "distance_m": None,
            }
        )


    return predictions


# =========================================================
# Depth 거리 계산
# =========================================================

def get_object_distance(
    depth_image,
    x1,
    y1,
    x2,
    y2
):

    height, width = depth_image.shape[:2]

    # 영상 범위 안으로 제한
    x1 = max(0, min(x1, width - 1))
    x2 = max(0, min(x2, width - 1))

    y1 = max(0, min(y1, height - 1))
    y2 = max(0, min(y2, height - 1))


    # =====================================================
    # Bounding Box 전체를 쓰면 배경까지 포함될 수 있으므로
    # 중앙 50% 영역만 사용
    # =====================================================

    box_width = x2 - x1
    box_height = y2 - y1

    inner_x1 = int(
        x1 + box_width * 0.25
    )

    inner_x2 = int(
        x2 - box_width * 0.25
    )

    inner_y1 = int(
        y1 + box_height * 0.25
    )

    inner_y2 = int(
        y2 - box_height * 0.25
    )


    roi = depth_image[
        inner_y1:inner_y2,
        inner_x1:inner_x2
    ]


    if roi.size == 0:

        return None


    # Depth가 0인 픽셀 제거
    valid_depth = roi[
        roi > 0
    ]


    if len(valid_depth) == 0:

        return None


    # 너무 이상한 값 제거
    distance_values = (
        valid_depth.astype(
            np.float32
        )
        * depth_scale
    )


    distance_values = distance_values[
        (distance_values > 0.10)
        &
        (distance_values < 5.0)
    ]


    if len(distance_values) == 0:

        return None


    # 중앙값 사용
    distance_m = float(
        np.median(
            distance_values
        )
    )


    return distance_m


# =========================================================
# Prediction에 거리 추가
# =========================================================

def add_distance(
    predictions,
    depth_image
):

    for p in predictions:

        distance = get_object_distance(
            depth_image,

            p["x1"],
            p["y1"],
            p["x2"],
            p["y2"],
        )


        p["distance_m"] = (
            distance
        )


    return predictions


# =========================================================
# Local inference - 모델 1개씩 번갈아 실행
# =========================================================

def run_inference(
    frame,
    depth_image,
    model_type
):

    predictions = []

    if model_type == "can":

        try:
            result = can_model.infer(frame)[0]

            predictions = convert_result(
                result,
                CAN_THRESHOLD
            )

            predictions = add_distance(
                predictions,
                depth_image
            )

        except Exception as e:
            print("CAN inference 오류:", e)


    elif model_type == "paper":

        try:
            result = paper_model.infer(frame)[0]

            predictions = convert_result(
                result,
                PAPER_THRESHOLD
            )

            predictions = add_distance(
                predictions,
                depth_image
            )

        except Exception as e:
            print("PAPER inference 오류:", e)


    elif model_type == "plastic":

        try:
            result = plastic_model.infer(frame)[0]

            predictions = convert_result(
                result,
                PLASTIC_THRESHOLD
            )

            predictions = add_distance(
                predictions,
                depth_image
            )

        except Exception as e:
            print("PLASTIC inference 오류:", e)


    return model_type, predictions


# =========================================================
# Bounding Box + 거리 표시
# =========================================================

def draw_predictions(
    frame,
    predictions,
    color,
    prefix
):

    for p in predictions:

        x1 = p["x1"]
        y1 = p["y1"]
        x2 = p["x2"]
        y2 = p["y2"]

        center_x = p["center_x"]
        center_y = p["center_y"]

        class_name = p["class"]

        confidence = (
            p["confidence"]
        )

        distance_m = (
            p["distance_m"]
        )


        # -------------------------------------------------
        # Bounding Box
        # -------------------------------------------------

        cv2.rectangle(
            frame,
            (x1, y1),
            (x2, y2),
            color,
            2,
        )


        # -------------------------------------------------
        # Bounding Box 중심점
        # -------------------------------------------------

        cv2.circle(
            frame,
            (
                center_x,
                center_y
            ),
            5,
            (0, 255, 255),
            -1,
        )


        # -------------------------------------------------
        # 거리 문자열
        # -------------------------------------------------

        if distance_m is not None:

            distance_text = (
                f"{distance_m:.2f}m"
            )

        else:

            distance_text = (
                "Depth:N/A"
            )


        # -------------------------------------------------
        # Label
        # -------------------------------------------------

        label = (
            f"{prefix}"
            f"{class_name} "
            f"{confidence:.2f} "
            f"{distance_text}"
        )


        cv2.putText(
            frame,
            label,
            (
                x1,
                max(
                    y1 - 10,
                    25
                )
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
        )


# =========================================================
# 비동기 inference
# =========================================================

executor = ThreadPoolExecutor(
    max_workers=1
)


inference_future = None


last_can_predictions = []

last_paper_predictions = []

last_plastic_predictions = []


# =========================================================
# 모델 실행 순서
# =========================================================

model_order = [
    "can",
    "paper",
    "plastic",
]

model_index = 0
current_inference_model = None


# =========================================================
# 시작
# =========================================================

print()
print("====================================")
print("RealSense Waste Detection Start")
print()

print("CAN:")
print(CAN_MODEL_ID)

print()

print("PAPER:")
print(PAPER_MODEL_ID)

print()

print("PLASTIC:")
print(PLASTIC_MODEL_ID)

print()

print(
    "RealSense : "
    "640 x 480 / 15 FPS"
)

print(
    "Color + Aligned Depth"
)

print(
    "Inference : CAN -> PAPER -> PLASTIC 반복"
)

print(
    "q : 종료"
)

print("====================================")


# =========================================================
# Main Loop
# =========================================================

try:

    while True:

        # =================================================
        # RealSense Frame
        # =================================================

        frames = (
            pipeline.wait_for_frames()
        )


        # Depth를 Color 영상 기준으로 정렬
        aligned_frames = (
            align.process(
                frames
            )
        )


        depth_frame = (
            aligned_frames
            .get_depth_frame()
        )


        color_frame = (
            aligned_frames
            .get_color_frame()
        )


        if (
            not depth_frame
            or not color_frame
        ):

            continue


        # =================================================
        # numpy 변환
        # =================================================

        frame = np.asanyarray(
            color_frame.get_data()
        )


        depth_image = np.asanyarray(
            depth_frame.get_data()
        )


        # =================================================
        # inference 결과 확인
        # =================================================

        if (
            inference_future is not None
            and inference_future.done()
        ):

            try:

                model_type, predictions = (
                    inference_future.result()
                )

                # CAN 결과만 갱신
                if model_type == "can":

                    last_can_predictions = predictions

                    if last_can_predictions:
                        print()
                        print("--------- CAN ---------")

                        for p in last_can_predictions:

                            distance = p["distance_m"]

                            if distance is not None:
                                distance_text = f"{distance:.3f} m"
                            else:
                                distance_text = "N/A"

                            print(
                                f'{p["class"]:30s} '
                                f'{p["confidence"]:.2f} '
                                f'Distance: {distance_text}'
                            )

                # PAPER 결과만 갱신
                elif model_type == "paper":

                    last_paper_predictions = predictions

                    if last_paper_predictions:
                        print()
                        print("---- CRUMPLED PAPER ----")

                        for p in last_paper_predictions:

                            distance = p["distance_m"]

                            if distance is not None:
                                distance_text = f"{distance:.3f} m"
                            else:
                                distance_text = "N/A"

                            print(
                                f'{p["class"]:30s} '
                                f'{p["confidence"]:.2f} '
                                f'Distance: {distance_text}'
                            )

                # PLASTIC 결과만 갱신
                elif model_type == "plastic":

                    last_plastic_predictions = predictions

                    if last_plastic_predictions:
                        print()
                        print("------- PLASTIC -------")

                        for p in last_plastic_predictions:

                            distance = p["distance_m"]

                            if distance is not None:
                                distance_text = f"{distance:.3f} m"
                            else:
                                distance_text = "N/A"

                            print(
                                f'{p["class"]:30s} '
                                f'{p["confidence"]:.2f} '
                                f'Distance: {distance_text}'
                            )

            except Exception as e:
                print("Inference 오류:", e)

            inference_future = None
            current_inference_model = None


        # =================================================
        # 새로운 Frame inference
        # CAN -> PAPER -> PLASTIC 순서로 1개씩 실행
        # =================================================

        if inference_future is None:

            inference_frame = frame.copy()
            inference_depth = depth_image.copy()

            current_inference_model = (
                model_order[model_index]
            )

            inference_future = executor.submit(
                run_inference,
                inference_frame,
                inference_depth,
                current_inference_model,
            )

            model_index = (
                model_index + 1
            ) % len(model_order)


        # =================================================
        # CAN
        # =================================================

        draw_predictions(
            frame,
            last_can_predictions,
            (0, 255, 0),
            "[CAN] "
        )


        # =================================================
        # PAPER
        # =================================================

        draw_predictions(
            frame,
            last_paper_predictions,
            (0, 0, 255),
            "[PAPER] "
        )


        # =================================================
        # PLASTIC
        # =================================================

        draw_predictions(
            frame,
            last_plastic_predictions,
            (255, 0, 0),
            "[PLASTIC] "
        )


        # =================================================
        # Depth 화면 색상화
        # =================================================

        depth_colormap = (
            cv2.applyColorMap(
                cv2.convertScaleAbs(
                    depth_image,
                    alpha=0.03
                ),
                cv2.COLORMAP_JET
            )
        )


        # =================================================
        # 현재 추론 중인 모델 표시
        # =================================================

        if current_inference_model is not None:

            cv2.putText(
                frame,
                f"Inference: {current_inference_model.upper()}",
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
            )


        # =================================================
        # 화면 출력
        # =================================================

        cv2.imshow(
            "RealSense Waste Detection",
            frame
        )


        cv2.imshow(
            "RealSense Depth",
            depth_colormap
        )


        key = (
            cv2.waitKey(1)
            & 0xFF
        )


        if key == ord("q"):

            break


# =========================================================
# 종료
# =========================================================

finally:

    pipeline.stop()


    executor.shutdown(
        wait=False,
        cancel_futures=True
    )


    cv2.destroyAllWindows()


print("프로그램 종료")
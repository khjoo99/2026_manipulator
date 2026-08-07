import subprocess
import sys
from pathlib import Path


def main():

    # 1: path
    data_path = Path(__file__).parent / "data"

    pb_path = data_path / "frozen_inference_graph.pb"

    onnx_path = data_path / "faster_rcnn_resnet50_coco.onnx"

    print("input  =", pb_path)
    print("output =", onnx_path)

    if not pb_path.exists():
        raise FileNotFoundError(f"TensorFlow model not found: {pb_path}")

    # 2: TensorFlow -> ONNX
    cmd = [
        sys.executable,
        "-m",
        "tf2onnx.convert",
        "--graphdef",
        str(pb_path),
        "--output",
        str(onnx_path),
        "--inputs",
        "image_tensor:0",
        "--outputs",
        ("detection_boxes:0,detection_scores:0,detection_classes:0,num_detections:0"),
        "--opset",
        "13",
    ]

    print()
    print("===== TensorFlow -> ONNX =====")

    print("python =", sys.executable)

    print("command =", " ".join(cmd))

    print()

    subprocess.run(cmd, check=True)

    print()
    print("ONNX 변환 완료")
    print("model =", onnx_path)


if __name__ == "__main__":
    main()
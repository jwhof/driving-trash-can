import cv2
import base64
import requests
import os
import json

# Configuration
RF_API_KEY = os.getenv("ROBOFLOW_API_KEY") or "BYHnfaVCKFCJCWXxjkzE"
RF_WORKSPACE = "jws-workspace"
RF_WORKFLOW_ID = "find-papers"
RF_URL = f"https://serverless.roboflow.com/{RF_WORKSPACE}/workflows/{RF_WORKFLOW_ID}"

IMAGE_PATH = "test_frame.jpg"  # update this path to any test image

def run_roboflow(image_path):
    # Read and encode image
    img = cv2.imread(image_path)
    if img is None:
        raise RuntimeError(f"Could not read image at {image_path}")

    ok, buffer = cv2.imencode(".jpg", img)
    if not ok:
        raise RuntimeError("Failed to encode image to jpeg")

    img_base64 = base64.b64encode(buffer).decode("utf-8")

    payload = {
        "api_key": RF_API_KEY,
        "inputs": {
            "image": {
                "type": "base64",
                "value": img_base64
            }
        }
    }

    resp = requests.post(RF_URL, json=payload)
    print("HTTP status:", resp.status_code)
    print("Raw response:")
    print(resp.text)

    resp.raise_for_status()
    data = resp.json()

    # Quick summary
    outputs = data.get("outputs") or []
    if not outputs:
        print("No outputs field in response")
        return

    preds_block = outputs[0].get("predictions", {})
    boxes = preds_block.get("predictions") or []
    print(f"Number of detections: {len(boxes)}")
    if boxes:
        best = max(boxes, key=lambda b: b.get("confidence", 0.0))
        print("Best detection:")
        print(json.dumps(best, indent=2))

if __name__ == "__main__":
    run_roboflow(IMAGE_PATH)

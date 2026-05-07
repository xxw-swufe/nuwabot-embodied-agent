from __future__ import annotations

import httpx


def main() -> None:
    base_url = "http://127.0.0.1:8000"

    with httpx.Client(base_url=base_url, timeout=10.0) as client:
        print("healthz:", client.get("/healthz").json())
        print("status:", client.get("/robot/status").json())
        print("go_to:", client.post("/robot/go_to", json={"x": 1.0, "y": 2.0}).json())
        print("capture_image:", client.post("/robot/capture_image").json())
        print("task_log:", client.get("/robot/task_log").json())


if __name__ == "__main__":
    main()


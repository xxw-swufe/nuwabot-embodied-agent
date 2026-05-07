from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any


@dataclass
class TaskLogEntry:
    task_id: str
    action: str
    status: str
    message: str
    payload: dict[str, Any]
    created_at: str


class TaskLogger:
    def __init__(self, log_path: Path | None = None) -> None:
        base_dir = Path(__file__).resolve().parent
        self.log_path = log_path or (base_dir / "logs" / "task_log.jsonl")
        self.log_path.parent.mkdir(parents=True, exist_ok=True)

    def write(self, task_id: str, action: str, status: str, message: str, payload: dict[str, Any]) -> TaskLogEntry:
        entry = TaskLogEntry(
            task_id=task_id,
            action=action,
            status=status,
            message=message,
            payload=payload,
            created_at=datetime.now(timezone.utc).isoformat(),
        )
        with self.log_path.open("a", encoding="utf-8") as fp:
            fp.write(json.dumps(asdict(entry), ensure_ascii=False) + "\n")
        return entry

    def read_all(self) -> list[dict[str, Any]]:
        if not self.log_path.exists():
            return []
        rows: list[dict[str, Any]] = []
        with self.log_path.open("r", encoding="utf-8") as fp:
            for line in fp:
                line = line.strip()
                if line:
                    rows.append(json.loads(line))
        return rows


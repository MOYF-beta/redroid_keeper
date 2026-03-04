from __future__ import annotations

import json
import os
import shutil
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any


@dataclass
class TrajectoryBatch:
    batch_id: int
    started_at: float
    ended_at: float
    duration_sec: float
    call_string: str
    get_video: bool
    tool_returns: list[dict[str, Any]]
    status: dict[str, Any]
    evidence: list[str]
    interval_to_next_batch_sec: float | None = None


@dataclass
class ToolCallTrajectory:
    root_dir: str
    session_name: str | None = None
    session_dir: str = field(init=False)
    status_dir: str = field(init=False)
    evidence_dir: str = field(init=False)
    json_path: str = field(init=False)
    batches: list[TrajectoryBatch] = field(default_factory=list)

    def __post_init__(self) -> None:
        if not self.session_name:
            self.session_name = datetime.now().strftime("trajectory_%Y%m%d_%H%M%S")

        self.session_dir = os.path.abspath(os.path.join(self.root_dir, self.session_name))
        self.status_dir = os.path.join(self.session_dir, "status")
        self.evidence_dir = os.path.join(self.session_dir, "evidence")
        self.json_path = os.path.join(self.session_dir, "trajectory.json")

        os.makedirs(self.status_dir, exist_ok=True)
        os.makedirs(self.evidence_dir, exist_ok=True)

    def _copy_if_exists(self, src_path: str, dst_path: str) -> str | None:
        if not src_path:
            return None
        src_abs = os.path.abspath(src_path)
        if not os.path.exists(src_abs):
            return None
        session_abs = os.path.abspath(self.session_dir)
        if os.path.commonpath([src_abs, session_abs]) == session_abs:
            return os.path.relpath(src_abs, self.session_dir)
        os.makedirs(os.path.dirname(dst_path), exist_ok=True)
        shutil.copy2(src_abs, dst_path)
        return os.path.relpath(dst_path, self.session_dir)

    def add_batch(
        self,
        *,
        started_at: float,
        ended_at: float,
        call_string: str,
        get_video: bool,
        tool_returns: list[dict[str, Any]],
        status_type: str,
        status_src_path: str | None = None,
        status_image=None,
        evidence_src_paths: list[str] | None = None,
    ) -> TrajectoryBatch:
        if self.batches:
            self.batches[-1].interval_to_next_batch_sec = round(started_at - self.batches[-1].ended_at, 3)

        batch_id = len(self.batches) + 1
        status_rel = None

        if status_src_path:
            ext = os.path.splitext(status_src_path)[1] or (".mp4" if status_type == "video" else ".png")
            status_dst = os.path.join(self.status_dir, f"batch_{batch_id:04d}{ext}")
            status_rel = self._copy_if_exists(status_src_path, status_dst)
        elif status_image is not None:
            status_dst = os.path.join(self.status_dir, f"batch_{batch_id:04d}.png")
            status_image.save(status_dst)
            status_rel = os.path.relpath(status_dst, self.session_dir)

        evidence_rel_paths: list[str] = []
        for idx, src in enumerate(evidence_src_paths or [], start=1):
            ext = os.path.splitext(src)[1] or ".png"
            evidence_dst = os.path.join(self.evidence_dir, f"batch_{batch_id:04d}_{idx:02d}{ext}")
            rel = self._copy_if_exists(src, evidence_dst)
            if rel:
                evidence_rel_paths.append(rel)

        batch = TrajectoryBatch(
            batch_id=batch_id,
            started_at=started_at,
            ended_at=ended_at,
            duration_sec=round(max(0.0, ended_at - started_at), 3),
            call_string=call_string,
            get_video=get_video,
            tool_returns=tool_returns,
            status={"type": status_type, "path": status_rel},
            evidence=evidence_rel_paths,
            interval_to_next_batch_sec=None,
        )
        self.batches.append(batch)
        self.flush()
        return batch

    def flush(self) -> None:
        payload = {
            "session_name": self.session_name,
            "created_at": datetime.now().isoformat(),
            "batch_count": len(self.batches),
            "batches": [
                {
                    "batch_id": b.batch_id,
                    "started_at": b.started_at,
                    "ended_at": b.ended_at,
                    "duration_sec": b.duration_sec,
                    "interval_to_next_batch_sec": b.interval_to_next_batch_sec,
                    "call_string": b.call_string,
                    "get_video": b.get_video,
                    "tool_returns": b.tool_returns,
                    "status": b.status,
                    "evidence": b.evidence,
                }
                for b in self.batches
            ],
        }
        os.makedirs(self.session_dir, exist_ok=True)
        with open(self.json_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)

    def finish(self) -> str:
        self.flush()
        return self.json_path

    @staticmethod
    def load_batches(trajectory_path_or_dir: str) -> list[dict[str, Any]]:
        path = os.path.abspath(trajectory_path_or_dir)
        if os.path.isdir(path):
            path = os.path.join(path, "trajectory.json")
        with open(path, "r", encoding="utf-8") as f:
            payload = json.load(f)
        return payload.get("batches", [])

#!/usr/bin/env python3
"""Download missing WildOS files directly, with mirror and proxy fallback."""

from __future__ import annotations

import argparse
import json
import os
import random
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import requests


REPO_ID = "leggedrobotics/wildos"
REVISION = "4461d3c24f234b284c44050a81862386fa75540d"
PROXY = "http://172.21.16.1:7897"
REQUIRED_PREFIXES = (
    "RGB_rectified/",
    "SAM_boundaries/",
    "annotations/",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--tree",
        type=Path,
        default=Path("/home/admin1/cfy/data/wildos_tree.json"),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("/home/admin1/cfy/data/wildos"),
    )
    parser.add_argument("--workers", type=int, default=6)
    return parser.parse_args()


def download_one(
    relative_path: str,
    output_root: Path,
    mirror_session: requests.Session,
    proxy_session: requests.Session,
) -> tuple[str, str]:
    destination = output_root / relative_path
    if destination.is_file() and destination.stat().st_size > 0:
        return relative_path, "exists"

    destination.parent.mkdir(parents=True, exist_ok=True)
    mirror_url = (
        f"https://hf-mirror.com/datasets/{REPO_ID}/resolve/{REVISION}/"
        f"{relative_path}"
    )
    proxy_url = (
        f"https://huggingface.co/datasets/{REPO_ID}/resolve/{REVISION}/"
        f"{relative_path}"
    )

    for attempt in range(3):
        for session, url in (
            (mirror_session, mirror_url),
            (proxy_session, proxy_url),
        ):
            part = destination.with_suffix(destination.suffix + ".part")
            try:
                with session.get(
                    url,
                    stream=True,
                    timeout=(20, 120),
                    allow_redirects=True,
                ) as response:
                    if response.status_code == 404:
                        return relative_path, "not_found"
                    response.raise_for_status()
                    expected = response.headers.get("content-length")
                    with part.open("wb") as handle:
                        for chunk in response.iter_content(chunk_size=1024 * 1024):
                            if chunk:
                                handle.write(chunk)
                actual = part.stat().st_size
                if expected is not None and actual != int(expected):
                    raise IOError(f"size mismatch: expected {expected}, got {actual}")
                os.replace(part, destination)
                return relative_path, "downloaded"
            except Exception:
                if part.exists():
                    part.unlink()
                time.sleep(min(8, 2**attempt) + random.random())
    return relative_path, "failed"


def main() -> None:
    args = parse_args()
    files = json.loads(args.tree.read_text(encoding="utf-8"))
    relative_paths = {
        item["path"]
        for item in files
        if item.get("type") == "file"
        and item["path"].startswith(REQUIRED_PREFIXES)
    }
    relative_paths.update(
        f"SAM_boundaries/bound_{index:05d}.png" for index in range(390)
    )
    relative_paths.update(
        f"annotations/annotation_{index:05d}.json" for index in range(390)
    )
    relative_paths = sorted(relative_paths)

    mirror = requests.Session()
    mirror.trust_env = False
    mirror.headers["User-Agent"] = "Codex-WildOS-Downloader/1.0"
    proxied = requests.Session()
    proxied.trust_env = False
    proxied.proxies.update({"http": PROXY, "https": PROXY})
    proxied.headers["User-Agent"] = "Codex-WildOS-Downloader/1.0"

    counts = {"downloaded": 0, "exists": 0, "not_found": 0, "failed": 0}
    failures: list[str] = []
    start = time.perf_counter()

    with ThreadPoolExecutor(max_workers=args.workers) as executor:
        futures = {
            executor.submit(
                download_one,
                relative_path,
                args.output,
                mirror,
                proxied,
            ): relative_path
            for relative_path in relative_paths
        }
        for index, future in enumerate(as_completed(futures), start=1):
            relative_path, status = future.result()
            counts[status] += 1
            if status == "failed":
                failures.append(relative_path)
            if index % 10 == 0 or index == len(futures):
                elapsed = time.perf_counter() - start
                print(
                    f"[{index:04d}/{len(futures):04d}] "
                    f"downloaded={counts['downloaded']} "
                    f"exists={counts['exists']} "
                    f"failed={counts['failed']} "
                    f"elapsed={elapsed:.0f}s",
                    flush=True,
                )

    if failures:
        failure_path = args.output / "missing_files.txt"
        failure_path.write_text("\n".join(failures) + "\n", encoding="utf-8")
        print(f"Failures written to {failure_path}", flush=True)
        raise SystemExit(1)

    print(json.dumps(counts, indent=2), flush=True)


if __name__ == "__main__":
    main()

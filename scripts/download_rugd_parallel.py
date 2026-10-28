#!/usr/bin/env python3
"""Finish the RUGD archive with resumable parallel HTTP range requests."""

from __future__ import annotations

import math
import os
import shutil
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import requests


URL = "http://rugd.vision/data/RUGD_frames-with-annotations.zip"
TOTAL_SIZE = 5_671_298_594
PROXY = "http://172.21.16.1:7897"
OUTPUT = Path("/home/admin1/cfy/data/RUGD_frames-with-annotations.zip")
CHUNKS = 8
CHUNK_SIZE = 1024 * 1024

print_lock = threading.Lock()


def download_range(
    index: int,
    start: int,
    end: int,
) -> Path:
    part = OUTPUT.with_name(f"{OUTPUT.name}.part{index:02d}")
    expected = end - start + 1
    session = requests.Session()
    session.trust_env = False
    session.proxies.update({"http": PROXY, "https": PROXY})
    session.headers["User-Agent"] = "Codex-RUGD-Range-Downloader/1.0"

    while not part.exists() or part.stat().st_size < expected:
        offset = part.stat().st_size if part.exists() else 0
        request_start = start + offset
        try:
            with session.get(
                URL,
                headers={"Range": f"bytes={request_start}-{end}"},
                stream=True,
                timeout=(20, 180),
            ) as response:
                if response.status_code != 206:
                    raise IOError(
                        f"chunk {index}: expected HTTP 206, got {response.status_code}"
                    )
                with part.open("ab") as handle:
                    for block in response.iter_content(CHUNK_SIZE):
                        if block:
                            handle.write(block)
        except Exception as error:
            with print_lock:
                size = part.stat().st_size if part.exists() else 0
                print(
                    f"chunk {index}: retrying after {error!r}; "
                    f"{size / math.prod([1024] * 2):.1f} MiB saved",
                    flush=True,
                )
            time.sleep(2)

    actual = part.stat().st_size
    if actual != expected:
        raise IOError(f"chunk {index}: expected {expected}, got {actual}")
    return part


def main() -> None:
    if not OUTPUT.exists():
        raise FileNotFoundError(OUTPUT)
    base_size = OUTPUT.stat().st_size
    if base_size > TOTAL_SIZE:
        raise ValueError(f"Existing file is larger than expected: {base_size}")
    if base_size == TOTAL_SIZE:
        print("RUGD archive is already complete")
        return

    remaining = TOTAL_SIZE - base_size
    chunk_size = math.ceil(remaining / CHUNKS)
    ranges = []
    for index in range(CHUNKS):
        start = base_size + index * chunk_size
        end = min(TOTAL_SIZE - 1, start + chunk_size - 1)
        if start <= end:
            ranges.append((index, start, end))

    print(
        f"Existing={base_size / 2**30:.2f} GiB; "
        f"remaining={remaining / 2**30:.2f} GiB in {len(ranges)} chunks",
        flush=True,
    )

    with ThreadPoolExecutor(max_workers=len(ranges)) as executor:
        futures = [
            executor.submit(download_range, index, start, end)
            for index, start, end in ranges
        ]
        parts = [future.result() for future in futures]

    assembled = OUTPUT.with_name(f"{OUTPUT.name}.assembled")
    with assembled.open("wb") as target:
        with OUTPUT.open("rb") as source:
            shutil.copyfileobj(source, target, length=CHUNK_SIZE)
        for part in parts:
            with part.open("rb") as source:
                shutil.copyfileobj(source, target, length=CHUNK_SIZE)

    assembled_size = assembled.stat().st_size
    if assembled_size != TOTAL_SIZE:
        raise IOError(
            f"Assembled archive size mismatch: {assembled_size} != {TOTAL_SIZE}"
        )
    os.replace(assembled, OUTPUT)
    for part in parts:
        part.unlink()
    print(f"RUGD archive complete: {OUTPUT} ({assembled_size} bytes)", flush=True)


if __name__ == "__main__":
    main()

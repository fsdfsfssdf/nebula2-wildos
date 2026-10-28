#!/usr/bin/env python3
"""Evaluate the ExploRFM traversability head on the official RUGD split."""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import torch
from PIL import Image
from torchvision.transforms.functional import to_tensor

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from evaluate_rugd_sample import (
    load_colormap,
    make_preview,
    make_safe_mask,
    metrics_from_confusion,
)
from explorfm.explorfm_model import ExploRFMInference

TRAIN_SCENES = [
    "park-1",
    "park-2",
    "trail",
    "trail-3",
    "trail-4",
    "trail-5",
    "trail-6",
    "trail-7",
    "trail-9",
    "trail-10",
    "trail-11",
    "trail-12",
    "trail-13",
    "trail-14",
]
VAL_SCENES = ["park-8", "trail-15"]
SPLIT_SCENES = ["creek", "village"]


def build_official_paths(data_dir: Path) -> tuple[list[Path], list[Path]]:
    frame_dir = data_dir / "RUGD_frames"
    annotation_dir = data_dir / "RUGD_annotations"
    train_paths: list[Path] = []
    val_paths: list[Path] = []

    def add_scene(scene: str, target: list[Path]) -> list[str]:
        scene_dir = frame_dir / scene
        names = os.listdir(scene_dir)
        for name in names:
            image_path = scene_dir / name
            if (annotation_dir / scene / name).exists():
                target.append(image_path)
        return names

    for scene in TRAIN_SCENES:
        add_scene(scene, train_paths)
    for scene in VAL_SCENES:
        add_scene(scene, val_paths)

    for scene in SPLIT_SCENES:
        names = os.listdir(frame_dir / scene)
        split_index = int(len(names) * 0.8)
        names = np.random.permutation(names)
        for name in names[:split_index]:
            image_path = frame_dir / scene / name
            if (annotation_dir / scene / name).exists():
                train_paths.append(image_path)
        for name in names[split_index:]:
            image_path = frame_dir / scene / name
            if (annotation_dir / scene / name).exists():
                val_paths.append(image_path)

    return train_paths, val_paths


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--data-dir",
        type=Path,
        default=Path("/home/admin1/cfy/data/RUGD"),
    )
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path("/home/admin1/cfy/nebula2-wildos"),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("/home/admin1/cfy/nebula2-wildos/outputs/step2_rugd_full"),
    )
    parser.add_argument("--threshold", type=float, default=0.5)
    parser.add_argument("--limit", type=int, default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    # Recreate the deterministic split from the upstream datamodule without
    # requiring the optional Lightning training dependency.
    np.random.seed(42)
    _, _ = build_official_paths(args.data_dir)
    _, validation = build_official_paths(args.data_dir)
    image_count = len(validation) if args.limit is None else min(len(validation), args.limit)
    print(f"Official validation images: {len(validation)}")
    print(f"Evaluating images: {image_count}")
    colormap = load_colormap(
        args.data_dir
        / "RUGD_annotations"
        / "RUGD_annotation-colormap.txt"
    )

    model = ExploRFMInference(
        frontier_ckpt=str(args.repo_root / "ckpts/frontier_head.ckpt"),
        traversability_ckpt=str(args.repo_root / "ckpts/trav_head.ckpt"),
        model_version=str(args.repo_root / "ckpts/c-radio_v3-b_half.pth.tar"),
        adaptor_version=None,
        use_naclip=True,
        use_summary_for_spatial=False,
        radio_dim=768,
        static_scale_factor=1.0,
        model_precision="FP32",
    )

    true_positive = 0
    false_positive = 0
    false_negative = 0
    true_negative = 0
    rows: list[dict[str, float | int | str]] = []
    preview_items: list[
        tuple[str, np.ndarray, np.ndarray, np.ndarray, np.ndarray]
    ] = []

    start = time.perf_counter()
    with torch.inference_mode():
        for index in range(image_count):
            image_path = validation[index]
            annotation_path = (
                args.data_dir
                / "RUGD_annotations"
                / image_path.parent.name
                / image_path.name
            )
            image = np.array(Image.open(image_path).convert("RGB"), copy=True)
            annotation = np.array(
                Image.open(annotation_path).convert("RGB"),
                copy=True,
            )
            target = make_safe_mask(annotation, colormap)
            image_tensor = to_tensor(image).unsqueeze(0)
            traversability, _, _ = model.forward(image_tensor)
            probability = traversability[0, 0].float().cpu().numpy()
            prediction = (probability >= args.threshold).astype(np.uint8)
            image_name = image_path.name

            tp = int(np.sum((target == 1) & (prediction == 1)))
            fp = int(np.sum((target == 0) & (prediction == 1)))
            fn = int(np.sum((target == 1) & (prediction == 0)))
            tn = int(np.sum((target == 0) & (prediction == 0)))
            true_positive += tp
            false_positive += fp
            false_negative += fn
            true_negative += tn

            image_metrics = metrics_from_confusion(tp, fp, fn, tn)
            relative_name = f"{image_path.parent.name}/{image_name}"
            rows.append({"image": relative_name, **image_metrics})
            if (index + 1) % 25 == 0 or index == 0 or index + 1 == image_count:
                print(
                    f"[{index + 1:04d}/{image_count:04d}] "
                    f"{relative_name} "
                    f"IoU={image_metrics['iou_safe']:.4f} "
                    f"F1={image_metrics['f1']:.4f}"
                )

            if len(preview_items) < 4:
                preview_items.append(
                    (image_name, image, target, probability, prediction)
                )

    torch.cuda.synchronize()
    elapsed = time.perf_counter() - start
    aggregate = metrics_from_confusion(
        true_positive, false_positive, false_negative, true_negative
    )
    aggregate.update(
        {
            "images_validation": len(validation),
            "images_evaluated": image_count,
            "elapsed_seconds": elapsed,
            "threshold": args.threshold,
            "seed": 42,
            "val_scenes": VAL_SCENES,
            "split_scenes": SPLIT_SCENES,
        }
    )

    with (args.output_dir / "per_image_metrics.csv").open(
        "w", newline="", encoding="utf-8"
    ) as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    with (args.output_dir / "aggregate_metrics.json").open(
        "w", encoding="utf-8"
    ) as handle:
        json.dump(aggregate, handle, indent=2)

    for name, image, target, probability, prediction in preview_items:
        figure = make_preview(image, target, probability, prediction)
        figure.savefig(
            args.output_dir / f"preview_{Path(name).stem}.png",
            dpi=150,
            bbox_inches="tight",
        )
        plt.close(figure)

    print("\nAggregate metrics")
    for key in (
        "images_validation",
        "images_evaluated",
        "threshold",
        "accuracy",
        "precision",
        "recall",
        "f1",
        "iou_safe",
        "iou_unsafe",
        "miou",
        "elapsed_seconds",
    ):
        print(f"{key}: {aggregate[key]}")
    print(f"Saved outputs to {args.output_dir}")


if __name__ == "__main__":
    main()

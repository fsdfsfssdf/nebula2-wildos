#!/usr/bin/env python3
"""Evaluate the ExploRFM traversability head on the small RUGD sample."""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import torch
from PIL import Image

# Running a script by path does not automatically add the repository root to
# sys.path, while the editable install only registers the prototyping subpackage.
REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from explorfm.explorfm_model import ExploRFMInference


SAFE_LABELS = {
    "dirt",
    "sand",
    "grass",
    "asphalt",
    "gravel",
    "mulch",
    "rock-bed",
    "concrete",
}


def load_colormap(path: Path) -> dict[str, tuple[int, int, int]]:
    colormap: dict[str, tuple[int, int, int]] = {}
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            parts = line.strip().split()
            if not parts:
                continue
            if len(parts) != 5:
                raise ValueError(f"Invalid colormap line: {line!r}")
            _, label, red, green, blue = parts
            colormap[label] = (int(red), int(green), int(blue))
    return colormap


def make_safe_mask(
    annotation: np.ndarray,
    colormap: dict[str, tuple[int, int, int]],
) -> np.ndarray:
    mask = np.zeros(annotation.shape[:2], dtype=np.uint8)
    for label in SAFE_LABELS:
        color = np.asarray(colormap[label], dtype=np.uint8)
        mask[np.all(annotation == color, axis=-1)] = 1
    return mask


def metrics_from_confusion(
    true_positive: int,
    false_positive: int,
    false_negative: int,
    true_negative: int,
) -> dict[str, float | int]:
    epsilon = 1e-12
    precision = true_positive / (true_positive + false_positive + epsilon)
    recall = true_positive / (true_positive + false_negative + epsilon)
    f1 = 2 * precision * recall / (precision + recall + epsilon)
    iou_safe = true_positive / (
        true_positive + false_positive + false_negative + epsilon
    )
    iou_unsafe = true_negative / (
        true_negative + false_negative + false_positive + epsilon
    )
    accuracy = (true_positive + true_negative) / (
        true_positive
        + false_positive
        + false_negative
        + true_negative
        + epsilon
    )
    return {
        "tp": true_positive,
        "fp": false_positive,
        "fn": false_negative,
        "tn": true_negative,
        "accuracy": accuracy,
        "precision": precision,
        "recall": recall,
        "f1": f1,
        "iou_safe": iou_safe,
        "iou_unsafe": iou_unsafe,
        "miou": (iou_safe + iou_unsafe) / 2,
    }


def make_preview(
    image: np.ndarray,
    target: np.ndarray,
    probability: np.ndarray,
    prediction: np.ndarray,
) -> plt.Figure:
    error = np.zeros((*target.shape, 3), dtype=np.float32)
    error[(target == 0) & (prediction == 0)] = (0.12, 0.12, 0.12)
    error[(target == 1) & (prediction == 1)] = (0.10, 0.75, 0.25)
    error[(target == 0) & (prediction == 1)] = (0.90, 0.10, 0.10)
    error[(target == 1) & (prediction == 0)] = (0.10, 0.35, 0.95)

    figure, axes = plt.subplots(1, 4, figsize=(18, 4.5))
    axes[0].imshow(image)
    axes[0].set_title("RGB")

    axes[1].imshow(target, cmap="gray", vmin=0, vmax=1)
    axes[1].set_title("RUGD traversability")

    heatmap = axes[2].imshow(probability, cmap="viridis", vmin=0.0, vmax=1.0)
    axes[2].set_title("ExploRFM probability")
    figure.colorbar(heatmap, ax=axes[2], fraction=0.046)

    axes[3].imshow(error)
    axes[3].set_title("TP green | FP red | FN blue")

    for axis in axes:
        axis.axis("off")
    figure.tight_layout()
    return figure


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--sample-root",
        type=Path,
        default=Path("/home/admin1/cfy/data/RUGD_sample/RUGD_sample-data"),
    )
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path("/home/admin1/cfy/nebula2-wildos"),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("/home/admin1/cfy/nebula2-wildos/outputs/step2_rugd_sample"),
    )
    parser.add_argument("--threshold", type=float, default=0.5)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    colormap_path = (
        args.sample_root / "RUGD_annotations" / "RUGD_annotation-colormap.txt"
    )
    colormap = load_colormap(colormap_path)
    missing_labels = SAFE_LABELS.difference(colormap)
    if missing_labels:
        raise ValueError(f"Missing safe labels in colormap: {sorted(missing_labels)}")

    image_dir = args.sample_root / "images"
    annotation_dir = args.sample_root / "annotations"
    image_paths = sorted(image_dir.glob("*.png"))
    if not image_paths:
        raise FileNotFoundError(f"No PNG images found in {image_dir}")

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
    preview_items: list[tuple[str, np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = []

    start = time.perf_counter()
    with torch.inference_mode():
        for index, image_path in enumerate(image_paths, start=1):
            annotation_path = annotation_dir / image_path.name
            if not annotation_path.exists():
                print(f"[SKIP] Missing annotation: {annotation_path.name}")
                continue

            image = np.array(Image.open(image_path).convert("RGB"), copy=True)
            annotation = np.array(
                Image.open(annotation_path).convert("RGB"), copy=True
            )
            if image.shape[:2] != annotation.shape[:2]:
                raise ValueError(
                    f"Shape mismatch for {image_path.name}: "
                    f"{image.shape[:2]} vs {annotation.shape[:2]}"
                )

            target = make_safe_mask(annotation, colormap)
            traversability, _, _ = model.forward_on_numpy(image)
            probability = traversability[0, 0].float().cpu().numpy()
            prediction = (probability >= args.threshold).astype(np.uint8)

            tp = int(np.sum((target == 1) & (prediction == 1)))
            fp = int(np.sum((target == 0) & (prediction == 1)))
            fn = int(np.sum((target == 1) & (prediction == 0)))
            tn = int(np.sum((target == 0) & (prediction == 0)))

            true_positive += tp
            false_positive += fp
            false_negative += fn
            true_negative += tn

            image_metrics = metrics_from_confusion(tp, fp, fn, tn)
            rows.append({"image": image_path.name, **image_metrics})
            print(
                f"[{index:02d}/{len(image_paths):02d}] {image_path.name} "
                f"IoU={image_metrics['iou_safe']:.4f} "
                f"F1={image_metrics['f1']:.4f}"
            )

            if len(preview_items) < 4:
                preview_items.append(
                    (image_path.stem, image, target, probability, prediction)
                )

    torch.cuda.synchronize()
    elapsed = time.perf_counter() - start
    aggregate = metrics_from_confusion(
        true_positive, false_positive, false_negative, true_negative
    )
    aggregate["images"] = len(rows)
    aggregate["elapsed_seconds"] = elapsed
    aggregate["threshold"] = args.threshold

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
            args.output_dir / f"preview_{name}.png",
            dpi=150,
            bbox_inches="tight",
        )
        plt.close(figure)

    print("\nAggregate metrics")
    for key in (
        "images",
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

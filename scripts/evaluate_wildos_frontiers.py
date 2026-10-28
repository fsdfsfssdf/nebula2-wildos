#!/usr/bin/env python3
"""Evaluate the ExploRFM frontier head on the official WildOS validation split."""

from __future__ import annotations

import argparse
import json as json_module
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
import cv2
from PIL import Image
from torch.utils.data import random_split
from torchvision.transforms.functional import to_tensor

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from explorfm.explorfm_model import ExploRFMInference


def discover_frontier_records(data_dir: Path) -> list[Path]:
    image_dir = data_dir / "RGB_rectified"
    boundary_dir = data_dir / "SAM_boundaries"
    annotation_dir = data_dir / "annotations"

    records = []
    for image_path in sorted(image_dir.glob("*.png")):
        boundary_path = boundary_dir / image_path.name.replace("rect", "bound")
        annotation_path = (
            annotation_dir
            / image_path.name.replace("rect", "annotation").replace(".png", ".json")
        )
        if boundary_path.exists() and annotation_path.exists():
            records.append(image_path)
    return records


def load_frontier_sample(
    image_path: Path,
    data_dir: Path,
    resize_factor: float,
    fixed_bbox_height: int,
) -> tuple[torch.Tensor, np.ndarray]:
    annotation_path = (
        data_dir
        / "annotations"
        / image_path.name.replace("rect", "annotation").replace(".png", ".json")
    )
    image = np.array(Image.open(image_path).convert("RGB"), copy=True)
    image = cv2.resize(
        image,
        (0, 0),
        fx=resize_factor,
        fy=resize_factor,
    )

    with annotation_path.open(encoding="utf-8") as handle:
        annotations = json_module.load(handle)

    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    scaled_bbox_height = int(fixed_bbox_height * resize_factor)
    image_height, image_width = image.shape[:2]

    for annotation in annotations:
        if annotation["label"] != "frontier":
            continue
        box = annotation["start"] + annotation["end"]
        box = [int(coordinate * resize_factor) for coordinate in box]
        x1, y1, x2, y2 = (
            min(box[0], box[2]),
            min(box[1], box[3]),
            max(box[0], box[2]),
            max(box[1], box[3]),
        )
        if y2 - y1 < scaled_bbox_height:
            y2 = y1 + scaled_bbox_height
        x1 = max(0, x1)
        y1 = max(0, y1)
        x2 = min(image_width - 1, x2)
        y2 = min(image_height - 1, y2)
        mask[y1:y2, x1:x2] = 1

    return to_tensor(image), mask


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
    iou_frontier = true_positive / (
        true_positive + false_positive + false_negative + epsilon
    )
    iou_background = true_negative / (
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
        "iou_frontier": iou_frontier,
        "iou_background": iou_background,
        "miou": (iou_frontier + iou_background) / 2,
    }


def make_preview(
    image: np.ndarray,
    target: np.ndarray,
    probability: np.ndarray,
    prediction: np.ndarray,
    image_name: str,
) -> plt.Figure:
    error = np.zeros((*target.shape, 3), dtype=np.float32)
    error[(target == 0) & (prediction == 0)] = (0.12, 0.12, 0.12)
    error[(target == 1) & (prediction == 1)] = (0.10, 0.75, 0.25)
    error[(target == 0) & (prediction == 1)] = (0.90, 0.10, 0.10)
    error[(target == 1) & (prediction == 0)] = (0.10, 0.35, 0.95)

    figure, axes = plt.subplots(1, 4, figsize=(18, 4.5))
    axes[0].imshow(image)
    axes[0].set_title(image_name)

    axes[1].imshow(target, cmap="gray", vmin=0, vmax=1)
    axes[1].set_title("Frontier bbox mask")

    heatmap = axes[2].imshow(probability, cmap="inferno", vmin=0.0, vmax=1.0)
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
        "--data-dir",
        type=Path,
        default=Path("/home/admin1/cfy/data/wildos"),
    )
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path("/home/admin1/cfy/nebula2-wildos"),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(
            "/home/admin1/cfy/nebula2-wildos/outputs/step2_wildos_frontiers"
        ),
    )
    parser.add_argument("--threshold", type=float, default=0.5)
    parser.add_argument("--resize-factor", type=float, default=0.75)
    parser.add_argument("--fixed-bbox-height", type=int, default=50)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    records = discover_frontier_records(args.data_dir)
    _, validation = random_split(
        dataset=records,
        lengths=[0.8, 0.2],
        generator=torch.Generator().manual_seed(42),
    )
    print(f"Filtered dataset: {len(records)} images")
    print(f"Validation split: {len(validation)} images")

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
        for index, image_path in enumerate(validation, start=1):
            image_tensor, target = load_frontier_sample(
                image_path=image_path,
                data_dir=args.data_dir,
                resize_factor=args.resize_factor,
                fixed_bbox_height=args.fixed_bbox_height,
            )
            image_tensor = image_tensor.unsqueeze(0)
            _, frontier_logits, _ = model.forward(image_tensor)
            probability = frontier_logits[0, 0].float().cpu().numpy()
            prediction = (probability >= args.threshold).astype(np.uint8)
            image = (
                image_tensor[0].permute(1, 2, 0).cpu().numpy() * 255
            ).clip(0, 255).astype(np.uint8)
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
            rows.append({"image": image_name, **image_metrics})
            print(
                f"[{index:03d}/{len(validation):03d}] {image_name} "
                f"IoU={image_metrics['iou_frontier']:.4f} "
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
            "images_total": len(records),
            "images_validation": len(validation),
            "elapsed_seconds": elapsed,
            "threshold": args.threshold,
            "resize_factor": args.resize_factor,
            "fixed_bbox_height": args.fixed_bbox_height,
            "seed": 42,
            "split": "80/20",
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
        figure = make_preview(image, target, probability, prediction, name)
        figure.savefig(
            args.output_dir / f"preview_{Path(name).stem}.png",
            dpi=150,
            bbox_inches="tight",
        )
        plt.close(figure)

    print("\nAggregate metrics")
    for key in (
        "images_total",
        "images_validation",
        "threshold",
        "accuracy",
        "precision",
        "recall",
        "f1",
        "iou_frontier",
        "iou_background",
        "miou",
        "elapsed_seconds",
    ):
        print(f"{key}: {aggregate[key]}")
    print(f"Saved outputs to {args.output_dir}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Semantic shelf QR generator for robot logistics system.

Final label format:

Column QR:
  S03-C3

Layer QR:
  S03-L1
  S03-L2
  S03-L3

Package QR:
  PKG:C9999

Task code:
  C3-L2-C9999

Examples:

Single shelf:
  python3 generate_semantic_qr.py single-shelf --shelf S03 --column C3 --layers 3 --packages C9999

Batch shelves:
  python3 generate_semantic_qr.py batch --start 1 --end 6 --layers 3

Single QR:
  python3 generate_semantic_qr.py one --text S03-C3
"""

import argparse
import json
import re
from pathlib import Path
from typing import Dict, List, Any

import qrcode
from PIL import Image, ImageDraw, ImageFont


DEFAULT_OUT_DIR = Path("/host-home/tools/semantic_qr_tools/semantic_qr_output")


def safe_name(text: str) -> str:
    """
    Convert QR text into safe filename.
    """
    name = text.strip()
    name = name.replace(":", "_")
    name = name.replace("/", "_")
    name = name.replace("\\", "_")
    name = name.replace(" ", "_")
    return name


def find_font(size: int = 42) -> ImageFont.FreeTypeFont:
    """
    Try to use a readable system font. Fall back to PIL default font.
    """
    candidates = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
    ]

    for p in candidates:
        if Path(p).exists():
            return ImageFont.truetype(p, size=size)

    return ImageFont.load_default()


def make_qr_image(
    text: str,
    box_size: int = 16,
    border: int = 4,
) -> Image.Image:
    """
    Create pure QR image.
    """
    qr = qrcode.QRCode(
        version=None,
        error_correction=qrcode.constants.ERROR_CORRECT_H,
        box_size=box_size,
        border=border,
    )
    qr.add_data(text)
    qr.make(fit=True)

    img = qr.make_image(fill_color="black", back_color="white")
    return img.convert("RGB")


def add_label(
    qr_img: Image.Image,
    label: str,
    font_size: int = 46,
    padding: int = 24,
) -> Image.Image:
    """
    Add readable text below QR.
    """
    font = find_font(font_size)

    dummy = Image.new("RGB", (10, 10), "white")
    draw = ImageDraw.Draw(dummy)
    bbox = draw.textbbox((0, 0), label, font=font)
    text_w = bbox[2] - bbox[0]
    text_h = bbox[3] - bbox[1]

    width = max(qr_img.width, text_w + padding * 2)
    height = qr_img.height + text_h + padding * 3

    canvas = Image.new("RGB", (width, height), "white")

    qr_x = (width - qr_img.width) // 2
    canvas.paste(qr_img, (qr_x, padding))

    draw = ImageDraw.Draw(canvas)
    text_x = (width - text_w) // 2
    text_y = qr_img.height + padding * 2
    draw.text((text_x, text_y), label, fill="black", font=font)

    return canvas


def classify_qr(text: str) -> str:
    """
    Classify semantic QR type.
    """
    if re.fullmatch(r"S\d{2}-C\d+", text):
        return "column_qr"
    if re.fullmatch(r"S\d{2}-L\d+", text):
        return "layer_qr"
    if re.fullmatch(r"PKG:[A-Za-z0-9_-]+", text):
        return "package_qr"
    return "custom_qr"


def parse_shelf_column(text: str) -> Dict[str, str]:
    """
    Parse S03-C3 or S03-L2.
    """
    m = re.fullmatch(r"(S\d{2})-([CL]\d+)", text)
    if not m:
        return {}
    shelf_id = m.group(1)
    semantic_id = m.group(2)

    if semantic_id.startswith("C"):
        return {
            "shelf_id": shelf_id,
            "column_id": semantic_id,
        }

    if semantic_id.startswith("L"):
        return {
            "shelf_id": shelf_id,
            "layer_id": semantic_id,
        }

    return {}


def save_qr(
    text: str,
    out_dir: Path,
    box_size: int,
    border: int,
    font_size: int,
    with_label: bool = True,
) -> Dict[str, Any]:
    """
    Save pure QR and labeled QR.
    """
    out_dir.mkdir(parents=True, exist_ok=True)

    qr_type = classify_qr(text)
    filename_base = safe_name(text)

    pure_path = out_dir / f"{filename_base}.png"
    labeled_path = out_dir / f"{filename_base}_label.png"

    qr_img = make_qr_image(text, box_size=box_size, border=border)
    qr_img.save(pure_path)

    if with_label:
        labeled_img = add_label(qr_img, text, font_size=font_size)
        labeled_img.save(labeled_path)
    else:
        labeled_path = None

    item = {
        "text": text,
        "type": qr_type,
        "pure_png": str(pure_path),
        "label_png": str(labeled_path) if labeled_path else None,
    }

    item.update(parse_shelf_column(text))

    if qr_type == "package_qr":
        item["package_id"] = text.replace("PKG:", "", 1)

    return item


def shelf_id_from_number(n: int) -> str:
    return f"S{n:02d}"


def column_id_from_number(n: int) -> str:
    return f"C{n}"


def build_single_shelf_items(
    shelf: str,
    column: str,
    layers: int,
    packages: List[str],
) -> List[str]:
    items = []

    # Column QR
    items.append(f"{shelf}-{column}")

    # Layer QR
    for i in range(1, layers + 1):
        items.append(f"{shelf}-L{i}")

    # Package QR
    for pkg in packages:
        pkg = pkg.strip()
        if not pkg:
            continue
        if pkg.startswith("PKG:"):
            items.append(pkg)
        else:
            items.append(f"PKG:{pkg}")

    return items


def write_manifest(out_dir: Path, items: List[Dict[str, Any]]) -> None:
    manifest = {
        "format_version": "semantic_qr_v1",
        "rules": {
            "column_qr": "Sxx-Cx, example S03-C3",
            "layer_qr": "Sxx-Lx, example S03-L2",
            "package_qr": "PKG:xxxxx, example PKG:C9999",
            "task_code": "Cx-Lx-xxxxx, example C3-L2-C9999",
        },
        "items": items,
    }

    path = out_dir / "manifest.json"
    path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def cmd_one(args):
    out_dir = Path(args.out)
    item = save_qr(
        text=args.text,
        out_dir=out_dir,
        box_size=args.box_size,
        border=args.border,
        font_size=args.font_size,
        with_label=not args.no_label,
    )
    write_manifest(out_dir, [item])
    print(f"[OK] generated one QR: {args.text}")
    print(f"[OUT] {out_dir}")


def cmd_single_shelf(args):
    out_dir = Path(args.out) / f"{args.shelf}_{args.column}"
    qr_texts = build_single_shelf_items(
        shelf=args.shelf,
        column=args.column,
        layers=args.layers,
        packages=args.packages,
    )

    items = []
    for text in qr_texts:
        items.append(
            save_qr(
                text=text,
                out_dir=out_dir,
                box_size=args.box_size,
                border=args.border,
                font_size=args.font_size,
                with_label=not args.no_label,
            )
        )

    write_manifest(out_dir, items)

    print(f"[OK] generated shelf QR set: {args.shelf}-{args.column}")
    print(f"[OUT] {out_dir}")


def cmd_batch(args):
    out_dir = Path(args.out) / f"S{args.start:02d}_to_S{args.end:02d}"
    all_items = []

    for n in range(args.start, args.end + 1):
        shelf = shelf_id_from_number(n)
        column = column_id_from_number(n)

        qr_texts = build_single_shelf_items(
            shelf=shelf,
            column=column,
            layers=args.layers,
            packages=[],
        )

        shelf_dir = out_dir / f"{shelf}_{column}"
        for text in qr_texts:
            item = save_qr(
                text=text,
                out_dir=shelf_dir,
                box_size=args.box_size,
                border=args.border,
                font_size=args.font_size,
                with_label=not args.no_label,
            )
            all_items.append(item)

    write_manifest(out_dir, all_items)

    print(f"[OK] generated batch shelves: S{args.start:02d} to S{args.end:02d}")
    print(f"[OUT] {out_dir}")


def main():
    parser = argparse.ArgumentParser(
        description="Generate semantic QR codes for robot logistics shelves."
    )

    parser.add_argument(
        "--out",
        default=str(DEFAULT_OUT_DIR),
        help=f"Output directory. Default: {DEFAULT_OUT_DIR}",
    )
    parser.add_argument(
        "--box-size",
        type=int,
        default=16,
        help="QR pixel box size. Larger means bigger image.",
    )
    parser.add_argument(
        "--border",
        type=int,
        default=4,
        help="QR border size.",
    )
    parser.add_argument(
        "--font-size",
        type=int,
        default=46,
        help="Label font size.",
    )
    parser.add_argument(
        "--no-label",
        action="store_true",
        help="Only generate pure QR image, without text label image.",
    )

    sub = parser.add_subparsers(dest="cmd", required=True)

    p_one = sub.add_parser("one", help="Generate one QR code.")
    p_one.add_argument("--text", required=True, help="QR content, example S03-C3.")
    p_one.set_defaults(func=cmd_one)

    p_shelf = sub.add_parser("single-shelf", help="Generate QR set for one shelf.")
    p_shelf.add_argument("--shelf", required=True, help="Shelf id, example S03.")
    p_shelf.add_argument("--column", required=True, help="Column id, example C3.")
    p_shelf.add_argument("--layers", type=int, default=3, help="Layer count.")
    p_shelf.add_argument(
        "--packages",
        nargs="*",
        default=[],
        help="Package ids, example C9999 A1234. PKG: prefix is optional.",
    )
    p_shelf.set_defaults(func=cmd_single_shelf)

    p_batch = sub.add_parser("batch", help="Generate QR sets for shelves Sxx-Cx.")
    p_batch.add_argument("--start", type=int, required=True, help="Start shelf number.")
    p_batch.add_argument("--end", type=int, required=True, help="End shelf number.")
    p_batch.add_argument("--layers", type=int, default=3, help="Layer count.")
    p_batch.set_defaults(func=cmd_batch)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()

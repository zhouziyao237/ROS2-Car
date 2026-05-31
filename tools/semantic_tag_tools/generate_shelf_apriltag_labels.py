#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
from pathlib import Path

import cv2
from PIL import Image, ImageDraw, ImageFont


def get_dict():
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("cv2.aruco not found. Install python3-opencv.")

    aruco = cv2.aruco

    if not hasattr(aruco, "DICT_APRILTAG_36h11"):
        raise RuntimeError("OpenCV does not support DICT_APRILTAG_36h11.")

    if hasattr(aruco, "getPredefinedDictionary"):
        return aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)

    return aruco.Dictionary_get(aruco.DICT_APRILTAG_36h11)


def draw_marker(tag_id: int, size_px: int) -> Image.Image:
    aruco = cv2.aruco
    dictionary = get_dict()

    if hasattr(aruco, "generateImageMarker"):
        img = aruco.generateImageMarker(dictionary, int(tag_id), int(size_px))
    else:
        img = aruco.drawMarker(dictionary, int(tag_id), int(size_px))

    return Image.fromarray(img).convert("RGB")


def display_code(shelf_number: int, row: int) -> str:
    return f"{shelf_number:02d}{row:02d}"


def load_font(size: int, bold: bool = False):
    candidates = [
        "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ]

    for path in candidates:
        try:
            return ImageFont.truetype(path, size)
        except Exception:
            pass

    return ImageFont.load_default()


def make_label(
    tag_id: int,
    title: str,
    subtitle: str,
    marker_px: int,
    label_px: int,
) -> Image.Image:
    canvas = Image.new("RGB", (label_px, label_px), "white")
    draw = ImageDraw.Draw(canvas)

    marker = draw_marker(tag_id, marker_px)

    x = (label_px - marker_px) // 2
    y = 36
    canvas.paste(marker, (x, y))

    font_big = load_font(34, bold=True)
    font_small = load_font(22, bold=False)

    draw.text((36, y + marker_px + 22), title, fill="black", font=font_big)
    draw.text((36, y + marker_px + 64), subtitle, fill="black", font=font_small)
    draw.rectangle([2, 2, label_px - 3, label_px - 3], outline="black", width=2)

    return canvas


def add_home_label(db, out_dir: Path, marker_px: int, label_px: int):
    tag_id = 0
    item = {
        "tag_id": 0,
        "display_code": "HOME",
        "type": "home_tag",
        "obs_type": "home_tag",
        "face": "home",
        "shelf_id": "",
        "shelf_number": 0,
        "row": 0,
        "col": 0,
        "description": "Fixed semantic origin / map check anchor"
    }

    db["tags"][str(tag_id)] = item

    label = make_label(
        tag_id=0,
        title="HOME / MAP ORIGIN",
        subtitle="AprilTag ID 0 | fixed semantic origin",
        marker_px=marker_px,
        label_px=label_px,
    )

    path = out_dir / "tag_0_HOME_MAP_ORIGIN.png"
    label.save(path)
    return path


def add_shelf_labels(db, out_dir: Path, shelf_number: int, rows: int, marker_px: int, label_px: int):
    label_paths = []
    shelf_id = f"S{shelf_number:02d}"

    for row in range(0, rows + 1):
        tag_id = shelf_number * 10 + row
        code = display_code(shelf_number, row)

        if row == 0:
            item = {
                "tag_id": tag_id,
                "display_code": code,
                "type": "column_tag",
                "obs_type": "column_qr",
                "face": "side",
                "shelf_id": shelf_id,
                "shelf_number": shelf_number,
                "row": 0,
                "col": shelf_number
            }
            filename = f"tag_{tag_id}_{code}_{shelf_id}_SIDE.png"
            title = f"{code}  {shelf_id} SIDE"
            subtitle = f"AprilTag ID {tag_id} | side column"
        else:
            item = {
                "tag_id": tag_id,
                "display_code": code,
                "type": "row_tag",
                "obs_type": "layer_qr",
                "face": "front",
                "shelf_id": shelf_id,
                "shelf_number": shelf_number,
                "row": row,
                "col": shelf_number,
                "layer_id": f"L{row}"
            }
            filename = f"tag_{tag_id}_{code}_{shelf_id}_L{row}.png"
            title = f"{code}  {shelf_id}-L{row}"
            subtitle = f"AprilTag ID {tag_id} | front layer"

        db["tags"][str(tag_id)] = item

        label = make_label(
            tag_id=tag_id,
            title=title,
            subtitle=subtitle,
            marker_px=marker_px,
            label_px=label_px,
        )

        path = out_dir / filename
        label.save(path)
        label_paths.append(path)

    return label_paths


def make_pdf(label_paths, pdf_path: Path):
    """
    Make printable sheets.

    Some Ubuntu/Pillow builds do not have JPEG encoder enabled.
    PIL's PDF writer may internally call JPEG and fail with:
      KeyError: 'JPEG'

    Therefore:
      1. Always generate PNG sheet pages.
      2. Try to generate PDF.
      3. If PDF fails, keep PNG sheets as the printable output.
    """
    page_w = 2480
    page_h = 3508
    gap = 40

    pages = []
    page = Image.new("RGB", (page_w, page_h), "white")
    x = gap
    y = gap
    row_h = 0

    for path in label_paths:
        img = Image.open(path).convert("RGB")
        w, h = img.size

        if x + w + gap > page_w:
            x = gap
            y += row_h + gap
            row_h = 0

        if y + h + gap > page_h:
            pages.append(page)
            page = Image.new("RGB", (page_w, page_h), "white")
            x = gap
            y = gap
            row_h = 0

        page.paste(img, (x, y))
        x += w + gap
        row_h = max(row_h, h)

    pages.append(page)

    # Always save PNG sheets.
    sheet_paths = []
    for i, im in enumerate(pages, start=1):
        sheet_path = pdf_path.with_name(f"{pdf_path.stem}_page_{i}.png")
        im.save(sheet_path)
        sheet_paths.append(sheet_path)

    # Try PDF, but do not fail if Pillow has no JPEG support.
    try:
        pages[0].save(pdf_path, save_all=True, append_images=pages[1:])
        print("Generated PDF:", pdf_path)
    except Exception as e:
        print("WARNING: PDF generation failed, but PNG sheets were generated.")
        print("PDF error:", repr(e))

    print("Generated printable PNG sheets:")
    for sp in sheet_paths:
        print(" ", sp)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--shelf-start", type=int, default=1)
    parser.add_argument("--shelf-end", type=int, default=5)
    parser.add_argument("--rows", type=int, default=3)
    parser.add_argument("--out-dir", default="/host-home/maps/apriltag_labels")
    parser.add_argument("--db", default="/host-home/maps/apriltag_shelf_database.json")
    parser.add_argument("--marker-px", type=int, default=520)
    parser.add_argument("--label-px", type=int, default=720)
    parser.add_argument("--no-home", action="store_true")
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    db = {
        "family": "DICT_APRILTAG_36h11",
        "rule": {
            "home": "tag_id = 0",
            "shelf": "tag_id = shelf_number * 10 + row",
            "row_0": "side column tag",
            "row_positive": "front layer tag"
        },
        "tags": {}
    }

    label_paths = []

    if not args.no_home:
        label_paths.append(add_home_label(db, out_dir, args.marker_px, args.label_px))

    for shelf_number in range(args.shelf_start, args.shelf_end + 1):
        label_paths.extend(
            add_shelf_labels(
                db=db,
                out_dir=out_dir,
                shelf_number=shelf_number,
                rows=args.rows,
                marker_px=args.marker_px,
                label_px=args.label_px,
            )
        )

    db_path = Path(args.db)
    db_path.parent.mkdir(parents=True, exist_ok=True)
    db_path.write_text(json.dumps(db, ensure_ascii=False, indent=2), encoding="utf-8")

    pdf_path = out_dir / "apriltag_shelf_labels.pdf"
    make_pdf(label_paths, pdf_path)

    print("Generated labels:", out_dir)
    print("Generated database:", db_path)
    print("Generated PDF:", pdf_path)
    print("Rule:")
    print("  HOME: tag_id 0")
    print("  Shelf: tag_id = shelf_number * 10 + row")
    print("Examples:")
    print("  HOME = 0")
    print("  S03 side = 30 display=0300")
    print("  S03-L1 = 31 display=0301")


if __name__ == "__main__":
    main()

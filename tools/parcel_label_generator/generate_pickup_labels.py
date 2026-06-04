#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import csv
import hashlib
import logging
import re
import sys
import tempfile
from pathlib import Path
from typing import Dict, List

import yaml

# ============================================================
# ReportLab md5 compatibility patch
# ============================================================
try:
    import reportlab.pdfbase.pdfdoc as pdfdoc

    def _safe_md5(*args, **kwargs):
        kwargs.pop("usedforsecurity", None)
        return hashlib.md5(*args, **kwargs)

    pdfdoc.md5 = _safe_md5
except Exception:
    pass

from PIL import Image, ImageChops
from reportlab.lib import colors
from reportlab.lib.units import mm
from reportlab.lib.utils import ImageReader
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.cidfonts import UnicodeCIDFont
from reportlab.pdfgen import canvas
from reportlab.graphics.barcode import createBarcodeDrawing
from reportlab.graphics import renderPDF

try:
    from pystrich.datamatrix import DataMatrixEncoder
except ImportError:
    DataMatrixEncoder = None


logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
log = logging.getLogger("parcel_label_generator")


ALNUM_RE = re.compile(r"^[A-Za-z0-9]{1,4}$")
PICKUP_CODE_RE = re.compile(r"^[A-Za-z0-9]{4}$")


# ============================================================
# 固定标签参数
# ============================================================
LABEL_W_MM = 80.0
LABEL_H_MM = 60.0

# 右侧 Data Matrix 固定 3.2cm × 3.2cm
DM_SIZE_MM = 32.0

# 底部条形码区域上边界
SPLIT_Y_MM = 18.5


def pt_mm(value: float) -> float:
    return float(value) * mm


def load_yaml(path: str | Path) -> dict:
    path = Path(path)
    log.info(f"读取配置文件: {path.resolve()}")

    if not path.exists():
        raise FileNotFoundError(f"找不到配置文件: {path.resolve()}")

    with path.open("r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    if not config:
        raise ValueError("config.yaml 为空")

    return config


def register_fonts(config: dict) -> Dict[str, str]:
    zh_font = config.get("font", {}).get("chinese_font", "STSong-Light")

    try:
        pdfmetrics.registerFont(UnicodeCIDFont(zh_font))
    except Exception:
        zh_font = "STSong-Light"
        pdfmetrics.registerFont(UnicodeCIDFont(zh_font))

    return {
        "zh": zh_font,
        "latin": config.get("font", {}).get("latin_font", "Helvetica"),
        "latin_bold": config.get("font", {}).get("latin_bold_font", "Helvetica-Bold"),
    }


def read_rows(path: str | Path, delimiter: str = ",") -> List[Dict[str, str]]:
    path = Path(path)
    log.info(f"读取数据文件: {path.resolve()}")

    if not path.exists():
        raise FileNotFoundError(f"找不到数据文件: {path.resolve()}")

    lines = [
        line.strip()
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.strip().startswith("#")
    ]

    if not lines:
        raise ValueError("labels.txt 没有有效数据")

    first = lines[0].lower().replace(" ", "")
    has_header = all(k in first for k in ["col", "row", "pickup_code", "order_no"])

    rows: List[Dict[str, str]] = []

    if has_header:
        reader = csv.DictReader(lines, delimiter=delimiter)
        for item in reader:
            rows.append({
                (k or "").strip(): (v or "").strip()
                for k, v in item.items()
            })
    else:
        reader = csv.reader(lines, delimiter=delimiter)
        for parts in reader:
            parts = [p.strip() for p in parts]

            if len(parts) < 4:
                raise ValueError("无表头格式至少需要 4 列：col,row,pickup_code,order_no[,date]")

            rows.append({
                "col": parts[0],
                "row": parts[1],
                "pickup_code": parts[2],
                "order_no": parts[3],
                "date": parts[4] if len(parts) >= 5 else "",
            })

    log.info(f"解析到标签数据: {len(rows)} 条")
    return rows


def normalize_row(row: Dict[str, str], default_date: str = "") -> Dict[str, str]:
    col = row.get("col", "").strip().upper()
    row_id = row.get("row", "").strip().upper()
    pickup_code = row.get("pickup_code", "").strip().upper()
    order_no = row.get("order_no", "").strip()
    date = row.get("date", "").strip() or default_date

    if not ALNUM_RE.match(col):
        raise ValueError(f"列 col 必须是 1-4 位数字或字母组合，当前为: {col}")

    if not ALNUM_RE.match(row_id):
        raise ValueError(f"行 row 必须是 1-4 位数字或字母组合，当前为: {row_id}")

    if not PICKUP_CODE_RE.match(pickup_code):
        raise ValueError(f"pickup_code 必须是 4 位数字或字母组合，当前为: {pickup_code}")

    if not order_no:
        raise ValueError("order_no 不能为空")

    pickup_full = f"{col}-{row_id}-{pickup_code}"

    return {
        "col": col,
        "row": row_id,
        "pickup_code": pickup_code,
        "pickup_full": pickup_full,
        "order_no": order_no,
        "date": date,
    }


def fit_font_size(
    c: canvas.Canvas,
    text: str,
    font_name: str,
    max_size: float,
    min_size: float,
    max_width: float,
) -> float:
    size = float(max_size)

    while size > min_size and c.stringWidth(text, font_name, size) > max_width:
        size -= 0.5

    return size


def draw_text_fit(
    c: canvas.Canvas,
    text: str,
    x: float,
    y: float,
    font_name: str,
    max_size: float,
    min_size: float,
    max_width: float,
) -> float:
    size = fit_font_size(c, text, font_name, max_size, min_size, max_width)
    c.setFont(font_name, size)
    c.drawString(x, y, text)
    return size


def create_single_datamatrix_image(value: str) -> Image.Image:
    """
    用 pystrich 生成“单个 Data Matrix”，不是 ReportLab 那种 44x44 拼接样式。
    然后裁掉多余白边，再重新加安静区。
    """
    if DataMatrixEncoder is None:
        raise ImportError(
            "缺少 pystrich，请先运行：pip install pystrich pillow"
        )

    encoder = DataMatrixEncoder(value)

    with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as tmp:
        tmp_path = Path(tmp.name)

    try:
        encoder.save(str(tmp_path))

        img = Image.open(tmp_path).convert("L")

        # 二值化
        img = img.point(lambda p: 0 if p < 128 else 255, "L")

        # 裁剪黑色内容区域
        inv = ImageChops.invert(img)
        bbox = inv.getbbox()

        if bbox is None:
            raise ValueError("Data Matrix 图片生成失败，未检测到内容")

        img = img.crop(bbox)

        # 重新加白色安静区
        quiet = max(8, int(min(img.size) * 0.18))
        canvas_img = Image.new(
            "L",
            (img.width + quiet * 2, img.height + quiet * 2),
            255,
        )
        canvas_img.paste(img, (quiet, quiet))

        # 转 RGB，避免 PDF 嵌入异常
        return canvas_img.convert("RGB")

    finally:
        try:
            tmp_path.unlink(missing_ok=True)
        except Exception:
            pass


def draw_datamatrix_single_32mm(
    c: canvas.Canvas,
    value: str,
    x: float,
    y: float,
    debug_frame: bool = False,
) -> None:
    """
    绘制一个 Data Matrix 图片，整体强制显示为 32mm × 32mm。
    注意：这里不是 ReportLab 的 ECC200DataMatrix。
    """
    box = pt_mm(DM_SIZE_MM)

    img = create_single_datamatrix_image(value)
    reader = ImageReader(img)

    c.drawImage(
        reader,
        x,
        y,
        width=box,
        height=box,
        preserveAspectRatio=False,
        mask="auto",
    )

    if debug_frame:
        c.saveState()
        c.setStrokeColor(colors.red)
        c.setLineWidth(0.2 * mm)
        c.rect(x, y, box, box, stroke=1, fill=0)
        c.restoreState()


def draw_code128(
    c: canvas.Canvas,
    value: str,
    x: float,
    y: float,
    width: float,
    height: float,
    bar_width_mm: float,
) -> None:
    barcode = createBarcodeDrawing(
        "Code128",
        value=value,
        barHeight=height,
        barWidth=pt_mm(bar_width_mm),
        humanReadable=False,
    )

    scale = min(width / barcode.width, height / barcode.height)

    c.saveState()
    c.translate(x + (width - barcode.width * scale) / 2, y)
    c.scale(scale, scale)
    renderPDF.draw(barcode, c, 0, 0)
    c.restoreState()


def draw_label(
    c: canvas.Canvas,
    data: Dict[str, str],
    config: dict,
    fonts: Dict[str, str],
) -> None:
    label_w = pt_mm(LABEL_W_MM)
    label_h = pt_mm(LABEL_H_MM)

    outer_margin = pt_mm(1.8)
    left_x = pt_mm(3.2)

    split_y = pt_mm(SPLIT_Y_MM)

    # 右侧固定 32mm × 32mm Data Matrix
    dm_box = pt_mm(DM_SIZE_MM)
    dm_x = label_w - pt_mm(4.0) - dm_box
    dm_y = split_y + pt_mm(3.0)

    # 左侧文字安全宽度
    left_max_w = dm_x - pt_mm(3.0) - left_x

    # 外框
    c.setStrokeColor(colors.black)
    c.setFillColor(colors.black)

    c.setLineWidth(pt_mm(0.35))
    c.roundRect(
        outer_margin,
        outer_margin,
        label_w - outer_margin * 2,
        label_h - outer_margin * 2,
        pt_mm(1.8),
        stroke=1,
        fill=0,
    )

    # 分割线
    c.setLineWidth(pt_mm(0.25))
    c.line(outer_margin, split_y, label_w - outer_margin, split_y)

    # 标题
    c.setFont(fonts["zh"], 11.5)
    c.drawString(
        left_x,
        label_h - pt_mm(5.5),
        config.get("text", {}).get("header", "快递驿站取件标签"),
    )

    # 大取件码
    draw_text_fit(
        c,
        data["pickup_full"],
        left_x,
        label_h - pt_mm(16.5),
        fonts["latin_bold"],
        22,
        14,
        left_max_w,
    )

    # 黑底取件码
    badge_y = label_h - pt_mm(24.0)
    badge_w = pt_mm(14.5)
    badge_h = pt_mm(5.2)

    c.setFillColor(colors.black)
    c.roundRect(
        left_x,
        badge_y,
        badge_w,
        badge_h,
        pt_mm(0.8),
        stroke=0,
        fill=1,
    )

    c.setFillColor(colors.white)
    c.setFont(fonts["zh"], 9.0)
    c.drawString(
        left_x + pt_mm(1.4),
        badge_y + pt_mm(1.2),
        config.get("text", {}).get("pickup_badge", "取件码"),
    )

    c.setFillColor(colors.black)

    # 小字
    date_text = f"{config.get('text', {}).get('date_label', '日期：')} {data['date']}"
    order_text = f"{config.get('text', {}).get('order_label', '订单号：')} {data['order_no']}"
    slot_text = config.get("text", {}).get("slot_format", "货位格式：列-行-4位取件码")

    draw_text_fit(
        c,
        date_text,
        left_x,
        pt_mm(28.0),
        fonts["zh"],
        7.2,
        5.6,
        left_max_w,
    )

    draw_text_fit(
        c,
        order_text,
        left_x,
        pt_mm(24.0),
        fonts["zh"],
        7.0,
        5.4,
        left_max_w,
    )

    c.setDash(1.2, 1.2)
    c.setLineWidth(pt_mm(0.16))
    c.line(left_x, pt_mm(22.0), dm_x - pt_mm(3.0), pt_mm(22.0))
    c.setDash()

    draw_text_fit(
        c,
        slot_text,
        left_x,
        pt_mm(19.8),
        fonts["zh"],
        6.8,
        5.2,
        left_max_w,
    )

    # 右侧唯一 Data Matrix：单个码，整体 32mm × 32mm
    debug_frame = bool(config.get("debug", {}).get("dm_frame", False))
    draw_datamatrix_single_32mm(
        c,
        data["pickup_full"],
        dm_x,
        dm_y,
        debug_frame=debug_frame,
    )

    # 底部 Code128 条形码
    barcode_x = pt_mm(16.0)
    barcode_y = pt_mm(6.8)
    barcode_w = label_w - barcode_x * 2
    barcode_h = pt_mm(8.5)

    draw_code128(
        c,
        data["order_no"],
        barcode_x,
        barcode_y,
        barcode_w,
        barcode_h,
        0.25,
    )

    # 条形码下方订单号
    if config.get("barcode", {}).get("draw_text_below", True):
        text_size = 7.2
        c.setFont(fonts["latin"], text_size)
        text_w = c.stringWidth(data["order_no"], fonts["latin"], text_size)
        c.drawString((label_w - text_w) / 2, pt_mm(3.4), data["order_no"])


def generate_pdf(config_path: str, input_path: str, output_path: str) -> None:
    log.info("开始生成标签")

    config = load_yaml(config_path)
    fonts = register_fonts(config)

    delimiter = config.get("input", {}).get("delimiter", ",")
    default_date = config.get("input", {}).get("default_date", "")

    raw_rows = read_rows(input_path, delimiter)
    rows = [normalize_row(row, default_date) for row in raw_rows]

    output = Path(output_path)

    label_w = pt_mm(LABEL_W_MM)
    label_h = pt_mm(LABEL_H_MM)

    log.info(f"标签尺寸固定为: {LABEL_W_MM}mm × {LABEL_H_MM}mm")
    log.info(f"Data Matrix 使用 pystrich 生成单个码")
    log.info(f"Data Matrix 显示尺寸固定为: {DM_SIZE_MM}mm × {DM_SIZE_MM}mm")
    log.info(f"输出文件: {output.resolve()}")

    c = canvas.Canvas(str(output), pagesize=(label_w, label_h))

    for i, item in enumerate(rows, start=1):
        log.info(f"生成第 {i} 张: {item['pickup_full']} / {item['order_no']}")
        draw_label(c, item, config, fonts)
        c.showPage()

    c.save()

    log.info(f"生成完成: {output.resolve()}")
    log.info(f"共生成 {len(rows)} 张标签")


def main() -> None:
    parser = argparse.ArgumentParser(description="快递驿站取件标签批量生成工具")
    parser.add_argument("--config", "-c", default="config.yaml")
    parser.add_argument("--input", "-i", default="labels.txt")
    parser.add_argument("--output", "-o", default="output_labels.pdf")
    args = parser.parse_args()

    try:
        generate_pdf(args.config, args.input, args.output)
    except Exception as e:
        log.exception(f"生成失败: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
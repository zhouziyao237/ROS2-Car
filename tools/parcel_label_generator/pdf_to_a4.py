#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path

from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm

from generate_pickup_labels import (
    load_yaml,
    register_fonts,
    read_rows,
    normalize_row,
    draw_label,
)

# =========================
# A4 拼版参数
# =========================
A4_MARGIN_MM = 8      # A4 页边距
GAP_X_MM = 4          # 标签左右间距
GAP_Y_MM = 4          # 标签上下间距

LABEL_W_MM = 80       # 单张标签宽度
LABEL_H_MM = 60       # 单张标签高度


def pdf_to_a4_direct(
    config_path="config.yaml",
    input_path="labels.txt",
    output_path="labels_a4.pdf",
):
    config = load_yaml(config_path)
    fonts = register_fonts(config)

    delimiter = config.get("input", {}).get("delimiter", ",")
    default_date = config.get("input", {}).get("default_date", "")

    raw_rows = read_rows(input_path, delimiter)
    rows = [normalize_row(row, default_date) for row in raw_rows]

    page_w, page_h = A4

    label_w = LABEL_W_MM * mm
    label_h = LABEL_H_MM * mm

    margin = A4_MARGIN_MM * mm
    gap_x = GAP_X_MM * mm
    gap_y = GAP_Y_MM * mm

    cols = int((page_w - 2 * margin + gap_x) // (label_w + gap_x))
    rows_per_page = int((page_h - 2 * margin + gap_y) // (label_h + gap_y))
    labels_per_page = cols * rows_per_page

    print(f"每页 {cols} 列 × {rows_per_page} 行 = {labels_per_page} 张标签")
    print(f"总共 {len(rows)} 张标签")

    c = canvas.Canvas(output_path, pagesize=A4)

    for index, item in enumerate(rows):
        position_in_page = index % labels_per_page

        if index > 0 and position_in_page == 0:
            c.showPage()

        col = position_in_page % cols
        row = position_in_page // cols

        x = margin + col * (label_w + gap_x)
        y = page_h - margin - label_h - row * (label_h + gap_y)

        c.saveState()
        c.translate(x, y)

        # 直接复用原来的标签绘制函数
        draw_label(c, item, config, fonts)

        c.restoreState()

    c.save()

    print(f"生成完成：{Path(output_path).resolve()}")


if __name__ == "__main__":
    pdf_to_a4_direct(
        config_path="config.yaml",
        input_path="labels_20.txt",
        output_path="labels_a4.pdf",
    )
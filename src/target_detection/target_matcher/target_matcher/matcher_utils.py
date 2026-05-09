import re
from typing import List, Tuple


def normalize_code(value: str) -> str:
    text = str(value or "").upper().strip()
    return "".join(ch for ch in text if ch.isalnum())


def extract_digit_sequences(value: str) -> List[str]:
    try:
        return re.findall(r"\d+", str(value or ""))
    except Exception:
        return []


def match_target_code(target_code: str, raw_text: str) -> Tuple[bool, str]:
    """Match user target code against OCR merged text.

    Supports:
    - full normalized substring matching, e.g. 2-3-2050 -> 232050
    - short numeric code <= 5 digits using suffix/contains matching
    """
    target_norm = normalize_code(target_code)
    text_norm = normalize_code(raw_text)

    if not target_norm or not text_norm:
        return False, ""

    if target_norm in text_norm or text_norm in target_norm:
        return True, target_norm

    if target_norm.isdigit() and len(target_norm) <= 5:
        digit_tokens = extract_digit_sequences(raw_text)
        if not digit_tokens:
            digit_tokens = extract_digit_sequences(text_norm)

        for token in digit_tokens:
            if token and token.endswith(target_norm):
                return True, token

        for token in digit_tokens:
            if token and target_norm in token:
                return True, token

    return False, ""


def merge_ocr_lines(lines: List[str]) -> str:
    return " ".join(str(line or "").strip() for line in lines if str(line or "").strip())


def score_ocr_text(lines: List[str], confidences: List[float]) -> float:
    total_conf = 0.0
    total_chars = 0
    for text, conf in zip(lines, confidences):
        clean = str(text or "").strip()
        if not clean:
            continue
        total_conf += float(conf)
        total_chars += len(clean)
    return total_conf + total_chars * 0.02

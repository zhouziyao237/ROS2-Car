#!/usr/bin/env python3
import argparse
import glob
import json
from pathlib import Path

import cv2
import numpy as np
import yaml


def find_corners(gray, pattern_size):
    if hasattr(cv2, 'findChessboardCornersSB'):
        flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
        ok, corners = cv2.findChessboardCornersSB(gray, pattern_size, flags)
        if ok and corners is not None:
            return True, corners.astype(np.float32)

    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
    if not ok or corners is None:
        return False, None
    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, corners


def reprojection_errors(objpoints, imgpoints, rvecs, tvecs, camera_matrix, dist_coeffs):
    errors = []
    total_err = 0.0
    total_points = 0
    for objp, imgp, rvec, tvec in zip(objpoints, imgpoints, rvecs, tvecs):
        projected, _ = cv2.projectPoints(objp, rvec, tvec, camera_matrix, dist_coeffs)
        err = cv2.norm(imgp, projected, cv2.NORM_L2)
        n = len(projected)
        rms = float(np.sqrt((err * err) / n))
        errors.append(rms)
        total_err += err * err
        total_points += n
    overall = float(np.sqrt(total_err / max(total_points, 1)))
    return overall, errors


def parse_args():
    parser = argparse.ArgumentParser(description='Calibrate RGB camera intrinsics from chessboard images.')
    parser.add_argument('--image-dir', required=True)
    parser.add_argument('--cols', type=int, default=9, help='inner corners per chessboard row')
    parser.add_argument('--rows', type=int, default=6, help='inner corners per chessboard column')
    parser.add_argument('--square-mm', type=float, default=25.0)
    parser.add_argument('--output', default='oak_rgb_intrinsics.yaml')
    parser.add_argument('--preview-dir', default=None, help='optional directory to save corner preview images')
    parser.add_argument('--fix-k3', action='store_true', help='fix k3 to zero')
    parser.add_argument('--rational-model', action='store_true', help='estimate k4-k6 too')
    return parser.parse_args()


def main():
    args = parse_args()
    pattern_size = (args.cols, args.rows)
    image_paths = []
    for ext in ('*.png', '*.jpg', '*.jpeg', '*.bmp'):
        image_paths.extend(glob.glob(str(Path(args.image_dir) / ext)))
    image_paths = sorted(image_paths)

    if not image_paths:
        raise SystemExit(f'No images found in {args.image_dir}')

    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2)
    objp *= float(args.square_mm)

    objpoints = []
    imgpoints = []
    used_images = []
    image_size = None

    preview_dir = Path(args.preview_dir) if args.preview_dir else None
    if preview_dir:
        preview_dir.mkdir(parents=True, exist_ok=True)

    for path in image_paths:
        img = cv2.imread(path)
        if img is None:
            print(f'[SKIP] cannot read {path}')
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = (gray.shape[1], gray.shape[0])
        elif image_size != (gray.shape[1], gray.shape[0]):
            print(f'[SKIP] size mismatch {path}: {gray.shape[1]}x{gray.shape[0]} != {image_size}')
            continue

        ok, corners = find_corners(gray, pattern_size)
        if not ok:
            print(f'[NO CORNERS] {path}')
            continue

        objpoints.append(objp.copy())
        imgpoints.append(corners)
        used_images.append(path)
        print(f'[OK] {path}')

        if preview_dir:
            vis = img.copy()
            cv2.drawChessboardCorners(vis, pattern_size, corners, ok)
            cv2.imwrite(str(preview_dir / Path(path).name), vis)

    if len(objpoints) < 12:
        raise SystemExit(f'Only {len(objpoints)} valid images. Use at least 12, preferably 30-60.')

    flags = 0
    if args.fix_k3:
        flags |= cv2.CALIB_FIX_K3
    if args.rational_model:
        flags |= cv2.CALIB_RATIONAL_MODEL

    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints,
        imgpoints,
        image_size,
        None,
        None,
        flags=flags,
    )

    overall_error, per_image_errors = reprojection_errors(
        objpoints,
        imgpoints,
        rvecs,
        tvecs,
        camera_matrix,
        dist_coeffs,
    )

    dist = dist_coeffs.reshape(-1).astype(float).tolist()
    # ROS plumb_bob normally uses 5 coefficients. If rational model used, keep all but also provide first five.
    first5 = (dist + [0.0] * 5)[:5]

    fx = float(camera_matrix[0, 0])
    fy = float(camera_matrix[1, 1])
    cx = float(camera_matrix[0, 2])
    cy = float(camera_matrix[1, 2])
    width, height = image_size

    projection = [
        fx, 0.0, cx, 0.0,
        0.0, fy, cy, 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]

    output = {
        'camera_name': 'oak_rgb_1080p',
        'image_width': int(width),
        'image_height': int(height),
        'fx': fx,
        'fy': fy,
        'cx': cx,
        'cy': cy,
        'camera_matrix': {
            'rows': 3,
            'cols': 3,
            'data': camera_matrix.reshape(-1).astype(float).tolist(),
        },
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': {
            'rows': 1,
            'cols': len(first5),
            'data': first5,
        },
        'dist_coeffs_all': dist,
        'rectification_matrix': {
            'rows': 3,
            'cols': 3,
            'data': np.eye(3).reshape(-1).astype(float).tolist(),
        },
        'projection_matrix': {
            'rows': 3,
            'cols': 4,
            'data': projection,
        },
        'square_mm': float(args.square_mm),
        'pattern_inner_corners': [int(args.cols), int(args.rows)],
        'valid_image_count': len(used_images),
        'input_image_count': len(image_paths),
        'opencv_rms': float(ret),
        'reprojection_error_px': overall_error,
        'per_image_reprojection_error_px': per_image_errors,
        'used_images': used_images,
        'recommended_oak_parcel_pose_launch_args': {
            'use_camera_info': False,
            'manual_fx': fx,
            'manual_fy': fy,
            'manual_cx': cx,
            'manual_cy': cy,
        },
    }

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(output, f, allow_unicode=True, sort_keys=False)

    print('\nCalibration complete')
    print(f'valid images: {len(used_images)} / {len(image_paths)}')
    print(f'image size: {width}x{height}')
    print(f'fx={fx:.6f}')
    print(f'fy={fy:.6f}')
    print(f'cx={cx:.6f}')
    print(f'cy={cy:.6f}')
    print(f'dist_coeffs first5={first5}')
    print(f'OpenCV RMS={float(ret):.6f}')
    print(f'Reprojection error={overall_error:.6f} px')
    print(f'output: {out_path}')
    print('\nUse these launch args if oak_parcel_pose supports manual intrinsics:')
    print(f'  use_camera_info:=false manual_fx:={fx:.9f} manual_fy:={fy:.9f} manual_cx:={cx:.9f} manual_cy:={cy:.9f}')


if __name__ == '__main__':
    main()

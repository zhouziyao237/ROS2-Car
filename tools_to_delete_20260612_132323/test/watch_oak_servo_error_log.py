#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Standalone helper to show OAK servo error values.

Usage after source install/setup.bash:
  python3 tools/watch_oak_servo_error_log.py
  python3 tools/watch_oak_servo_error_log.py --csv /tmp/oak_servo_error.csv
"""

from oak_package_servo.oak_servo_error_watch import main

if __name__ == '__main__':
    main()

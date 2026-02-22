#!/usr/bin/env python3
import sys
sys.path.append('../build/bin/lib')

from build.bin.lib import sdk_py

# 测试基本功能
print("SDK版本:", sdk_py.get_version())
print("可用的类:", [attr for attr in dir(sdk_py) if not attr.startswith('_')])

# 测试Dev类
try:
    dev = sdk_py.Dev()
    print("Dev对象创建成功")
except Exception as e:
    print("Dev对象创建失败:", e)
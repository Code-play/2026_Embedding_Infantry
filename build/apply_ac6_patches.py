# -*- coding: utf-8 -*-
"""apply_ac6_patches.py -- 把 AC5->AC6 迁移必需的三处 user_code 补丁打到一份 user_code 副本上。

用法：
    python fw/keil_ac6/apply_ac6_patches.py --user-code <user_code 目录>

三处补丁（都是幂等的，可重复执行）：
  1. weak 回调：bsp_fdcan.h 的 `__weak void fdcanN_rx_callback(void);` 去掉 __weak，
     并删掉 bsp_fdcan.cpp 里三个空兜底实现。
     原因：armclang/GCC 会把声明上的 weak 属性传染到 Communicate.cpp 里的真定义，
     链接器选中空实现 → CAN 接收回调永不执行。ARMCC 无此行为（D10 已记录）。
  2. __packed：VT13.h / Vision.h 的 `typedef __packed struct` 在 AC6 会被**忽略**
     （-Wignored-attributes）。Vision.cpp:117 是 raw memcpy 进结构体，错位就直接解析错。
     改成 `typedef struct __attribute__((packed))`/嵌套同理。
  3. 静态断言：给两个协议结构体加 28 / 34 字节的编译期长度断言，防止回归。
"""
import argparse
import io
import os
import sys

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

WEAK_BODIES_OLD = """__weak void fdcan1_rx_callback(void){

}

__weak void fdcan2_rx_callback(void)
{

}

__weak void fdcan3_rx_callback(void){
\t
}"""

WEAK_BODIES_NEW = """/* [AC6 迁移补丁] 删掉三个空的弱回调兜底实现（否则链接器会选中空实现，
 * CAN 接收回调永不执行；等价于 fw/params_whitelist.json 的 weak_callback_* 补丁）。 */"""

VT13_ASSERT = """

/* [AC6 迁移] packed 必须真的生效（AC5 的 __packed 布局 = attribute((packed))）：
 * rc 15 + mouse 9 + key 2 + crc16 2 = 28 字节。 */
#if defined(__clang__)
_Static_assert(sizeof(VT13_RC_ctrl_t) == 28, "VT13_RC_ctrl_t must be 28 bytes (packed)");
#endif
"""

VISION_ASSERT = """

/* [AC6 迁移] 视觉接收包必须 34 字节：Vision.cpp 直接 memcpy 进这个结构体，
 * 若 packed 失效（AC6 会忽略 typedef 上的 __packed）就会错位。 */
#if defined(__clang__)
_Static_assert(sizeof(VisionRecvData_t) == 34, "VisionRecvData_t must be 34 bytes (packed)");
#endif
"""


def _rw(path, fn):
    s = io.open(path, encoding='utf-8', errors='replace').read()
    s2 = fn(s)
    if s2 != s:
        io.open(path, 'w', encoding='utf-8', newline='\n').write(s2)
        return True
    return False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--user-code', required=True)
    a = ap.parse_args()
    root = a.user_code
    changed = []

    # 1. weak 回调
    h = os.path.join(root, 'bsp', 'bsp_fdcan.h')
    if _rw(h, lambda s: s.replace('__weak void fdcan', 'void fdcan')):
        changed.append('bsp/bsp_fdcan.h (remove __weak on rx callbacks)')
    cpp = os.path.join(root, 'bsp', 'bsp_fdcan.cpp')
    if _rw(cpp, lambda s: s.replace(WEAK_BODIES_OLD, WEAK_BODIES_NEW)):
        changed.append('bsp/bsp_fdcan.cpp (drop empty weak bodies)')

    # 2. __packed
    for name in ('VT13.h', 'Vision.h'):
        p = os.path.join(root, 'device', name)
        if os.path.isfile(p) and _rw(p, lambda s: s.replace('__packed struct', 'struct __attribute__((packed))')):
            changed.append('device/' + name + ' (__packed -> attribute((packed)))')

    # 3. 静态断言
    p = os.path.join(root, 'device', 'VT13.h')
    if os.path.isfile(p) and _rw(p, lambda s: s if 'VT13_RC_ctrl_t must be 28' in s else s.rstrip() + VT13_ASSERT):
        changed.append('device/VT13.h (+ sizeof assert 28)')
    p = os.path.join(root, 'device', 'Vision.h')
    if os.path.isfile(p) and _rw(p, lambda s: s if 'VisionRecvData_t must be 34' in s else s.rstrip() + VISION_ASSERT):
        changed.append('device/Vision.h (+ sizeof assert 34)')

    if changed:
        print('applied patches:')
        for c in changed:
            print('  -', c)
    else:
        print('nothing to do (already patched)')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

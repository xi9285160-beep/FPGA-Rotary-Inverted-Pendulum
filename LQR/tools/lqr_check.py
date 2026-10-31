#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lqr_check.py — LQR 增益换算核对（IP570 旋转倒立摆 -> J280 FPGA 工程）

重要前提（已核对）：
    附送资料 `5.LQR控制MATLAB程序源码/*/parameter.m` 里的
        m, M, l, J, R, r, K_m, K_e, I （M = 小车质量、r = 同步带轮半径）
    是 **直线倒立摆** 的模型 —— 用它的 Q=diag(6000,0,2000,0) 算出的
        K = [-61.1, -50.5, -107.0, -19.0]
    与直线版源码里发布的
        K = [-64.32, -51.71, -109.48, -19.56]
    几乎一致。用同一份 A/B 去验证「旋转版」源码里的
        K = [-1.6349, -1.7563, +26.8, +2.3682]
    会得到发散（谱半径 1.28），因为两者根本不是同一个模型。

    旋转倒立摆的外购资料里没有给出建模脚本（缺摆臂长度、减速比、
    电机转子惯量），所以本工程**直接采用 WHEELTEC 旋转版源码里的 K**，
    它是在同一套 IP570 机构上发布并使用的。

本脚本做两件有用的事：
  1. 把 K 从 (rad, V) 换算成 (count, PWM)，核对 lqr_ctrl.v 里的 Q8 参数
  2. 把两套 PD 与两套 LQR 都折算到物理单位 (V/rad, V/(rad/s)) 对比，
     判断增益量级是否可信
"""

import math

# ---------------------------------------------------------------- 单位换算
CNT_PER_REV = 1040.0            # 编码器（与 STM32 / FPGA 一致）
CNT_PER_DEG = 11.92             # 角度传感器实测（下垂 936 / 竖直 3082，差 2146 = 180°）
PWM_PERIOD = 5400.0             # motor_drive.v 载波
VOLT_NOM = 12.0                 # 假设电池 12V

rad_per_cnt_al = math.pi / 180.0 / CNT_PER_DEG
rad_per_cnt_th = 2 * math.pi / CNT_PER_REV
pwm_per_volt = PWM_PERIOD / VOLT_NOM

# 旋转版源码发布的 K（rad / rad/s / 电压）
K_REF = [-1.6349, -1.7563, 26.8000, 2.3682]

# lqr_ctrl.v 里的 Q8 参数
Q8 = {"K_TH ": 1138, "K_THD": 61130, "K_AL ": 4520, "K_ALD": 19973}

print("=== 1. 换算系数 ===")
print("  alpha : %.6e rad/count   (%.2f counts/deg)" % (rad_per_cnt_al, CNT_PER_DEG))
print("  theta : %.6e rad/count   (%.0f counts/rev)" % (rad_per_cnt_th, CNT_PER_REV))
print("  PWM/V : %.1f  (载波 %.0f / %.0fV)" % (pwm_per_volt, PWM_PERIOD, VOLT_NOM))
print("  20ms 差分的速度换算：theta %.6f rad/s per (count/20ms)，alpha %.6f"
      % (rad_per_cnt_th * 50, rad_per_cnt_al * 50))

print("\n=== 2. K 换算（(rad,V) -> (count,PWM)）===")
items = [
    ("K_TH ", K_REF[0], rad_per_cnt_th,      "theta"),
    ("K_THD", K_REF[1], rad_per_cnt_th * 50, "theta_dot"),
    ("K_AL ", K_REF[2], rad_per_cnt_al,      "alpha"),
    ("K_ALD", K_REF[3], rad_per_cnt_al * 50, "alpha_dot"),
]
ok = True
for name, k, scale, st in items:
    exact = abs(k) * scale * pwm_per_volt
    q = Q8[name] / 256.0
    err = (q - exact) / exact * 100
    ok &= abs(err) < 0.5
    print("  %s  %9.4f PWM/count   Q8=%6d -> %9.4f  (误差 %+.3f%%)  [%s]"
          % (name, exact, Q8[name], q, err, st))
print("  Q8 参数一致性：%s" % ("OK" if ok else "需要修正"))

# ---------------------------------------------------------------- 物理量对比
def v_units(k_al, k_ald, pwm_per_v, rad_per_cnt):
    """把「每 count 的增益」折回 V/rad 与 V/(rad/s)"""
    return (k_al / pwm_per_v / rad_per_cnt,
            k_ald / pwm_per_v / (rad_per_cnt * 50.0))

print("\n=== 3. 同一机构的 PD / LQR 物理增益对比（V/rad, V/(rad/s)）===")
# 参考板（STM32）：载波 7200 -> 600 PWM/V，0.0879 deg/count
ref_rad_per_cnt = 0.0879 * math.pi / 180.0
p_ref = v_units(50.0, 244.0 * 4.0, 600.0, ref_rad_per_cnt)     # KD 为 5ms 差分，*4 折到 20ms
k_ref = (26.8, 2.3682)      # 见 control.c：u(V) = -(k1*th+k2*th'+k3*al+k4*al')
p_our = v_units(50.0, 230.0 * 4.0, 450.0, rad_per_cnt_al)
k_our = v_units(Q8["K_AL "] / 256.0, Q8["K_ALD"] / 256.0, 450.0, rad_per_cnt_al)

print("  参考板 PD  (KP=50, KD=244@5ms) : K_alpha=%7.2f  K_alphad=%6.2f" % p_ref)
print("  参考板 LQR (26.8 / 2.3682)     : K_alpha=%7.2f  K_alphad=%6.2f" % k_ref)
print("  本板   PD  (KP=50, KD=230@5ms) : K_alpha=%7.2f  K_alphad=%6.2f" % p_our)
print("  本板   LQR (17.66 / 78.0)      : K_alpha=%7.2f  K_alphad=%6.2f" % k_our)
print("  -> 本板 LQR 与参考板 LQR 物理增益应相等，只差 4.8%% 的角度标定误差")

print("\n=== 4. 典型状态下的输出（PWM，限幅 ±5300）===")
def u_pwm(al_cnt, ald_cnt20, th_cnt=0, thd_cnt20=0):
    acc = (Q8["K_TH "] * th_cnt + Q8["K_THD"] * thd_cnt20 +
           Q8["K_AL "] * al_cnt + Q8["K_ALD"] * ald_cnt20)
    return max(-5300, min(5300, -(acc >> 8)))

print("  ---- 角度项 ----")
for al_cnt in (10, 30, 60, 120, 240, 450):
    u = u_pwm(al_cnt, 0)
    print("  alpha=%4d cnt (%5.2f°)  -> u=%+6d  (%.1f%% 满占空比)"
          % (al_cnt, al_cnt / CNT_PER_DEG, u, abs(u) / PWM_PERIOD * 100))
print("  ---- 角速度项 ----")
for ald in (1, 2, 5, 10, 20):
    print("  alpha_dot=%3d cnt/20ms (%6.1f°/s) -> u=%+6d"
          % (ald, ald * 50.0 / CNT_PER_DEG, u_pwm(0, ald)))
print("  ---- 位置项 ----")
for th in (30, 87, 300, 1000):
    print("  theta_err=%4d cnt (%5.1f°) -> u=%+6d"
          % (th, th / 2.8889, u_pwm(0, 0, th, 0)))

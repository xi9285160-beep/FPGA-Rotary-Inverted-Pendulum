// ============================================================================
// pendulum_ctrl.v  —  旋转倒立摆控制核心（移植自 IP570 旋转倒立摆 STM32 源码）
//
// 结构（与 control.c 对应）：
//   Balance()  角度环 PD      -> bal_pwm_w
//   Position() 位置环 PD      -> pos_pwm_w
//   Moto = Balance_Pwm - Position_Pwm
//   Auto_run() 自动起摆状态机 -> S_SWING（能量泵）
//   手动辅助起摆               -> S_BALANCE（启动时记录当前位置）
//   保持直立转圈               -> key_inc/key_dec 修改 pos_cmd，pos_target 斜坡跟随
//
// 控制周期：
//   5ms  ：角度环、状态机、输出 PWM
//   25ms ：位置环（与 IP570 的 Position_Target>4 对应）
//
// 参数说明见 README.md
// ============================================================================
module pendulum_ctrl #(
    // =========================================================================
    // STM32_MODE —— 「完全照抄 STM32 原版」兼容模式
    //   = 1 时本模块行为与 WHEELTEC IP570 control.c 一一对应：
    //     · 位置低通用 1/5（0.2/0.8），不是移位 1/8
    //     · 捕获后平衡参考点 = 甩杆终点 FLING_STEP（STM32 的 Position_Zero=step1_target）
    //     · 甩杆单相（不做回杆/减速）
    //     · 起摆超时 600ms（try_count>120）
    //     · 编码器突变阈值 15，且保护无宽限期（STM32 在中断末尾无条件检查）
    //     · 关闭 D 项低通、关闭小增益区
    //     · 手动/自动判定窗口 ±500
    //     · 位置环输出不额外限幅（只受最终 PWM 限幅约束）
    //   刷新率：控制 5ms、位置环 25ms、捕获后前 550ms 位置环 5ms —— 与 STM32 相同。
    //   = 0 时使用本工程调好的参数（默认）。
    // =========================================================================
    parameter integer STM32_MODE       = 0,

    // =========================================================================
    // BAL_STM32 —— 「平衡部分完全照抄 STM32」模式（可与自定义起摆组合）
    //   只影响平衡/位置/零点/保护，不影响起摆方式。
    //   与 STM32_MODE 的区别：STM32_MODE 连起摆一起照抄；
    //   BAL_STM32 只照抄平衡，起摆仍用本工程的甩杆/能量泵。
    //   生效内容（全部对齐 control.c / main.c）：
    //     · 位置低通 1/5（0.2/0.8）
    //     · 平衡参考点 = 甩杆终点（Position_Zero = step1_target，过冲补偿）
    //     · 位置环无积分、无额外限幅
    //     · 关闭 D 项低通、关闭小增益区
    //     · 手动/自动判定窗口 ±500
    //     · 编码器突变 15、无条件检查、无宽限期
    //     · 上电自动角度标定 ZHONGZHI = 下垂 + CAL_SPAN（失败用 ZHONGZHI_DEF）
    //     · PWM 限幅比例对齐 STM32 的 6900/7200
    //   刷新率与 STM32 一致：控制 5ms / 位置环 25ms / 捕获后前 550ms 位置环 5ms。
    // =========================================================================
    parameter integer BAL_STM32        = 0,

    // ---- 上电自动角度标定（对应 STM32 的 Angle_Calibrate）----
    //   STM32 在 main() 里调一次：采样摆杆自然下垂位置 down，
    //   若 |down-CAL_DOWN_TYP|<=CAL_TOL 且 down+CAL_SPAN<=4095，则 ZHONGZHI = down+CAL_SPAN，
    //   否则退回默认 ZHONGZHI_DEF。
    //
    //   【本机取值】实测下垂 ≈1010、竖直 = 2968，故跨度 = 2968-1010 = 1958。
    //   （STM32 原值 2080 是 IP570 的 180° 跨度，与本机机械不同。）
    //   ⚠ 当前 AUTO_CAL = 0（已取消），上电直接使用 ZHONGZHI_DEF = 2968。
    parameter integer AUTO_CAL         = 0,          // 1 = 启用上电自动标定；0 = 直接使用 ZHONGZHI_DEF
    parameter [15:0]  CAL_TICKS        = 16'd64,     // 采样节拍数，必须是 2 的幂
    parameter integer CAL_SHIFT        = 6,          // 必须 = log2(CAL_TICKS)
    parameter [11:0]  CAL_DOWN_TYP     = 12'd1020,   // STM32: ANGLE_DOWN_TYP
    parameter [11:0]  CAL_TOL          = 12'd200,    // STM32: ANGLE_DOWN_TOL
    parameter [11:0]  CAL_SPAN         = 12'd1958,   // 下垂→竖直的跨度（2968-1010）
    parameter [11:0]  ZHONGZHI_DEF     = 12'd2968,   // 标定失败时的平衡点默认值

    // 竖直平衡位置：角度传感器（电位器 ADC）实测读数，直接固定写死，不做任何自动反推。
    // 实测：下垂 ≈1010，真正竖直 = 2968。
    // 若需微调，可在停止状态下拨 B3_SW1，把当前角度设为新的平衡点。
    parameter [11:0] ANGLE_MID_DEFAULT = 12'd2968,
    parameter integer ANGLE_INV        = 0,          // 1 = 角度方向取反
    parameter integer ENC_INV          = 0,          // 1 = 编码器方向取反
    parameter integer SWING_INV        = 0,          // 1 = 起摆能量泵方向取反

    parameter [15:0] MANUAL_WINDOW     = 16'd700,    // 手动模式允许启动的角度窗口
    parameter [15:0] ANGLE_LIMIT       = 16'd800,    // 平衡时摆角超限保护（对应 IP570 的 ZHONGZHI±800）
    // 角度超限的持续确认拍数（×5ms）：必须连续这么多拍都超限才停机。
    //
    //   【为何需要】原来是单拍判定，而 angle 是 10-bit ADC 左移 2 位的 12-bit 值：
    //     正常平衡 ang_err ≈ ±100；若 ADC 读数瞬跳到 0 -> ang_err = -2968，
    //     瞬跳到 4092 -> ang_err = +1124，两者都远超 800，**一拍就停机**。
    //     电机 PWM(9.26kHz) 耦合、电源波动、接插件抖动都可能造成这种单拍毛刺。
    //     表现就是「平衡中偶发停机，LED4 闪 1 次」，而摆杆其实没倒。
    //
    //   真倒下时摆杆越过 ±ANGLE_LIMIT 后会持续外倒，20 拍(100ms)内必然仍超限，
    //   保护依然有效；而 <100ms 的毛刺会被滤掉。
    //   = 1 时退化为原来的单拍判定。
    parameter [15:0] ANGLE_LIMIT_TICKS = 16'd20,
    // 角度超限保护使能：1 = 生效（默认，安全底线）；0 = 临时禁用。
    //   禁用后摆杆倒出 ±ANGLE_LIMIT 也不会停机，仅剩编码器突变保护（err_code=2）。
    //   ⚠ 仅用于调试（例如观察摆杆能倒到多斜而不中断），不要用于正式运行。
    parameter integer ANGLE_LIMIT_EN   = 1,
    parameter [15:0] PWM_MAX           = 16'd5300,   // PWM 限幅（载波 5400 -> 98.1%，对齐 IP570 的接近满占空比）

    parameter signed [31:0] POS_STEP   = 32'sd2080,  // 旧按键步长（保留兼容，当前用 MOVE_STEP_30）
    parameter signed [31:0] RAMP_STEP  = 32'sd1,     // 目标斜坡：每 5ms 变化量

    // ---------------- 摇杆（手轮）参数（与 FPGA_PID_Pendulum 完全一致）----------------
    // 手轮可以被转得很快，而摆臂受「摆杆必须保持直立」的物理限制，不可能瞬时就位。
    // 策略：允许滞后，但限速——
    //   ① wheel_pos 保存手轮的真实目标位置（最终一定要走到这里）
    //   ② 实际给定按 WHEEL_RAMP_STEP 限速靠近 wheel_pos
    //   ③ 给定不得超前当前实际位置 WHEEL_LAG_MAX（防止位置环长期饱和把摆杆甩倒）
    parameter signed [31:0] WHEEL_RAMP_STEP = 32'sd2,     // 摇杆目标速度上限：2 counts/5ms ≈ 138°/s
    parameter signed [31:0] WHEEL_LAG_MAX   = 32'sd150,   // 给定允许超前实际位置的最大量（150 counts ≈ 52°）
    parameter signed [31:0] WHEEL_POS_MAX   = 32'sd20000, // 摇杆模式目标限幅（±19 圈）
    parameter signed [31:0] WHEEL_DEADBAND  = 32'sd2,     // 手轮增量死区（过滤静止抖动）
    parameter integer       WHEEL_INV       = 0,          // 1 = 手轮方向取反（转手轮时摆臂反向就跟）

    // 角度环：PD（严禁加 I）
    //   旋转倒立摆是欠驱动系统，摆杆稍偏就必须加速摆臂才能拉回。
    //   若角度环带积分，它会把"长期偏一点"当成要消掉的误差，持续给摆臂加速，
    //   而摆臂加速又反过来改角度 —— 表现为摆臂越跑越快 / 低频大摆。
    //   机械上摆杆竖直、摆臂不加速时力矩为 0，稳态误差本就应为 0，不需要 I。
    //
    //   ⚠ 以下带 [RW] 标记的参数都是【运行时可写】的：
    //     parameter 只作为上电默认值，实际运算用的是 r_xxx 寄存器。
    //     通过串口 0xAA 0x55 0x01 <ADDR> <DATA_H> <DATA_L> <CHK> 0x0D 0x0A 修改。
    parameter [15:0] BAL_KP            = 16'd35,     // [RW] 角度环 P
    parameter [15:0] BAL_KD            = 16'd244,    // [RW] 角度环 D

    // =========================================================================
    // α-β 滤波器（位置 + 速度估计）
    //   启用后：角度环 P 项用【滤波后角度】，D 项用【估计速度】
    //           替代原来的「原始角度 + 5ms 裸差分」
    //   只影响 S_BALANCE 路径；S_SWING 仍用原始角度（捕获要的就是实时性）
    //
    //   α 决定“多相信测量”：大 → 跟随快、噪声大；小 → 平滑、滞后大
    //   β 决定“速度学得多快”：约定 β = α²/(2-α) 使两个极点重合（临界阻尼）
    //     α=0.60 → β=0.25    （当前，实测摆动 4Hz 处滞后仅 13°）
    //     α=0.20 → β=0.023   （旧值，4Hz 处滞后 66°，D 项失效）
    //   两个值都可串口在线调（0x1B / 0x1C），Q8 格式
    //   AB_EN = 0 时完全退化回原来的裸差分实现，方便 A/B 对比
    // =========================================================================
    //   【2026-10-30 实测数据整定】
    //   抱杆振荡周期实测约 240ms -> 4Hz。在该频率下：
    //     α=51/β=6  -> 速度估计相位滞后 66°  ← D 项几乎失效，阻尼变负！
    //     α=153/β=65 -> 滞后仅 13.5°         ← 推荐
    //   实测静止时角度 σ 仅 0.4 计数，噪声极小，完全可以换成快滤波器。
    parameter integer AB_EN            = 1,          // 1 = 启用 α-β；0 = 旧行为
    parameter [15:0]  AB_ALPHA_Q8      = 16'd153,    // [RW] α = 153/256 ≈ 0.598
    parameter [15:0]  AB_BETA_Q8       = 16'd65,     // [RW] β =  65/256 ≈ 0.254
    // 平衡点附近的小增益区（线性过渡，不是死区也不是硬阶跃！）
    //   增益随 |ang_err| 从中心向边界线性上升：
    //     gain(|e|) = 1 - (1/2^SHIFT) * (1 - |e|/DZ)     |e| < DZ
    //     gain(|e|) = 1                                  |e| >= DZ
    //   即中心处增益 1-1/2^SHIFT，边界处正好 1（连续，无跳变）。
    //   用于压制平衡点附近由 ADC 量化噪声引起的高频细颤，同时避免硬阶跃
    //   造成的「区内区外反复跳」极限环。
    //   区内仍保留恢复力矩，摆杆不会失去支撑。
    //   ANGLE_DZ_SHIFT = 0 时整个小增益区被综合优化掉，回到原始 PD。
    //
    //   ⚠ ANGLE_DZ 必须是 2 的幂，且 ANGLE_DZ_LOG2 必须等于 log2(ANGLE_DZ)。
    //     改 DZ 时两个参数要一起改（32/5、16/4、64/6）。
    parameter [15:0] ANGLE_DZ          = 16'd32,     // 小增益区半宽（计数，32 计数 ≈ 2.77°）
    parameter integer ANGLE_DZ_SHIFT   = 1,          // 中心增益 = 1 - 1/2^SHIFT（0 = 关闭）
    parameter integer ANGLE_DZ_LOG2    = 5,          // 必须 = log2(ANGLE_DZ)

    // 角度环 D 项一阶低通（治理量化噪声）
    //   ang_d5 = ang_err - ang_err_prev 是 5ms 裸差分，而 ADC 只有 10-bit：
    //   1 LSB = 4 计数，一个 LSB 的抖动等效 0.7°/5ms ≈ 140°/s 的虚假角速度，
    //   经 BAL_KD 放大后就是 BAL_KD*4 的 PWM 力矩冲击（KD=140 时达 560，占 10.6%）。
    //   这就是「平衡后摆杆细颤 + 摆臂跟着震颤」的主要来源。
    //   一阶 IIR：y[n] = y[n-1] + (x[n] - y[n-1]) / 2^SHIFT，时间常数 = 2^SHIFT * 5ms。
    //   SHIFT=2 -> 20ms，对角度环带宽(1~3Hz)只引入约 20° 相位滞后，噪声降 4 倍。
    //   SHIFT=0 时直通，等价于原来的裸差分。
    parameter integer ANG_D_LPF_SHIFT  = 2,          // 0=关闭；2 => 20ms；3 => 40ms

    // 位置环：PID
    //   位置环有真实稳态误差（摆臂静摩擦 / 电机死区 / 机械偏心），
    //   纯 PD 会停在「pos_err*POS_KP = 摩擦所需力矩」处，回不到零位，所以要 I。
    parameter [15:0] POS_KP            = 16'd25,     // [RW] 位置环 P
    parameter [15:0] POS_KD            = 16'd600,    // [RW] 位置环 D
    parameter [15:0] POS_KI            = 16'd32,     // [RW] 位置环 I（0 = 退化为纯 PD）
    parameter integer POS_I_SHIFT      = 8,          // 积分定点：每 25ms 累加 KI*pos_filt/2^SHIFT
    parameter [15:0] POS_I_BAND        = 16'd40,     // [RW] |位置误差| 超过此值就暂停积分（抗饱和）
    // 积分只在摆臂【基本静止】时累加（|Δenc| < POS_I_VEL_BAND 计数/5ms）
    //
    //   【为何需要】摆臂有静摩擦 Fs：纯 P 会停在 e = Fs/POS_KP 处（Kp=25、Fs=250PWM -> e=10 计数）。
    //   积分随后以 e/8 = 1.25 PWM/25ms 慢慢累积，约 5s 后力矩超过 Fs，静摩擦突然被突破，
    //   动摩擦更小 -> 摆臂突然窜出去。此时积分已累积 250+ PWM 且不会立刻消失，
    //   继续同方向推 -> 摆臂走得更远 -> 误差反向 -> 积分反向重新累积 -> 再等几秒 -> 再次突破。
    //   这就是「收敛 -> 突然偏离 -> 再收敛」的粘滑极限环（周期 10~30s）。
    //
    //   冻结条件：一旦摆臂开始运动（积分正要造成危害的时刻）就停止累加，
    //   等它重新静止后再慢慢修正。这样既保留了消除稳态误差的能力，又不会 windup。
    parameter [15:0] POS_I_VEL_BAND    = 16'd2,      // [RW] 静止判定：|Δenc| < 此值（计数/5ms）

    // ================= 自动起摆方式 =================
    // SWING_MODE = 0 : 能量泵（连续换向，原方案）
    // SWING_MODE = 1 : 甩杆（移植 STM32 Auto_run 的逻辑：一次性位置 PD 甩杆）
    // 参数是编译期常量，综合只会保留被选中的那一路。
    parameter integer SWING_MODE       = 1,

    // ---- 甩杆（SWING_MODE = 1）----
    //   STM32 的做法：step1_target 从 10000 改成 10341，即把摆臂甩 341 计数
    //   （约 1/3 圈），用 Position_KP_1=90 / Position_KD_1=180 的位置 PD 驱动。
    //   第一次进入时目标 = 当前位置（误差 0，不输出），第二次起才真正甩。
    parameter signed [31:0] FLING_STEP      = 32'sd341,  // 甩杆位移（计数，1 圈=1040）
    parameter [15:0] FLING_KP               = 16'd90,    // 甩杆位置 PD 的 P
    parameter [15:0] FLING_KD               = 16'd180,   // 甩杆位置 PD 的 D
    parameter [15:0] FLING_START_WIN        = 16'd20,    // 允许起摆的摆臂初始偏差（同 STM32 的 9980~10020）

    // ---- 甩杆后的回杆 + 减速（三相甩杆）----
    //   物理意义：甩杆到位后摆臂急停，摆杆因惯性继续向前甩。
    //   此时主动把摆臂反向拉回一小段，相当于再"鞭"一次，
    //   给摆杆补充一份冲量，比单向甩杆更容易把摆杆送到竖直。
    //
    //   相位：0 = 甩向 FLING_STEP；1 = 回向 FLING_STEP2；2 = 减速到 FLING_STEP3 并保持。
    //     甩杆相  0 -> 341        快速前冲
    //     回杆相  1 -> 261        反向拉回，补冲量
    //     减速相  2 -> 231        继续同向走一小段，缓停（避免硬刹车）
    //
    //   减速相的作用：回杆到位时摆臂仍有反向速度，若直接硬刹车会被 D 项
    //   猛制住并可能反向，产生对摆杆的反向冲击。让目标再往前挪一小段，
    //   摆臂就沿着同一方向自然减速，避免方向反转。
    //
    //   FLING_BACK_STEP = 0 时退化为单向甩杆；
    //   FLING_DECEL_STEP = 0 时退化为两相甩杆（回到旧行为）。
    parameter signed [31:0] FLING_BACK_STEP  = 32'sd80,  // 回杆量（计数，80 ≈ 0.077 圈）
    parameter signed [31:0] FLING_DECEL_STEP = 32'sd30,  // 减速段额外行程（计数，30 ≈ 0.029 圈）
    parameter [15:0] FLING_ARRIVE_WIN       = 16'd15,    // 到位判定窗口（计数）
    parameter [15:0] FLING_PHASE_TICKS      = 16'd60,    // 单相超时（60×5ms=300ms），防卡住

    // ---- 能量泵（SWING_MODE = 0）----
    parameter [15:0] SWING_GAIN        = 16'd1200,   // 起摆能量泵 PWM 幅值
    parameter [15:0] SWING_DAMP        = 16'd20,     // 起摆时摆臂速度阻尼
    // QUARTER 是「cos 变号点」的计数阈值，物理上就是 90°（摆杆水平）。
    //   ⚠⚠ 旧标度（3003 计数/180°）时写的是 1000（其实更像 60°），
    //   换成新标度（534 计数/180°）后，整个角度量程只有 ±534 计数，
    //   |ang_err| > 1000 【永远为假】—— 泵律退化成纯阻尼 u = -sign(v)·GAIN，
    //   摆臂只会被刹住，根本泵不进能量。
    //   新标度：90° = 534/2 = 267 计数。
    parameter [15:0] QUARTER           = 16'd267,    // ≈90° 对应的 ADC 计数（新标度）

    // ---- 捕获 ----
    parameter [15:0] CATCH_WINDOW      = 16'd350,    // 进入平衡捕获的角度窗口
    parameter [15:0] CATCH_VEL         = 16'd150,    // 进入平衡捕获的角速度窗口
    parameter [15:0] SUCCESS_NEED      = 16'd10,     // 连续满足捕获条件的次数
    parameter signed [31:0] SWING_TIMEOUT_MAX = 32'sd1000, // 5s 起摆超时（STM32 为 600ms）
    // 起摆「放弃窗口」：只有摆杆偏出这个范围才允许判超时
    //   STM32 原逻辑：超时条件是
    //       (不在 ZHONGZHI±500 内) && fail_count>2 && try_count>120
    //   即「摆杆已经在竖直附近慢慢收敛」时绝不停机。
    //   若只用纯计时，摆杆正在收拢时会被硬掐断，表现为「平衡永远不接管」。
    parameter [15:0] SWING_GIVEUP_WIN  = 16'd500,

    // ---- 捕获期软参数（对应 STM32 在 success_count==0 时改写 PID 全局变量）----
    //   STM32: Balance_KP=30, Balance_KD=150, Position_KP=15, Position_KD=500
    //   本机 BAL_KD 已因 D 项低通降到 140，所以按比例略作修正。
    parameter [15:0] BAL_KP_CATCH      = 16'd35,     // [RW]
    parameter [15:0] BAL_KD_CATCH      = 16'd90,     // [RW]
    parameter [15:0] POS_KP_CATCH      = 16'd15,     // [RW]
    parameter [15:0] POS_KD_CATCH      = 16'd500,    // [RW]
    //   刚进平衡的前 POS_FAST_TICKS 个节拍，位置环按 5ms 跑（STM32 的 success_count<=109），
    //   之后恢复 25ms。摆臂刚甩到位时速度未散，需要快速刹车。
    parameter [15:0] POS_FAST_TICKS    = 16'd110,    // 110 × 5ms = 550ms
    // 位置环速率选择
    //   1 = 位置环【始终】按 5ms 跑（推荐）
    //   0 = 只有进平衡后前 POS_FAST_TICKS 拍按 5ms，之后回 25ms（旧行为）
    //
    //   【为何要常驻 5ms】25ms 采样 + 1/5 低通（τ≈112ms）在 10Hz 处相位滞后：
    //       滤波 arctan(2π·10·0.112) = 82° + 零阶保持 ωT/2 = 18°  →  合计 100°
    //   D 项只有 90° 超前，被吃光后变成 -10°，即【负阻尼】，会自激出十几 Hz 抖动。
    //   改 5ms 后零阶保持滞后从 18° 降到 3.6°，D 项净超前回到 +4°。
    //   注意：本参数只影响位置环速率，不影响软参数切换（见 CATCH_SOFT_TICKS）。
    parameter integer POS_LOOP_FAST_ALWAYS = 0,
    // 捕获期软参数（BAL_KP_CATCH 等）的生效时长（×5ms）。
    //   原来看的是 POS_FAST_TICKS，但两者语义不同：软参数只需覆盖刚站稳的 550ms，
    //   而位置环速率可能希望常驻 5ms。拆开后互不影响。
    parameter [15:0] CATCH_SOFT_TICKS  = 16'd110,    // 110 × 5ms = 550ms
    parameter [15:0] ENC_JUMP_MAX      = 16'd20,     // 编码器 5ms 突变保护阈值（对应 IP570 的 >15）
    // 编码器突变保护的宽限期（× 5ms），两种模式都生效
    //   【为何需要】起摆时摆臂在高速运动：
    //     · 甩杆模式：FLING_KP×341 远大于限幅，起步就是满功率
    //     · 能量泵模式：SWING_GAIN/SWING_DAMP = 1200/20 = 60 计数/5ms
    //   而保护阈值只有 15~20 计数/5ms。S_SWING 期间摆臂不受限，
    //   切换到 S_BALANCE 后的第一个 5ms 就会检查 Δenc，
    //   此时位置环可能还没产生制动力 -> 保护在刹车前把系统停掉，
    //   表现为「起摆成功后突然失去动力」或「手动起摆立刻停止」。
    //   这里在进入平衡后先屏蔽 PROT_GRACE_TICKS 个节拍，给位置环留刹车时间。
    //   PROT_GRACE_TICKS = 0 即退化为无宽限期（STM32 原行为）。
    //   注意：角度超限保护（ANGLE_LIMIT）不受宽限期影响，始终生效。
    parameter [15:0] PROT_GRACE_TICKS  = 16'd60,     // 60 × 5ms = 300ms
    parameter [15:0] POS_ENABLE_TICKS  = 16'd100,    // 进入平衡后 100*5ms = 500ms 才接入位置环（移植 IP570 的 success_count>109）
    parameter integer POS_FILT_SHIFT   = 3           // 位置一阶低通：1/8
)(
    input  wire                clk,
    input  wire                rst_n,
    input  wire                tick_5ms,
    input  wire                tick_25ms,

    input  wire [11:0]         angle,       // ADC 角度
    input  wire [15:0]         enc,         // 编码器位置（1 圈 = 1040）
    input  wire [15:0]         enc_wheel,   // 手轮编码器（XH2，仅摇杆模式使用）

    input  wire                key_start,   // SW1：启动/停止
    input  wire                key_mode,    // SW2：手动/自动切换
    input  wire                key_inc,     // SW3：模式0=目标+30°；其他=幅值+
    input  wire                key_dec,     // SW4：模式0=目标-30°；其他=幅值-
    input  wire                cal_sw,      // DIP1：低有效，停止时校准角度零点
    input  wire                dir_inv,     // DIP2：低有效，编码器方向取反
    // ---- 运动模式（由 top.v 的按键状态机选择）----
    //   mode_sel   : 0=纯稳摆 1=正弦 2=三角 3=线性 4=四四拍 5=摇杆（手轮）
    //   move_armed : 1 = 用户已按 SW1 确认执行该模式（否则只预选不动）
    input  wire [3:0]          mode_sel,
    input  wire signed [15:0]  move_amp,
    input  wire signed [15:0]  move_beat_amp,  // 模式4（四四拍）单拍峰值（计数，独立于 move_amp）
    input  wire [15:0]         move_period,
    input  wire signed [15:0]  move_toff,   // 模式0：已确认的目标偏移
    input  wire                move_armed,  // 1 = 模式正在执行
    // ---- 摇杆（手轮）----
    //   wheel_en  : 1 = 摇杆模式已按 SW1 确认（top.v 用 mode_sel==5 && move_armed 生成）
    //   enc_wheel : 手轮编码器（与驱动通道互补的那一路，XH2）
    input  wire                wheel_en,
    // 串口命令：强制停止（单周期脉冲，优先级高于 key_start）
    input  wire                cmd_stop,

    // ---- 运行时参数写入总线（来自 uart_cmd）----
    //   只覆盖「平衡控制」相关的 PID 参数，地址表见下方 RPARAM_* 定义。
    //   we 为单周期脉冲；收到后先进影子寄存器，tick_5ms 时整体提交，
    //   保证同一控制周期内 P/D 项用同一组增益（避免撕裂）。
    input  wire [7:0]          param_addr,
    input  wire [15:0]         param_data,
    input  wire                param_we,

    output reg  signed [15:0]  pwm_cmd,
    output reg  [3:0]          state_dbg,
    output reg                 led_run,
    output reg                 led_manual,
    output reg                 led_auto,
    output reg                 led_err,
    output reg  [11:0]         angle_mid_dbg,
    // 控制器实际使用的角度（平衡时 = 尖峰抑制后；起摆时 = 原始）
    //   遥测应发这个，否则波形里会看到已被滤掉的尖峰伪迹
    output wire [11:0]         angle_used_dbg,
    // ---- 位置环状态（供上位机显示/调试）----
    output wire signed [15:0]  pos_target_dbg,  // 斜坡跟随后的目标位置
    output wire signed [15:0]  pos_pwm_dbg,     // 位置环输出（限幅后）
    // ---- α-β 滤波器速度估计（供上位机调试）----
    //   单位：计数/5ms（已按 ANGLE_INV 取反，与 ang_err 的导数同向）
    output wire signed [15:0]  ab_v_dbg
);

    // ---------------- 状态机定义 ----------------
    localparam [3:0] S_IDLE    = 4'd0;
    localparam [3:0] S_SWING   = 4'd1;
    localparam [3:0] S_BALANCE = 4'd2;

    // =========================================================================
    // 运行时可调参数（平衡控制）
    //   r_xxx  ：当前生效值，参与运算
    //   s_xxx  ：影子寄存器，串口写入后先进这里，tick_5ms 时整体提交
    //   为什么需要影子：若 BAL_KP 在 5ms 周期中途变化，同一拍内 P 项和 D 项
    //   可能用到不同增益，造成力矩跳变。整体提交保证同一周期用同一组参数。
    //
    //   声明必须放在 EFF_* 常量之前（EFF_POS_KI 引用了 r_pos_ki）。
    // =========================================================================
    reg [15:0] r_bal_kp,       s_bal_kp;
    reg [15:0] r_bal_kd,       s_bal_kd;
    reg [15:0] r_bal_kp_catch, s_bal_kp_catch;
    reg [15:0] r_bal_kd_catch, s_bal_kd_catch;
    reg [15:0] r_pos_kp,       s_pos_kp;
    reg [15:0] r_pos_kd,       s_pos_kd;
    reg [15:0] r_pos_ki,       s_pos_ki;
    reg [15:0] r_pos_kp_catch, s_pos_kp_catch;
    reg [15:0] r_pos_kd_catch, s_pos_kd_catch;
    // 能量泵参数（运行时可调，便于现场快速调起摆）
    reg [15:0] r_swing_gain,    s_swing_gain;
    reg [15:0] r_swing_damp,    s_swing_damp;
    reg [15:0] r_swing_quarter, s_swing_quarter;
    // 位置环输出每 tick 幅度限幅（运行时可调）
    reg [15:0] r_pos_slew,      s_pos_slew;
    // 平衡点（角度零点）：运行时可调，单位 = ADC 计数
    reg [11:0] r_bal_target,    s_bal_target;
    // ---- α-β 滤波器参数（运行时可调，Q8 格式）----
    //   51 = 0.199，6 = 0.0234。约定 β = α²/(2-α) 使双极点重合。
    reg [15:0] r_ab_alpha,      s_ab_alpha;
    reg [15:0] r_ab_beta,       s_ab_beta;
    // ---- 甩杆参数（运行时可调）----
    //   甩杆是一次性开环动作，参数必须在现场快速试（不改固件），
    //   否则每次调 FLING_STEP 都要重新综合烧写。
    //   注意：这些是 16-bit 有符号数量，实际量级：
    //     STEP/BACK/DECEL <= ±2000 计数（<2 圈）
    //     KP/KD <= 1000
    reg signed [15:0] r_fling_step,    s_fling_step;    // 甩杆位移（计数）
    reg [15:0]        r_fling_kp,      s_fling_kp;      // 甩杆 PD 的 P
    reg [15:0]        r_fling_kd,      s_fling_kd;      // 甩杆 PD 的 D
    reg [15:0]        r_fling_back,    s_fling_back;    // 回杆量（正数）
    reg [15:0]        r_fling_decel,   s_fling_decel;   // 减速段行程（正数）
    reg [15:0]        r_fling_win,     s_fling_win;     // 到位判定窗口
    reg [15:0]        r_fling_pticks,  s_fling_pticks;  // 单相超时（×5ms）

    // =========================================================================
    // STM32 兼容模式的生效值（编译期常量，未选中的分支会被综合优化掉）
    //   BAL_MODE = 平衡部分照抄 STM32（STM32_MODE 或 BAL_STM32 任一开启）
    //   ALL_MODE = 连起摆一起照抄（仅 STM32_MODE）
    // =========================================================================
    localparam integer BAL_MODE            = (STM32_MODE != 0) || (BAL_STM32 != 0);
    localparam integer ALL_MODE            = (STM32_MODE != 0);

    // ---- SW3/SW4 步长：30° = 1040 × 30/360 ≈ 87 计数 ----
    //   模式 0：改目标位置；模式 1~4：改曲线幅值（由 top.v 应用到 move_amp）
    localparam signed [31:0] MOVE_STEP_30 = 32'sd87;

    localparam integer EFF_ANG_D_LPF_SHIFT = BAL_MODE ? 0 : ANG_D_LPF_SHIFT;
    localparam integer EFF_ANGLE_DZ_SHIFT  = BAL_MODE ? 0 : ANGLE_DZ_SHIFT;
    // ---- 「摆杆已在竖直附近 -> 直接进平衡」的判定窗口 ----
    //   ⚠⚠ 原来是 BAL_MODE ? 16'd500 : MANUAL_WINDOW，这个 500 是【角度计数】，
    //   换标度后 500 计数 = 500 × 0.337° = 168.5°，几乎等于「任何位置都算竖直」！
    //   后果：摆杆下垂时 ang_err = 235-768 = -533，|533| > 500 勉强进 S_SWING；
    //         但只要把摆杆抬高到 268 计数（ang_err = -500），按启动就会
    //         【直接进 S_BALANCE】，此时摆杆偏离 168°，立刻触发 ANGLE_LIMIT=142
    //         报 err_code=1（LED4 闪 1 次）。这是换标度后才出现的隐患。
    //   top.v 已经传入按标度缩放好的值（124 计数 ≈ ±42°），这里直接使用，
    //   不再用 BAL_MODE 硬编码覆盖。
    localparam [15:0]  EFF_MANUAL_WINDOW   = MANUAL_WINDOW;
    // 编码器突变保护阈值
    //   STM32 原值 15 计数/5ms = 3000 计数/s = 2.88 圈/s，是 IP570（大惯量摆臂）的值。
    //   本机摆臂轻、加速快，正常平衡时也可能瞬时超过 2.88 圈/s，
    //   导致「平衡中偶发停机、LED4 闪 2 次」。
    //   放宽到 40 计数/5ms = 8000 计数/s = 7.7 圈/s —— 这才是真正的失控阈值。
    localparam [15:0]  EFF_ENC_JUMP_MAX    = BAL_MODE ? 16'd40  : ENC_JUMP_MAX;
    // 编码器突变保护的宽限期：两种模式都启用（不再因照抄 STM32 而取消）
    //   原因：STM32 的手动起摆场景下摆臂几乎是静止的，而本机手动起摆时
    //   摆臂往往正在运动，无宽限期会在进入平衡后第一拍就误停机。
    //   PROT_GRACE_TICKS = 0 时即退化为「无宽限期」。
    localparam [15:0]  EFF_PROT_GRACE      = PROT_GRACE_TICKS;
    // 位置环积分系数：不再被 BAL_MODE 强制清零（STM32 的 Position() 无积分，
    // 但本机摆臂有静摩擦，纯 PD 会停在「误差×POS_KP = 摩擦」处回不到零位）。
    // 设 POS_KI = 0 即退化为 STM32 的纯 PD 行为。
    // 位置环积分系数：运行时可写，所以这里是 wire 而非 localparam
    wire [15:0]  EFF_POS_KI          = r_pos_ki;
    // 起摆相关：只有 ALL_MODE（STM32_MODE）才照抄
    localparam signed [31:0] EFF_SWING_TMO = ALL_MODE ? 32'sd120 : SWING_TIMEOUT_MAX;
    localparam signed [31:0] EFF_BACK_STEP = ALL_MODE ? 32'sd0 : FLING_BACK_STEP;
    localparam signed [31:0] EFF_DECEL_STEP= ALL_MODE ? 32'sd0 : FLING_DECEL_STEP;
    // 捕获成功后位置环参考点：
    //   STM32 用 Position_Zero = step1_target（甩杆终点，比实际停靠点更远），
    //   注释原话「平衡的目标位置比甩杆的距离更远，有利于平衡的稳定性，过冲补偿」。
    //
    //   ⚠ 这个「过冲补偿」只在【甩杆模式】下成立：甩杆会把摆臂驱动到 FLING_STEP 附近，
    //     所以参考点取 FLING_STEP 只是比停靠点多几十计数。
    //     能量泵模式下摆臂在捕获时停在哪里是任意的，若仍强制参考点 = 341，
    //     位置环一上来就有 |enc_rel-341| 的巨大误差（且 BAL_MODE 下位置环不限幅），
    //     会立刻把摆臂猛拽走 -> 表现为「刚捕获就倒」。
    //   因此这里加 SWING_MODE == 1 条件：能量泵模式用捕获时的实际位置。
    //
    //   【参考点取捕获时的实际位置（不再用 FLING_STEP）】
    //     三相甩杆的停靠点是 fling_base + FLING_STEP3，而 fling_base 是起动时的
    //     实际位置（任意值）。因此平衡参考点无法在编译期确定，
    //     直接用捕获那一刻的 enc_rel 最准（此时摆臂就停在停靠点上，误差=0）。
    localparam signed [31:0] EFF_BAL_REF = 32'sd0;  // 0 = 用捕获时的 enc_rel

    // 角度零点默认值：固定使用 ZHONGZHI_DEF（BAL_MODE）或 ANGLE_MID_DEFAULT
    localparam [11:0] EFF_ANGLE_MID_DEF    = BAL_MODE ? ZHONGZHI_DEF : ANGLE_MID_DEFAULT;
    // 上电自动标定：完全由 AUTO_CAL 控制（不再被 BAL_MODE 强制启用）
    localparam integer EFF_AUTO_CAL        = AUTO_CAL;
    // PWM 限幅：STM32 是 ±6900（载波 7200）-> 95.83%；本机载波 5400
    localparam [15:0] EFF_PWM_MAX          = BAL_MODE ? 16'd5175 : PWM_MAX;

    // ---------------- 角度换算 ----------------
    // ---- 角度尖峰滤波（3 点中值）--------------------------------------
    //   【为何需要】实测 ADC 偶发单帧尖峰：正常读数 ~2968，但会突然跳到
    //   ~4008（接近满量程 4092）或直接跳到 0，下一帧立刻恢复。
    //   这种跳变持续 20ms，经 BAL_KP 放大后是 ±4000 PWM 的力矩冲击
    //   （实测：角度 +1032 -> PWM -4401，下一帧 -16 -> PWM +4258），
    //   足以把摆杆踢出大幅摆动，是「周期性偏离再收敛」的直接原因。
    //
    //   中值滤波对单帧尖峰完全免疫（三个值里取中间那个，尖峰永远被排除），
    //   代价是 1 帧（5ms）延迟。真实摆杆运动在 5ms 内变化远小于尖峰幅度，
    //   不受影响。
    //
    //   位运算中值：median(a,b,c) = (a&b) | (b&c) | (a&c)（无符号数成立）
    //
    //   【作用范围】只用于 S_BALANCE。S_SWING（含捕获阶段）与启动判定
    //   一律使用原始 angle，保证起摆行为与改动前逐拍一致。
    //
    //   【实现】限幅对象 = 位置环输出 pos_pwm 的【每 tick 幅度】，不是角度。
    //
    //   ⚠ 角度侧曾用三种方案，都有缺陷：
    //     1) 位运算「中值」(a&b)|(b&c)|(a&c) —— 按位多数表决而非数值中值，
    //        进位边界输出错误值，控制直接失稳。
    //     2) 保持型（超阈值冻结，最多 3 帧后强制接受）——
    //        冻结期 D 项=0；第 4 帧阶跃（实测 100~266 计数），
    //        BAL_KD×阶跃 打满 PWM（180×137=24660 -> 5175），一次台阶一次满力踢击。
    //     3) 角度速率限幅 —— 虽平滑，但会【扭曲角度信号本身】：
    //        角度同时供 P 和 D 使用，限速后 P 项失真、D 项相位改变。
    //
    //   【现在改为限幅位置环输出】优点：
    //     · 不扭曲任何测量信号，角度原样进入角度环
    //     · 直接约束执行器指令的跳变率，物理意义明确
    //     · 摆臂不会因为位置环输出的突跳而受到冲击
    //
    //   POS_SLEW_MAX 单位是 PWM/tick（tick = 位置环周期）。
    //     位置环限幅 ±2587，取 250 表示满量程需要 10 tick（50ms）。
    //     偏小（80~150）-> 位置环响应更柔，但收敛变慢
    //     偏大（500+）-> 响应快，但对摆杆扰动大
    localparam [15:0] POS_SLEW_MAX = 16'd250;   // 上电默认值（PWM / tick）

    // 角度直通 / α-β 滤波----------------------------------------------------
    //
    //   AB_EN=1（默认）：角度与角速度都由 α-β 滤波器给出
    //     · P 项用 x_ab（平滑后的角度）—— 不引入相位滞后
    //     · D 项用 v_ab（估计速度）—— 取代 5ms 裸差分，噪声降低约 10 倍
    //   AB_EN=0：完全恢复原来的实现（angle 直通 + 裸差分），用于 A/B 对比
    //
    //   注意：滤波器工作在【原始角度】上，而不是 ang_err。
    //     因为 angle_mid 运行时会变（串口/B3_SW1），若滤波误差则改零点
    //     会凭空产生一个速度尖峰；滤原始角度就没有这个问题（零点只影响 P）。
    wire [11:0]        ab_x;
    wire signed [15:0] ab_v;

    alpha_beta #(
        .ALPHA_Q8(AB_ALPHA_Q8),
        .BETA_Q8 (AB_BETA_Q8)
    ) u_ab (
        .clk      (clk),
        .rst_n    (rst_n),
        .tick     (tick_5ms),
        .alpha_q8 (r_ab_alpha),
        .beta_q8  (r_ab_beta),
        .z        (angle),
        .x_out    (ab_x),
        .v_out    (ab_v)
    );

    wire [11:0] angle_f = (AB_EN != 0) ? ab_x : angle;

    // ---------------- 角度换算 ----------------
    wire signed [15:0] mid_now     = $signed({4'b0, angle_mid});

    // ---- 原始角度（起摆路径，与改动前完全一致）----
    wire signed [15:0] ang_now     = $signed({4'b0, angle});
    wire signed [15:0] ang_err_raw = ang_now - mid_now;
    wire signed [15:0] ang_err     = (ANGLE_INV != 0) ? -ang_err_raw : ang_err_raw;

    // ---- 中值滤波后的角度（仅平衡路径使用）----
    wire signed [15:0] ang_now_f     = $signed({4'b0, angle_f});
    wire signed [15:0] ang_err_raw_f = ang_now_f - mid_now;
    wire signed [15:0] ang_err_f     = (ANGLE_INV != 0) ? -ang_err_raw_f : ang_err_raw_f;

    // 遥测用：控制器本拍实际采信的角度
    //   ⚠ 改成【始终】输出 angle_f（不再按状态切换），原因：
    //     原来只有 S_BALANCE 才发滤波值，而现场看读数时几乎总是停机状态，
    //     于是上位机看到的永远是原始角度，无法判断滤波器到底有没有工作。
    //     现在固定发控制实际使用的值，原始值可用另一路 rawAngle 对比。
    assign angle_used_dbg = angle_f;

    // 遥测用：位置环状态
    //   pos_target 是 32-bit，实际量级远小于 ±32767（1 圈 = 1040 计数），
    //   饱和处理后再输出，避免回绕显示成怪值。
    function signed [15:0] sat16;
        input signed [31:0] v;
        begin
            if(v >  32'sd32767) sat16 =  16'sd32767;
            else if(v < -32'sd32768) sat16 = -16'sd32768;
            else sat16 = v[15:0];
        end
    endfunction

    assign pos_target_dbg = sat16(pos_target);
    assign pos_pwm_dbg    = pos_pwm;

    // α-β 速度估计遥测：按 ANGLE_INV 取反，与 ang_err 的导数同向。
    //   注意 ab_v 是【原始角度】的导数 d(angle)/dt，而控制用的 D 项是
    //   d(ang_err)/dt = ±d(angle)/dt，所以这里也要乘同样的符号，
    //   否则上位机看到的符号和实际参与运算的 D 项相反，调参时会误导。
    assign ab_v_dbg = (ANGLE_INV != 0) ? -ab_v : ab_v;

    // ---------------- 编码器换算（相对 10000） ----------------
    wire signed [31:0] enc_rel_raw = $signed({16'b0, enc}) - 32'sd10000;
    wire               inv_enc     = ((ENC_INV != 0) ^ dir_inv);
    wire signed [31:0] enc_rel     = inv_enc ? -enc_rel_raw : enc_rel_raw;

    // ========================================================================
    // 控制节拍延后一拍（tick_c）
    // ------------------------------------------------------------------------
    //   α-β 滤波器的 x_out/v_out 是寄存器输出，在 tick_5ms 边沿更新。
    //   若控制逻辑在同一个边沿采样，拿到的是【上一拍】的滤波值，
    //   等于在角度环里凭空多出 5ms 纯延迟（会吃掉 D 项相位超前）。
    //   把控制节拍延后 1 个时钟再采样，延迟就从 5ms 降到 20ns。
    //
    //   ⚠ 滤波器实例仍用原始 tick_5ms，只有主 always 块里的控制逻辑用 tick_c。
    // ========================================================================
    reg tick_c, tick25_c;
    always @(posedge clk or negedge rst_n)
        if(!rst_n) begin
            tick_c   <= 1'b0;
            tick25_c <= 1'b0;
        end else begin
            tick_c   <= tick_5ms;
            tick25_c <= tick_25ms;
        end

    // ---------------- 寄存器 ----------------
    reg [11:0]        angle_mid;
    reg               run;
    reg [3:0]         state;
    reg               mode_manual;       // 0 = 自动起摆, 1 = 手动辅助

    reg signed [31:0] pos_cmd;           // 按键设定的目标
    reg signed [31:0] pos_target;        // 斜坡跟随后的目标
    reg signed [31:0] pos_filt;          // 位置误差一阶滤波
    reg signed [31:0] pos_vel;           // 位置误差微分
    reg signed [31:0] pos_int;           // 位置环积分累加器（直接以 PWM 单位累加）
    reg signed [15:0] pos_pwm;           // 位置环输出

    reg signed [15:0] ang_err_prev;      // 5ms 角误差历史（起摆路径用，原始角度）
    reg signed [15:0] ang_err_f_prev;    // 5ms 角误差历史（平衡路径用，中值滤波后）
    reg signed [15:0] ang_d_lpf;         // 角度环 D 项低通状态（一阶 IIR）

    // ---- 小增益区流水寄存器 ----------------------------------------
    // 线性插值乘法与 BAL_KP/BAL_KD 乘法如果直接串联，会形成两级乘法器级联，
    // 把 Fmax 从 37MHz 拖到 33.9MHz。这里插一级寄存器把两级乘法拆开。
    // 延迟 1 个时钟（20ns），相对 5ms 控制周期完全可以忽略。
    reg signed [15:0] ang_err_gr;        // 小增益区处理后的 P 项误差（已打拍）
    reg signed [15:0] ang_d5_gr;         // 小增益区处理后的 D 项差分（已打拍）
    reg signed [15:0] pos_pwm_gr;        // 小增益区处理后的位置环输出（已打拍）
    reg signed [15:0] ang_err_prev25;    // 25ms 角误差历史（保留，当前未参与 PD）
    reg signed [15:0] ang_vel_fast;      // 5ms 角速度（起摆用）
    reg signed [15:0] ang_vel_pd;        // 25ms 角速度（保留，当前未参与 PD，勿用于角度环）

    reg [31:0]        swing_timer;
    reg [15:0]        success_count;
    reg               fling_flag;        // 甩杆已启动（对应 STM32 的 step1_flag）
    reg signed [31:0] fling_base;        // 甩杆起始位置（相对 10000）
    reg signed [31:0] fling_target;      // 当前甩杆目标（寄存器，跟随相位）
    reg [1:0]         fling_phase;       // 0=甩杆相, 1=回杆相, 2=减速相(保持)
    reg [15:0]        fling_timer;       // 相位计时，用于超时推进
    reg signed [31:0] enc_rel_prev;

    // ---- 摇杆（手轮）状态 ----
    reg [15:0]        enc_wheel_prev;    // 手轮编码器上一拍（5ms 前）
    reg signed [31:0] wheel_acc;         // 手轮增量累积（死区判断用）
    reg signed [31:0] wheel_pos;         // 手轮真实目标位置（无限速，最终位置以它为准）

    reg [31:0]        err_timer;
    reg               err_flag;
    // 错误码（决定 LED4 闪几次）：
    //   1 = 角度超限（ANGLE_LIMIT）
    //   2 = 编码器突变（ENC_JUMP_MAX）
    //   3 = 起摆超时（SWING_TIMEOUT_MAX）
    //   4 = 启动被拒绝（手动模式不在竖直窗口 / 摆臂不在起始位置）
    reg [2:0]         err_code;

    reg [15:0]        bal_timer;         // 进平衡后的计时，用于延迟接入位置环
    reg [15:0]        prot_timer;        // 进平衡后的计时，用于屏蔽编码器突变保护
    reg [15:0]        ang_over_cnt;      // 角度连续超限计数（滤除单拍毛刺）
    reg               tick5_d;           // tick_5ms 延后一拍（位置环流水第二级）
    reg               tick25_d;          // tick_25ms 延后一拍

    // 参数地址表（与 tools/serial_plot.html 的表格一一对应）
    localparam [7:0] RP_BAL_KP        = 8'h01;
    localparam [7:0] RP_BAL_KD        = 8'h02;
    localparam [7:0] RP_POS_KP        = 8'h03;
    localparam [7:0] RP_POS_KD        = 8'h04;
    localparam [7:0] RP_POS_KI        = 8'h05;
    localparam [7:0] RP_BAL_KP_CATCH  = 8'h06;
    localparam [7:0] RP_BAL_KD_CATCH  = 8'h07;
    localparam [7:0] RP_POS_KP_CATCH  = 8'h08;
    localparam [7:0] RP_POS_KD_CATCH  = 8'h09;
    localparam [7:0] RP_SWING_GAIN    = 8'h0A;
    localparam [7:0] RP_SWING_DAMP    = 8'h0B;
    localparam [7:0] RP_SWING_QUARTER = 8'h0C;
    localparam [7:0] RP_POS_SLEW      = 8'h0D;
    localparam [7:0] RP_BAL_TARGET    = 8'h0E;
    // 位置环目标位置直接设定（单位：编码器计数，相对 10000）
    //   与 key_inc/key_dec（每次 ±POS_STEP，斜坡跟随）不同，
    //   这里是【直接写 pos_cmd】，再由 RAMP_STEP 斜坡追上。
    //   用途：上位机做位置环控制台（直接输入目标角度）。
    localparam [7:0] RP_POS_TARGET    = 8'h0F;

    // ---- 甩杆参数（运行时可调，地址 0x11~0x17）----
    //   甩杆是一次性开环动作，必须能现场快速试参数，否则每次改
    //   FLING_STEP 都要重新综合烧写（约 1~2 分钟），调试效率极低。
    //   全部为 16-bit 有符号数量（web 端补码收发）。
    localparam [7:0] RP_FLING_STEP    = 8'h11;   // 甩杆位移（计数，可负）
    localparam [7:0] RP_FLING_KP      = 8'h12;   // 甩杆 PD 的 P
    localparam [7:0] RP_FLING_KD      = 8'h13;   // 甩杆 PD 的 D
    localparam [7:0] RP_FLING_BACK    = 8'h14;   // 回杆量（正数，相位1回退）
    localparam [7:0] RP_FLING_DECEL   = 8'h15;   // 减速段行程（正数，相位2再前进）
    localparam [7:0] RP_FLING_WIN     = 8'h16;   // 到位判定窗口（计数）
    localparam [7:0] RP_FLING_PTICKS  = 8'h17;   // 单相超时（×5ms）
    localparam [7:0] RP_AB_ALPHA      = 8'h1B;   // α-β 的 α（Q8）
    localparam [7:0] RP_AB_BETA       = 8'h1C;   // α-β 的 β（Q8）

    // ---- 上电自动角度标定（对应 STM32 的 Angle_Calibrate）----
    reg [15:0]        cal_cnt;           // 已采样节拍数
    reg [17:0]        cal_sum;           // 累加和（64 × 4095 = 262080，需 18-bit）
    reg               cal_done;          // 标定完成

    // 标定中间量（显式限位宽，避免 13-bit 截断警告）
    wire [11:0]       cal_down   = (cal_sum >> CAL_SHIFT);          // 下垂位置
    wire [12:0]       cal_up     = cal_down + CAL_SPAN;             // 竖直位置（可能 >4095）
    wire              cal_valid  =
        (cal_down <= (CAL_DOWN_TYP + CAL_TOL)) &&
        ((cal_down + CAL_TOL) >= CAL_DOWN_TYP) &&
        (cal_up <= 13'd4095);

    // ---------------- 组合逻辑 ----------------
    // ---- 摇杆（手轮）目标生成 ------------------------------------------
    //   移植自 FPGA_PID_Pendulum 的 pendulum_ctrl.v（traj_sel == 5 分支）：
    //     ① 手轮增量累加成 wheel_pos（死区过滤静止抖动）
    //     ② 实际给定每 5ms 最多靠近 wheel_pos 一个 WHEEL_RAMP_STEP（速度上限）
    //     ③ 给定不得超前实际位置 WHEEL_LAG_MAX（防止位置环长期饱和把摆杆甩倒）
    //   手轮停下后给定继续慢慢追上 wheel_pos，最终位置与手轮一致（只是滞后）。
    //   与 PID 版的唯一区别：PID 版用 pos_cmd 做给定基准，这里用 pos_target
    //   （本工程 pos_target 才是位置环实际跟踪的目标）。
    wire wheel_sel = (mode_sel == 4'd5);

    function signed [31:0] clamp_wheel;
        input signed [31:0] v;
        begin
            if(v > WHEEL_POS_MAX)
                clamp_wheel = WHEEL_POS_MAX;
            else if(v < -WHEEL_POS_MAX)
                clamp_wheel = -WHEEL_POS_MAX;
            else
                clamp_wheel = v;
        end
    endfunction

    // 手轮增量：两台电机型号参数完全相同，1:1 映射（手轮转多少，摆臂目标转多少）
    wire signed [31:0] wheel_delta_raw =
        $signed({16'b0, enc_wheel}) - $signed({16'b0, enc_wheel_prev});
    wire signed [31:0] wheel_delta =
        (WHEEL_INV != 0) ? -wheel_delta_raw : wheel_delta_raw;
    wire signed [31:0] wheel_acc_next = wheel_acc + wheel_delta;

    // 限速 + 超前限幅后的实际给定
    wire signed [31:0] wheel_diff = wheel_pos - pos_target;
    wire signed [31:0] wheel_step = (wheel_diff >  WHEEL_RAMP_STEP) ?  WHEEL_RAMP_STEP :
                                    (wheel_diff < -WHEEL_RAMP_STEP) ? -WHEEL_RAMP_STEP :
                                    wheel_diff;
    wire signed [31:0] wheel_cmd_s = pos_target + wheel_step;
    wire signed [31:0] wheel_cmd_next =
        (wheel_cmd_s > enc_rel + WHEEL_LAG_MAX) ? (enc_rel + WHEEL_LAG_MAX) :
        (wheel_cmd_s < enc_rel - WHEEL_LAG_MAX) ? (enc_rel - WHEEL_LAG_MAX) :
        wheel_cmd_s;

    // ---- 运动曲线发生器 ------------------------------------------------
    //   只有「已平衡 + 已确认执行 + 不是纯稳摆模式 + 不是摇杆模式」才使能。
    //   enable=0 时相位归零、输出 0，保证每次确认后曲线都从 0 开始（无跳变）。
    //   用 tick_5ms（而不是 tick_c）驱动，使主块的 tick_c 读到本拍新鲜值。
    wire signed [15:0] motion_off;
    motion_gen u_motion (
        .clk       (clk),
        .rst_n     (rst_n),
        .tick_5ms  (tick_5ms),
        .enable    ((state == S_BALANCE) && move_armed && (mode_sel != 4'd0) && !wheel_sel),
        .mode      (mode_sel[2:0]),
        .amp       (move_amp),
        .beat_amp  (move_beat_amp),
        .period    (move_period),
        .offset    (motion_off)
    );

    // 目标位置 = 手动目标 pos_cmd + 模式相关偏移
    //   模式 0（纯稳摆）：加上预设的 ±30° 目标偏移 move_toff
    //   模式 1~4（曲线） ：加上实时曲线偏移 motion_off
    //   模式 5（摇杆）   ：偏移恒 0，给定完全由手轮增量生成（见上方 wheel_cmd_next）
    //   两者都是相对【起摆时的位置】的偏移，所以按 SW1 后从当前位置平滑走过去。
    wire signed [31:0] toff32       = {{16{move_toff[15]}}, move_toff};
    wire signed [31:0] motion_off32 = {{16{motion_off[15]}}, motion_off};
    wire signed [31:0] mode_off32   = (mode_sel == 4'd0) ? toff32
                                    : wheel_sel          ? 32'sd0
                                    :                      motion_off32;
    wire signed [31:0] pos_cmd_sum  = pos_cmd + mode_off32;

    // 【关键】把合成目标打一拍再用。
    //   不打拍时，这条 32-bit 加法 + 后面 pos_err 的减法和 32x16 乘法
    //   会串成一条超长组合链，实测把 setup slack 从 -11ns 拉到 -14.2ns。
    //   打一拍只引入 1 个时钟（20ns）延迟 —— 相对 5ms 控制周期完全可忽略，
    //   而且 pos_cmd/move_toff/motion_off 本来每 5ms 才变一次。
    reg signed [31:0] pos_cmd_eff;
    always @(posedge clk or negedge rst_n)
        if(!rst_n)          pos_cmd_eff <= 32'sd0;
        else if(tick_c)     pos_cmd_eff <= pos_cmd_sum;

    // 目标位置斜坡（每 5ms 移动 RAMP_STEP，模拟 IP570 的 Position_Zero++）
    wire signed [31:0] pos_target_next =
        (pos_target < pos_cmd_eff) ? (pos_target + RAMP_STEP) :
        (pos_target > pos_cmd_eff) ? (pos_target - RAMP_STEP) : pos_target;

    wire signed [31:0] pos_err_w    = enc_rel - pos_target_next;

    // ---- 位置一阶低通 ----
    //   STM32 原式：Position_Bias = Position_Bias*0.8 + Position_Least*0.2
    //   0.2 ≈ 205/1024 = 0.200195（在 25ms 采样下 τ ≈ 112ms）
    //
    //   【按 5ms 重定标】位置环提速到 5ms 后，要维持同样的 τ≈112ms，
    //   系数需从 0.2 降到 1-exp(-5/112) ≈ 0.044 → 45/1024。
    //   若不改，τ 会缩短到 22ms（滤波过快，位置环噪声直通）。
    //
    //   【位宽】乘法前把 (pos_err_q4 - pos_filt) 截断到 20-bit：
    //     不截断时是 32x16 乘法器，会把位置环关键路径拖到 Fmax 28.9MHz。
    //     20-bit Q4 可表示 ±32767 计数，远超实际位置误差（1 圈=1040 计数）。
    //
    //   【Q4 定点】为什么不能用整数：
    //     系数 45/1024 时，整数运算的截断死区 = 1024/45 ≈ 22 计数。
    //     即 |pos_err - pos_filt| ≤ 22 时增量恒为 0，滤波器停止更新，
    //     而 pos_filt 正是 P 项的输入 → 位置环永远留 22 计数（≈7.6°）残差，
    //     表现就是「位置环不收敛」。
    //     Q4 后死区降到 22/16 ≈ 1.4 计数，再加 +512 四舍五入后仅 0.7 计数。
    //     四舍五入同时消除了「正方向死区 22、负方向 1」的取整不对称。
    localparam [15:0]  POS_FILT_K    = 16'd45;              // 固定常量（不开放运行时修改）
    localparam integer POS_FILT_KS   = 10;                  // 系数分母 2^10
    localparam integer POS_FILT_FRAC = 4;                   // Q4 定点小数位
    // 非 BAL_MODE 用原来的 1/2^POS_FILT_SHIFT，换算成同一分母下的系数
    localparam [15:0]  POS_FILT_K_USE =
        BAL_MODE ? POS_FILT_K : (16'd1024 >>> POS_FILT_SHIFT);

    wire signed [31:0] pos_err_q4   = pos_err_w <<< POS_FILT_FRAC;
    wire signed [31:0] pos_diff_q4  = pos_err_q4 - pos_filt;   // pos_filt 存 Q4
    wire signed [19:0] pos_diff20   = pos_diff_q4[19:0];

    // y[n] = y[n-1] + (x[n]-y[n-1])·K/2^10，带 +2^9 四舍五入
    wire signed [31:0] pos_filt_next =
        pos_filt +
        ((($signed(pos_diff20) * $signed(POS_FILT_K_USE)) + 32'sd512) >>> POS_FILT_KS);
    wire signed [31:0] pos_vel_next = pos_filt_next - pos_filt;   // Q4 速度

    // 角度环 PD：balance = -(KP*Bias + KD*D_Bias)
    //
    // 【关键】IP570 的 Balance() 在 TIM1 的 5ms 中断里执行，
    //   D_Bias = Bias - Last_Bias   （5ms 差分）
    // 所以这里必须同样用 5ms 差分。若在 25ms 里算差分，同样的角度变化率会
    // 得到 5 倍大的数值，等效 KD = 244*5 = 1220，平衡点附近必然剧烈抖动、站不住。
    // ang_err_prev 每 5ms 更新一次，因此 (ang_err - ang_err_prev) 就是 5ms 差分。
    // 起摆路径（能量泵、捕获角速度、超时判定）一律使用这个原始差分。
    wire signed [15:0] ang_d5   = ang_err - ang_err_prev;

    // ---- 平衡路径的信号源选择 ----------------------------------------
    //   S_BALANCE ：用中值滤波后的角度（消除 ADC 单帧尖峰造成的力矩冲击）
    //   其他状态 ：用原始角度（S_SWING 捕获阶段调用 bal_pwm_w，必须与改动前一致）
    //
    // 差分也用各自的源：平衡用滤波角度的差分，起摆用原始差分。
    //   ang_err_f_prev 只在平衡时更新，避免起摆期间被原始值污染。
    wire signed [15:0] ang_d5_filt_raw = ang_err_f - ang_err_f_prev;

    // α-β 启用时，平衡路径的 D 项直接取【估计速度】，不再对位置做差分。
    //   ang_d5 原本是 d(ang_err)/dt，而 ang_err = ±(angle - mid)，mid 为常数，
    //   所以 d(ang_err)/dt = ±d(angle)/dt，符号随 ANGLE_INV 变化。
    wire signed [15:0] ab_v_signed = (ANGLE_INV != 0) ? -ab_v : ab_v;
    wire signed [15:0] ang_d5_filt =
        (AB_EN != 0) ? ab_v_signed : ang_d5_filt_raw;

    wire signed [15:0] ang_err_src = (state == S_BALANCE) ? ang_err_f : ang_err;
    wire signed [15:0] ang_d5_src  = (state == S_BALANCE) ? ang_d5_filt : ang_d5;

    // ---- 角度环 D 项一阶低通 ----------------------------------------
    // y[n] = y[n-1] + (x[n] - y[n-1]) / 2^SHIFT
    // 用组合形式（而非打一拍）是为了让 D 项不额外增加 5ms 延迟；
    // ang_d_lpf 是独立的一级寄存器，不在 enc/pos 那条关键路径上。
    wire signed [15:0] ang_d5_f =
        (EFF_ANG_D_LPF_SHIFT == 0) ? ang_d5_src
                                   : ang_d_lpf + ((ang_d5_src - ang_d_lpf) >>> EFF_ANG_D_LPF_SHIFT);

    // ---- 平衡点附近的小增益区（线性过渡）------------------------------
    // 只在 S_BALANCE 生效（bal_pwm_w 只在那里用）；
    // S_SWING 的捕获阶段仍用满增益 ang_err/ang_d5，避免削弱"抓杆"力度。
    //
    // 【重要】区内必须同时把位置环输出按同样比例缩小（见 pos_pwm_g），
    //   否则角度环被缩小、位置环不变，两个环的相对权限被放大，
    //   积分累积后净力矩会归零甚至反向 → "平衡一段时间后失去动力"。
    wire in_angle_dz = (ang_err_src <=  $signed(ANGLE_DZ)) &&
                       (ang_err_src >= -$signed(ANGLE_DZ));

    // |ang_err_src|（有符号 16-bit，范围远小于 -32768，不会溢出）
    wire [15:0] ae_abs = ang_err_src[15] ? (~ang_err_src + 16'd1) : ang_err_src;

    // 增益分子 = (2^SHIFT - 1)*DZ + |e|，分母 = 2^(SHIFT+LOG2_DZ)
    //   |e| = 0  -> 分子 (2^SHIFT-1)*DZ -> 增益 1 - 1/2^SHIFT
    //   |e| = DZ -> 分子 2^SHIFT*DZ     -> 增益 1（与区外连续）
    //
    // 【位宽】区内 gain_num <= 2^SHIFT*DZ - 1，只需 GN_W 位。
    //   若不收窄，综合会生成 16x16 乘法器（关键路径变长、Fmax 从 37MHz 掉到 33MHz）。
    //   收窄后是 16x7 乘法器，只有少量部分积。区外取值被截断但会走 bypass，不影响结果。
    localparam [15:0] DZ_BIAS = (ANGLE_DZ << EFF_ANGLE_DZ_SHIFT) - ANGLE_DZ;
    localparam integer GN_W   = ANGLE_DZ_LOG2 + EFF_ANGLE_DZ_SHIFT + 1;

    wire [15:0]     gain_num_full = DZ_BIAS + ae_abs;
    wire [GN_W-1:0] gain_num      = gain_num_full[GN_W-1:0];
    wire signed [GN_W:0] gn_s     = $signed({1'b0, gain_num});

    wire signed [31:0] ang_p_lin =
        ($signed(ang_err_src) * gn_s) >>> (EFF_ANGLE_DZ_SHIFT + ANGLE_DZ_LOG2);
    wire signed [31:0] ang_d_lin =
        ($signed(ang_d5_f)    * gn_s) >>> (EFF_ANGLE_DZ_SHIFT + ANGLE_DZ_LOG2);

    wire signed [15:0] ang_err_g =
        (EFF_ANGLE_DZ_SHIFT == 0 || !in_angle_dz) ? ang_err_src : ang_p_lin[15:0];
    wire signed [15:0] ang_d5_g  =
        (EFF_ANGLE_DZ_SHIFT == 0 || !in_angle_dz) ? ang_d5_f : ang_d_lin[15:0];

    // ---- 捕获期 / 平衡期的参数切换（对应 STM32 改写 PID 全局变量）--------
    //   STM32 在 success_count==0（刚捕获）时把 PID 改成软参数，
    //   success_count==110（约 550ms）后恢复原参数。
    //   这里 catch_phase 覆盖两个阶段：
    //     1) S_SWING  ：捕获时用软参数，避免残余动能造成过冲
    //     2) S_BALANCE 且 bal_timer < CATCH_SOFT_TICKS：刚站稳的 550ms 内也用软参数
    wire catch_phase = (state == S_SWING) ||
                       (state == S_BALANCE && bal_timer < CATCH_SOFT_TICKS);

    // ---- 位置环速率选择 ----
    //   pos_fast_en：本拍按 5ms 跑位置环（pos_filt/pos_vel 每 5ms 更新）
    //   pos_slow_en：本拍按 25ms 跑位置环
    //   POS_LOOP_FAST_ALWAYS=1 时 pos_fast_en 在平衡态恒为真、pos_slow_en 恒为假。
    wire pos_fast_en = (state == S_BALANCE) &&
                       ((POS_LOOP_FAST_ALWAYS != 0) || (bal_timer < POS_FAST_TICKS));
    wire pos_slow_en = (state == S_BALANCE) &&
                       ((POS_LOOP_FAST_ALWAYS == 0) && (bal_timer >= POS_FAST_TICKS));

    wire signed [15:0] bal_kp_sel = catch_phase ? $signed(r_bal_kp_catch) : $signed(r_bal_kp);
    wire signed [15:0] bal_kd_sel = catch_phase ? $signed(r_bal_kd_catch) : $signed(r_bal_kd);
    wire signed [15:0] pos_kp_sel = catch_phase ? $signed(r_pos_kp_catch) : $signed(r_pos_kp);
    wire signed [15:0] pos_kd_sel = catch_phase ? $signed(r_pos_kd_catch) : $signed(r_pos_kd);

    // 乘积先按 32-bit 计算，避免综合出 48x48 乘法器
    wire signed [31:0] bal_p32 = bal_kp_sel * $signed(ang_err_gr);
    wire signed [31:0] bal_d32 = bal_kd_sel * $signed(ang_d5_gr);
    wire signed [47:0] bal_pwm_w = -($signed(bal_p32) + $signed(bal_d32));

    // ---- 甩杆用的位置 PD（SWING_MODE = 1）------------------------------
    //   与 STM32 的 Position_PID 同构：Pwm = KP*(Enc-Target) + KD*Δ(Enc-Target)
    //   注意输出符号：STM32 把 Position_PID 直接送 Set_Pwm()，
    //   而平衡时是 Moto = Balance_Pwm - Position_Pwm，两者符号约定一致，
    //   所以这里直接输出 fling_pwm_w（不取负）。
    //
    //   回杆相的第二目标：从 FLING_STEP 往反方向退 FLING_BACK_STEP。
    //   减速相的第三目标：从 FLING_STEP2 继续同向再走 FLING_DECEL_STEP。
    //
    //   ⚠ 这里全部改成【运行期计算】（wire 而非 localparam），
    //     因为 STEP/BACK/DECEL 都是串口可写的寄存器。
    //     摆臂实际行程方向由 fling_step 的符号决定：
    //       step > 0 → 前冲后回退（+STEP → +STEP-BACK → +STEP-BACK-DECEL）
    //       step < 0 → 反向同理
    wire signed [31:0] fling_step_w  = r_fling_step;
    wire signed [31:0] fling_seq     = (fling_step_w >= 0) ? 32'sd1 : -32'sd1;
    wire signed [31:0] fling_step2_w = fling_step_w
                                     - $signed({16'b0, r_fling_back})  * fling_seq;
    wire signed [31:0] fling_step3_w = fling_step2_w
                                     - $signed({16'b0, r_fling_decel}) * fling_seq;

    // 目标用寄存器 fling_target（随相位切换），到位判定基于它
    wire signed [31:0] fling_bias  = enc_rel - fling_target;
    wire signed [31:0] fling_p32   = $signed({16'b0, r_fling_kp}) * fling_bias;
    // D 项直接对 enc_rel 差分（纯速度阻尼）。
    // 若像 STM32 那样对 bias 差分，回杆相切目标时 Last_Bias 会跳变，
    // 产生一个不可控的 D 冲量；这里用速度差分，相位切换时行为可预测。
    wire signed [31:0] fling_d32   = $signed({16'b0, r_fling_kd}) * (enc_rel - enc_rel_prev);
    wire signed [47:0] fling_pwm_w = $signed(fling_p32) + $signed(fling_d32);

    // 甩杆到位：进入 FLING_ARRIVE_WIN 窗口即认为到达当前目标
    wire fling_arrived =
        (enc_rel >= fling_target - $signed({16'b0, r_fling_win})) &&
        (enc_rel <= fling_target + $signed({16'b0, r_fling_win}));

    // ---- 甩杆为【单次动作】，失败不重试 ---------------------------------
    //   fling_flag 置 1 后不再复位，整次起摆只尝试一次：
    //     甩杆 -> 回杆 -> 减速 -> 摆臂挂在 base+step3 -> 摆杆若没上去，
    //     3s 后 SWING_TIMEOUT_MAX 触发，停机关断（err_code=3，LED4 闪 3 次）。
    //   要重试就再按一次启动。
    //   （曾经加过「掉回下方就重甩」的逻辑，实测会让甩杆与自然摆荡互相打架，
    //    相位错乱反而起不来，故撤销。）

    // 位置环 PID：Position_PWM = KP*Bias + KD*D_Bias + I
    //
    // 【流水】这里用【已寄存】的 pos_filt / pos_vel（而不是 pos_filt_next /
    //   pos_vel_next），把「滤波乘法」与「PD 乘法」拆到相邻两个时钟：
    //     时钟 N   ：由 pos_err 算 pos_filt_next，写入 pos_filt
    //     时钟 N+1 ：由 pos_filt 算 PD，写入 pos_pwm（用 tick25_d / tick5_d 触发）
    //   否则两级乘法串联，25ms 位置环关键路径达 35.6ns，Fmax 仅 28MHz。
    //   多出的 20ns 延迟相对 5ms/25ms 周期完全可以忽略。
    // 位置环 PD：pos_filt / pos_vel 均为 Q4，乘完再移回真实 PWM 量纲
    wire signed [31:0] pos_p32 = ($signed(pos_kp_sel) * pos_filt) >>> POS_FILT_FRAC;

    // ---- D 项的采样周期补偿（关键！）----------------------------------
    //   pos_vel 是【单拍差分】，所以 D 项的物理微分增益是
    //       K_d = POS_KD × T            （T = 位置环采样周期）
    //
    //   POS_KD 的标定基准是 25ms（STM32 的 Position() 调用周期），
    //   即原设计 K_d = 600 × 0.025 = 15。
    //   位置环提速到 5ms 后，同样的物理角速度对应的 pos_vel 只有 1/5，
    //   若直接沿用 POS_KD，则 K_d = 600 × 0.005 = 3，只剩 1/5。
    //
    //   而摆臂动力学近似为双积分器（力矩 -> 加速度 -> 位置）：
    //       J·θ̈ = K·u,  u = -(Kp·θ + Kd·θ̇)
    //       ζ = K·Kd / (2·sqrt(K·Kp·J))
    //   P 项不受 T 影响，但 K_d 直接乘了 T。所以 5ms 时阻尼比掉到 1/5，
    //   接近无阻尼 —— 纯 P 加在双积分器上就是【无阻尼简谐振荡】，永不收敛。
    //   表现为：位置环来回摆不停，幅度大了就绕圈转。
    //
    //   这里按速率自动补偿：5ms 模式乘 5，恢复与 25ms 相同的物理阻尼。
    //   这样 POS_KD 始终可以用 STM32 的数值，切换 POS_LOOP_FAST_ALWAYS 也不会错。
    localparam integer POS_D_SCALE = (POS_LOOP_FAST_ALWAYS != 0) ? 5 : 1;

    wire signed [31:0] pos_d32 =
        (($signed(pos_kd_sel) * pos_vel) * POS_D_SCALE) >>> POS_FILT_FRAC;

    // ---- 位置环积分项 ------------------------------------------------
    // 用寄存器的 pos_filt（上一拍滤波值）而不是 pos_filt_next，把乘法从
    // "enc-pos_target → 滤波 → 乘 → 加" 那条关键路径上摘出来，避免继续拉低 Fmax。
    // 差一个 25ms 节拍，对积分环节无影响。
    // POS_KI=0 时综合会整条优化掉，等价于原来的纯 PD。
    // 积分贡献上限：PWM_MAX/8（原 PWM_MAX/4）。
    //   上限太大会让积分能独立把摆臂推走，造成大幅偏离；
    //   只留给它「抵消静摩擦」的量就够，超出部分应由角度环持有。
    localparam signed [31:0] POS_I_MAX = EFF_PWM_MAX / 8;

    wire signed [31:0] pos_int_step =
        (EFF_POS_KI == 0) ? 32'sd0
                          : ($signed(pos_filt) * $signed(EFF_POS_KI))
                            >>> (POS_I_SHIFT + POS_FILT_FRAC);
    wire signed [31:0] pos_int_sum  = pos_int + pos_int_step;
    wire signed [31:0] pos_int_next =
        (pos_int_sum >  POS_I_MAX) ?  POS_I_MAX :
        (pos_int_sum < -POS_I_MAX) ? -POS_I_MAX : pos_int_sum;

    wire signed [47:0] pos_pwm_w =
        $signed(pos_p32) + $signed(pos_d32) + $signed(pos_int);

    // ---- 小增益区内位置环同比例缩小 -----------------------------------
    // 与角度环用同一个 gain_num / in_angle_dz，保证区内两个环的相对权限不变，
    // 避免"积分把角度环架空 → 摆杆失去动力"。
    // pos_pwm 已被 clamp_pos 限制在 ±5300，乘 gain_num(≤64) 后仍在 16-bit 内。
    wire signed [31:0] pos_lin =
        ($signed(pos_pwm) * gn_s) >>> (EFF_ANGLE_DZ_SHIFT + ANGLE_DZ_LOG2);

    wire signed [15:0] pos_pwm_g =
        (EFF_ANGLE_DZ_SHIFT == 0 || !in_angle_dz) ? pos_pwm : pos_lin[15:0];

    // 起摆能量泵：u = -sign(v * cos(angle_err))
    //   |err| < QUARTER -> cos>0 -> u = -sign(v)
    //   否则            -> cos<0 -> u = +sign(v)
    //
    // ---- 【速度源选择】----------------------------------------------
    //   泵律只取 v 的【符号】，所以 v 的符号必须在过零点附近稳定。
    //   原来用 ang_vel_fast = ang_err - ang_err_prev（裸差分）：
    //     · 旧标度：摆杆速度可达 50~100 计数/拍，噪声 σ≈31，SNR 尚可
    //     · 新标度（534 计数/180°）：同样的物理角速度只有 1~3 计数/拍，
    //       而噪声 σ≈0.57 —— SNR 只有 3~5:1，过零点附近符号频繁翻转，
    //       泵力来回抖，能量根本泵不进去。
    //   改用 α-β 估计速度 ab_v_signed（σ≈0.02，4Hz 处滞后仅 13°），
    //   符号判定干净得多。AB_EN=0 时自动退回裸差分，行为与旧版一致。
    //
    //   注意：只改泵律的输入，CATCH_VEL 判定仍用 ang_vel_fast，
    //   以免影响「捕获到平衡」那一段已验证过的行为。
    wire signed [15:0] ang_vel_pump = (AB_EN != 0) ? ab_v_signed : ang_vel_fast;

    wire signed [15:0] swing_cmd_base =
        ((ang_err > $signed(r_swing_quarter)) || (ang_err < -$signed(r_swing_quarter)))
        ? ((ang_vel_pump >= 0) ? $signed(r_swing_gain) : -$signed(r_swing_gain))
        : ((ang_vel_pump >= 0) ? -$signed(r_swing_gain) : $signed(r_swing_gain));

    wire signed [15:0] swing_cmd =
        (SWING_INV != 0) ? -swing_cmd_base : swing_cmd_base;

    // ---- 起摆驱动输出（两种方式二选一，编译期常量）--------------------
    //   SWING_MODE = 0：能量泵 + 摆臂速度阻尼
    //   SWING_MODE = 1：甩杆位置 PD（STM32 风格，第一次进入不输出）
    wire signed [31:0] arm_vel = enc_rel - enc_rel_prev;

    // 泵输出：泵力 - 比例阻尼
    //   阻尼正比于摆臂速度，用于防止摆臂越转越快；但它同时会抵消泵力，
    //   而能量传递效率最高的时刻恰好是摆臂速度峰值 -> 阻尼也达峰。
    //   因此阻尼应取较小值（让摆臂跑得起来），由下面的硬限速兜底。
    wire signed [47:0] swing_pump_w =
        $signed(swing_cmd) - $signed({1'b0, r_swing_damp}) * arm_vel;

    // ---- 硬限速（安全网）--------------------------------------------
    //   |摆臂速度| 超过 SWING_VMAX 且泵方向与当前速度同向时，停止加速（滑行）。
    //   只在「同向」时切断，因为反向时泵本身就在刹车，不应被拦。
    //   140 计数/5ms = 28000 计数/s ≈ 27 圈/s，是防止飞车的硬上限。
    //   即使把 SWING_DAMP 调成 0，摆臂也不会超过这个速度。
    localparam signed [15:0] SWING_VMAX = 16'sd140;

    wire arm_over_speed = (arm_vel >  SWING_VMAX) || (arm_vel < -SWING_VMAX);
    wire pump_with_vel  = (swing_cmd >= 0) ? (arm_vel >= 0) : (arm_vel < 0);

    wire signed [47:0] swing_drive_w =
        (SWING_MODE == 0)
        ? ((arm_over_speed && pump_with_vel) ? 48'sd0 : swing_pump_w)
        : (fling_flag ? fling_pwm_w : 48'sd0);

    // 编码器 5ms 突变判据（宽限期之外才参与保护）
    wire enc_jump = ((enc_rel - enc_rel_prev) >  $signed({16'b0, EFF_ENC_JUMP_MAX})) ||
                    ((enc_rel - enc_rel_prev) < -$signed({16'b0, EFF_ENC_JUMP_MAX}));

    // 角度超限原始判据（仅平衡态，单拍）
    wire ang_raw_over = (state == S_BALANCE) &&
                        ((ang_err_src >  $signed(ANGLE_LIMIT)) ||
                         (ang_err_src < -$signed(ANGLE_LIMIT)));

    // 连续 ANGLE_LIMIT_TICKS 拍超限才认定（滤除 ADC 单拍毛刺）
    wire ang_over = (ANGLE_LIMIT_EN != 0) &&
                    (ang_over_cnt >= ANGLE_LIMIT_TICKS);

    // ---- ADC 削顶 = 已倾倒 ----
    //   【原理】电位器超出电气行程后，ADC 读数会削顶在 0 或 4092 不变，
    //   所以「读数贴近 0 或 4092」等价于「摆杆已倒到行程末端」。
    //
    //   ⚠⚠ 【当前已关闭】！
    //   该判据依赖一个前提：平衡点读数离 0/4092 很远（旧标度竖直 = 3227）。
    //   现在新标度下 180° 只有 534 计数，如果这 534 计数的窗口恰好贴近
    //   量程端点（例如 0~534），那么摆杆正常下垂时读数就 ≤12，
    //   削顶判据会把「正常下垂」当成「已倾倒」，一起摆就立刻停机。
    //
    //   而且 ANGLE_LIMIT 现在已经是按新标度正确缩放的（142 计数 = ±48°），
    //   保护覆盖完整，削顶检测属于冗余。
    //   → 用 ANG_RAIL_EN 关闭。若将来换回宽量程电位器，把它改回 1 即可。
    //   注意：这里写在模块体内，Verilog 视作 localparam（不能在实例化时覆盖）。
    localparam integer ANG_RAIL_EN = 0;         // 1 = 启用削顶判倾倒；0 = 关闭（当前）
    localparam [11:0] ANG_RAIL_LO = 12'd12;     // ≤ 12 视为削在低端
    localparam [11:0] ANG_RAIL_HI = 12'd4080;   // ≥ 4080 视为削在高端

    // 削顶检测用【原始角度】而不是 angle_f：
    //   α-β 滤波会平滑信号，若用 angle_f 判定削顶会晚 5~10 拍才触发。
    //   削顶是硬件量程端点，必须看未经平滑的原值。
    wire ang_rail = (ANG_RAIL_EN != 0) && (state == S_BALANCE) &&
                    ((angle <= ANG_RAIL_LO) || (angle >= ANG_RAIL_HI));

    // 削顶后直接判倾倒：不需要长连续确认，但为避免单帧误判，用 2 拍（10ms）确认。
    //   削顶若真发生会持续存在，2 拍足够滤掉偶发干扰。
    //   计数器在主 always 块的 tick_5ms 里维护（不能另起 always，否则多处驱动）。
    reg [1:0] ang_rail_cnt;

    wire ang_rail_over = (ANGLE_LIMIT_EN != 0) && (ang_rail_cnt >= 2'd2);

    // ---- LED 闪烁指示时序 ----
    //   每次闪烁 = 335ms 亮 + 335ms 灭（2^24 = 16.78M 周期 @50MHz）
    //   总时长 = err_code × 2 × 2^24
    localparam integer BLINK_BIT = 24;
    wire [31:0] blink_total = {29'b0, err_code} << (BLINK_BIT + 1);

    // ---- 起摆超时门控（对应 STM32 的 try_count 逻辑）----
    //   in_catch_ang：摆杆已进入捕获角度窗口（STM32 在这里 try_count=0）
    //   swing_ready ：甩杆相位已走完（SWING_MODE=0 时恒为真）
    //   计时器只在「甩杆已完成 且 摆杆不在捕获窗口」时累加，
    //   否则清零。这样摆杆一旦进入窗口，超时就不会把它提前停掉。
    wire in_catch_ang = (ang_err <  $signed(CATCH_WINDOW)) &&
                        (ang_err > -$signed(CATCH_WINDOW));
    wire swing_ready  = (SWING_MODE == 0) || (fling_phase == 2'd2);
    wire swing_hold   = in_catch_ang || !swing_ready;   // 1 = 计时器清零

    // 超时必须同时满足：计时到 + 摆杆确实偏出放弃窗口（STM32 的 ±500 条件）
    wire swing_timeout =
        (swing_timer > EFF_SWING_TMO) &&
        !in_catch_ang &&
        ((ang_err >  $signed(SWING_GIVEUP_WIN)) ||
         (ang_err < -$signed(SWING_GIVEUP_WIN)));

    // 保护命中：
    //   角度超限保护在状态机里单独判断（始终生效）。
    //   编码器突变保护：仅 S_BALANCE 且过了宽限期才生效。
    //     · 起摆阶段（S_SWING）摆臂本来就在高速运动，不应报错
    //     · 刚进平衡的前 PROT_GRACE_TICKS 给位置环留刹车时间
    wire prot_hit =
        (state == S_BALANCE) && (prot_timer >= EFF_PROT_GRACE) && enc_jump;

    // ---------------- PWM 限幅函数 ----------------
    //   用 EFF_PWM_MAX：BAL_MODE 时为 5175（对齐 STM32 的 6900/7200 = 95.83%），
    //   普通模式为 top.v 传入的 PWM_MAX。
    localparam signed [15:0] PWM_MAX_16 = EFF_PWM_MAX;
    localparam signed [47:0] PWM_MAX_48 = EFF_PWM_MAX;

    function signed [15:0] clamp_pwm;
        input signed [47:0] v;
        begin
            if(v > PWM_MAX_48)
                clamp_pwm = PWM_MAX_16;
            else if(v < -PWM_MAX_48)
                clamp_pwm = -PWM_MAX_16;
            else
                clamp_pwm = v[15:0];
        end
    endfunction
    // ---------------- 位置环限幅函数 ----------------
    // 位置环的输出必须明显小于总限幅，否则一旦摆臂有位移/速度，
    // pos_pwm 会一个人把整个力矩预算（PWM_MAX）占满，角度环被彻底架空，
    // 表现就是“方向对但托不住，摆臂被一路推着转好几圈”。
    //
    // 【为何两种模式都要限幅】
    //   按「抵消关系」算：25 × pos_filt = 50 × ang_err  =>  pos_filt = 2·ang_err
    //   换算成角度（摆臂 1 计数 = 0.346°，摆杆 1 计数 = 0.0865°）：
    //       摆臂偏 8°  ⇔  完全抵消摆杆 1° 的修正力矩
    //   若位置环不限幅，摆臂只要偏 8° 净力矩就归零，摆杆继续倒。
    //   STM32 的 Position_PWM 确实不限幅，但 IP570 的摆臂几乎不动；
    //   本机摆臂动作大得多，必须给角度环留出余地。
    //
    // 限幅值 = EFF_PWM_MAX / 2：位置环最多占一半预算，角度环永远占优。
    // 若出现「摆臂回中太慢」，先把 POS_KP 提上去，不要取消限幅。
    localparam signed [15:0] POS_PWM_MAX    = EFF_PWM_MAX / 2;
    localparam signed [47:0] POS_PWM_MAX_48 = EFF_PWM_MAX / 2;

    function signed [15:0] clamp_pos;
        input signed [47:0] v;
        begin
            if(v > POS_PWM_MAX_48)
                clamp_pos = POS_PWM_MAX;
            else if(v < -POS_PWM_MAX_48)
                clamp_pos = -POS_PWM_MAX;
            else
                clamp_pos = v[15:0];
        end
    endfunction

    // ---------------- 位置环输出的每 tick 幅度限幅 ----------------
    //   目标值先经 clamp_pos 限幅，再限制相对【当前输出 pos_pwm】的变化量
    //   不超过 POS_SLEW_MAX（PWM / tick）。
    //
    //   目的：即使 pos_filt / pos_vel 因量化或扰动突跳，摆臂收到的指令
    //   也是平滑变化的，不会产生冲击。这是【执行器侧】的保护，
    //   不扭曲任何测量信号（角度环拿到的仍是原始 angle）。
    //
    //   pos_pwm 范围 ±2587（= EFF_PWM_MAX/2），默认 250 -> 满量程需 10 tick。
    //     偏小（80~150）-> 位置环更柔，收敛慢但扰动小
    //     偏大（500+）  -> 响应快，但对摆杆扰动大
    //   设成 >= 5175 等价于关闭限幅。
    wire signed [15:0] pos_tgt_w = clamp_pos(pos_pwm_w);
    wire signed [16:0] pos_dlt_w = $signed(pos_tgt_w) - $signed(pos_pwm);

    // 加减后是 17-bit，但实际范围只有 ±2837（2587+250），
    // 远在 16-bit 有符号范围内，显式取低 16 位消除截断警告。
    wire signed [16:0] pos_up_w = $signed(pos_pwm) + $signed({1'b0, r_pos_slew});
    wire signed [16:0] pos_dn_w = $signed(pos_pwm) - $signed({1'b0, r_pos_slew});

    wire signed [15:0] pos_pwm_nxt =
        (pos_dlt_w >  $signed({1'b0, r_pos_slew})) ? pos_up_w[15:0] :
        (pos_dlt_w < -$signed({1'b0, r_pos_slew})) ? pos_dn_w[15:0] :
        pos_tgt_w;

    // ---- B3_SW1 极性（关键！）----------------------------------------
    //   .cst 里 cal_sw 配的是 PULL_MODE=UP，外部开关接 GND（低有效）：
    //       没按 -> 上拉 -> cal_sw = 1
    //       按下 -> 导通 -> cal_sw = 0
    //   所以「是否按下」必须是 !cal_sw。
    //   ⚠ 这里曾经写成 cal_sw（漏了取反），后果非常隐蔽且致命：
    //     条件在【没按】时恒成立 -> angle_mid 每个节拍都被覆写成当前角度
    //     -> ang_err 永远是 0 -> 按启动后 |ang_err| ≤ MANUAL_WINDOW
    //     -> 直接进 S_BALANCE 但误差为 0 -> PWM 恒为 0 -> 【完全不动】
    //     同时串口下发的平衡点也会被立刻覆盖。
    wire cal_pressed = !cal_sw;

    // ========================================================================
    // 主状态机
    // ========================================================================
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            angle_mid      <= EFF_ANGLE_MID_DEF;
            run            <= 1'b0;
            state          <= S_IDLE;
            mode_manual    <= 1'b0;

            // ---- 平衡参数：上电取 parameter 默认值 ----
            r_bal_kp       <= BAL_KP;
            r_bal_kd       <= BAL_KD;
            r_bal_kp_catch <= BAL_KP_CATCH;
            r_bal_kd_catch <= BAL_KD_CATCH;
            r_pos_kp       <= POS_KP;
            r_pos_kd       <= POS_KD;
            r_pos_ki       <= POS_KI;
            r_pos_kp_catch <= POS_KP_CATCH;
            r_pos_kd_catch <= POS_KD_CATCH;
            r_swing_gain    <= SWING_GAIN;
            r_swing_damp    <= SWING_DAMP;
            r_swing_quarter <= QUARTER;
            r_pos_slew      <= POS_SLEW_MAX;
            r_bal_target    <= EFF_ANGLE_MID_DEF;
            r_ab_alpha      <= AB_ALPHA_Q8;
            r_ab_beta       <= AB_BETA_Q8;
            // 影子初值同步，否则第一次提交会覆盖成 0
            s_bal_kp       <= BAL_KP;
            s_bal_kd       <= BAL_KD;
            s_bal_kp_catch <= BAL_KP_CATCH;
            s_bal_kd_catch <= BAL_KD_CATCH;
            s_pos_kp       <= POS_KP;
            s_pos_kd       <= POS_KD;
            s_pos_ki       <= POS_KI;
            s_pos_kp_catch <= POS_KP_CATCH;
            s_pos_kd_catch <= POS_KD_CATCH;
            s_swing_gain    <= SWING_GAIN;
            s_swing_damp    <= SWING_DAMP;
            s_swing_quarter <= QUARTER;
            s_pos_slew      <= POS_SLEW_MAX;
            s_fling_step    <= FLING_STEP[15:0];
            s_fling_kp      <= FLING_KP;
            s_fling_kd      <= FLING_KD;
            s_fling_back    <= FLING_BACK_STEP[15:0];
            s_fling_decel   <= FLING_DECEL_STEP[15:0];
            s_fling_win     <= FLING_ARRIVE_WIN;
            s_fling_pticks  <= FLING_PHASE_TICKS;
            s_bal_target    <= EFF_ANGLE_MID_DEF;
            s_ab_alpha      <= AB_ALPHA_Q8;
            s_ab_beta       <= AB_BETA_Q8;

            pos_cmd        <= 32'sd0;
            pos_target     <= 32'sd0;
            pos_filt       <= 32'sd0;
            pos_vel        <= 32'sd0;
            pos_int        <= 32'sd0;
            pos_pwm        <= 16'sd0;

            ang_err_prev   <= 16'sd0;
            ang_err_f_prev <= 16'sd0;
            ang_d_lpf      <= 16'sd0;
            ang_err_gr     <= 16'sd0;
            ang_d5_gr      <= 16'sd0;
            pos_pwm_gr     <= 16'sd0;
            ang_err_prev25 <= 16'sd0;
            ang_vel_fast   <= 16'sd0;
            ang_vel_pd     <= 16'sd0;

            swing_timer    <= 32'd0;
            success_count  <= 16'd0;
            fling_flag     <= 1'b0;
            fling_base     <= 32'sd0;
            fling_target   <= 32'sd0;
            fling_phase    <= 2'd0;
            fling_timer    <= 16'd0;
            enc_rel_prev   <= 32'sd0;
            // 摇杆（手轮）：基准与 encoder.v 的复位初值 10000 一致，避免首拍出现巨大增量
            enc_wheel_prev <= 16'd10000;
            wheel_acc      <= 32'sd0;
            wheel_pos      <= 32'sd0;

            err_timer      <= 32'd0;
            err_flag       <= 1'b0;
            err_code       <= 3'd0;

            bal_timer      <= 16'd0;
            prot_timer     <= 16'd0;
            ang_over_cnt   <= 16'd0;
            ang_rail_cnt   <= 2'd0;
            tick5_d        <= 1'b0;
            tick25_d       <= 1'b0;

            cal_cnt        <= 16'd0;
            cal_sum        <= 18'd0;
            cal_done       <= 1'b0;

            pwm_cmd        <= 16'sd0;
            state_dbg      <= S_IDLE;
            led_run        <= 1'b0;
            led_manual     <= 1'b0;
            led_auto       <= 1'b1;
            led_err        <= 1'b0;
            angle_mid_dbg  <= EFF_ANGLE_MID_DEF;
        end else begin
            // ---------------- 状态灯 ----------------
            led_run       <= run;
            led_manual    <= mode_manual;
            led_auto      <= ~mode_manual;
            state_dbg     <= state;
            angle_mid_dbg <= angle_mid;

            // ---------------- 小增益区流水（每时钟打拍）----------------
            // 把「插值乘法」与「KP/KD 乘法」拆到两个时钟周期，修复时序。
            // ang_err/ang_d5_f/pos_pwm 在 5ms 内保持不变，因此打一拍不影响控制结果。
            ang_err_gr <= ang_err_g;
            ang_d5_gr  <= ang_d5_g;
            pos_pwm_gr <= pos_pwm_g;

            // ---------------- 位置环流水第二级使能 ----------------
            //   主块已经延后一拍（tick_c），这里再延一拍保持原有两级流水关系
            tick5_d  <= tick_c;
            tick25_d <= tick25_c;

            // ================================================================
            // 运行时参数：影子寄存器写入 + 5ms 原子提交
            // ================================================================
            // 1) 串口写入先落到影子寄存器（任意时刻，逐字节/逐帧到达）
            if(param_we) begin
                case(param_addr)
                    RP_BAL_KP:       s_bal_kp       <= param_data;
                    RP_BAL_KD:       s_bal_kd       <= param_data;
                    RP_POS_KP:       s_pos_kp       <= param_data;
                    RP_POS_KD:       s_pos_kd       <= param_data;
                    RP_POS_KI:       s_pos_ki       <= param_data;
                    RP_BAL_KP_CATCH: s_bal_kp_catch <= param_data;
                    RP_BAL_KD_CATCH: s_bal_kd_catch <= param_data;
                    RP_POS_KP_CATCH: s_pos_kp_catch <= param_data;
                    RP_POS_KD_CATCH: s_pos_kd_catch <= param_data;
                    RP_SWING_GAIN:   s_swing_gain    <= param_data;
                    RP_SWING_DAMP:   s_swing_damp    <= param_data;
                    RP_SWING_QUARTER:s_swing_quarter <= param_data;
                    RP_POS_SLEW:     s_pos_slew      <= param_data;
                    RP_FLING_STEP:   s_fling_step    <= param_data;
                    RP_FLING_KP:     s_fling_kp      <= param_data;
                    RP_FLING_KD:     s_fling_kd      <= param_data;
                    RP_FLING_BACK:   s_fling_back    <= param_data;
                    RP_FLING_DECEL:  s_fling_decel   <= param_data;
                    RP_FLING_WIN:    s_fling_win     <= param_data;
                    RP_FLING_PTICKS: s_fling_pticks  <= param_data;
                    RP_AB_ALPHA:     s_ab_alpha      <= param_data;
                    RP_AB_BETA:      s_ab_beta       <= param_data;
                    RP_BAL_TARGET:   s_bal_target    <= param_data[11:0];
                    default: ;   // 未知地址：忽略（不影响控制）
                endcase
            end

            // 2) tick 时整体搬到工作寄存器
            //    保证同一控制周期内 P 项和 D 项用同一组增益，避免参数撕裂。
            //    串口帧 9 字节 @115200 ≈ 0.78ms，远小于 5ms，不会跨周期丢失。
            if(tick_c) begin
                r_bal_kp       <= s_bal_kp;
                r_bal_kd       <= s_bal_kd;
                r_bal_kp_catch <= s_bal_kp_catch;
                r_bal_kd_catch <= s_bal_kd_catch;
                r_pos_kp       <= s_pos_kp;
                r_pos_kd       <= s_pos_kd;
                r_pos_ki       <= s_pos_ki;
                r_pos_kp_catch <= s_pos_kp_catch;
                r_pos_kd_catch <= s_pos_kd_catch;
                r_swing_gain    <= s_swing_gain;
                r_swing_damp    <= s_swing_damp;
                r_swing_quarter <= s_swing_quarter;
                r_pos_slew      <= s_pos_slew;
                r_fling_step    <= s_fling_step;
                r_fling_kp      <= s_fling_kp;
                r_fling_kd      <= s_fling_kd;
                r_fling_back    <= s_fling_back;
                r_fling_decel   <= s_fling_decel;
                r_fling_win     <= s_fling_win;
                r_fling_pticks  <= s_fling_pticks;
                r_ab_alpha      <= s_ab_alpha;
                r_ab_beta       <= s_ab_beta;
                r_bal_target    <= s_bal_target;
            end

            // ---------------- 角度零点 ----------------
            // 两种工作方式：
            //   A) 自动标定（EFF_AUTO_CAL=1，BAL_MODE 下强制启用）——对应 STM32 的
            //      Angle_Calibrate()：上电后让摆杆自然下垂，采样 CAL_TICKS 个节拍取平均，
            //      若 |down-CAL_DOWN_TYP| <= CAL_TOL 且 down+CAL_SPAN <= 4095，
            //      则 angle_mid = down + CAL_SPAN（下垂 + 180°）；否则退回 ZHONGZHI_DEF。
            //   B) 固定值 + 手动覆盖（原方式）：停止状态下拨 B3_SW1，把当前角度设为平衡点。
            if(EFF_AUTO_CAL != 0) begin
                if(!cal_done && tick_c) begin
                    if(cal_cnt < CAL_TICKS) begin
                        cal_sum <= cal_sum + {6'b0, angle};
                        cal_cnt <= cal_cnt + 16'd1;
                    end else begin
                        cal_done <= 1'b1;
                        // 与 STM32 一致：读数明显偏离下垂位置、或反推结果超量程，则保留默认值
                        angle_mid <= cal_valid ? cal_up[11:0] : ZHONGZHI_DEF;
                    end
                end else if(cal_done && !run && cal_pressed && !key_start) begin
                    // 标定完成后仍允许 B3_SW1 手动覆盖
                    angle_mid <= angle;
                end
            end else if(!run && cal_pressed && !key_start) begin
                // 按住 B3_SW1（低电平有效）= 把当前角度设为平衡点。
                //   旧的「angle > 2800」守卫已去掉：那是旧标度（3003 计数/180°）
                //   下的「接近竖直」判断，新标度（534 计数/180°）下读数不可能到 2800，
                //   留着会让手动标定永远失效。现在由操作者保证摆杆已竖直。
                angle_mid <= angle;
            end

            // ---- 运行时平衡点覆盖（串口下发）----
            //   双向同步，避免互相覆盖：
            //     · 串口写了新值（param_we 且地址 = RP_BAL_TARGET）-> 本拍同步到 angle_mid
            //     · 拨 B3_SW1 手动标定 -> 同步回 r_bal_target（保持两者一致）
            //   用「本拍是否有该地址的写入」作为判据，而不是值比较，
            //   否则 B3_SW1 改动 angle_mid 后会被旧 r_bal_target 立刻覆盖回去。
            //   ⚠ 同步条件必须与上面 B3_SW1 分支完全一致（这里原来是 !cal_sw +
            //     angle>2800，与上面的写法不匹配，会把手动标定的结果覆盖回去）。
            if(param_we && (param_addr == RP_BAL_TARGET))
                angle_mid <= param_data[11:0];
            else if(!run && cal_pressed && !key_start)
                r_bal_target <= angle;   // 手动标定后同步回去

            // ---------------- 手动/自动切换（仅停止时） ----------------
            if(key_mode && !run)
                mode_manual <= ~mode_manual;

            // ---------------- 启动 / 停止 ----------------
            if(cmd_stop) begin
                // 串口强制停止：优先级最高，不受 key_start 影响
                run     <= 1'b0;
                state   <= S_IDLE;
                pwm_cmd <= 16'sd0;
            end else if(key_start) begin
                if(run) begin
                    run     <= 1'b0;
                    state   <= S_IDLE;
                    pwm_cmd <= 16'sd0;
                end else begin
                    run <= 1'b1;

                    // 如果摆杆已经在竖直附近，无论手动/自动模式都直接进入平衡
                    if(ang_err <= $signed(EFF_MANUAL_WINDOW) &&
                       ang_err >= -$signed(EFF_MANUAL_WINDOW)) begin
                        state          <= S_BALANCE;
                        pos_target     <= enc_rel;
                        pos_cmd        <= enc_rel;
                        pos_filt       <= 32'sd0;
                        pos_vel        <= 32'sd0;
                        pos_int        <= 32'sd0;   // 必须清零，否则残留积分会立刻把摆臂推走
                        pos_pwm        <= 16'sd0;
                        bal_timer      <= 16'd0;    // 必须清零，否则软参数/保护宽限期被跳过
                        prot_timer     <= 16'd0;
                        ang_over_cnt   <= 16'd0;
                        ang_rail_cnt   <= 2'd0;
                        // 必须同步微分历史，否则第一拍 D 项 = KD*ang_err 会打满 PWM
                        ang_err_prev   <= ang_err;
                        ang_err_f_prev <= ang_err_f;   // 平衡路径的 D 项历史
                        ang_err_prev25 <= ang_err;
                        enc_rel_prev   <= enc_rel;
                    end else if(mode_manual) begin
                        // 手动模式但摆杆不在竖直附近：拒绝启动
                        run       <= 1'b0;
                        err_flag  <= 1'b1;
                        err_code  <= 3'd4;
                        err_timer <= 32'd0;
                    end else if((SWING_MODE == 1) &&
                                ((enc_rel >  $signed({16'b0, FLING_START_WIN})) ||
                                 (enc_rel < -$signed({16'b0, FLING_START_WIN})))) begin
                        // 摆臂不在起始位置，拒绝自动起摆
                        // （对应 STM32 的 Encoder<9980 || Encoder>10020 检查）
                        run       <= 1'b0;
                        err_flag  <= 1'b1;
                        err_code  <= 3'd4;
                        err_timer <= 32'd0;
                    end else begin
                        // 自动起摆
                        state         <= S_SWING;
                        swing_timer   <= 32'd0;
                        success_count <= 16'd0;
                        fling_flag    <= 1'b0;
                        fling_base    <= 32'sd0;
                        fling_target  <= fling_step_w;
                        fling_phase   <= 2'd0;
                        fling_timer   <= 16'd0;
                        ang_err_prev  <= ang_err;
                        enc_rel_prev  <= enc_rel;
                        pwm_cmd       <= 16'sd0;
                    end
                end
            end else if(run) begin
                // ================= 5ms 控制 =================
                if(tick_c) begin
                    ang_vel_fast <= ang_err - ang_err_prev;
                    ang_err_prev <= ang_err;
                    // 平衡路径的历史：只在平衡时跟随，避免起摆期间被原始值污染
                    if(state == S_BALANCE)
                        ang_err_f_prev <= ang_err_f;
                    ang_d_lpf    <= ang_d5_f;      // D 项低通状态更新
                    enc_rel_prev <= enc_rel;
                    enc_wheel_prev <= enc_wheel;   // 手轮增量基准（5ms 前的值）

                    // 目标位置给定：
                    //   摇杆模式（手轮）：手轮增量累加成 wheel_pos，再用限速/超前限幅生成给定
                    //   其他模式：按 RAMP_STEP 斜坡追随 pos_cmd + 模式偏移
                    // bal_timer：进平衡后的 5ms 计时，到达 POS_ENABLE_TICKS 才接入位置环
                    if(state == S_BALANCE) begin
                        if(wheel_en && wheel_sel) begin
                            // ① 手轮增量累加成真实目标（死区过滤静止抖动）
                            if(wheel_acc_next > WHEEL_DEADBAND) begin
                                wheel_pos <= clamp_wheel(wheel_pos + wheel_acc_next);
                                wheel_acc <= 32'sd0;
                            end else if(wheel_acc_next < -WHEEL_DEADBAND) begin
                                wheel_pos <= clamp_wheel(wheel_pos + wheel_acc_next);
                                wheel_acc <= 32'sd0;
                            end else begin
                                wheel_acc <= wheel_acc_next;   // 未超死区：继续累积
                            end
                            // ② 限速 + 超前限幅后的实际给定（旁路 RAMP_STEP 斜坡）
                            //    pos_cmd 同步跟随，退出摇杆模式时不会突然跳变
                            pos_cmd    <= wheel_cmd_next;
                            pos_target <= wheel_cmd_next;
                        end else begin
                            pos_target <= pos_target_next;
                            wheel_acc  <= 32'sd0;
                            wheel_pos  <= pos_target;   // 非摇杆模式：基准跟着给定走
                        end
                        // bal_timer 自由计数到饱和，同时供「软参数时长」和「位置环速率」判定
                        if(bal_timer != 16'hFFFF)
                            bal_timer <= bal_timer + 16'd1;
                        if(prot_timer < EFF_PROT_GRACE)
                            prot_timer <= prot_timer + 16'd1;
                    end else begin
                        bal_timer  <= 16'd0;
                        prot_timer <= 16'd0;
                        wheel_acc  <= 32'sd0;
                    end

                    // 角度超限计数器：连续超限累加，一旦回到范围内立即清零。
                    //   配合 ang_over 的 >= ANGLE_LIMIT_TICKS 判据，把「单拍毛刺」
                    //   和「真倒下」区分开：前者 cnt 加不到阈值就被清零。
                    if(ang_raw_over) begin
                        if(ang_over_cnt < ANGLE_LIMIT_TICKS)
                            ang_over_cnt <= ang_over_cnt + 16'd1;
                    end else begin
                        ang_over_cnt <= 16'd0;
                    end

                    // ADC 削顶计数器：连续 2 拍削顶即认定已倾倒
                    //   削顶（读数贴 0 或 4092）说明摆杆已倒到传感器行程末端，
                    //   此时角度读数失去意义，继续控制只会满力空转。
                    if(ang_rail) begin
                        if(ang_rail_cnt < 2'd2)
                            ang_rail_cnt <= ang_rail_cnt + 2'd1;
                    end else begin
                        ang_rail_cnt <= 2'd0;
                    end

                    // ---- 保护 ----
                    // 角度超限（连续确认）：始终生效，不因宽限期放宽
                    // ADC 削顶（2 拍确认）：等价于已倾倒到行程末端
                    // 编码器突变：仅 S_BALANCE 且过宽限期后生效（prot_hit 已含门控）
                    //   起摆阶段摆臂本来就在高速运动，不应报错；
                    //   刚进平衡的前 PROT_GRACE_TICKS 给位置环留刹车时间。
                    // 错误码：1 = 角度超限 / 削顶（摆杆已经倒了才是根本原因）
                    //         2 = 编码器突变
                    if(prot_hit || ang_over || ang_rail_over) begin
                        run       <= 1'b0;
                        state     <= S_IDLE;
                        pwm_cmd   <= 16'sd0;
                        err_flag  <= 1'b1;
                        err_code  <= (ang_over || ang_rail_over) ? 3'd1 : 3'd2;
                        err_timer <= 32'd0;
                    end else begin
                        case(state)
                            // ---------- 自动起摆 ----------
                            S_SWING: begin
                                // 计时器：甩杆未完成或已进入捕获窗口时清零
                                if(swing_hold)
                                    swing_timer <= 32'd0;
                                else
                                    swing_timer <= swing_timer + 32'd1;

                                if(ang_err <  $signed(CATCH_WINDOW) &&
                                   ang_err > -$signed(CATCH_WINDOW) &&
                                   ang_vel_fast <  $signed(CATCH_VEL) &&
                                   ang_vel_fast > -$signed(CATCH_VEL)) begin
                                    // ===== 已进入平衡捕获窗口 =====
                                    if(success_count >= SUCCESS_NEED) begin
                                        // ---- 确认起摆成功，转入稳摆 ----
                                        // bal_timer 从 0 开始，前 POS_FAST_TICKS(550ms)
                                        // 位置环按 5ms 跑 + 用软参数（STM32 的做法）。
                                        state          <= S_BALANCE;
                                        // STM32_MODE 时参考点 = 甩杆终点（过冲补偿），否则用当前位置
                                        pos_target     <= (EFF_BAL_REF != 0) ? EFF_BAL_REF : enc_rel;
                                        pos_cmd        <= (EFF_BAL_REF != 0) ? EFF_BAL_REF : enc_rel;
                                        pos_filt       <= 32'sd0;
                                        pos_vel        <= 32'sd0;
                                        pos_int        <= 32'sd0;
                                        pos_pwm        <= 16'sd0;
                                        bal_timer      <= 16'd0;
                                        prot_timer     <= 16'd0;
                                        ang_err_prev   <= ang_err;
                                        ang_err_f_prev <= ang_err_f;   // 平衡路径的 D 项历史
                                        ang_err_prev25 <= ang_err;
                                        enc_rel_prev   <= enc_rel;
                                        // 切换瞬间保留 PD 输出，避免一拍断电
                                        pwm_cmd        <= clamp_pwm(bal_pwm_w);
                                    end else begin
                                        // ---- 计数阶段 ----
                                        success_count <= success_count + 16'd1;
                                        if(SWING_MODE == 0)
                                            // 原逻辑：用角度 PD 拉一把
                                            pwm_cmd <= clamp_pwm(bal_pwm_w);
                                        else
                                            // STM32 逻辑：计数期间继续维持甩杆 PD，
                                            // 把摆臂稳稳压在 FLING_STEP，让摆杆安定下来
                                            pwm_cmd <= clamp_pwm(swing_drive_w);
                                    end
                                end else begin
                                    // ===== 未进入捕获窗口：继续起摆驱动 =====
                                    success_count <= 16'd0;
                                    if(SWING_MODE == 1 && !fling_flag) begin
                                        // 第一次进入：记录起始位置，下一拍开始甩
                                        //   【关键】甩杆是【相对】动作：目标 = 起始位置 + FLING_STEP。
                                        //   不能直接用绝对 FLING_STEP（相对 10000），因为摆臂
                                        //   实际位置是任意的（实测 -982 ~ +536），若按绝对值甩，
                                        //   行程可能变成 1300+ 计数（1.3 圈），甩力完全不对。
                                        fling_flag   <= 1'b1;
                                        fling_base   <= enc_rel;
                                        fling_target <= enc_rel + fling_step_w;
                                        fling_phase  <= 2'd0;
                                        fling_timer  <= 16'd0;
                                        pwm_cmd      <= 16'sd0;
                                    end else begin
                                        // 三相推进：甩杆 -> 回杆 -> 减速(保持)
                                        if(SWING_MODE == 1 && fling_phase < 2'd2) begin
                                            fling_timer <= fling_timer + 16'd1;
                                            if(fling_arrived || (fling_timer > r_fling_pticks)) begin
                                                fling_timer <= 16'd0;
                                                if(fling_phase == 2'd0) begin
                                                    fling_phase  <= 2'd1;
                                                    fling_target <= fling_base + fling_step2_w;   // 回杆
                                                end else begin
                                                    fling_phase  <= 2'd2;
                                                    fling_target <= fling_base + fling_step3_w;   // 减速后保持
                                                end
                                            end
                                        end
                                        pwm_cmd <= clamp_pwm(swing_drive_w);
                                    end
                                end

                                // 超时停机：必须同时满足计时到 + 确实偏出放弃窗口
                                // （swing_timeout 已包含 !in_catch_ang，所以切换那一拍不会被覆盖）
                                if(swing_timeout) begin
                                    run       <= 1'b0;
                                    state     <= S_IDLE;
                                    pwm_cmd   <= 16'sd0;
                                    err_flag  <= 1'b1;
                                    err_code  <= 3'd3;
                                    err_timer <= 32'd0;
                                end
                            end

                            // ---------- 保持直立 + 位置环 ----------
                            S_BALANCE: begin
                                // 位置环每 25ms 更新一次，两次更新之间保持 pos_pwm；
                                // 区内用 pos_pwm_gr（与角度环同比例缩小，已打拍）。
                                pwm_cmd <= clamp_pwm(bal_pwm_w - $signed(pos_pwm_gr));
                            end

                            default: begin
                                pwm_cmd <= 16'sd0;
                            end
                        endcase
                    end

                    // ================= 刚进平衡的快速位置环 =================
                    // 对应 STM32 的 success_count<=109 阶段：位置环每 5ms 跑一次，
                    // 把刚甩到位、仍在高速旋转的摆臂尽快刹住。
                    // 550ms 后交由 25ms 位置环接管。
                    if(pos_fast_en) begin
                        pos_vel  <= pos_vel_next;
                        pos_filt <= pos_filt_next;
                    end
                end

                // ============ 快速位置环第二级（延后一拍算 PD）============
                if(tick5_d && pos_fast_en)
                    pos_pwm <= pos_pwm_nxt;

                // ================= 25ms 位置环 =================
                // 仅在快速阶段（bal_timer >= POS_FAST_TICKS）之后才接管。
                //   ⚠ 用 tick25_c（tick_25ms 延后一拍），与主块的 tick_c 对齐；
                //     对应的第二级在下面用 tick25_d，保持原有两级流水关系。
                if(tick25_c) begin
                    ang_vel_pd     <= ang_err - ang_err_prev25;
                    ang_err_prev25 <= ang_err;

                    if(pos_slow_en) begin
                        pos_vel    <= pos_vel_next;
                        pos_filt   <= pos_filt_next;
                    end

                    // ---- 积分累加（固定 25ms 速率）----
                    //   【重要】不能放在 pos_slow_en 分支里：POS_LOOP_FAST_ALWAYS=1 时
                    //   pos_slow_en 恒为假，积分将永远不累加。
                    //   固定按 tick_25ms 累加，正好匹配 POS_KI / POS_I_SHIFT 的 25ms 标定基准，
                    //   因此位置环提速到 5ms 也不需要额外缩放（只取每第 5 拍的值）。
                    //
                    //   累加条件（四个必须同时满足，缺一不可）：
                    //     1) 目标斜坡已走完 —— 否则 SW3/SW4 转圈时的跟踪误差会灌满积分
                    //     2) 误差在 POS_I_BAND 内 —— 误差过大时 P 已饱和，再积分只会 windup
                    //     3) 摆臂基本静止（|pos_vel| < POS_I_VEL_BAND）—— 防止粘滑极限环
                    //   pos_filt / pos_vel 是 Q4 值，所以比较阈值都要左移 4 位。
                    if(state == S_BALANCE) begin
                        if((pos_target == pos_cmd_eff) &&
                           (pos_filt <=  ($signed(POS_I_BAND)     <<< POS_FILT_FRAC)) &&
                           (pos_filt >= -($signed(POS_I_BAND)     <<< POS_FILT_FRAC)) &&
                           (pos_vel  <=  ($signed(POS_I_VEL_BAND) <<< POS_FILT_FRAC)) &&
                           (pos_vel  >= -($signed(POS_I_VEL_BAND) <<< POS_FILT_FRAC)))
                            pos_int <= pos_int_next;
                    end else begin
                        pos_int <= 32'sd0;
                    end
                end

                // ============ 25ms 位置环第二级（延后一拍算 PD）============
                if(tick25_d && pos_slow_en)
                    pos_pwm <= pos_pwm_nxt;

                // ================= 目标位置按键 =================
                //   SW3/SW4 已全部移到 top.v 处理：
                //     模式0 -> 调 move_toff（目标偏移）
                //     模式1~4 -> 调 move_amp（曲线幅值）
                //   所以这里不再改 pos_cmd，避免两处同时改造成冲突。
            end

            // ================= 串口设定位置环目标（不受 run 门控）=================
            //   与按键等效，但直接写 pos_cmd（不用累加），便于上位机做控制台。
            //   pos_target 由斜坡逐拍追随 pos_cmd，不会瞬变。
            //   仅在平衡态生效，避免起摆期间被意外改写。
            //   ⚠ 必须符号扩展：param_data 是 16-bit，负值（如 -500 = 0xFE0C）
            //     若用 {16'b0, param_data} 会变成 +65036。
            if(param_we && (param_addr == RP_POS_TARGET) && (state == S_BALANCE))
                pos_cmd <= {{16{param_data[15]}}, param_data};

            // ---------------- 错误指示：按错误码闪 N 次 ----------------
            //   LED4 闪烁次数 = err_code，用于区分停机原因（无需串口）：
            //     1 次 = 角度超限（摆杆倒出 ±ANGLE_LIMIT）
            //     2 次 = 编码器突变（摆臂 5ms 位移超 ENC_JUMP_MAX）
            //     3 次 = 起摆超时
            //     4 次 = 启动被拒绝
            //   每次闪烁 = 335ms 亮 + 335ms 灭。
            if(err_flag) begin
                if(err_timer < blink_total) begin
                    err_timer <= err_timer + 32'd1;
                    led_err   <= ~err_timer[BLINK_BIT];
                end else begin
                    err_flag  <= 1'b0;
                    err_timer <= 32'd0;
                    led_err   <= 1'b0;
                end
            end else begin
                led_err   <= 1'b0;
                err_timer <= 32'd0;
            end
        end
    end

endmodule

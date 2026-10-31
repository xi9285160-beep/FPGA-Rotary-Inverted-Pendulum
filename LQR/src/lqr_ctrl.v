// ============================================================================
// lqr_ctrl.v — 旋转倒立摆 LQR 状态反馈控制核心（J280 板 + IP570 机构）
//
// ---------------------------------------------------------------------------
// 1. 控制律
// ---------------------------------------------------------------------------
//   u = -( K_th*th + K_thd*th_dot + K_al*al + K_ald*al_dot )
//
//   th      = enc - ENC_ZERO            摆臂角度      (1040 counts/圈 => 6.043e-3 rad/count)
//   th_dot  = enc - enc_h4              20ms 差分      (counts / 20ms)
//   al      = ang_err（已过 ANGLE_INV）  摆杆角        (11.92 counts/° => 1.464e-3 rad/count)
//   al_dot  = ang_err - ang_h4          20ms 差分      (counts / 20ms)
//
//   四项系数在本工程单位下**同号为正**（推导见 README_LQR.md）：
//       K_TH  = +4.446   PWM / count
//       K_THD = +238.8   PWM / (count/20ms)
//       K_AL  = +17.66   PWM / count
//       K_ALD = +78.0    PWM / (count/20ms)
//
// ---------------------------------------------------------------------------
// 2. 为什么这几组数能用
// ---------------------------------------------------------------------------
//   WHEELTEC 旋转倒立摆 LQR 源码（MATLAB dlqr，Ts=20ms，Q=diag(6000,0,2000,0), R=1）
//       k = [ -1.6349, -1.7563, +26.8000, +2.3682 ]   (rad, rad/s, 12V 电压)
//   该源码的运行条件：Timer1_Init(199,7199) => 50Hz => **20ms** 控制周期，
//   且 Moto = 7200*(u/12)，即 600 PWM/V。本工程 PWM 载波 5400 => 450 PWM/V。
//   把增益从 (rad, V) 换到 (count, PWM)：
//       K = |k| * (state_rad_per_count) * (PWM_per_volt)
//   再乘上位运放/传感器方向换算后符号一致，故四项全为正。
//   注意：若本机构摆杆长度/质量与 IP570 不同，需要重新用 MATLAB 算 K，
//         或直接按 README_LQR.md 里的「调参表」微调。
//
// ---------------------------------------------------------------------------
// 3. 采样周期
// ---------------------------------------------------------------------------
//   反馈算式每 5ms 算一次（tick_5ms），但**微分项固定用 20ms 窗口**（4 级历史）。
//   这样定点增益与参考源码（Ts=20ms）严格对应，同时 P 项用最新采样、延迟更小；
//   微分窗口 20ms 而不是 5ms，噪声也被压低 4 倍（PD 版抖动的根因之一）。
//   若要完全复刻参考源码的 20ms 更新率，把 S_BALANCE 里的 pwm_cmd 赋值
//   改成每 4 个 5ms 拍才执行一次即可（不影响稳定性，只是延迟变大）。
//
// ---------------------------------------------------------------------------
// 4. 数值精度（抖动就藏在这里）
// ---------------------------------------------------------------------------
//   状态量的 1 个 LSB 乘上增益，直接变成力矩跳变：
//       theta     × 4.45  =   4.4 PWM  (0.08%)   <- 可以忽略
//       alpha     × 17.7  =  17.7 PWM  (0.33%)
//       theta_dot × 238.8 = 238.8 PWM  (4.42%)   <- 抖动首恶
//       alpha_dot × 78.0  =  78.0 PWM  (1.44%)
//   编码器速度只有 1 count/20ms 的分辨率，因此速度项必须：
//     ① 升到 Q4 小数（分辨率 1/16 count -> 14.9 PWM/步）
//     ② 加一阶低通（VEL_FILT_SHIFT，默认 2 -> 时间常数约 20ms）
//   否则摆臂编码器每跳 1 个计数，电机就"踢"一下，表现就是持续抖动。
//
// ---------------------------------------------------------------------------
// 5. 与 PD 版的区别
// ---------------------------------------------------------------------------
//   - 取消 25ms 位置环 PD（位置保持由 LQR 的 K_th 项承担，本质是"倾斜摆杆
//     产生反作用力把摆臂带回去"）；
//   - 角度环 PD 换成四状态线性组合；
//   - 起摆（能量泵）、下垂点自标定、角度毛刺门、保护逻辑全部沿用。
// ============================================================================
module lqr_ctrl #(
    // ===================== 平衡点 / 角度换算 =====================
    // 2026-10-27 实测：下垂 936，竖直 3082，差 2146 counts = 180° => 11.92 counts/°
    parameter [11:0] ANGLE_MID_DEFAULT = 12'd3082,
    parameter [11:0] MID_FROM_DOWN     = 12'd2146,
    parameter [11:0] ANGLE_DOWN        = 12'd936,
    parameter integer AUTO_CALIB       = 1,
    parameter [11:0] CALIB_TOL         = 12'd250,
    parameter [3:0]  CALIB_SHIFT       = 4'd6,      // 64 拍 = 320ms 标定窗口
    parameter signed [19:0] CALIB_DRIFT_MAX = 20'sd128,
    parameter [12:0] CALIB_STILL_MAX   = 13'd24,
    parameter integer ANGLE_INV        = 1,         // 摆杆角度方向取反（实测必须为 1）
    parameter integer ENC_INV          = 0,         // 摆臂编码器方向取反
    parameter integer SWING_INV        = 1,         // 起摆能量泵方向取反

    // ===================== LQR 增益（Q8 定点：实际值 = 参数值 / 256） =====================
    parameter signed [16:0] K_TH  = 17'sd1138,      //  4.4459 PWM/count
    parameter signed [16:0] K_THD = 17'sd61130,     // 238.79  PWM/(count/20ms)
    parameter signed [16:0] K_AL  = 17'sd4520,      // 17.656  PWM/count
    parameter signed [16:0] K_ALD = 17'sd19973,     // 78.02   PWM/(count/20ms)

    // 平衡死区：|α| ≤ BAL_DEADBAND（原始 ADC counts）时把角度误差视为 0。
    //   死区是「连续型」的：带内归零，带外减去阈值，所以边界不会跳变。
    //   它同时作用于 α 和 α̇：因为历史寄存器里存的就是死区后的值，
    //   所以摆杆在死区内时 α̇ 也≈0，把噪声侵入 D 项这条路直接封死。
    parameter [11:0] BAL_DEADBAND = 12'd5,
    parameter integer VEL_FILT_SHIFT = 2,

    // ===================== 输出 / 保护 =====================
    parameter [15:0] PWM_MAX        = 16'd5300,     // 载波 5400 -> 98%
    parameter [11:0] MANUAL_WINDOW  = 12'd350,      // 手动启动允许的角度窗口（≈29°）
    parameter [11:0] ANGLE_LIMIT    = 12'd600,      // 平衡时超限判倒（≈50°，LQR 有效区约 ±45°）
    parameter [15:0] ENC_JUMP_MAX   = 16'd30,       // 摆臂 5ms 突变保护（飞车）
    parameter [15:0] PROT_DEBOUNCE  = 16'd3,        // 连续 4×5ms 异常才判倒
    parameter [12:0] ANGLE_JUMP_MAX = 13'd400,      // 角度单拍最大可信跳变（≈34°/5ms）

    // ===================== 位置给定（θ_ref） =====================
    // 摆臂 2.889 counts/°，30° ≈ 87 counts
    parameter signed [31:0] STEP_POS  = 32'sd87,    // SW3/SW4 每次 ±30°
    parameter signed [31:0] POS_MAX   = 32'sd30000, // 给定限幅（±10 圈）
    parameter signed [31:0] RAMP_STEP = 32'sd1,     // 给定斜坡 counts/5ms（1 => 17°/s）

    // ===================== 起摆（与 PD 版相同） =====================
    parameter [15:0] SWING_GAIN      = 16'd1000,
    parameter [15:0] SWING_DAMP      = 16'd10,
    parameter [15:0] CATCH_WINDOW    = 16'd450,
    parameter [15:0] CATCH_VEL       = 16'd150,
    parameter [15:0] QUARTER         = 16'd1070,
    parameter [15:0] SUCCESS_NEED    = 16'd10,
    parameter signed [31:0] SWING_TIMEOUT_MAX = 32'sd4000,  // 20s
    parameter [11:0] DOWN_WINDOW     = 12'd300,
    parameter [15:0] WAIT_VEL_MAX    = 16'd20,
    parameter [15:0] WAIT_STABLE_CNT = 16'd10,
    parameter [15:0] SWING_VEL_MAX   = 16'd100,
    parameter [15:0] ENC_ZERO_INIT   = 16'd10000    // 与 encoder.v 复位值一致
)(
    input  wire                clk,
    input  wire                rst_n,
    input  wire                tick_5ms,

    input  wire [11:0]         angle,       // ADC 角度（12-bit）
    input  wire [15:0]         enc,         // 摆臂编码器

    input  wire                key_start,   // SW1：启动 / 停机
    input  wire                key_mode,    // 保留（手动/自动）
    input  wire                key_inc,     // SW3：摆臂目标 +30°
    input  wire                key_dec,     // SW4：摆臂目标 -30°
    input  wire                dir_inv,     // DIP2：编码器方向取反（低有效）
    input  wire [1:0]          preset,      // SW2：阻尼档位 0~3

    output reg  signed [15:0]  pwm_cmd,
    output reg  [3:0]          state_dbg,
    output reg                 led_run,
    output reg                 led_manual,
    output reg                 led_auto,
    output reg                 led_err,
    output reg  [11:0]         angle_mid_dbg,
    output wire                db_flag        // 1 = 当前落在 ±BAL_DEADBAND 死区内
);

    // ---------------- 状态机 ----------------
    localparam [3:0] S_IDLE    = 4'd0;
    localparam [3:0] S_SWING   = 4'd1;
    localparam [3:0] S_BALANCE = 4'd2;
    localparam [3:0] S_WAIT    = 4'd3;

    // ========================================================================
    // 限幅函数（必须在使用之前声明）
    // ========================================================================
    localparam signed [15:0] PWM_MAX_16 = PWM_MAX;
    localparam signed [47:0] PWM_MAX_48 = PWM_MAX;
    localparam signed [32:0] PWM_MAX_32 = PWM_MAX;

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

    // LQR 输出限幅（33-bit，位宽窄、时序好）
    function signed [15:0] clamp32;
        input signed [32:0] v;
        begin
            if(v > PWM_MAX_32)
                clamp32 = PWM_MAX_16;
            else if(v < -PWM_MAX_32)
                clamp32 = -PWM_MAX_16;
            else
                clamp32 = v[15:0];
        end
    endfunction

    // 可切换限幅（限幅档位用）
    function signed [15:0] clamp_lim;
        input signed [32:0] v;
        input signed [32:0] lim;
        begin
            if(v > lim)
                clamp_lim = lim[15:0];
            else if(v < -lim)
                clamp_lim = -lim[15:0];
            else
                clamp_lim = v[15:0];
        end
    endfunction

    // ========================================================================
    // 寄存器
    // ========================================================================
    reg [11:0]        angle_mid;
    reg               run;
    reg [3:0]         state;
    reg               mode_manual;

    reg [15:0]        enc_zero;          // 固定原点（复位 = ENC_ZERO_INIT，不再改写）

    reg signed [31:0] pos_cmd;           // 摆臂目标（counts，绝对，相对 enc_zero）
    reg signed [31:0] pos_target;        // 斜坡后的给定 = LQR 的 θ_ref

    reg signed [15:0] ang_err_prev5;     // 5ms 差分历史（起摆 / 捕获用）
    reg signed [15:0] ang_vel_fast;      // 5ms 角速度（起摆、捕获用）
    reg signed [15:0] ang_h1, ang_h2, ang_h3, ang_h4;   // 5/10/15/20ms 前的 ang_err
    reg [15:0]        enc_h1, enc_h2, enc_h3, enc_h4;   // 5/10/15/20ms 前的原始编码器值
    reg signed [15:0] enc_rel_prev5;     // 5ms 前的相对位置（起摆阻尼 / 保护）

    reg [31:0]        swing_timer;
    reg [15:0]        success_count;
    reg [31:0]        err_timer;
    reg               err_flag;
    reg [15:0]        wait_cnt;
    reg [15:0]        wait_bal_cnt;
    reg [15:0]        prot_cnt;

    // 角度毛刺门
    reg [11:0]        angle_f;
    reg [11:0]        angle_f_prev;
    reg               ang_held;

    // 下垂点自动标定
    reg [11:0]        angle_down_now;
    reg [19:0]        calib_acc;
    reg [19:0]        calib_acc_h;
    reg [15:0]        calib_cnt;

    localparam [15:0] CALIB_TICKS = (16'd1 << CALIB_SHIFT);
    localparam [15:0] CALIB_HALF  = (CALIB_TICKS >> 1);

    // ========================================================================
    // 角度采样毛刺门（与 PD 版一致）
    //   ADC 偶尔出野值，单拍丢弃；连续两拍同向跳变说明是真实运动，照常接受。
    // ========================================================================
    wire signed [12:0] ang_jump_s  = $signed({1'b0, angle}) - $signed({1'b0, angle_f});
    wire               ang_jump_big = (ang_jump_s >  $signed(ANGLE_JUMP_MAX)) ||
                                      (ang_jump_s < -$signed(ANGLE_JUMP_MAX));

    wire signed [12:0] ang_still_d = $signed({1'b0, angle_f}) - $signed({1'b0, angle_f_prev});
    wire               calib_still = (ang_still_d <  $signed(CALIB_STILL_MAX)) &&
                                     (ang_still_d > -$signed(CALIB_STILL_MAX));

    // ========================================================================
    // 角度换算
    // ========================================================================
    wire signed [15:0] ang_err_raw = $signed({4'b0, angle_f}) - $signed({4'b0, angle_mid});
    wire signed [15:0] ang_err     = (ANGLE_INV != 0) ? -ang_err_raw : ang_err_raw;

    // ========================================================================
    // 编码器换算
    //   一律用 16-bit 回绕减法，摆臂转多圈也不会算错（±31.5 圈内有效）
    // ========================================================================
    wire               inv_enc = (ENC_INV != 0) ^ dir_inv;

    wire signed [15:0] enc_rel    = inv_enc ? -$signed(enc - enc_zero)
                                            :  $signed(enc - enc_zero);
    wire signed [15:0] enc_d5     = enc_rel - enc_rel_prev5;
    wire signed [15:0] ang_d5     = ang_err - ang_err_prev5;   // 5ms 角度差分（起摆判断用）

    // ========================================================================
    // LQR 状态量
    //   θ_ref 只在平衡态取 pos_target；起摆/等待时直接取当前位置，
    //   这样切换瞬间位置项恒为 0，不会因为旧目标而踢一下。
    // ========================================================================
    wire signed [31:0] th_ref     = (state == S_BALANCE)
                                    ? pos_target
                                    : {{16{enc_rel[15]}}, enc_rel};
    wire signed [31:0] th_err_raw = $signed(enc_rel) - th_ref;

    // 限幅到 ±4095：再大输出早已饱和，限幅只是为了把乘法器压到 13 位
    wire signed [12:0] th_err_c = (th_err_raw >  32'sd4095) ?  13'sd4095 :
                                  (th_err_raw < -32'sd4095) ? -13'sd4095 :
                                                               th_err_raw[12:0];
    wire signed [12:0] ang_c    = ang_err[12:0];       // |ang_err| <= 4095 天然成立

    // ---- 平衡死区（±BAL_DEADBAND counts）----
    //   带内：α = 0（电机不再因测量噪声微动）
    //   带外：α = ang_err 减去阈值（连续，无跳变）
    //   注意：起摆捕获、保护仍然用原始 ang_err，不受死区影响。
    localparam signed [15:0] DB_S = BAL_DEADBAND;
    wire signed [15:0] ang_db = (ang_err >  DB_S) ? (ang_err - DB_S) :
                                (ang_err < -DB_S) ? (ang_err + DB_S) : 16'sd0;
    wire signed [12:0] ang_db_c = ang_db[12:0];

    // 死区判定（给上位机 / LED 用）
    wire db_in = (ang_err <=  DB_S) && (ang_err >= -DB_S);
    assign db_flag = db_in;

    // ========================================================================
    // 速度项：选哪一段历史做差分（SW2 预设）
    //   20ms 窗：量化台阶小，但相位滞后大（等效 ~15ms）
    //   10ms 窗：折中
    //    5ms 窗：与 PD 版完全一致，相位好、量化台阶大
    //   窗变短后同一转速的差分值变大，增益同步 ×2 / ×4 补偿，阻尼量保持不变。
    //
    //   实测记录（2026-10-27）：20ms 窗 + 不额外滤波是唯一能稳住的配置。
    //   给速度项再加任何低通（哪怕 5ms）都会立刻失稳 → 这个闭环对速度通道
    //   的相位极其敏感，滤波这条路走不通；要么缩短窗口（预设 1/2），
    //   要么改用模型观测器。
    // ========================================================================
    reg [1:0] win_sel;   // 0 = 20ms(enc_h4)，1 = 10ms(enc_h2)，2 = 5ms(enc_h1)
    reg [1:0] g_sh;      // 速度增益倍数：0/1/2 -> ×1/×2/×4
    reg [1:0] p_sh;      // 位置项倍数：0/1 -> ×1/×2
    reg [1:0] g_div;     // 整体增益右移位数（0 = ×1）
    reg [1:0] ald_sh;    // K_ALD 右移（诊断用）
    reg       lim_bypass;// 1 = 完全不限幅（档 0）
    reg       integ_en;  // 1 = 启用摆臂位置积分（LQI）
    reg [1:0] lim_tbl;   // 用哪张限幅表：1=A / 2=B / 3=C

    always @(*) begin
        win_sel = 2'd0; g_sh = 2'd0; p_sh = 2'd0; g_div = 2'd0; ald_sh = 2'd0;
        lim_bypass = 1'b0; integ_en = 1'b1; lim_tbl = 2'd3;
        case(preset)
            2'd1: lim_tbl = 2'd1;   // 表 A：保守（权限大）
            2'd2: lim_tbl = 2'd2;   // 表 B：中等
            2'd3: lim_tbl = 2'd3;   // 表 C：激进（抖动抑制强）
            default: begin lim_bypass = 1'b1; integ_en = 1'b0; lim_tbl = 2'd1; end
        endcase
    end

    // ========================================================================
    // ★★★ 限幅分级表（要调就改这里）★★★
    //
    //   共 4 级 + 满幅，每级有：
    //     一个「摆杆倾角 |α|」阈值  A0~A3   （单位：ADC counts，1 count ≈ 0.0839°）
    //     一个「摆臂位置误差 |θe|」阈值 T0~T3（单位：编码器 counts，1 count ≈ 0.346°）
    //     一个「该级的 PWM 输出上限」L0~L3 （单位：PWM，5300 = 100% 占空比）
    //
    //   判定规则（每一级都是双重判断）：
    //     只要 |α| ≥ A_i  或  |θe| ≥ T_i   → 就进入第 i 级
    //   即取「两个判据里更危险的那个」，谁先越线听谁的。
    //
    //   第 4 级（满幅）：|α| ≥ A3 或 |θe| ≥ T3 → 放开到 PWM_MAX(5300)
    //
    //   另外 LV_HY 是回滞量：升级用 A_i，降级要退回到 A_i - LV_HY 才降，
    //   防止变量在阈值附近抖动时输出上限来回跳。
    // ========================================================================
    parameter [15:0] LV_HY = 16'd4;      // 回滞（counts）

    // ---------------- 表 A（档 1）：保守，近平衡权限大 ----------------
    parameter [15:0] A_A0 = 16'd16,  A_A1 = 16'd40,  A_A2 = 16'd80,  A_A3 = 16'd128;
    parameter [15:0] A_T0 = 16'd24,  A_T1 = 16'd56,  A_T2 = 16'd104, A_T3 = 16'd176;
    parameter [15:0] A_L0 = 16'd2400,A_L1 = 16'd3300,A_L2 = 16'd4200,A_L3 = 16'd4900;

    // ---------------- 表 B（档 2）：中等 ----------------
    parameter [15:0] B_A0 = 16'd122,  B_A1 = 16'd124,  B_A2 = 16'd126,  B_A3 = 16'd128;
    parameter [15:0] B_T0 = 16'd24,  B_T1 = 16'd26,  B_T2 = 16'd28, B_T3 = 16'd30;
    parameter [15:0] B_L0 = 16'd1000,B_L1 = 16'd1200,B_L2 = 16'd1400,B_L3 = 16'd4000;

    // ---------------- 表 C（档 3）：激进，抖动抑制最强 ----------------
    parameter [15:0] C_A0 = 16'd16,  C_A1 = 16'd40,  C_A2 = 16'd80,  C_A3 = 16'd128;
    parameter [15:0] C_T0 = 16'd24,  C_T1 = 16'd56,  C_T2 = 16'd104, C_T3 = 16'd176;
    parameter [15:0] C_L0 = 16'd600, C_L1 = 16'd1800,C_L2 = 16'd3200,C_L3 = 16'd4400;

    // ---- 绝对值（无符号；|α| 在极限环里能到 128，8 位有符号装不下）----
    wire [15:0] ang_abs = ang_err[15]  ? (~ang_err  + 16'd1) : ang_err;
    wire [12:0] th_abs  = th_err_c[12] ? (~th_err_c + 13'd1) : th_err_c;
    wire [15:0] th_mag  = {3'b0, th_abs};

    // ---- 判定：返回 0..4，表示「有几个判据越线 = 该进第几级」----
    function [2:0] lvl_of;
        input [1:0]  tbl;
        input [15:0] a, t;
        reg [15:0] a0,a1,a2,a3,t0,t1,t2,t3;
        begin
            if(tbl == 2'd2) begin
                a0=B_A0; a1=B_A1; a2=B_A2; a3=B_A3;
                t0=B_T0; t1=B_T1; t2=B_T2; t3=B_T3;
            end else if(tbl == 2'd3) begin
                a0=C_A0; a1=C_A1; a2=C_A2; a3=C_A3;
                t0=C_T0; t1=C_T1; t2=C_T2; t3=C_T3;
            end else begin
                a0=A_A0; a1=A_A1; a2=A_A2; a3=A_A3;
                t0=A_T0; t1=A_T1; t2=A_T2; t3=A_T3;
            end
            if((a >= a3) || (t >= t3))      lvl_of = 3'd4;
            else if((a >= a2) || (t >= t2)) lvl_of = 3'd3;
            else if((a >= a1) || (t >= t1)) lvl_of = 3'd2;
            else if((a >= a0) || (t >= t0)) lvl_of = 3'd1;
            else                            lvl_of = 3'd0;
        end
    endfunction

    // ---- 查表：第 lv 级对应的输出上限 ----
    function [15:0] lim_of;
        input [1:0] tbl;
        input [2:0] lv;
        begin
            if(tbl == 2'd2) begin
                if(lv == 3'd0)      lim_of = B_L0;
                else if(lv == 3'd1) lim_of = B_L1;
                else if(lv == 3'd2) lim_of = B_L2;
                else if(lv == 3'd3) lim_of = B_L3;
                else                lim_of = PWM_MAX;
            end else if(tbl == 2'd3) begin
                if(lv == 3'd0)      lim_of = C_L0;
                else if(lv == 3'd1) lim_of = C_L1;
                else if(lv == 3'd2) lim_of = C_L2;
                else if(lv == 3'd3) lim_of = C_L3;
                else                lim_of = PWM_MAX;
            end else begin
                if(lv == 3'd0)      lim_of = A_L0;
                else if(lv == 3'd1) lim_of = A_L1;
                else if(lv == 3'd2) lim_of = A_L2;
                else if(lv == 3'd3) lim_of = A_L3;
                else                lim_of = PWM_MAX;
            end
        end
    endfunction

    // ---- 带回滞的级数跟踪 ----
    wire [2:0] lv_up = lvl_of(lim_tbl, ang_abs, th_mag);
    wire [2:0] lv_dn = lvl_of(lim_tbl, ang_abs + LV_HY, th_mag + LV_HY);
    reg  [2:0] lv_cur;
    wire [2:0] lv_new = (lv_up > lv_cur) ? lv_up :
                        ((lv_dn < lv_cur) ? lv_dn : lv_cur);

    wire [32:0] lim_cur  = lim_bypass ? PWM_MAX_32 : {17'b0, lim_of(lim_tbl, lv_cur)};
    reg  [32:0] lim_r;

    wire [15:0]        enc_tap = (win_sel == 2'd0) ? enc_h4 :
                                 (win_sel == 2'd1) ? enc_h2 : enc_h1;
    wire signed [15:0] ang_tap = (win_sel == 2'd0) ? ang_h4 :
                                 (win_sel == 2'd1) ? ang_h2 : ang_h1;

    wire signed [15:0] enc_dw = inv_enc ? -$signed(enc - enc_tap)
                                        :  $signed(enc - enc_tap);
    wire signed [15:0] al_dw  = ang_db - ang_tap;

    // Q4 表示（预留给以后的状态观测器；当前不做额外滤波，等价于整数差分）
    wire signed [15:0] enc_dw_c = (enc_dw >  16'sd127) ?  16'sd127 :
                                  (enc_dw < -16'sd127) ? -16'sd127 : enc_dw;
    wire signed [15:0] al_dw_c  = (al_dw  >  16'sd127) ?  16'sd127 :
                                  (al_dw  < -16'sd127) ? -16'sd127 : al_dw;
    wire signed [12:0] thd_v = enc_dw_c[12:0] <<< 4;
    wire signed [12:0] ald_v = al_dw_c[12:0]  <<< 4;

    // ========================================================================
    // 摆臂位置积分项（LQI）
    //   病根：摆臂静摩擦约 90~180 PWM，而 K_TH 只有 4.4459 PWM/count，
    //   需要 20~40 counts（7~14°）的偏差才能克服 -> 这就是「位置环好像没作用、
    //   时不时滑很远」的来源（典型的粘滑 / stick-slip），不是符号写错。
    //   积分器负责把这段死区填掉，慢速（约 0.2Hz）把残余偏差磨到 0。
    //
    //   标定：3° 偏差（~9 counts）时约 0.5s 积到 150 PWM（刚好推开静摩擦）。
    //     u_i = th_i * KI_Q8 / 256   [PWM]
    //     KI_Q8 = 43  <=>  0.168 PWM per (count · 5ms拍)
    // ========================================================================
    localparam signed [15:0] I_MAX = 16'sd4096;   // 累积上限 -> u_i 最大约 688 PWM
    localparam signed [15:0] I_WIN = 16'sd120;    // |θ误差| 超过此值（~41°）就清零重来
    localparam signed [7:0]  KI_Q8 = 8'sd43;

    reg  signed [15:0] th_i;      // 摆臂位置误差累加量（counts × 拍）
    wire i_win_ok = (th_err_raw <= I_WIN) && (th_err_raw >= -I_WIN);
    // 17 位中间量：避免 16 位自决定宽度把加法结果截掉
    wire signed [16:0] th_i_nxt = {{1{th_i[15]}}, th_i}
                                + {{4{th_err_c[12]}}, th_err_c};
    // 乘法两端补到同一宽度，否则 Verilog 只按较宽操作数出结果，高位被无声截断
    wire signed [23:0] m_i_p = {{8{th_i[15]}}, th_i} * {{16{KI_Q8[7]}}, KI_Q8};
    wire signed [32:0] m_i_w = {{9{m_i_p[23]}}, m_i_p} <<< 4;   // Q8 PWM -> Q12

    // ---- 增益：P 项用 Q12（K×16 只是连线），D 项 Q8×Q4 也是 Q12 ----
    localparam signed [15:0] K_TH12 = K_TH * 16;
    localparam signed [17:0] K_AL12 = K_AL * 16;

    // ---- LQR 计算流水线（乘 / 加 / 限幅分开拍，避免 50MHz 下单拍跑完全链）----
    reg               lq_v1, lq_v2, lq_v3;
    reg signed [12:0] s_th_err, s_ang, s_thd, s_ald;
    reg signed [32:0] m_th, m_thd, m_al, m_ald;
    reg signed [15:0] u_x_r;          // 粗限后的输出（±5300）

    wire signed [32:0] m_th_raw  = K_TH12 * s_th_err;
    wire signed [32:0] m_thd_raw = K_THD  * s_thd;
    wire signed [32:0] m_al_raw  = K_AL12 * s_ang;
    wire signed [32:0] m_ald_raw = K_ALD  * s_ald;

    wire signed [32:0] m_sum = m_th + m_thd + m_al + m_ald;    // Q12
    wire signed [15:0] u_sat = clamp_lim(-((m_sum >>> g_div) >>> 12), PWM_MAX_32);

    // 再拆一级：lq_v2 锁存 u_sat，lq_v3 按分级表削顶输出
    wire signed [15:0] u_out  = clamp_lim({{17{u_x_r[15]}}, u_x_r}, lim_r);

    wire lq_active = lq_v2 && run &&
                     ((state == S_BALANCE) || ((state == S_SWING) && in_window));

    // ========================================================================
    // 目标斜坡
    // ========================================================================
    wire signed [31:0] pos_target_next =
        (pos_target < pos_cmd) ?
            (((pos_cmd - pos_target) > RAMP_STEP) ? (pos_target + RAMP_STEP) : pos_cmd) :
        (pos_target > pos_cmd) ?
            (((pos_target - pos_cmd) > RAMP_STEP) ? (pos_target - RAMP_STEP) : pos_cmd) :
            pos_target;

    // ========================================================================
    // 起摆前的下垂判断 / 捕获判断 / 保护条件
    // ========================================================================
    wire signed [12:0] down_diff =
        $signed({1'b0, angle_f}) - $signed({1'b0, angle_down_now});

    wire down_ok     = (down_diff <  $signed(DOWN_WINDOW)) &&
                       (down_diff > -$signed(DOWN_WINDOW));
    wire wait_vel_ok  = (ang_d5 <  $signed(WAIT_VEL_MAX)) &&
                        (ang_d5 > -$signed(WAIT_VEL_MAX));
    wire can_swing    = down_ok && wait_vel_ok;

    wire in_window    = (ang_err <  $signed(CATCH_WINDOW)) &&
                        (ang_err > -$signed(CATCH_WINDOW));
    wire catch_vel_ok = (ang_vel_fast <  $signed(CATCH_VEL)) &&
                        (ang_vel_fast > -$signed(CATCH_VEL));

    // S_WAIT 自恢复：保护可能是误触发，摆杆其实还在竖直附近
    wire bal_ok_now   = in_window && catch_vel_ok;

    wire bal_prot_bad =
        (ang_err >  $signed(ANGLE_LIMIT)) ||
        (ang_err < -$signed(ANGLE_LIMIT)) ||
        (enc_d5 >  $signed({1'b0, ENC_JUMP_MAX})) ||
        (enc_d5 < -$signed({1'b0, ENC_JUMP_MAX}));

    // 起摆能量泵：u = -sign(v * cos(err))
    wire signed [15:0] swing_cmd_base =
        ((ang_err > $signed(QUARTER)) || (ang_err < -$signed(QUARTER)))
        ? ((ang_vel_fast >= 0) ? $signed(SWING_GAIN) : -$signed(SWING_GAIN))
        : ((ang_vel_fast >= 0) ? -$signed(SWING_GAIN) : $signed(SWING_GAIN));
    wire signed [15:0] swing_cmd = (SWING_INV != 0) ? -swing_cmd_base : swing_cmd_base;

    // 标定结算判据
    wire [19:0]        calib_sum_avg = calib_acc >> CALIB_SHIFT;
    wire [11:0]        calib_avg     = calib_sum_avg[11:0];
    wire signed [20:0] calib_drift_sum =
        $signed({1'b0, calib_acc}) - $signed({1'b0, calib_acc_h << 1});
    wire calib_quiet = (calib_drift_sum <= CALIB_DRIFT_MAX) &&
                       (calib_drift_sum >= -CALIB_DRIFT_MAX);

    // ========================================================================
    // 主逻辑
    // ========================================================================
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            angle_mid      <= ANGLE_MID_DEFAULT;
            run            <= 1'b0;
            state          <= S_IDLE;
            mode_manual    <= 1'b0;

            enc_zero       <= ENC_ZERO_INIT;
            pos_cmd        <= 32'sd0;
            pos_target     <= 32'sd0;

            ang_err_prev5  <= 16'sd0;
            ang_vel_fast   <= 16'sd0;
            ang_h1         <= 16'sd0;
            ang_h2         <= 16'sd0;
            ang_h3         <= 16'sd0;
            ang_h4         <= 16'sd0;
            lq_v1          <= 1'b0;
            lq_v2          <= 1'b0;
            lq_v3          <= 1'b0;
            u_x_r          <= 16'sd0;
            lv_cur         <= 3'd0;
            s_th_err       <= 13'sd0;
            s_ang          <= 13'sd0;
            s_thd          <= 13'sd0;
            s_ald          <= 13'sd0;
            m_th           <= 33'sd0;
            m_thd          <= 33'sd0;
            m_al           <= 33'sd0;
            m_ald          <= 33'sd0;
            lim_r          <= PWM_MAX_32;
            lv_cur         <= 3'd0;
            th_i           <= 16'sd0;
            enc_h1         <= ENC_ZERO_INIT;
            enc_h2         <= ENC_ZERO_INIT;
            enc_h3         <= ENC_ZERO_INIT;
            enc_h4         <= ENC_ZERO_INIT;
            enc_rel_prev5  <= 16'sd0;

            swing_timer    <= 32'd0;
            success_count  <= 16'd0;
            err_timer      <= 32'd0;
            err_flag       <= 1'b0;
            wait_cnt       <= 16'd0;
            wait_bal_cnt   <= 16'd0;
            prot_cnt       <= 16'd0;

            angle_f        <= ANGLE_MID_DEFAULT;
            angle_f_prev   <= ANGLE_MID_DEFAULT;
            ang_held       <= 1'b0;

            angle_down_now <= ANGLE_DOWN;
            calib_acc      <= 20'd0;
            calib_acc_h    <= 20'd0;
            calib_cnt      <= 16'd0;

            pwm_cmd        <= 16'sd0;
            state_dbg      <= S_IDLE;
            led_run        <= 1'b0;
            led_manual     <= 1'b0;
            led_auto       <= 1'b1;
            led_err        <= 1'b0;
            angle_mid_dbg  <= ANGLE_MID_DEFAULT;
        end else begin
            // ---------------- 指示灯 ----------------
            led_run       <= run;
            led_manual    <= mode_manual;
            led_auto      <= ~mode_manual;
            state_dbg     <= state;
            angle_mid_dbg <= angle_mid;

            // =============================================================
            // 5ms 采样相关：角度毛刺门 / 下垂点标定 / 20ms 历史
            // =============================================================
            if(tick_5ms) begin
                angle_f_prev <= angle_f;
                if(ang_jump_big && !ang_held) begin
                    ang_held <= 1'b1;          // 第一拍判为毛刺，angle_f 保持
                end else begin
                    angle_f  <= angle;
                    ang_held <= 1'b0;
                end

                // ---- 下垂点自动标定 ----
                if((AUTO_CALIB != 0) && (state != S_BALANCE) && (state != S_SWING) &&
                   down_ok && calib_still) begin
                    if(calib_cnt < CALIB_TICKS) begin
                        calib_acc <= calib_acc + {8'b0, angle_f};
                        if(calib_cnt < CALIB_HALF)
                            calib_acc_h <= calib_acc_h + {8'b0, angle_f};
                        calib_cnt <= calib_cnt + 16'd1;
                    end else begin
                        if(calib_quiet &&
                           (calib_avg >= ANGLE_DOWN - CALIB_TOL) &&
                           (calib_avg <= ANGLE_DOWN + CALIB_TOL)) begin
                            angle_down_now <= calib_avg;
                            angle_mid      <= calib_avg + MID_FROM_DOWN;
                        end
                        calib_cnt   <= 16'd0;
                        calib_acc   <= 20'd0;
                        calib_acc_h <= 20'd0;
                    end
                end else begin
                    calib_cnt   <= 16'd0;
                    calib_acc   <= 20'd0;
                    calib_acc_h <= 20'd0;
                end

                // ---- 20ms 微分窗口：4 级历史，每个 5ms 拍刷新一次 ----
                //   这样每个拍都能拿到「正好 20ms 前」的采样，
                //   窗口长度不会随拍数变化（否则速度会被低估 1~4 倍）。
                enc_h1 <= enc;
                enc_h2 <= enc_h1;
                enc_h3 <= enc_h2;
                enc_h4 <= enc_h3;
                // 存死区后的角度：这样 20ms 差分自然也在带内归零
                ang_h1 <= ang_db;
                ang_h2 <= ang_h1;
                ang_h3 <= ang_h2;
                ang_h4 <= ang_h3;

                // ---- 摆臂位置积分器累加 ----
                //   仅在平衡态、且偏差在窗口内时积分；离开平衡立即清零，
                //   防止起摆/摔倒过程中积出巨大的值。
                if(!run || (state != S_BALANCE))
                    th_i <= 16'sd0;
                else if(!i_win_ok)
                    th_i <= 16'sd0;            // 偏差太大 -> 交给 LQR 自己救，别积分
                else if(th_i_nxt >  I_MAX)
                    th_i <=  I_MAX;
                else if(th_i_nxt < -I_MAX)
                    th_i <= -I_MAX;
                else
                    th_i <= th_i_nxt[15:0];

                // ---- 限幅级数跟踪（带回滞）----
                lv_cur <= lv_new;
            end

            // =============================================================
            // 5ms 周期控制
            // =============================================================
            if(run && tick_5ms) begin
                ang_vel_fast  <= ang_err - ang_err_prev5;
                ang_err_prev5 <= ang_err;
                enc_rel_prev5 <= enc_rel;

                // ---- 目标斜坡（仅平衡态）----
                if(state == S_BALANCE)
                    pos_target <= pos_target_next;

                // ---- 保护：连续 PROT_DEBOUNCE 拍异常才判定 ----
                if(state != S_BALANCE)
                    prot_cnt <= 16'd0;
                else if(bal_prot_bad && (prot_cnt < PROT_DEBOUNCE))
                    prot_cnt <= prot_cnt + 16'd1;
                else if(!bal_prot_bad)
                    prot_cnt <= 16'd0;

                if((state == S_BALANCE) && (prot_cnt >= PROT_DEBOUNCE)) begin
                    // 摆杆真倒了 / 摆臂飞车 -> 回等待，等摆杆垂下来再起摆
                    state         <= S_WAIT;
                    wait_cnt      <= 16'd0;
                    wait_bal_cnt  <= 16'd0;
                    prot_cnt      <= 16'd0;
                    swing_timer   <= 32'd0;
                    success_count <= 16'd0;
                    pwm_cmd       <= 16'sd0;
                    err_flag      <= 1'b1;
                    err_timer     <= 32'd0;
                end else begin
                    case(state)
                        // ---------- 等待摆杆回到下垂位置 ----------
                        S_WAIT: begin
                            pwm_cmd <= 16'sd0;
                            if(bal_ok_now) begin
                                // 误触发：摆杆还在竖直附近，25ms 后直接回平衡
                                wait_cnt <= 16'd0;
                                if(wait_bal_cnt >= 16'd4) begin
                                    state        <= S_BALANCE;
                                    prot_cnt     <= 16'd0;
                                    wait_bal_cnt <= 16'd0;
                                    // 以当前位置为新目标，避免旧目标造成位置项踢动
                                    pos_target   <= {{16{enc_rel[15]}}, enc_rel};
                                    pos_cmd      <= {{16{enc_rel[15]}}, enc_rel};
                                end else
                                    wait_bal_cnt <= wait_bal_cnt + 16'd1;
                            end else begin
                                wait_bal_cnt <= 16'd0;
                                if(can_swing) begin
                                    if(wait_cnt >= WAIT_STABLE_CNT) begin
                                        state         <= S_SWING;
                                        swing_timer   <= 32'd0;
                                        success_count <= 16'd0;
                                        wait_cnt      <= 16'd0;
                                    end else
                                        wait_cnt <= wait_cnt + 16'd1;
                                end else
                                    wait_cnt <= 16'd0;
                            end
                        end

                        // ---------- 自动起摆（能量泵） ----------
                        S_SWING: begin
                            swing_timer <= swing_timer + 32'd1;

                            if(in_window) begin
                                // 进入角度窗口：立刻交给 LQR（输出由下方流水线写入），不再泵能量
                                if(catch_vel_ok) begin
                                    if(success_count >= SUCCESS_NEED) begin
                                        // 稳住 50ms，正式进平衡；θ_ref 取为当前位置，无突变
                                        state         <= S_BALANCE;
                                        pos_target    <= {{16{enc_rel[15]}}, enc_rel};
                                        pos_cmd       <= {{16{enc_rel[15]}}, enc_rel};
                                        success_count <= 16'd0;
                                    end else
                                        success_count <= success_count + 16'd1;
                                end else
                                    success_count <= 16'd0;
                            end else begin
                                success_count <= 16'd0;
                                pwm_cmd <= clamp_pwm($signed(swing_cmd)
                                          - $signed(SWING_DAMP) * $signed(enc_d5));
                            end

                            // 超时 / 摆臂转速过快 -> 回等待
                            if((swing_timer > SWING_TIMEOUT_MAX) ||
                               (enc_d5 >  $signed({1'b0, SWING_VEL_MAX})) ||
                               (enc_d5 < -$signed({1'b0, SWING_VEL_MAX}))) begin
                                state         <= S_WAIT;
                                wait_cnt      <= 16'd0;
                                wait_bal_cnt  <= 16'd0;
                                swing_timer   <= 32'd0;
                                success_count <= 16'd0;
                                pwm_cmd       <= 16'sd0;
                                err_flag      <= 1'b1;
                                err_timer     <= 32'd0;
                            end
                        end

                        // ---------- LQR 稳摆 ----------
                        // 输出由下方流水线写入（每 5ms 一次，微分项是 20ms 窗口）
                        S_BALANCE: begin end

                        default: pwm_cmd <= 16'sd0;
                    endcase
                end
            end

            // =============================================================
            // LQR 计算流水线（与 5ms 节拍同步启动）
            //   第 0 拍：锁存状态 -> 第 1 拍：4 个乘法 -> 第 2 拍：求和/限幅输出
            //   50MHz 下单拍跑不完 4 个乘法 + 32 位加法 + 限幅，必须拆开。
            // =============================================================
            lq_v1 <= tick_5ms && run;
            lq_v2 <= lq_v1;
            lq_v3 <= lq_active;

            if(tick_5ms) begin
                s_th_err <= th_err_c;
                s_ang    <= ang_db_c;
                s_thd    <= thd_v;
                s_ald    <= ald_v;
                lim_r    <= lim_cur;
            end

            if(lq_v1) begin
                m_th  <= (m_th_raw  <<< p_sh) + (integ_en ? m_i_w : 33'sd0);
                m_thd <= m_thd_raw <<< g_sh;
                m_al  <= m_al_raw;
                m_ald <= (m_ald_raw <<< g_sh) >>> ald_sh;
                lim_r <= lim_cur;          // 流水第二级：锁存本级限幅值
            end

            if(lq_active)
                u_x_r <= u_sat;            // 流水第三级：粗限幅

            if(lq_v3)
                pwm_cmd <= u_out;          // 流水第四级：削顶 / 乘系数

            // =============================================================
            // 按键（1 拍脉冲，必须在 clk 域处理，不能放进 tick_5ms 里）
            // =============================================================
            if(key_mode && !run)
                mode_manual <= ~mode_manual;

            if(key_start) begin
                if(run) begin
                    // 任何状态下按 SW1 = 停机（第一版调试用，方便随时断电）
                    run      <= 1'b0;
                    state    <= S_IDLE;
                    pwm_cmd  <= 16'sd0;
                    prot_cnt <= 16'd0;
                end else begin
                    run <= 1'b1;
                    if((ang_err <= $signed(MANUAL_WINDOW)) &&
                       (ang_err >= -$signed(MANUAL_WINDOW))) begin
                        // 摆杆已在竖直附近：直接闭环
                        state      <= S_BALANCE;
                        pos_target <= {{16{enc_rel[15]}}, enc_rel};
                        pos_cmd    <= {{16{enc_rel[15]}}, enc_rel};
                        prot_cnt   <= 16'd0;
                        pwm_cmd    <= 16'sd0;
                    end else if(mode_manual) begin
                        run       <= 1'b0;
                        err_flag  <= 1'b1;
                        err_timer <= 32'd0;
                    end else if(can_swing) begin
                        state         <= S_SWING;
                        swing_timer   <= 32'd0;
                        success_count <= 16'd0;
                        pwm_cmd       <= 16'sd0;
                    end else begin
                        state      <= S_WAIT;
                        wait_cnt   <= 16'd0;
                        pwm_cmd    <= 16'sd0;
                    end
                end
            end

            // 平衡中调摆臂目标（±30°，斜坡慢速跟随）
            if(run && (state == S_BALANCE)) begin
                if(key_inc && (pos_cmd < POS_MAX))
                    pos_cmd <= pos_cmd + STEP_POS;
                if(key_dec && (pos_cmd > -POS_MAX))
                    pos_cmd <= pos_cmd - STEP_POS;
            end

            // =============================================================
            // 错误指示：亮 1s
            // =============================================================
            if(err_flag) begin
                led_err <= 1'b1;
                if(err_timer < 32'd50_000_000)
                    err_timer <= err_timer + 32'd1;
                else begin
                    err_flag  <= 1'b0;
                    err_timer <= 32'd0;
                end
            end else begin
                led_err   <= 1'b0;
                err_timer <= 32'd0;
            end
        end
    end

endmodule

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
    // 竖直平衡位置：角度传感器（电位器 ADC）实测读数，直接固定写死，不做任何自动反推。
    // 实测：下垂 ≈1010，真正竖直 ≈2995。
    // 若需微调，可在停止状态下拨 B3_SW1，把当前角度设为新的平衡点。
    // 竖直平衡位置：角度传感器（电位器 ADC）实测读数。
    // 2026-10-27 实测：下垂 ≈ 936，竖直 ≈ 3082，差 2146 counts。
    //   2146 / 180° = 11.92 counts/°，满量程 4092/11.92 = 343°，
    //   与 WDD35D4 手册的 345° 只差 0.5% —— 两个独立测量互相印证，可以直接用。
    // （旧值 1010/3005 差 1995，只对应 168°，平衡点偏了 77 counts ≈ 6.5°，是错误的）
    // AUTO_CALIB = 1 时，这个值只在第一次掉下垂标定之前起作用。
    parameter [11:0] ANGLE_MID_DEFAULT = 12'd3082,
    parameter [11:0] MID_FROM_DOWN     = 12'd2146,   // 下垂点 -> 竖直点的计数差（=180°）
    parameter integer AUTO_CALIB       = 1,          // 1 = 摆杆垂到最低点且静止时自动标定平衡点
    parameter integer ANGLE_INV        = 0,          // 1 = 角度方向取反
    parameter integer ENC_INV          = 0,          // 1 = 编码器方向取反
    parameter integer SWING_INV        = 0,          // 1 = 起摆能量泵方向取反

    parameter [15:0] MANUAL_WINDOW     = 16'd700,    // 手动模式允许启动的角度窗口
    parameter [15:0] ANGLE_LIMIT       = 16'd800,    // 平衡时摆角超限保护（对应 IP570 的 ZHONGZHI±800）
    parameter [15:0] PWM_MAX           = 16'd5300,   // PWM 限幅（载波 5400 -> 98.1%，对齐 IP570 的接近满占空比）

    parameter signed [31:0] POS_STEP   = 32'sd2080,  // 旧功能，保留未用
    parameter signed [31:0] RAMP_STEP  = 32'sd1,     // 目标斜坡：每 5ms 移动量（≈69°/s，接近要求的 60°/s）
    parameter signed [31:0] STEP_D10   = 32'sd300,   // SW3/SW4 每次按键目标变化：30.0°（单位 0.1°）
    parameter signed [31:0] POS_D10_MAX= 32'sd14400, // 待确认目标限幅：±1440°（±4 圈）

    parameter [15:0] BAL_KP            = 16'd50,     // 角度环 P
    parameter [15:0] BAL_KD            = 16'd244,    // 角度环 D
    parameter [15:0] POS_KP            = 16'd25,     // 位置环 P
    parameter [15:0] POS_KD            = 16'd600,    // 位置环 D（实测 600 最好，300/500 都变差）

    parameter [15:0] SWING_GAIN        = 16'd1000,   // 起摆能量泵 PWM 幅值（约 19% 满量程）—— 越小能量粒度越细
    parameter [15:0] SWING_DAMP        = 16'd10,     // 起摆时摆臂速度阻尼（摆臂高速时会吃掉泵能量，不宜大）
    parameter [15:0] CATCH_WINDOW      = 16'd350,    // 进入平衡捕获的角度窗口（±29°）
    parameter [15:0] CATCH_VEL         = 16'd150,    // 进入平衡捕获的角速度窗口
    parameter [15:0] QUARTER           = 16'd1070,   // 90° 对应的 ADC 计数（11.92 counts/° × 90）
    parameter signed [31:0] SWING_TIMEOUT_MAX = 32'sd4000, // 20s 起摆超时（泵幅值降了，需要多甩几下）
    parameter [15:0] SUCCESS_NEED      = 16'd10,     // 连续满足捕获条件的次数
    parameter [15:0] ENC_JUMP_MAX      = 16'd30,     // 编码器 5ms 突变保护阈值（30 counts/5ms ≈2080°/s，只抓飞车）
    parameter [15:0] PROT_DEBOUNCE     = 16'd3,      // 保护连击次数：必须连续 4×5ms 都异常才停机
    parameter [12:0] ANGLE_JUMP_MAX    = 13'd400,    // 角度单拍最大可信跳变（≈34°/5ms），超出判为采样毛刺
    parameter [11:0] ANGLE_DOWN        = 12'd936,    // 摆杆自然下垂时的 ADC 读数（2026-10-27 实测）
    parameter [11:0] CALIB_TOL         = 12'd250,    // 标定结果允许偏离 ANGLE_DOWN 的范围（必须放宽，否则零漂时标定会被丢弃）
    parameter [3:0]  CALIB_SHIFT       = 4'd6,       // 标定窗口 2^6 = 64 拍 = 320ms（必须远大于摆杆摆动周期）
    parameter signed [19:0] CALIB_DRIFT_MAX = 20'sd128, // 前后半段均值差上限（÷32 = 4 counts）
    parameter [12:0] CALIB_STILL_MAX   = 13'd24,     // 单拍角度变化上限（≈4σ，只挡掉大动作）
    parameter [11:0] DOWN_WINDOW       = 12'd300,    // 允许开始起摆的下垂窗口（±300 counts ≈ ±27°）
    parameter [15:0] WAIT_VEL_MAX      = 16'd20,     // 等待时允许的角速度（counts/5ms，≈360°/s）
    parameter [15:0] WAIT_STABLE_CNT   = 16'd10,     // 连续满足下垂条件 10×5ms = 50ms 才起摆
    parameter [15:0] SWING_VEL_MAX     = 16'd100,    // 起摆中摆臂速度上限（counts/5ms ≈1240°/s），超了回等待
    parameter [15:0] POS_ENABLE_TICKS  = 16'd200,    // 进入平衡后 200×5ms = 1s 才接入位置环（等摆臂停稳，避免捕获瞬间被拽）
    // ---------------- 摇杆（手轮）参数 ----------------
    // 手轮可以被转得很快，而摆臂受“摆杆必须保持直立”的物理限制，
    // 不可能瞬时就位。处理策略：允许滞后，但限速——
    //   ① wheel_pos 保存手轮的真实目标位置（最终一定要走到这里）
    //   ② 实际给定按 WHEEL_RAMP_STEP 限速靠近 wheel_pos
    //   ③ 给定不得超前当前实际位置 WHEEL_LAG_MAX（防止位置环长期饱和把摆杆甩倒）
    parameter signed [31:0] WHEEL_RAMP_STEP = 32'sd2,     // 摇杆目标速度上限：2 counts/5ms ≈ 138°/s
    parameter signed [31:0] WHEEL_LAG_MAX   = 32'sd150,   // 给定允许超前实际位置的最大量（150 counts ≈ 52°）
    parameter signed [31:0] WHEEL_POS_MAX   = 32'sd20000, // 摇杆模式目标限幅（±19 圈）
    parameter signed [31:0] WHEEL_DEADBAND  = 32'sd2,     // 手轮增量死区（过滤静止抖动）
    parameter signed [31:0] POS_ERR_MAX     = 32'sd600,   // 位置环最大误差（防止大滞后把角度环架空）
    parameter integer       WHEEL_INV       = 0,          // 1 = 手轮方向取反（转手轮时摆臂反向就跟）
    parameter integer POS_FILT_SHIFT   = 3           // 位置一阶低通：1/8
)(
    input  wire                clk,
    input  wire                rst_n,
    input  wire                tick_5ms,
    input  wire                tick_25ms,

    input  wire [11:0]         angle,       // ADC 角度
    input  wire [15:0]         enc,         // 编码器位置（1 圈 = 1040，摆臂）
    input  wire [15:0]         enc_wheel,   // 手轮编码器（XH2，仅摇杆模式使用）

    input  wire                key_start,   // SW1：起摆 / 曲线暂停-恢复 / 位置模式确认
    input  wire                key_mode,    // 保留（当前未用）
    input  wire                key_inc,     // SW3：位置模式 +30° / 曲线模式 幅度+
    input  wire                key_dec,     // SW4：位置模式 −30° / 曲线模式 幅度−
    input  wire [2:0]          traj_sel,    // SW2：0=不运动 1=正弦 2=三角 3=S形 4=匀速
    input  wire                dir_inv,     // DIP2：低有效，编码器方向取反

    output reg  signed [15:0]  pwm_cmd,
    output reg  [3:0]          state_dbg,
    output reg                 led_run,
    output reg                 led_manual,
    output reg                 led_auto,
    output reg                 led_err,
    output reg  [11:0]         angle_mid_dbg
);

    // ---------------- 状态机定义 ----------------
    localparam [3:0] S_IDLE    = 4'd0;
    localparam [3:0] S_SWING   = 4'd1;
    localparam [3:0] S_BALANCE = 4'd2;
    localparam [3:0] S_WAIT    = 4'd3;   // 等待摆杆回到下垂位置再起摆

    // ---------------- 角度采样毛刺门 ----------------
    // ADC 偶尔会出一次野值（并行总线干扰、转换未就绪等）。单靠 16 次平均挡不住，
    // 而一个野值就足以让 ang_err 跳过 ±800 触发保护，或者让 KD 项猛踢一脚。
    // 这里只做“单拍丢弃”：跳变超过 ANGLE_JUMP_MAX 就沿用上一拍的读数，
    // 连续第二拍还在跳则说明是真实变化，直接接受（不会被永久冻结）。
    // 正常工作时该电路不引入任何滞后。
    reg  [11:0]       angle_f;
    reg  [11:0]       angle_f_prev;      // 仅用于标定时判断“是否静止”
    reg               ang_held;
    reg  [15:0]       enc_zero;          // 编码器原点：每次进入平衡时以当前位置作为新原点

    wire signed [12:0] ang_jump_s = $signed({1'b0, angle}) - $signed({1'b0, angle_f});
    wire               ang_jump_big = (ang_jump_s >  $signed(ANGLE_JUMP_MAX)) ||
                                      (ang_jump_s < -$signed(ANGLE_JUMP_MAX));

    // 标定时用的“静止”判断，不依赖平衡态的历史寄存器（run=0 时那些寄存器不刷新）
    wire signed [12:0] ang_still_d = $signed({1'b0, angle_f}) - $signed({1'b0, angle_f_prev});
    wire               calib_still = (ang_still_d <  $signed(CALIB_STILL_MAX)) &&
                                     (ang_still_d > -$signed(CALIB_STILL_MAX));

    // ---------------- 角度换算 ----------------
    wire signed [15:0] ang_now     = $signed({4'b0, angle_f});
    wire signed [15:0] mid_now     = $signed({4'b0, angle_mid});
    wire signed [15:0] ang_err_raw = ang_now - mid_now;
    wire signed [15:0] ang_err     = (ANGLE_INV != 0) ? -ang_err_raw : ang_err_raw;

    // ---------------- 编码器换算（相对原点） ----------------
    // enc_zero 复位时为 10000（与 encoder.v 一致）；每次进入平衡态时改成当时的 enc，
    // 也就是「摆臂停在哪里，那里就是新原点」。这样位置环不需要把摆臂拽回上电位置。
    wire signed [31:0] enc_rel_raw = $signed({16'b0, enc}) - $signed({16'b0, enc_zero});
    wire               inv_enc     = ((ENC_INV != 0) ^ dir_inv);
    wire signed [31:0] enc_rel     = inv_enc ? -enc_rel_raw : enc_rel_raw;

    // ---------------- 寄存器 ----------------
    reg [11:0]        angle_mid;
    reg               run;
    reg [3:0]         state;
    reg               mode_manual;       // 0 = 自动起摆, 1 = 手动辅助

    reg signed [31:0] pos_cmd_d10;       // 待确认目标，单位 0.1°（SW3/SW4 累加修改）
    reg signed [31:0] pos_cmd;           // 已确认目标，单位编码器计数（SW1 确认时换算）
    reg signed [31:0] pos_target;        // 斜坡跟随后的目标
    reg signed [31:0] pos_filt;          // 位置误差一阶滤波
    reg signed [31:0] pos_vel;           // 位置误差微分
    reg signed [15:0] pos_pwm;           // 位置环输出

    reg signed [15:0] ang_err_prev;      // 5ms 角误差历史（角度环 D 项 + 起摆共用）
    reg signed [15:0] ang_err_prev25;    // 25ms 角误差历史（保留，当前未参与 PD）
    reg signed [15:0] ang_vel_fast;      // 5ms 角速度（起摆用）
    reg signed [15:0] ang_vel_pd;        // 25ms 角速度（保留，当前未参与 PD，勿用于角度环）

    reg [31:0]        swing_timer;
    reg [15:0]        success_count;
    reg signed [31:0] enc_rel_prev;

    reg [31:0]        err_timer;
    reg               err_flag;

    reg [15:0]        bal_timer;         // 进平衡后的计时，用于延迟接入位置环
    reg               traj_run;          // 曲线运行标志（SW1 在平衡状态下切换）
    reg [2:0]         traj_sel_d;        // 上一拍的曲线号，用于检测换曲线
    reg [15:0]        enc_wheel_prev;    // 手轮编码器上一拍（5ms 前）
    reg signed [31:0] wheel_acc;         // 手轮增量累积（死区判断用）
    reg [31:0]        wait_timer;        // S_WAIT 停留计时
    reg [15:0]        wait_cnt;          // 连续满足下垂条件的次数
    reg [15:0]        wait_bal_cnt;      // S_WAIT 中连续满足“竖直附近”的次数（自恢复用）
    reg [15:0]        prot_cnt;          // 保护条件连续成立的拍数（去抖）
    reg signed [31:0] wheel_pos;         // 手轮真实目标位置（无限速，最终位置以它为准）

    // 下垂点自动标定
    reg [11:0]        angle_down_now;    // 当前认定的下垂点（标定后会更新）
    reg [19:0]        calib_acc;         // 全窗口累加（64 × 4095 = 261888）
    reg [19:0]        calib_acc_h;       // 前半段累加（32 × 4095）
    reg [15:0]        calib_cnt;         // 标定计数
    reg               calib_ok;          // 已经成功标定过至少一次
    localparam [15:0] CALIB_TICKS = (16'd1 << CALIB_SHIFT);   // 64
    localparam [15:0] CALIB_HALF  = (CALIB_TICKS >> 1);       // 32

    // ---------------- 组合逻辑 ----------------
    // 待确认目标（0.1°）-> 编码器计数：1040 counts/圈 ÷ 360° = 2.8889 counts/°
    //   0.1° = 0.28889 counts ≈ 74/256 = 0.28906（误差 0.06%，用乘法+移位避免除法器）
    wire signed [31:0] pos_cmd_conv = ($signed(pos_cmd_d10) * 32'sd74) >>> 8;

    // ---------------- 模式译码 ----------------
    // traj_sel: 0 = 点位（不运动），1~4 = 四条曲线，5 = 摇杆（手轮）
    wire curve_mode   = (traj_sel >= 3'd1) && (traj_sel <= 3'd4);
    wire wheel_mode   = (traj_sel == 3'd5);
    wire traj_sel_act = curve_mode || wheel_mode;
    wire [1:0] traj_mode = traj_sel[1:0] - 2'd1;   // 1..4 -> 0..3

    // 曲线/摇杆都要在平衡态 + SW1 确认 + 起摆稳定期之后才生效
    wire sel_confirmed = (state == S_BALANCE) && traj_run && run
                         && (bal_timer >= POS_ENABLE_TICKS);
    wire traj_en  = curve_mode && sel_confirmed;
    wire wheel_en = wheel_mode && sel_confirmed;

    // 手轮增量：两台电机型号参数完全相同，1:1 映射（手轮转多少，摆臂目标转多少）
    wire signed [31:0] wheel_delta_raw =
        $signed({16'b0, enc_wheel}) - $signed({16'b0, enc_wheel_prev});
    wire signed [31:0] wheel_delta = (WHEEL_INV != 0) ? -wheel_delta_raw : wheel_delta_raw;
    wire signed [31:0] wheel_acc_next = wheel_acc + wheel_delta;
    wire signed [31:0] traj_pos;
    wire signed [31:0] traj_amp_dbg;

    traj_gen #(
        .AMP_INIT  (32'sd100),      // 默认幅度 100 counts ≈ 34.6°
        .AMP_MIN   (32'sd20),
        .AMP_MAX   (32'sd300),
        .AMP_STEP  (32'sd29),       // 每次按键约 ±10°
        .PHASE_STEP(16'd82),        // ≈4s 一个周期
        .V_CONST   (32'sd1),        // 匀速曲线 ≈69°/s
        .POS_LIMIT (32'sd20000)
    ) u_traj (
        .clk      (clk),
        .rst_n    (rst_n),
        .tick_5ms (tick_5ms),
        .en       (traj_en),
        .mode     (traj_mode),
        .amp_up   (key_inc && curve_mode && (state == S_BALANCE)),
        .amp_dn   (key_dec && curve_mode && (state == S_BALANCE)),
        .pos_ref  (traj_pos),
        .amp_dbg  (traj_amp_dbg)
    );

    // 目标位置斜坡（每 5ms 移动 RAMP_STEP，模拟 IP570 的 Position_Zero++）
    // 摇杆模式下不再使用这条斜坡，改由下面带限速的 wheel_cmd_next 生成
    wire signed [31:0] ramp_step_sel = wheel_mode ? WHEEL_RAMP_STEP : RAMP_STEP;
    wire signed [31:0] pos_target_next =
        (pos_target < pos_cmd) ? (pos_target + ramp_step_sel) :
        (pos_target > pos_cmd) ? (pos_target - ramp_step_sel) : pos_target;

    // ---------------- 摇杆目标生成：限速 + 超前限幅 ----------------
    // ① 限速：每 5ms 最多靠近手轮目标 WHEEL_RAMP_STEP（这就是“速度上限”）
    // ② 超前限幅：给定不得超前实际位置 WHEEL_LAG_MAX
    //    没有这一条时，手轮猛转会把给定推到很远，位置环长期饱和 →
    //    角度环被架空 → 摆臂越跑越快 → 摆杆必倒。
    // 手轮停下后，给定会继续慢慢追上 wheel_pos，最终位置与手轮一致（只是滞后）。
    wire signed [31:0] wheel_diff = wheel_pos - pos_cmd;
    wire signed [31:0] wheel_step = (wheel_diff >  WHEEL_RAMP_STEP) ?  WHEEL_RAMP_STEP :
                                    (wheel_diff < -WHEEL_RAMP_STEP) ? -WHEEL_RAMP_STEP :
                                    wheel_diff;
    wire signed [31:0] wheel_cmd_s = pos_cmd + wheel_step;
    wire signed [31:0] wheel_cmd_next =
        (wheel_cmd_s > enc_rel + WHEEL_LAG_MAX) ? (enc_rel + WHEEL_LAG_MAX) :
        (wheel_cmd_s < enc_rel - WHEEL_LAG_MAX) ? (enc_rel - WHEEL_LAG_MAX) :
        wheel_cmd_s;

    wire signed [31:0] pos_err_raw = enc_rel - pos_target_next;
    wire signed [31:0] pos_err_w   =
        (pos_err_raw >  POS_ERR_MAX) ?  POS_ERR_MAX :
        (pos_err_raw < -POS_ERR_MAX) ? -POS_ERR_MAX :
        pos_err_raw;
    wire signed [31:0] pos_filt_next =
        pos_filt - (pos_filt >>> POS_FILT_SHIFT) + (pos_err_w >>> POS_FILT_SHIFT);
    wire signed [31:0] pos_vel_next = pos_filt_next - pos_filt;

    // 角度环 PD：balance = -(KP*Bias + KD*D_Bias)
    //
    // 【关键】IP570 的 Balance() 在 TIM1 的 5ms 中断里执行，
    //   D_Bias = Bias - Last_Bias   （5ms 差分）
    // 所以这里必须同样用 5ms 差分。若在 25ms 里算差分，同样的角度变化率会
    // 得到 5 倍大的数值，等效 KD = 244*5 = 1220，平衡点附近必然剧烈抖动、站不住。
    // ang_err_prev 每 5ms 更新一次，因此 (ang_err - ang_err_prev) 就是 5ms 差分。
    wire signed [15:0] ang_d5   = ang_err - ang_err_prev;

    // ---------------- 起摆前的下垂位置判断 ----------------
    // 摆杆倒下后不能立刻用能量泵，否则会在半空乱推导致甩飞。
    // 必须等到角度传感器读数接近自然下垂点、且角速度很小，才允许起摆。
    wire signed [12:0] down_diff =
        $signed({1'b0, angle_f}) - $signed({1'b0, angle_down_now});
    wire down_ok = (down_diff <  $signed({1'b0, DOWN_WINDOW})) &&
                   (down_diff > -$signed({1'b0, DOWN_WINDOW}));
    wire wait_vel_ok = (ang_d5 <  $signed(WAIT_VEL_MAX)) &&
                       (ang_d5 > -$signed(WAIT_VEL_MAX));
    wire can_swing   = down_ok && wait_vel_ok;

    // 摆杆是否仍在竖直附近（S_WAIT 自恢复用）：误触发保护后不用等它倒下重起
    wire bal_ok_now  = (ang_err <  $signed(CATCH_WINDOW)) &&
                       (ang_err > -$signed(CATCH_WINDOW)) &&
                       (ang_vel_fast <  $signed(CATCH_VEL)) &&
                       (ang_vel_fast > -$signed(CATCH_VEL));

    // 平衡态保护条件（不含去抖，去抖在 5ms 块里用 prot_cnt 做）
    wire bal_prot_bad =
        (ang_err >  $signed(ANGLE_LIMIT)) ||
        (ang_err < -$signed(ANGLE_LIMIT)) ||
        ((enc_rel - enc_rel_prev) >  $signed({16'b0, ENC_JUMP_MAX})) ||
        ((enc_rel - enc_rel_prev) < -$signed({16'b0, ENC_JUMP_MAX}));

    // 乘积先按 32-bit 计算，避免综合出 48x48 乘法器
    wire signed [31:0] bal_p32 = $signed(BAL_KP) * $signed(ang_err);
    wire signed [31:0] bal_d32 = $signed(BAL_KD) * $signed(ang_d5);
    wire signed [47:0] bal_pwm_w = -($signed(bal_p32) + $signed(bal_d32));

    // 位置环 PD：Position_PWM = Position_Bias*KP + Position_Differential*KD
    wire signed [31:0] pos_p32 = $signed(POS_KP) * pos_filt_next;
    wire signed [31:0] pos_d32 = $signed(POS_KD) * pos_vel_next;
    wire signed [47:0] pos_pwm_w = $signed(pos_p32) + $signed(pos_d32);

    // ---------------- 标定结算 ----------------
    // 窗口结束时 calib_acc 里已是 CALIB_TICKS 个样本。
    // 静判定：前半段与后半段的平均值之差必须很小：
    //   差值×32 = calib_acc − 2×calib_acc_h = 后半段和 − 前半段和
    // 这一步才是真正在判断「摆杆到底有没有在晃」，看单拍差分会被噪声骗过。
    // 噪声下该判据 σ≈0.84 counts，而 1°（约 12 counts）的漂移就会被抓住。
    wire [19:0] calib_sum_avg = calib_acc >> CALIB_SHIFT;
    wire [11:0] calib_avg     = calib_sum_avg[11:0];
    wire signed [20:0] calib_drift_sum =
        $signed({1'b0, calib_acc}) - $signed({1'b0, calib_acc_h << 1});
    wire calib_quiet = (calib_drift_sum <= CALIB_DRIFT_MAX) &&
                       (calib_drift_sum >= -CALIB_DRIFT_MAX);

    // 起摆能量泵：u = -sign(v * cos(angle_err))
    //   |err| < QUARTER -> cos>0 -> u = -sign(v)
    //   否则            -> cos<0 -> u = +sign(v)
    wire signed [15:0] swing_cmd_base =
        ((ang_err > $signed(QUARTER)) || (ang_err < -$signed(QUARTER)))
        ? ((ang_vel_fast >= 0) ? $signed(SWING_GAIN) : -$signed(SWING_GAIN))
        : ((ang_vel_fast >= 0) ? -$signed(SWING_GAIN) : $signed(SWING_GAIN));

    wire signed [15:0] swing_cmd =
        (SWING_INV != 0) ? -swing_cmd_base : swing_cmd_base;

    // ---------------- PWM 限幅函数 ----------------
    localparam signed [15:0] PWM_MAX_16 = PWM_MAX;
    localparam signed [47:0] PWM_MAX_48 = PWM_MAX;

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
    // IP570 的 Position_PWM 虽然不限幅，但它的摆臂几乎不动；本机 10-bit ADC +
    // 轻质摆杆下摆臂动作大得多，必须给角度环留出余地。
    localparam signed [15:0] POS_PWM_MAX    = PWM_MAX/2    ;   // 2650
    localparam signed [47:0] POS_PWM_MAX_48 = PWM_MAX / 2;

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

    // ---------------- 手轮目标限幅函数 ----------------
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

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            angle_mid      <= ANGLE_MID_DEFAULT;
            run            <= 1'b0;
            state          <= S_IDLE;
            mode_manual    <= 1'b0;

            pos_cmd_d10    <= 32'sd0;
            pos_cmd        <= 32'sd0;
            pos_target     <= 32'sd0;
            pos_filt       <= 32'sd0;
            pos_vel        <= 32'sd0;
            pos_pwm        <= 16'sd0;

            ang_err_prev   <= 16'sd0;
            ang_err_prev25 <= 16'sd0;
            ang_vel_fast   <= 16'sd0;
            ang_vel_pd     <= 16'sd0;

            swing_timer    <= 32'd0;
            success_count  <= 16'd0;
            enc_rel_prev   <= 32'sd0;
            enc_zero       <= 16'd10000;   // 与 encoder.v 复位初值一致

            err_timer      <= 32'd0;
            err_flag       <= 1'b0;

            bal_timer      <= 16'd0;
            traj_run       <= 1'b0;      // 复位后不自动跑曲线，需按 SW1 确认
            traj_sel_d     <= 3'd0;
            enc_wheel_prev <= 16'd10000;   // 与 encoder.v 的复位初值保持一致，避免首拍出现巨大增量
            wheel_acc      <= 32'sd0;
            wheel_pos      <= 32'sd0;
            wait_timer     <= 32'd0;
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
            calib_ok       <= 1'b0;

            pwm_cmd        <= 16'sd0;
            state_dbg      <= S_IDLE;
            led_run        <= 1'b0;
            led_manual     <= 1'b0;
            led_auto       <= 1'b1;
            led_err        <= 1'b0;
            angle_mid_dbg  <= ANGLE_MID_DEFAULT;
        end else begin
            // ---------------- 状态灯 ----------------
            led_run       <= run;
            led_manual    <= mode_manual;
            led_auto      <= ~mode_manual;
            state_dbg     <= state;
            angle_mid_dbg <= angle_mid;

            // ---------------- 角度毛刺门（每 5ms，无条件运行） ----------------
            if(tick_5ms) begin
                angle_f_prev <= angle_f;
                if(ang_jump_big && !ang_held) begin
                    ang_held <= 1'b1;          // 第一拍：判为毛刺，angle_f 保持不变
                end else begin
                    angle_f  <= angle;         // 正常 / 连续第二拍跳变：接受新值
                    ang_held <= 1'b0;
                end
            end

            // ---------------- 下垂点自动标定 ----------------
            // 条件：“在下垂窗口内” + “每拍变化很小”，连续采 8 拍（40ms）平均，
            // 再用固定的 180° 计数差 MID_FROM_DOWN 反推竖直点。
            // 只在没在起摆/平衡的时候做（S_IDLE / S_WAIT 都可以），
            // 所以上电时摆杆垂着会自动标；起摆失败回 S_WAIT 时也会重新标，跟踪零点漂移。
            if(tick_5ms) begin
                if((AUTO_CALIB != 0) && (state != S_BALANCE) && (state != S_SWING) &&
                   down_ok && calib_still) begin
                    if(calib_cnt < CALIB_TICKS) begin
                        // 累加：同时记录前半段，结算时用于判断是否真的静止
                        calib_acc <= calib_acc + {8'b0, angle_f};
                        if(calib_cnt < CALIB_HALF)
                            calib_acc_h <= calib_acc_h + {8'b0, angle_f};
                        calib_cnt <= calib_cnt + 16'd1;
                    end else begin
                        // 结算：真的静止 + 结果落在标称下垂点附近，才采用
                        if(calib_quiet &&
                           (calib_avg >= ANGLE_DOWN - CALIB_TOL) &&
                           (calib_avg <= ANGLE_DOWN + CALIB_TOL)) begin
                            angle_down_now <= calib_avg;
                            angle_mid      <= calib_avg + MID_FROM_DOWN;
                            calib_ok       <= 1'b1;
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
            end

            // 换曲线后自动暂停：避免旧曲线跑到一半突然跳到新曲线造成摆臂冲击，
            // 必须重新按 SW1 确认才会开始跑新曲线。
            traj_sel_d <= traj_sel;
            if(traj_sel != traj_sel_d)
                traj_run <= 1'b0;

            // ---------------- 角度零点 ----------------
            // 竖直平衡点固定为 ANGLE_MID_DEFAULT（= 3005），上电后不做任何自动校准。
            // （B3_SW1 已改作「位置模式 / 曲线模式」选择，不再用于零点校准）

            // ---------------- 手动/自动切换（仅停止时） ----------------
            if(key_mode && !run)
                mode_manual <= ~mode_manual;

            // ---------------- 启动 / 起摆 / 曲线控制 ----------------
            // SW1：
            //   停止状态     -> 起摆
            //   平衡 + 曲线模式 -> 曲线暂停/恢复
            //   平衡 + 位置模式 -> 确认，按 ±30° 目标平滑旋转
            //   起摆过程中   -> 无效（按设计要求，只有 SW5 硬复位能停机）
            if(key_start) begin
                if(run && (state == S_BALANCE)) begin
                    if(traj_sel_act)
                        traj_run <= ~traj_run;      // 曲线：运行 ↔ 暂停
                    else
                        pos_cmd <= pos_cmd_conv;    // 点位模式：确认 ±30° 目标
                end else if(!run) begin
                    run <= 1'b1;

                    // 如果摆杆已经在竖直附近，无论手动/自动模式都直接进入平衡
                    if(ang_err <= $signed(MANUAL_WINDOW) &&
                       ang_err >= -$signed(MANUAL_WINDOW)) begin
                        state          <= S_BALANCE;
                        enc_zero       <= enc;        // 以当前位置为新原点，摆臂不用再回上电位置
                        pos_target     <= 32'sd0;     // 新原点就是当前位置
                        pos_cmd        <= 32'sd0;
                        pos_cmd_d10    <= 32'sd0;
                        pos_filt       <= 32'sd0;
                        pos_vel        <= 32'sd0;
                        pos_pwm        <= 16'sd0;
                        bal_timer      <= 16'd0;
                        wheel_pos      <= 32'sd0;
                        traj_run       <= 1'b0;      // 等待 SW1 确认后才跑曲线
                        // 必须同步微分历史，否则第一拍 D 项 = KD*ang_err 会打满 PWM
                        ang_err_prev   <= ang_err;
                        ang_err_prev25 <= ang_err;
                        enc_rel_prev   <= 32'sd0;    // 新原点下 enc_rel 立刻变 0
                    end else if(mode_manual) begin
                        // 手动模式但摆杆不在竖直附近：拒绝启动
                        run       <= 1'b0;
                        err_flag  <= 1'b1;
                        err_timer <= 32'd0;
                    end else if(can_swing) begin
                        // 摆杆已在下垂位置且基本静止：立即开始起摆
                        state         <= S_SWING;
                        swing_timer   <= 32'd0;
                        success_count <= 16'd0;
                        ang_err_prev  <= ang_err;
                        enc_rel_prev  <= enc_rel;
                        pwm_cmd       <= 16'sd0;
                    end else begin
                        // 摆杆还在半空乱晃：先等待摆杆垂下，不输出任何力矩
                        state         <= S_WAIT;
                        wait_timer    <= 32'd0;
                        wait_cnt      <= 16'd0;
                        ang_err_prev  <= ang_err;
                        enc_rel_prev  <= enc_rel;
                        pwm_cmd       <= 16'sd0;
                    end
                end
            end else if(run) begin
                // ================= 5ms 控制 =================
                if(tick_5ms) begin
                    ang_vel_fast <= ang_err - ang_err_prev;
                    ang_err_prev <= ang_err;
                    enc_rel_prev <= enc_rel;
                    enc_wheel_prev <= enc_wheel;   // 手轮增量基准（5ms 前的值）

                    // ---------------- 目标位置给定 ----------------
                    //   曲线模式：直接跟随 traj_gen（曲线本身已平滑）
                    //   摇杆模式：手轮增量累加成 wheel_pos，再用限速/超前限幅生成给定
                    //   点位模式：按 RAMP_STEP 斜坡平滑移动
                    // bal_timer：进平衡后的 5ms 计时，到达 POS_ENABLE_TICKS 才接入位置环
                    if(state == S_BALANCE) begin
                        if(traj_en) begin
                            pos_cmd    <= traj_pos;
                            pos_target <= traj_pos;
                            wheel_pos  <= traj_pos;    // 手轮基准跟随曲线，切模式时不跳变
                            wheel_acc  <= 32'sd0;
                        end else if(wheel_en) begin
                            // ① 手轮增量累加成真实目标（死区过滤静止抖动）
                            if(wheel_acc_next >  WHEEL_DEADBAND) begin
                                wheel_pos <= clamp_wheel(wheel_pos + wheel_acc_next);
                                wheel_acc <= 32'sd0;
                            end else if(wheel_acc_next < -WHEEL_DEADBAND) begin
                                wheel_pos <= clamp_wheel(wheel_pos + wheel_acc_next);
                                wheel_acc <= 32'sd0;
                            end else begin
                                wheel_acc <= wheel_acc_next;   // 未超死区：继续累积
                            end
                            // ② 限速 + 超前限幅后的实际给定
                            pos_cmd    <= wheel_cmd_next;
                            pos_target <= wheel_cmd_next;
                        end else begin
                            wheel_acc <= 32'sd0;
                            wheel_pos <= pos_target;   // 非摇杆模式：基准跟着走
                            pos_target <= pos_target_next;
                        end
                        if(bal_timer < POS_ENABLE_TICKS) begin
                            bal_timer <= bal_timer + 16'd1;
                            // 位置环接入前一刻，把「此刻」的摆臂位置定为原点。
                            // 捕获那一瞬间摆臂往往还在转，若以捕获点为原点，位置环一接入
                            // 就会把摆臂一路拽回去 —— 表现就是“捕获后还在回中”。
                            if(bal_timer == POS_ENABLE_TICKS - 16'd1) begin
                                enc_zero     <= enc;
                                pos_target   <= 32'sd0;
                                pos_cmd      <= 32'sd0;
                                pos_filt     <= 32'sd0;
                                pos_vel      <= 32'sd0;
                                enc_rel_prev <= 32'sd0;
                            end
                        end
                    end else begin
                        bal_timer  <= 16'd0;
                        wheel_acc  <= 32'sd0;
                        wheel_pos  <= 32'sd0;
                    end

                    // ---- 保护：仅平衡状态生效，连续 PROT_DEBOUNCE 拍都异常才判定 ----
                    // 以前是「单拍满足就停机」，ADC 野值 / 编码器噪声一下就能把电机断掉，
                    // 而 S_WAIT 里只等摆杆垂下，于是摆杆明明还直立着却只能看着它倒下。
                    if(state != S_BALANCE)
                        prot_cnt <= 16'd0;
                    else if(bal_prot_bad && (prot_cnt < PROT_DEBOUNCE))
                        prot_cnt <= prot_cnt + 16'd1;
                    else if(!bal_prot_bad)
                        prot_cnt <= 16'd0;

                    if(state == S_BALANCE && (prot_cnt >= PROT_DEBOUNCE)) begin
                        // 摆杆真倒了 / 摆臂飞车：先回到等待状态，等摆杆垂下来再起摆。
                        // 不能立刻启动能量泵，否则会在半空乱推导致甩飞。
                        state         <= S_WAIT;
                        wait_timer    <= 32'd0;
                        wait_cnt      <= 16'd0;
                        wait_bal_cnt  <= 16'd0;
                        prot_cnt      <= 16'd0;
                        swing_timer   <= 32'd0;
                        success_count <= 16'd0;
                        ang_err_prev  <= ang_err;
                        enc_rel_prev  <= enc_rel;
                        pwm_cmd       <= 16'sd0;
                        err_flag      <= 1'b1;
                        err_timer     <= 32'd0;
                    end else begin
                        case(state)
                            // ---------- 等待摆杆回到下垂位置 ----------
                            // 倒下后先让电机不出力，等摆杆靠重力自然垂到接近最下端、
                            // 且角速度足够小，再进入能量泵起摆。
                            S_WAIT: begin
                                pwm_cmd    <= 16'sd0;
                                wait_timer <= wait_timer + 32'd1;

                                // ---- 自恢复：保护可能是误触发 ----
                                // 摆杆其实还在竖直附近，就别等它倒下再重新起摆，
                                // 连续 5 拍（25ms）确认后直接回平衡态，电机只断 25ms。
                                if(bal_ok_now) begin
                                    wait_cnt <= 16'd0;
                                    if(wait_bal_cnt >= 16'd4) begin
                                        // 保留 pos_cmd / pos_target / traj_run：
                                        // 曲线或点位目标不丢失，恢复后接着走原目标，不会突然甩回 0
                                        state          <= S_BALANCE;
                                        pos_filt       <= 32'sd0;
                                        pos_vel        <= 32'sd0;
                                        pos_pwm        <= 16'sd0;
                                        bal_timer      <= 16'd0;      // 位置环重新延迟 500ms 接入
                                        wheel_pos      <= pos_cmd;
                                        wheel_acc      <= 32'sd0;
                                        ang_err_prev   <= ang_err;
                                        ang_err_prev25 <= ang_err;
                                        enc_rel_prev   <= enc_rel;
                                        wait_bal_cnt   <= 16'd0;
                                    end else begin
                                        wait_bal_cnt <= wait_bal_cnt + 16'd1;
                                    end
                                end else begin
                                    wait_bal_cnt <= 16'd0;
                                    if(can_swing) begin
                                        if(wait_cnt >= WAIT_STABLE_CNT) begin
                                            // 连续 50ms 稳定在下垂窗口：开始起摆
                                            state         <= S_SWING;
                                            swing_timer   <= 32'd0;
                                            success_count <= 16'd0;
                                            ang_err_prev  <= ang_err;
                                            enc_rel_prev  <= enc_rel;
                                            wait_cnt      <= 16'd0;
                                        end else begin
                                            wait_cnt <= wait_cnt + 16'd1;
                                        end
                                    end else begin
                                        wait_cnt <= 16'd0;
                                    end
                                end
                            end

                            // ---------- 自动起摆 ----------
                            S_SWING: begin
                                swing_timer <= swing_timer + 32'd1;

                                // ---- 分层判断（关键）----
                                // 只要摆杆进入角度窗口，就立刻从「泵能量」切到「角度 PD」，
                                // 不再看速度。以前速度超限时会继续泵，结果是摆杆冲得越快、
                                // 泵得越狠，越泵越收不住，最后只能等超时 —— 这就是起摆有时失败的主因。
                                if(ang_err <  $signed(CATCH_WINDOW) &&
                                   ang_err > -$signed(CATCH_WINDOW)) begin
                                    // 角度环 PD（与平衡态同一个式子，5ms 差分）
                                    pwm_cmd <= clamp_pwm(
                                        -($signed(BAL_KP) * $signed(ang_err) +
                                          $signed(BAL_KD) * $signed(ang_d5)));

                                    // 速度也够小，才累计「已捕获」次数
                                    if(ang_vel_fast <  $signed(CATCH_VEL) &&
                                       ang_vel_fast > -$signed(CATCH_VEL)) begin
                                        if(success_count >= SUCCESS_NEED) begin
                                            // 转入稳摆：以捕获时的摆臂位置作为新的原点
                                            state          <= S_BALANCE;
                                            enc_zero       <= enc;        // 摆臂停在哪里，那里就是原点
                                            pos_target     <= 32'sd0;     // 不再把摆臂拽回上电位置
                                            pos_cmd        <= 32'sd0;
                                            pos_cmd_d10    <= 32'sd0;
                                            pos_filt       <= 32'sd0;
                                            pos_vel        <= 32'sd0;
                                            pos_pwm        <= 16'sd0;
                                            bal_timer      <= 16'd0;
                                            wheel_pos      <= 32'sd0;
                                            traj_run       <= 1'b0;      // 等待 SW1 确认后才跑曲线
                                            ang_err_prev   <= ang_err;
                                            ang_err_prev25 <= ang_err;
                                            enc_rel_prev   <= 32'sd0;    // 新原点下 enc_rel 立刻变 0
                                            pwm_cmd        <= 16'sd0;
                                        end else begin
                                            success_count <= success_count + 16'd1;
                                        end
                                    end else begin
                                        // 冲得太快：继续用 PD 制动，但绝对不再泵能量
                                        success_count <= 16'd0;
                                    end
                                end else begin
                                    success_count <= 16'd0;
                                    // 能量泵 + 摆臂速度阻尼，防止摆臂越转越快
                                    pwm_cmd <= clamp_pwm(
                                        $signed(swing_cmd) -
                                        $signed(SWING_DAMP) * (enc_rel - enc_rel_prev));
                                end

                                // 超时 或 摆臂转速过快（甩飞风险）→ 回等待状态重新来
                                if((swing_timer > SWING_TIMEOUT_MAX) ||
                                   ((enc_rel - enc_rel_prev) >  $signed({16'b0, SWING_VEL_MAX})) ||
                                   ((enc_rel - enc_rel_prev) < -$signed({16'b0, SWING_VEL_MAX}))) begin
                                    state         <= S_WAIT;
                                    wait_timer    <= 32'd0;
                                    wait_cnt      <= 16'd0;
                                    swing_timer   <= 32'd0;
                                    success_count <= 16'd0;
                                    ang_err_prev  <= ang_err;
                                    enc_rel_prev  <= enc_rel;
                                    pwm_cmd       <= 16'sd0;
                                    err_flag      <= 1'b1;
                                    err_timer     <= 32'd0;
                                end
                            end

                            // ---------- 保持直立 + 位置环 ----------
                            S_BALANCE: begin
                                // 位置环每 25ms 更新一次，两次更新之间保持 pos_pwm
                                pwm_cmd <= clamp_pwm(bal_pwm_w - $signed(pos_pwm));
                            end

                            default: begin
                                pwm_cmd <= 16'sd0;
                            end
                        endcase
                    end

                    // ---- SW2：本模块不再使用（在 top.v 用作曲线选择）----
                end

                // ================= 25ms 位置环 =================
                if(tick_25ms) begin
                    ang_vel_pd     <= ang_err - ang_err_prev25;
                    ang_err_prev25 <= ang_err;

                    if(state == S_BALANCE) begin
                        pos_vel    <= pos_vel_next;
                        pos_filt   <= pos_filt_next;
                        // 移植 IP570 的思路：刚进平衡的前 500ms 只靠角度环，
                        // 等摆杆稳下来再接入位置环，避免一上来就把臂拽走。
                        if(bal_timer >= POS_ENABLE_TICKS)
                            pos_pwm <= clamp_pos(pos_pwm_w);
                        else
                            pos_pwm <= 16'sd0;
                    end
                end

                // ================= 点位模式目标按键 =================
                // 选了曲线时 SW3/SW4 由 traj_gen 用作幅度调节，这里不再改 ±30° 目标
                if(state == S_BALANCE && !traj_sel_act) begin
                    if(key_inc && (pos_cmd_d10 < POS_D10_MAX))
                        pos_cmd_d10 <= pos_cmd_d10 + STEP_D10;
                    if(key_dec && (pos_cmd_d10 > -POS_D10_MAX))
                        pos_cmd_d10 <= pos_cmd_d10 - STEP_D10;
                end
            end

            // ---------------- 错误指示：亮 1s ----------------
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

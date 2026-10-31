// ============================================================================
// top.v  —  J280 旋转倒立摆 FPGA 控制顶层
//
// 资源分配（与 FPGA_PID_Pendulum 一致）：
//   - 摆臂电机：接在 MOTOR_SEL 指定的那一路（A 或 B），由 pwm_cmd 驱动
//   - 手轮电机：接在另一路，只读编码器、不驱动（摇杆模式使用）
//   - UART 返回 {编码器1, 编码器2}
//
// 摆臂电机必须和它的编码器插在同一个 XH 接口上，所以驱动通道与反馈编码器
// 由同一个参数 USE_ENC2 对应：驱动通道由 MOTOR_SEL 选。
// ============================================================================
module top #(
    // 只驱动其中一个电机通道，另一路恒为 0（接在另一路的电机完全自由）
    //   0 = 驱动 A 通道：PWMA/AIN1/AIN2 (Y18/Y17/Y19)
    //   1 = 驱动 B 通道：PWMB/BIN1/BIN2 (V14/V15/AA17)
    // 实测：MOTOR_SEL = 1 时摆臂正常 → B 通道 = XH1（摆臂），A 通道 = XH2（手轮）
    parameter integer MOTOR_SEL = 1,
    // 摆臂反馈编码器：0 = enc1（串口第 4~5 字节，对应 XH1）
    //                 1 = enc2（串口第 6~7 字节，对应 XH2）
    // 手轮编码器自动取另一路。当前摆臂在 XH1 → 保持 0。
    parameter integer USE_ENC2 = 0   // 1 = B 通道(enc2/XH2)；0 = A 通道(enc1/XH1)
)(
    input  wire        clk_50m,
    input  wire        rst_n,

    input  wire [3:0]  key_sw,        // SW1~SW4，低有效
    input  wire        cal_sw,        // B3_SW1，低有效
    input  wire        dir_sw,        // B3_SW2，低有效

    output wire        uart_tx,
    input  wire        uart_rx,

    // 3PA1030 ADC
    output wire        adc_clk,
    output wire        adc_oe_n,
    output wire        adc_stby_n,
    input  wire [9:0]  adc_data_in,

    // 编码器 1（A 通道 / XH1 接口）
    input  wire        enc1_a,
    input  wire        enc1_b,
    // 编码器 2（B 通道 / XH2 接口）
    input  wire        enc2_a,
    input  wire        enc2_b,

    // TB6612 A 通道（XH1 接口）
    output wire        ain1,
    output wire        ain2,
    output wire        pwma,
    // TB6612 B 通道（XH2 接口）
    output wire        bin1,
    output wire        bin2,
    output wire        pwmb,

    output wire [3:0]  led,

    // ---- MP34DT01 PDM 数字麦克风（VDD = 3.3V，LR 接 GND）----
    output wire        mic_clk,       // 3.125MHz PDM 时钟
    input  wire        mic_data       // PDM 数据
);

    // ========================================================================
    // 控制节拍：5ms / 25ms / 20ms
    // ========================================================================
    reg [17:0] cnt_5ms;
    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n)                       cnt_5ms <= 18'd0;
        else if(cnt_5ms == 18'd249_999)  cnt_5ms <= 18'd0;
        else                             cnt_5ms <= cnt_5ms + 18'd1;
    wire tick_5ms = (cnt_5ms == 18'd249_999);

    reg [2:0] div25;
    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n)                div25 <= 3'd0;
        else if(tick_5ms)         div25 <= (div25 == 3'd4) ? 3'd0 : div25 + 3'd1;
    wire tick_25ms = tick_5ms && (div25 == 3'd4);

    reg [19:0] cnt_20ms;
    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n)                       cnt_20ms <= 20'd0;
        else if(cnt_20ms == 20'd999_999) cnt_20ms <= 20'd0;
        else                             cnt_20ms <= cnt_20ms + 20'd1;
    wire tick_20ms = (cnt_20ms == 20'd999_999);

    // ========================================================================
    // 串口命令信号的提前声明
    //   软复位生成需要 param_we/param_addr/param_data，而它们来自 uart_cmd，
    //   故先声明，实例化放在后面。
    // ========================================================================
    wire [7:0]  param_addr;
    wire [15:0] param_data;
    wire        param_we;

    // 命令解码（地址 0x10 = CMD）
    //   1 = 启动（起摆）      2 = 停止
    //   3 = 软复位            4 = 确认执行当前模式（arm，等价板上的 SW1）
    //   5 = 取消当前模式（disarm，等价平衡中按 SW2）
    wire cmd_start = param_we && (param_addr == 8'h10) && (param_data == 16'd1);
    wire cmd_stop  = param_we && (param_addr == 8'h10) && (param_data == 16'd2);
    wire cmd_reset = param_we && (param_addr == 8'h10) && (param_data == 16'd3);
    wire cmd_arm   = param_we && (param_addr == 8'h10) && (param_data == 16'd4);
    wire cmd_disarm= param_we && (param_addr == 8'h10) && (param_data == 16'd5);

    // ========================================================================
    // 软复位生成（串口 0x10 = 3）
    //   拉低内部复位 131072 个时钟（2.6ms），只复位内部逻辑，不影响 FPGA 配置。
    //   rst_hold 自身用原始 rst_n，否则复位期间无法退出。
    //   所有子模块用 rst_n_all；uart_rx / uart_cmd 仍用 rst_n，
    //   保证软复位后仍能继续接收命令（否则会把自己锁死）。
    // ========================================================================
    reg [16:0] rst_hold;
    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n)                 rst_hold <= 17'd0;
        else if(cmd_reset)         rst_hold <= 17'h1FFFF;
        else if(rst_hold != 17'd0) rst_hold <= rst_hold - 17'd1;

    wire rst_n_all = rst_n & (rst_hold == 17'd0);

    // ========================================================================
    // 按键消抖
    // ========================================================================
    wire [3:0] key_pulse;
    genvar i;
    generate
        for(i = 0; i < 4; i = i + 1) begin : g_key
            key_filter #(
                .CNT_MAX(20'd1_000_000)     // 20ms @50MHz
            ) u_key (
                .sys_clk  (clk_50m),
                .sys_rst_n(rst_n),
                .key_in   (key_sw[i]),
                .key_flag (key_pulse[i])
            );
        end
    endgenerate

    // ========================================================================
    // ADC 角度采集
    // ========================================================================
    wire [11:0] angle;
    adc u_adc (
        .clk_50m    (clk_50m),
        .rst_n      (rst_n_all),
        .tick_5ms   (tick_5ms),      // 采样相位锁定（250 样本 / 5ms = 50kSPS）
        .adc_clk    (adc_clk),
        .adc_oe_n   (adc_oe_n),
        .adc_stby_n (adc_stby_n),
        .adc_data_in(adc_data_in),
        .theta_out  (angle)
    );

    // ========================================================================
    // 两路编码器（A 通道 / B 通道）
    // ========================================================================
    wire [15:0] enc1, enc2;

    encoder u_enc1 (
        .clk_50m(clk_50m),
        .rst_n  (rst_n),
        .enc_a  (enc1_a),
        .enc_b  (enc1_b),
        .pos    (enc1)
    );

    encoder u_enc2 (
        .clk_50m(clk_50m),
        .rst_n  (rst_n),
        .enc_a  (enc2_a),
        .enc_b  (enc2_b),
        .pos    (enc2)
    );

    // 控制核心使用哪一路编码器
    wire [15:0] enc_ctrl = USE_ENC2 ? enc2 : enc1;

    // 手轮编码器：自动用「不是摆臂的那一路」
    wire [15:0] enc_wheel = USE_ENC2 ? enc1 : enc2;

    // ========================================================================
    // 控制核心
    // ========================================================================
    wire signed [15:0] pwm_cmd;
    wire               led_run;
    wire               led_manual;
    wire               led_err;
    wire               mic_led;        // 麦克风幅值超阈值
    wire [15:0]        mic_level;      // 单频幅值，随遥测上报
    wire signed [15:0] mic_smp;        // 麦克风时域采样（15.625 kSPS，已去直流）
    wire               mic_vld;        // 采样有效脉冲
    wire [11:0]        angle_mid_dbg;
    wire [11:0]        angle_used_dbg;
    wire signed [15:0] pos_target_dbg;
    wire signed [15:0] pos_pwm_dbg;
    wire signed [15:0] ab_v_dbg;

    // ---- 串口接收：参数在线修改 ----
    wire [7:0]  rx_byte;
    wire        rx_byte_valid;
    wire [7:0]  cmd_err_cnt;
    wire [7:0]  cmd_ok_cnt;

    uart_rx #(
        .CLK_FREQ(50_000_000),
        .UART_BPS(115200)
    ) u_rx (
        .clk      (clk_50m),
        .rst_n    (rst_n),          // 用原始复位：软复位后仍需能收命令
        .rx       (uart_rx),
        .rx_data  (rx_byte),
        .rx_valid (rx_byte_valid)
    );

    uart_cmd u_cmd (
        .clk          (clk_50m),
        .rst_n        (rst_n),      // 同上，不能被软复位掐断
        .rx_byte      (rx_byte),
        .rx_byte_valid(rx_byte_valid),
        .param_addr   (param_addr),
        .param_data   (param_data),
        .param_we     (param_we),
        .err_cnt      (cmd_err_cnt),
        .ok_cnt       (cmd_ok_cnt)
    );

    // 启动/停止：串口命令与 SW1 等效（OR 后送 key_start）
    //
    // ⚠ SW1 现在有【两种角色】，必须按状态分流：
    //     未进入平衡（IDLE / SWING）：SW1 = 起摆 / 取消起摆  -> 送 u_ctrl 的 key_start
    //     已进入平衡（BALANCE）    ：SW1 = 确认执行所选模式  -> 不能送 u_ctrl，
    //                                否则会被当成「停止」把平衡环关掉
    //   提前声明 state_dbg（下方 pendulum_ctrl 实例驱动），否则这里会生成 1-bit 隐式线网。
    wire [3:0] state_dbg;

    wire in_bal   = (state_dbg == 4'd2);          // 平衡态
    wire in_swing = (state_dbg == 4'd1);          // 起摆中

    wire sw1_press    = key_pulse[0];
    wire sw1_confirm  = sw1_press &  in_bal;                    // 平衡中：确认模式
    wire key_start_eff = (sw1_press & ~in_bal) | cmd_start;     // 其他状态：起摆/取消

    // ========================================================================
    // 角度标度换算（改一个数字即可整体重标定）
    // ------------------------------------------------------------------------
    //   【实测 2026-10-30】摆杆从最上（竖直）到最下（下垂），180° 只跨越
    //     534 个 ADC 计数。
    //   旧标度是 3003 计数 / 180°（下垂 224 → 竖直 3227）。
    //
    //     计数/度 :  534/180 = 2.967     （旧 16.68）
    //     度/计数 :  180/534 = 0.337     （旧 0.060）
    //     ADC 1 LSB(4 计数) = 1.348°     （旧 0.240°）
    //
    //   ★ 两类参数必须朝【相反方向】缩放：
    //
    //     A) 增益型（输入是计数，输出是 PWM）：BAL_KP / BAL_KD / *_CATCH
    //        要维持同样的「每度多少 PWM」，计数变少 → 增益必须变大
    //        new = old × (3003/534) ≈ old × 5.62
    //
    //     B) 阈值型（拿计数做比较）：ANGLE_LIMIT / ANGLE_DZ / CATCH_* 等
    //        同样的角度对应的计数变少 → 阈值必须变小
    //        new = old × (534/3003) ≈ old × 0.178
    //
    //     C) 编码器域参数（位置环、甩杆、编码器保护）完全不受影响，保持不变。
    //
    //   ⚠【安全隐患】若不改阈值：ANGLE_LIMIT=800 在新标度下相当于 270°，
    //     摆杆已倒下也绝不触发超限保护。因此必须同步缩放。
    // ========================================================================
    localparam integer SPAN_CNT_180 = 534;    // 新标度：180° 对应计数
    localparam integer SPAN_OLD_180 = 3003;   // 旧标度：180° 对应计数（换算基准）

    // ---- A) 增益型：[2026-10-30 现场实测确定] ----
    //   ⚠ 纯按标度换算得到的是 KP=197 / KD=1012，但实测发现：
    //     速度估计的【整数输出量化】= 1 计数/拍 = 67°/s。
    //     KD=1012 时，D 项一拍一个台阶就是 1012 PWM（满量程 20%）。
    //     控制量变成阶梯式跳动 → 适得其反。
    //   所以 KD 取 500（约等效值的一半）。相位滞后的问题交给 α-β 带宽
    //   （AB_ALPHA/BETA = 153/65）解决，而不是堆 D 增益。
    //
    //   BAL_KP=120 -> 356 PWM/度，饱和于 ±14.5°（5175/356）
    //   BAL_KD=500 -> 7.4 PWM/(°/s)
    localparam integer G_BAL_KP       = 120;   // 现场实测值（2026-10-30）
    localparam integer G_BAL_KD       = 525;   // 现场实测值；受速度量化限制不要超过 800
    localparam integer G_BAL_KP_CATCH = 102;   // 捕获期 = 主值 × 0.85
    localparam integer G_BAL_KD_CATCH = 436;   // 捕获期 = 主值 × 0.83

    // ---- B) 阈值型：× (新/旧) ----
    localparam integer T_ANGLE_LIMIT   = 800 * SPAN_CNT_180 / SPAN_OLD_180; // 142 = ±48°
    localparam integer T_ANGLE_DZ      = 32  * SPAN_CNT_180 / SPAN_OLD_180; // 5
    localparam integer T_CATCH_WINDOW  = 350 * SPAN_CNT_180 / SPAN_OLD_180; // 62 = ±21°
    localparam integer T_CATCH_VEL     = 150 * SPAN_CNT_180 / SPAN_OLD_180; // 26
    localparam integer T_MANUAL_WINDOW = 700 * SPAN_CNT_180 / SPAN_OLD_180; // 124 = ±42°
    localparam integer T_QUARTER       = 534 / 2;                            // 267 = 90°
    localparam integer T_GIVEUP_WIN    = 500 * SPAN_CNT_180 / SPAN_OLD_180; // 88  = ±30°

    // ---- 自动起摆方式（只改这一处，关联参数自动跟随）----
    //   0 = 能量泵（连续换向，可反复尝试；起摆慢但不需要初始位置）
    //   1 = 甩杆（一次性位置 PD，快；但只试一次，失败就 3s 超时）
    //   ⚠ 两种模式的超时常量差很多：甩杆 3s 够，泵 3s 根本来不及（应该 15s）。
    //     以前这个得手动改两处，现在由 SWING_MODE_SEL 自动决定。
    localparam integer SWING_MODE_SEL  = 0;            // <<< 当前：能量泵
    localparam signed [31:0] SWING_TMO_SEL =
        (SWING_MODE_SEL == 0) ? 32'sd3000 : 32'sd600;   // 15s / 3s

    // ========================================================================
    // 按键状态机 + 运动模式
    // ------------------------------------------------------------------------
    //   【完整操作流程】
    //     1) 停止状态   按 SW1        -> 起摆（能量泵）
    //     2) 起摆中     按 SW1        -> 取消起摆
    //     3) 已平衡     按 SW2        -> 轮换模式（仅预选，不动）
    //     4) 已平衡     按 SW1        -> 确认执行当前模式
    //     5) 模式运行中 按 SW2        -> 停掉该模式，回到纯稳摆（保持平衡）
    //     6) 任何时候   按 SW5        -> 硬件复位（rst_n）
    //
    //   即：SW1 = 起摆 / 确认；SW2 = 选模式（未装甲时）/ 停模式（已装甲时）。
    //
    //   【SW3 / SW4】
    //     模式 0（纯稳摆）：调目标偏移 sel_toff，按 SW1 确认后生效
    //     模式 1~4（曲线）：调幅值 sel_amp，实时生效（运行中也能改）
    //     模式 5（摇杆）  ：不用，摆臂目标由手轮（XH2 编码器）1:1 跟随
    //
    //   【LED1~3 = 模式号 + 1】
    //     0 纯稳摆=001  1 正弦=010  2 三角=011  3 线性=100  4 四四拍往复=101
    //     5 摇杆=111（全亮）
    //     慢闪 = 未执行（等你按 SW1 确认）   常亮 = 该模式正在执行
    //     LED4 = 错误闪烁（次数 = err_code）
    // ========================================================================
    localparam [3:0] MODE_MAX = 4'd5;               // 0~5（5 = 摇杆/手轮）
    localparam signed [15:0] AMP_STEP = 16'sd87;    // 30° = 1040×30/360 ≈ 87 计数
    localparam signed [15:0] BEAT_STEP = 16'sd29;   // 四四拍幅度步长 ≈ 10°
    localparam signed [15:0] AMP_MAX  = 16'sd520;   // ±180° 上限
    localparam [15:0]        PERIOD_DEF = 16'd800;  // 4 秒

    reg  [3:0]        sel_mode;      // 预选模式
    reg  signed [15:0] sel_amp;      // 预选幅值（曲线模式 1~3）
    reg  signed [15:0] sel_beat_amp; // 四四拍单拍峰值（模式 4，独立参数，默认 ≈15°）
    reg  [15:0]       sel_period;    // 预选周期
    reg  signed [15:0] sel_toff;     // 预选目标偏移（模式0）
    reg  signed [15:0] act_toff;     // 已确认的目标偏移
    reg               mode_armed;    // 1 = 所选模式正在执行

    // SW2 轮换用：5 之后回到 0
    wire [3:0] mode_nxt = (sel_mode == MODE_MAX) ? 4'd0 : (sel_mode + 4'd1);
    // LED 显示用：模式号 + 1（0→1 … 4→5），**不回绕**
    //   ⚠ 早期版本这里错用了 mode_nxt，导致 sel_mode=4（四四拍往复）时
    //     mode_nxt=0 -> LED 全灭，与「101」的设计不符，也不易辨认。
    wire [3:0] mode_disp = sel_mode + 4'd1;

    // 摇杆（手轮）模式：SW2 选到 5，再按 SW1 确认后生效
    wire wheel_sel = (sel_mode == 4'd5);

    wire signed [17:0] toff_up = $signed(sel_toff) + AMP_STEP;
    wire signed [17:0] toff_dn = $signed(sel_toff) - AMP_STEP;
    wire signed [17:0] amp_up  = $signed(sel_amp)  + AMP_STEP;
    wire signed [17:0] amp_dn  = $signed(sel_amp)  - AMP_STEP;
    wire signed [17:0] bamp_up = $signed(sel_beat_amp) + BEAT_STEP;
    wire signed [17:0] bamp_dn = $signed(sel_beat_amp) - BEAT_STEP;

    // ---- 预选值（SW2 轮换模式 / SW3,SW4 调参数 / 串口下发）----
    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n) begin
            sel_mode   <= 4'd0;       // 上电默认：纯稳摆
            sel_amp    <= 16'sd87;    // 30°
            sel_beat_amp <= 16'sd44;  // 四四拍强拍峰值 ≈15°（旧版是 87 的一半）
            sel_toff   <= 16'sd0;
            sel_period <= PERIOD_DEF;
        end else begin
            // 串口优先
            if(param_we) begin
                case(param_addr)
                    8'h1D: sel_amp    <= param_data[15:0];
                    8'h1E: sel_period <= (param_data < 16'd100) ? 16'd100 : param_data;
                    8'h1F: sel_mode   <= (param_data > 16'd5) ? MODE_MAX : param_data[3:0];
                    8'h20: sel_beat_amp <= param_data[15:0];  // 四四拍单拍峰值（独立参数）
                    8'h0F: sel_toff   <= 16'sd0;   // 上位机定位时清掉预设偏移
                    default: ;
                endcase
            end

            // SW2：只有「已平衡且模式未执行」时才轮换（运行中按 SW2 是停模式，见下方）
            if(key_pulse[1] && in_bal && !mode_armed)
                sel_mode <= mode_nxt;

            // SW3 / SW4
            //   模式 0（纯稳摆）：调目标偏移 sel_toff
            //   模式 1~3（曲线） ：调幅值 sel_amp
            //   模式 4（四四拍） ：调【独立】拍幅值 sel_beat_amp（步长 ≈10°）
            //   模式 5（摇杆）   ：摆臂目标完全由手轮决定，SW3/SW4 不参与
            if(!param_we) begin
                if(sel_mode == 4'd0) begin
                    if(key_pulse[2]) begin
                        if(toff_up > $signed({2'b0, AMP_MAX})) sel_toff <=  AMP_MAX;
                        else                                   sel_toff <= toff_up[15:0];
                    end
                    if(key_pulse[3]) begin
                        if(toff_dn < -$signed({2'b0, AMP_MAX})) sel_toff <= -AMP_MAX;
                        else                                    sel_toff <= toff_dn[15:0];
                    end
                end else if(sel_mode == 4'd4) begin
                    if(key_pulse[2]) begin
                        if(bamp_up > $signed({2'b0, AMP_MAX})) sel_beat_amp <= AMP_MAX;
                        else                                   sel_beat_amp <= bamp_up[15:0];
                    end
                    if(key_pulse[3]) begin
                        if(bamp_dn < 18'sd0) sel_beat_amp <= 16'sd0;
                        else                 sel_beat_amp <= bamp_dn[15:0];
                    end
                end else if(sel_mode != 4'd5) begin
                    if(key_pulse[2]) begin
                        if(amp_up > $signed({2'b0, AMP_MAX})) sel_amp <= AMP_MAX;
                        else                                  sel_amp <= amp_up[15:0];
                    end
                    if(key_pulse[3]) begin
                        if(amp_dn < 18'sd0) sel_amp <= 16'sd0;
                        else                sel_amp <= amp_dn[15:0];
                    end
                end
            end
        end

    // ---- 装甲（arm）状态 ----
    //   离开平衡态一律解除，且清掉已确认的目标偏移，
    //   保证每次起摆都从「纯稳摆、不偏移」开始，必须重新确认。
    //   确认来源：板上 SW1，或串口 CMD=4（网页的「手轮模式」按钮用它）；
    //   取消来源：板上 SW2（运行中），或串口 CMD=5。
    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n) begin
            mode_armed <= 1'b0;
            act_toff   <= 16'sd0;
        end else if(!in_bal) begin
            mode_armed <= 1'b0;
            act_toff   <= 16'sd0;
        end else if(sw1_confirm || cmd_arm) begin
            mode_armed <= 1'b1;              // 确认执行
            act_toff   <= sel_toff;          // 目标偏移此刻生效
        end else if((key_pulse[1] && mode_armed) || cmd_disarm) begin
            mode_armed <= 1'b0;              // 取消模式
            act_toff   <= 16'sd0;
        end

    // ---- 模式灯时序：未执行慢闪（1.5Hz），执行中常亮 ----
    reg [24:0] blink_cnt;
    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n)          blink_cnt <= 25'd0;
        else if(mode_armed) blink_cnt <= 25'd0;
        else                blink_cnt <= blink_cnt + 25'd1;

    wire mode_show = mode_armed ? 1'b1 : blink_cnt[24];   // 2^24@50MHz = 0.335s

    pendulum_ctrl #(
        //
        //   模式 A：STM32_MODE=0, BAL_STM32=0  → 本工程调优版（默认推荐）
        //   模式 B：STM32_MODE=0, BAL_STM32=1  → 平衡完全照抄 STM32，起摆用本工程甩杆
        //   模式 C：STM32_MODE=1, BAL_STM32=x  → 起摆+平衡全部照抄 STM32
        //
        // BAL_STM32 = 1 时强制生效（覆盖 top.v 中对应参数）：
        //   · 位置低通 1/5（0.2/0.8），非移位
        //   · 平衡参考点 = 甩杆终点 FLING_STEP（Position_Zero=step1_target，过冲补偿）
        //   · 位置环无积分（POS_KI 强制 0）；位置环独立限幅仍保留（±PWM_MAX/2）
        //   · 关闭 D 项低通、关闭小增益区
        //   · 手动/自动判定窗口 ±500
        //   · 编码器突变阈值 40（已从 STM32 的 15 放宽，见 ENC_JUMP_MAX）
        //   · 上电直接使用固定平衡点 ZHONGZHI_DEF（已取消自动标定）
        //   · PWM 限幅比例对齐 6900/7200 → 5175/5400
        //
        // STM32_MODE = 1 额外强制（起摆部分）：
        //   · 甩杆单相（无回杆/减速）
        //   · 起摆超时 600ms
        //
        // 刷新率三种模式一致：控制 5ms / 位置环 25ms / 捕获后前 550ms 位置环 5ms。
        // =====================================================================
        .STM32_MODE       (0),   // 起摆是否照抄 STM32
        .BAL_STM32        (1),   // 平衡是否照抄 STM32  <<< 当前启用

        // ---- 角度零点 ----
        //   [2026-10-30 现场标定] 摆杆竖直时 ADC 实测 = 768。
        //   换标度后旧值 3227 已完全失效（读数不超过 4092，但实际只在 235~768 区间）。
        //   需要微调时：网页「⭐ 当前设为平衡点」，或按住 B3_SW1。
        .AUTO_CAL         (0),         // 0 = 取消上电自动标定
        .CAL_TICKS        (16'd64),    // 64 × 5ms = 320ms 采样窗口（仅 AUTO_CAL=1 时生效）
        .CAL_SHIFT        (6),         // 必须 = log2(CAL_TICKS)
        .CAL_DOWN_TYP     (12'd235),   // 下垂实测值（仅 AUTO_CAL=1 时用于有效性判定）
        .CAL_TOL          (12'd36),    // 下垂读数允许偏差（36 计数 ≈ 12°）
        .CAL_SPAN         (12'd267),   // 下垂→竖直跨度 = 534/2（90°）
        .ZHONGZHI_DEF     (12'd768),   // ✅ 实测竖直值（2026-10-30）

        // ---- 竖直平衡位置 ----
        //   BAL_STM32=1 时使用 ZHONGZHI_DEF；否则用 ANGLE_MID_DEFAULT。
        //   两者都是「上电默认值」，运行时以串口 0x0E / B3_SW1 捕获值为准。
        .ANGLE_MID_DEFAULT(12'd768),    // ✅ 同上
        // 角度环极性：已实测确认必须取反。
        // 「摆臂往某方向加速 -> 摆杆相对臂往反方向倾」是旋转倒立摆的固有特性。
        .ANGLE_INV        (1),
        .ENC_INV          (0),
        .SWING_INV        (1),   // 与 ANGLE_INV 同步取反，起摆（能量泵）方向保持不变

        .MANUAL_WINDOW    (T_MANUAL_WINDOW[15:0]),   // 124 计数 ≈ ±42°
        // 角度超限保护：ANGLE_LIMIT_EN=1 生效（安全底线），0 = 临时禁用。
        //   ⚠ 新标度下必须用 T_ANGLE_LIMIT（=142），否则 800 计数 = 270° 永不触发。
        .ANGLE_LIMIT_EN   (1),
        .ANGLE_LIMIT      (T_ANGLE_LIMIT[15:0]),
        // 连续超限确认拍数（×5ms）。20 = 100ms：真倒下仍会停，单拍毛刺被滤掉。
        .ANGLE_LIMIT_TICKS(16'd20),
        .PWM_MAX          (16'd5300),

        .POS_STEP         (32'sd2080),
        .RAMP_STEP        (32'sd1),

        // ---- 平衡环 PID（基于 2026-10-30 实测数据重定）----
        //   实测：静止时角度 σ 仅 0.4 计数（分辨率充足）；
        //         起摇后摆动周期约 240ms（4Hz），从 9° 扰动过冲到 76° 然后失控。
        //   原因：α-β 速度滞后 66° → D 项在该频率下变成负阻尼。
        //   对策：① 提高 α-β 带宽（已改 153/65）  ② 适度加 P  ③ D 受量化限制取折中
        //
        //   KP=180 -> 534 PWM/度，饱和于 ±9.7°（超出后满功率，正是想要的）
        //   KD=500 -> 7.4 PWM/(度/秒)，比旧标度等效值(15)低一半，因为速度量化
        .BAL_KP           (G_BAL_KP[15:0]),    // 120
        .BAL_KD           (G_BAL_KD[15:0]),    // 525
        // 小增益区：BAL_MODE=1 时内部强制关闭，这里的值不生效。
        //   保留仅为将来若关闭 BAL_STM32 时与标度一致。
        .ANGLE_DZ         (T_ANGLE_DZ[15:0]),  // 5
        .ANGLE_DZ_SHIFT   (0),
        .ANGLE_DZ_LOG2    (3),
        // D 项低通：BAL_MODE 下强制 0（α-β 滤波器已接管降噪）
        .ANG_D_LPF_SHIFT  (0),
        .POS_KP           (16'd25),    // STM32: 25（实测线性：1 计数误差 ≈ 1.6 PWM）
        // 【位置环 D 项：[2026-10-30 现场实测] 600 -> 450】
        //   实际增益 = POS_KD × POS_D_SCALE(5) × v，v 为计数/5ms
        //     POS_KD=450 -> 2250 PWM per (计数/5ms)
        //     clamp_pos 限幅 2587 -> v 到 1.15 才饱和（比 600 时的 0.86 更宽松）
        //   原 600 时摆臂速度稍大就会把位置环输出打满，削弱角度环权限。
        .POS_KD           (16'd450),
        .POS_KI           (16'd0),     // 实测 0（开积分会引起粘滑极限环）
        //   积分固定按 25ms 累加（不随位置环提速变），公式：
        //     pos_int += pos_filt × POS_KI / 2^(POS_I_SHIFT+4)
        //   即误差 8 计数 → 0.5 PWM/25ms（20 PWM/s）；误差 100 计数 → 6.25 PWM/25ms。
        //   爬满上限 ±PWM_MAX/8（≈647）需 32s（误差 100 时）。
        //
        //   【为何从 32 降到 16】积分在跟摆臂静摩擦"较劲"：
        //     累积到超过静摩擦 → 摆臂突然窜出 → 积分仍同向推 → 走更远 → 误差反向 →
        //     积分反向重新累积 → 再突破。形成「收敛→突然偏离→再收敛」的粘滑极限环。
        //   现在配合「只在摆臂静止时积分」+ 窄抗饱和带(40) + 低上限(PWM_MAX/8)，
        //   把累积速度也降到一半，进一步减小突破瞬间的过冲。
        .POS_I_BAND       (16'd40),    // 原 300（=104°，几乎永不冻结）
        .POS_I_VEL_BAND   (16'd2),     // 静止判定：|Δenc| < 2 计数/5ms 才积分
        // POS_FILT_SHIFT 在 STM32_MODE=1 时内部改用 1/5 低通，此值不生效（见文件末尾）

        // ===== 自动起摆方式 =====
        // 0 = 能量泵（连续换向，按 sign(θ̇·cosθ) 实时换向，可反复尝试）
        // 1 = 甩杆（移植 STM32 Auto_run：一次性位置 PD 甩杆）
        .SWING_MODE       (SWING_MODE_SEL),   // <<< 顶部 SWING_MODE_SEL 统一控制

        // ---- 甩杆（SWING_MODE=1）----
        // STM32: step1_target 10000 -> 10341，甩 341 计数（约 1/3 圈），
        //        驱动用 Position_KP_1=90 / Position_KD_1=180。
        .FLING_STEP       (32'sd341),
        .FLING_KP         (16'd90),
        .FLING_KD         (16'd180),
        .FLING_START_WIN  (16'd20000), // 甩杆已改为【相对起始位置】，绝对位置不再重要
                                       // → 这个检查基本失效（仅防极端异常值）
                                       //   若需严格限制起始位置，改小即可（如 100）
        // ---- 甩杆后的回杆 + 减速（三相）----
        // 相位 0: 甩向 FLING_STEP (341)      快速前冲
        // 相位 1: 回向 FLING_STEP2 (261)     反向拉回，给摆杆补冲量
        // 相位 2: 减速到 FLING_STEP3 (231)   同向再走一小段，缓停避免硬刹车
        // BACK_STEP=0 退化为单向甩杆；DECEL_STEP=0 退化为两相。
        .FLING_BACK_STEP  (32'sd80),
        .FLING_DECEL_STEP (32'sd30),
        .FLING_ARRIVE_WIN (16'd15),    // 到位判定窗口
        .FLING_PHASE_TICKS(16'd60),    // 单相超时 300ms，防卡住
        // ---- 能量泵（SWING_MODE=0 时生效，当前启用）----
        //   律：u = SWING_INV ? -sign(θ̇·cosθ)·GAIN : +sign(θ̇·cosθ)·GAIN
        //   叠加摆臂速度阻尼 -SWING_DAMP×Δenc，防止摆臂越转越快。
        //
        //   【为何原来慢】三个原因：
        //     1) GAIN=1200 只有 PWM_MAX(5175) 的 23%，4 倍力矩余量没用上。
        //        能量泵每半周期传递的能量正比于力矩幅值。
        //     2) 比例阻尼正比于摆臂速度，而能量传递效率最高的时刻正是摆臂
        //        速度峰值 -> 阻尼也达峰，正好把泵力吃掉。
        //     3) 摆臂速度被钉在 GAIN/DAMP = 1200/20 = 60 计数/5ms。
        //
        //   【现在可在线调】GAIN / DAMP / QUARTER 都是运行时可写参数
        //   （地址 0x0A/0x0B/0x0C），配合网页滑块可以现场快速试。
        //   另外加了硬限速 SWING_VMAX=140 计数/5ms（≈27 圈/s）兜底，
        //   即使 DAMP 调到 0 也不会飞车。
        //
        //   调参方向：
        //     · 摆幅增大但上不去 -> GAIN 加到 3000~4000
        //     · 摆臂明显发热/噪音大 -> DAMP 加到 30~40
        //     · 摆臂完全不动/越摆越小 -> SWING_INV 取反（0↔1）
        .SWING_GAIN       (16'd1200),  // 实测值（原规划 2400 未验证）
        .SWING_DAMP       (16'd20),    // 实测值（原规划 30 未验证）
        .QUARTER          (T_QUARTER), // 267 = 90°（新标度）
                                       // ⚠ 旧值 1000 在 534 计数/180° 下永远触发不到，
                                       //   泵律会退化成纯阻尼（只刹车不泵能）。

        // ---- 捕获 ----
        .CATCH_WINDOW     (T_CATCH_WINDOW[15:0]),  // 62 计数 ≈ ±21°
        .CATCH_VEL        (T_CATCH_VEL[15:0]),     // 26 计数/拍
        .SUCCESS_NEED     (16'd10),    // STM32: success_count>10
        // 起摆超时（×5ms）。
        //   SWING_MODE_SEL=1（甩杆）：一次性动作，3s 内没上到竖直就该放弃
        //   SWING_MODE_SEL=0（能量泵）：需要反复泵能，15s
        //   STM32_MODE=1 时内部还会强制 120（600ms）。
        //   ⚠ 这是「起摆超时自动停机」的唯一来源。
        .SWING_TIMEOUT_MAX(SWING_TMO_SEL),
        // 起摆放弃窗口：只有摆杆偏出 ±SWING_GIVEUP_WIN 才允许判超时。
        // STM32 原逻辑同此（try_count>120 且 不在 ZHONGZHI±500 内）。
        // 缺了这条，摆杆正在收拢时会被硬掐断 -> 表现为「平衡永远不接管」。
        .SWING_GIVEUP_WIN (T_GIVEUP_WIN[15:0]),    // 88 计数 ≈ ±30°

        // ---- 捕获期软参数（按新标度等比放大）----
        .BAL_KP_CATCH     (G_BAL_KP_CATCH[15:0]),  // 102
        .BAL_KD_CATCH     (G_BAL_KD_CATCH[15:0]),  // 436
        .POS_KP_CATCH     (16'd15),    // STM32: 15
        .POS_KD_CATCH     (16'd500),   // STM32: 500
        // ---- 位置环速率 ----
        // 【方案一】位置环常驻 5ms：消除 25ms 采样 + 1/5 低通带来的约 100° 相位滞后。
        //   原 25ms 时 10Hz 处：滤波滞后 82° + 零阶保持 18° = 100°，
        //   D 项只有 90° 超前，被吃光后变成 -10°（负阻尼）→ 自激出十几 Hz 抖动。
        //   改 5ms 后零阶保持降到 3.6°，D 项净超前回到 +4°。
        .POS_LOOP_FAST_ALWAYS (1),     // 1 = 位置环始终 5ms（推荐）
        .POS_FAST_TICKS   (16'd110),   // 仅在 POS_LOOP_FAST_ALWAYS=0 时生效
        // 软参数（*_CATCH）只覆盖刚站稳的 550ms，与位置环速率解耦
        .CATCH_SOFT_TICKS (16'd110),   // STM32: success_count<=109 -> 550ms

        // 编码器突变保护阈值（5ms 位移，1 圈 = 1040 计数）
        //   20 计数/5ms = 4000 计数/s = 3.85 圈/s（普通模式）
        //   BAL_MODE 内部固定用 40（= 7.7 圈/s），因为 STM32 的 15（2.88 圈/s）
        //   对轻质摆臂过严，会在正常平衡时误触发。
        .ENC_JUMP_MAX     (16'd20),
        // 编码器突变保护宽限期：进平衡后屏蔽 PROT_GRACE_TICKS×5ms 再启用。
        // 两种模式都生效；设 0 则退化为 STM32 的无宽限期行为。
        // 起摆时摆臂速度远超阈值（甩杆满功率 / 能量泵 60 计数/5ms），
        // 没有宽限期会在进平衡第一拍就误停机。
        .PROT_GRACE_TICKS (16'd60),    // 300ms
        .POS_ENABLE_TICKS (16'd0),     // 保留（当前逻辑改用 POS_FAST_TICKS）
        .POS_FILT_SHIFT   (3),

        // ---- 摇杆（手轮）参数：与 FPGA_PID_Pendulum 现场实测值一致 ----
        .WHEEL_RAMP_STEP  (32'sd2),    // 摇杆速度上限 2 counts/5ms ≈ 138°/s
        .WHEEL_LAG_MAX    (32'sd150),  // 给定最多超前实际位置 150 counts（≈52°）
        .WHEEL_POS_MAX    (32'sd20000),// 目标限幅 ±19 圈
        .WHEEL_DEADBAND   (32'sd2),    // 手轮增量死区
        .WHEEL_INV        (1)          // 摇杆方向：转手轮时摆臂反向就跟
    ) u_ctrl (
        .clk       (clk_50m),
        .rst_n     (rst_n_all),
        .tick_5ms  (tick_5ms),
        .tick_25ms (tick_25ms),

        .angle     (angle),
        .enc       (enc_ctrl),
        .enc_wheel (enc_wheel),     // 手轮编码器：与驱动通道互补

        .key_start (key_start_eff),
        // SW2 已改作「轮换运动模式」，不再驱动手动/自动。
        //   必须固定为 0（=自动起摆），否则一旦 mode_manual=1，
        //   摆杆不在竖直附近时 SW1 会直接拒绝启动（err_code=4）。
        .key_mode  (1'b0),
        .key_inc   (key_pulse[2]),
        .key_dec   (key_pulse[3]),
        .cal_sw    (cal_sw),
        .dir_inv   (dir_sw),
        .cmd_stop  (cmd_stop),

        // ---- 运动模式（由上方按键状态机产生）----
        .mode_sel   (sel_mode),
        .move_amp   (sel_amp),
        .move_beat_amp(sel_beat_amp),
        .move_period(sel_period),
        .move_toff  (act_toff),      // 已确认的目标偏移（非预选值）
        .move_armed (mode_armed),    // 1 = 模式正在执行
        // 摇杆模式：SW2 选到 5 且已按 SW1 确认才跟随手轮
        .wheel_en   (mode_armed && wheel_sel),

        .param_addr(param_addr),
        .param_data(param_data),
        .param_we  (param_we),

        .pwm_cmd      (pwm_cmd),
        .state_dbg    (state_dbg),
        .led_run      (led_run),
        .led_manual   (led_manual),
        .led_err      (led_err),
        .angle_mid_dbg(angle_mid_dbg),
        .angle_used_dbg(angle_used_dbg),
        .pos_target_dbg(pos_target_dbg),
        .pos_pwm_dbg   (pos_pwm_dbg),
        .ab_v_dbg      (ab_v_dbg)
    );

    // ========================================================================
    // MP34DT01 数字麦克风：单频 DFT 幅值超阈值 -> LED3
    // ------------------------------------------------------------------------
    //   参数可在线改（串口 9 字节写帧）:
    //     0x18 MIC_FREQ  目标频率 Hz   （默认 1000，范围 1~7800）
    //     0x19 MIC_TH    点亮阈值      （默认 500，与遥测 micLvl 同标度）
    //     0x1A MIC_HOLD  保持时间×5ms  （默认 20 = 100ms）
    //   麦克风模块用原始 rst_n：软复位后不必重新做 2.1s 直流收敛。
    // ========================================================================
    reg [15:0] mic_freq;
    reg [15:0] mic_th;
    reg [15:0] mic_hold;
    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n) begin
            mic_freq <= 16'd1000;
            mic_th   <= 16'd500;
            mic_hold <= 16'd20;
        end else if(param_we) begin
            case(param_addr)
                8'h18:   mic_freq <= param_data;
                8'h19:   mic_th   <= param_data;
                8'h1A:   mic_hold <= param_data;
                default: ;
            endcase
        end

    mic_fft #(
        .OSR  (200),            // 3.125MHz / 200 = 15.625kSPS
        .LOG2N(9)               // 512 点相干积分，32.8ms/块
    ) u_mic (
        .clk_50m   (clk_50m),
        .rst_n     (rst_n),
        .mic_clk   (mic_clk),
        .mic_data  (mic_data),
        .freq_hz   (mic_freq),
        .th        (mic_th),
        .hold_ticks(mic_hold),
        .tick_5ms  (tick_5ms),
        .level_led (mic_led),
        .level     (mic_level),
        .sample    (mic_smp),       // 时域采样流（串口每帧带 39 点）
        .sample_vld(mic_vld)
    );

    // ========================================================================
    // 电机驱动：只输出到 MOTOR_SEL 选中的那一路，另一路恒 0
    //   （与 FPGA_PID_Pendulum 一致：手轮电机接在另一路时完全自由，
    //    不能跟着摆臂一起被驱动）
    // ========================================================================
    wire [15:0] pwm_a = (MOTOR_SEL == 0) ? pwm_cmd : 16'sd0;
    wire [15:0] pwm_b = (MOTOR_SEL == 0) ? 16'sd0   : pwm_cmd;

    motor_drive #(
        .PWM_INV(0)
    ) u_motor_a (
        .clk    (clk_50m),
        .rst_n  (rst_n),
        .pwm_cmd(pwm_a),
        .ain1   (ain1),
        .ain2   (ain2),
        .pwm_out(pwma)
    );

    motor_drive #(
        .PWM_INV(0)
    ) u_motor_b (
        .clk    (clk_50m),
        .rst_n  (rst_n),
        .pwm_cmd(pwm_b),
        .ain1   (bin1),
        .ain2   (bin2),
        .pwm_out(pwmb)
    );

    // ========================================================================
    // 串口状态上报（AA 55 + 角度 + 编码器1 + 编码器2 + 0D 0A）
    // ========================================================================
    uart_status #(
        .CLK_FREQ(50_000_000),
        .UART_BPS(115200)      // 原 9600：14 字节 @9600 = 14.6ms，占 20ms 周期 73%
                               // 115200 下仅 1.2ms，也为参数回传留出带宽
    ) u_uart (
        .clk      (clk_50m),
        .rst_n    (rst_n_all),
        .tick_20ms(tick_20ms),
        .angle    (angle_used_dbg),   // 控制器实际采信的角度（平衡时已抑制尖峰）
        .angle_raw(angle),            // 原始 ADC 值，用于诊断尖峰
        .enc1     (enc1),
        .enc2     (enc2),
        .pwm_cmd  (pwm_cmd),
        .state_dbg(state_dbg),
        // 遥测帧第 11 字节（索引 10）高 4 位：bit5 = 手轮模式选中，bit4 = 模式已确认
        .mode_flag({wheel_sel, mode_armed}),
        // 遥测帧第 12 字节（索引 11）高 4 位：当前预选模式号（0~5）
        .mode_dbg(sel_mode[2:0]),
        .pos_target(pos_target_dbg),
        .pos_pwm  (pos_pwm_dbg),
        .mic_level(mic_level),
        .ab_v     (ab_v_dbg),
        .mic_smp  (mic_smp),
        .mic_vld  (mic_vld),
        .uart_tx  (uart_tx)
    );

    // ========================================================================
    // LED
    //   LED1(U18) LED2(U19) LED3(U17) = 模式编号二进制（模式号+1）
    //     0 纯稳摆=001   1 正弦=010   2 三角=011   3 线性=100   4 四四拍往复=101
    //     5 摇杆（手轮）=111（全亮）
    //     待机：1Hz 慢闪（提示可选择）；运行：常亮
    //   LED4(T17) = 错误闪烁，次数 = err_code
    //
    //   ⚠ 麦克风幅值指示（原 LED3）已取消，仍可通过串口遥测 micLvl 查看。
    // ========================================================================
    wire [2:0] mode_led = wheel_sel ? (3'b111 & {3{mode_show}})
                                    : (mode_disp[2:0] & {3{mode_show}});
    assign led = {led_err, mode_led};

endmodule

// ============================================================================
// top.v  —  J280 旋转倒立摆 FPGA 控制顶层
//
// 资源分配：
//   - 摆臂电机：接在 MOTOR_USE_B 指定的那一路（A 或 B），由 pwm_cmd 驱动
//   - 手轮电机：接在另一路，只读编码器、不驱动（摇杆模式使用）
//   - UART 返回 {编码器1, 编码器2}
//
// 摆臂电机必须和它的编码器插在同一个 XH 接口上，所以驱动通道与反馈编码器
// 由同一个参数 MOTOR_USE_B 决定，不会配错。
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
    parameter integer USE_ENC2  = 0
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

    output wire [3:0]  led
);

    // ========================================================================
    // 控制节拍：5ms（LQR 反馈）/ 20ms（串口上报）
    //   LQR 的微分项内部固定用 20ms 窗口，由 lqr_ctrl 自己分频,
    //   所以这里不再需要 25ms 的位置环节拍。
    // ========================================================================
    reg [17:0] cnt_5ms;
    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n)                       cnt_5ms <= 18'd0;
        else if(cnt_5ms == 18'd249_999)  cnt_5ms <= 18'd0;
        else                             cnt_5ms <= cnt_5ms + 18'd1;
    wire tick_5ms = (cnt_5ms == 18'd249_999);

    reg [19:0] cnt_20ms;
    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n)                       cnt_20ms <= 20'd0;
        else if(cnt_20ms == 20'd999_999) cnt_20ms <= 20'd0;
        else                             cnt_20ms <= cnt_20ms + 20'd1;
    wire tick_20ms = (cnt_20ms == 20'd999_999);

    // ========================================================================
    // 按键消抖
    //   key_pulse[i]：按下瞬间的单脉冲（单次触发）
    //   key_level[i]：消抖后的电平，0 = 按住（用于按住持续旋转）
    // ========================================================================
    wire [3:0] key_pulse;
    wire [3:0] key_level;
    genvar i;
    generate
        for(i = 0; i < 4; i = i + 1) begin : g_key
            key_filter #(
                .CNT_MAX(20'd1_000_000)     // 20ms @50MHz
            ) u_key (
                .sys_clk  (clk_50m),
                .sys_rst_n(rst_n),
                .key_in   (key_sw[i]),
                .key_flag (key_pulse[i]),
                .key_level(key_level[i])
            );
        end
    endgenerate

    // ========================================================================
    // ADC 角度采集
    // ========================================================================
    wire [11:0] angle;
    adc #(
        // 采样间隔必须是 ADC 时钟周期（2 个 50MHz 时钟）的整数倍，否则采样相位每拍漂移
        .SAMPLE_CNT_MAX(14'd1_951),   // 1952 clk = 976 个 ADC 周期（偶数）= 39.04us
        .PHASE_INV     (0)            // 采样相位：若角度噪声大，改成 1 试试另一相位
    ) u_adc (
        .clk_50m    (clk_50m),
        .rst_n      (rst_n),
        .sync       (tick_5ms),       // 每个控制周期重开一个 128 样本窗口
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

    // 摆臂反馈编码器
    wire [15:0] enc_ctrl = USE_ENC2 ? enc2 : enc1;

    // 驱动：只输出到 MOTOR_SEL 选中的那一路，另一路恒 0
    wire [15:0] pwm_a = (MOTOR_SEL == 0) ? pwm_cmd : 16'sd0;
    wire [15:0] pwm_b = (MOTOR_SEL == 0) ? 16'sd0   : pwm_cmd;

    // ========================================================================
    // 控制核心
    // ========================================================================
    wire signed [15:0] pwm_cmd;
    wire [3:0]         state_dbg;
    wire               led_run;
    wire               led_manual;
    wire               led_auto;
    wire               led_err;
    wire [11:0]        angle_mid_dbg;
    wire               db_flag;

    // ========================================================================
    // 控制核心：LQR 状态反馈
    //     u = -( K_th*th + K_thd*th' + K_al*al + K_ald*al' )
    //   增益由 WHEELTEC 旋转倒立摆 LQR 源码（MATLAB dlqr, Ts=20ms）换算到本工程
    //   的 counts + PWM 单位，换算过程与调参方法见 README_LQR.md。
    //
    //   档位（SW2，停止状态下切换）—— 选哪张限幅表（表在 lqr_ctrl.v 里）：
    //     0 -> 直通：完全不限幅（±5300），不积分（基线）
    //     1 -> 表 A（保守，近平衡权限大）
    //     2 -> 表 B（中等）
    //     3 -> 表 C（激进，抖动抑制最强）
    //   三张表都在 lqr_ctrl.v 的「限幅分级表」段里，每级阈值和上限都能单独改。
    //   停止时 LED1/LED2 以二进制显示当前档位。
    // ========================================================================
    reg [1:0] lqr_preset;
    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n)
            lqr_preset <= 2'd0;
        else if(key_pulse[1] && !led_run)
            lqr_preset <= lqr_preset + 2'd1;

    lqr_ctrl #(
        // ---- 平衡点：2026-10-27 实测（下垂 936 / 竖直 3082，差 2146 = 180°）----
        .ANGLE_MID_DEFAULT(12'd3082),
        .ANGLE_DOWN       (12'd936),
        .MID_FROM_DOWN    (12'd2146),
        .AUTO_CALIB       (1),          // 摆杆垂下且静止时自动标定平衡点
        .CALIB_SHIFT      (4'd6),       // 标定窗口 64 拍 = 320ms
        .CALIB_TOL        (12'd250),
        .CALIB_DRIFT_MAX  (20'sd128),   // 前后半段均值差 ≤4 counts 才认为静止
        .CALIB_STILL_MAX  (13'd24),

        // ---- 极性：沿用 PD 版实测结论 ----
        .ANGLE_INV        (1),
        .ENC_INV          (0),
        .SWING_INV        (1),

        // ---- LQR 增益（Q8 定点：实际值 = 参数 / 256）----
        //    4.446 / 238.8 / 17.66 / 78.0  [PWM per count]
        .K_TH             (17'sd1138),    //   4.4459
        .K_THD            (17'sd61130),   // 238.79
        .K_AL             (17'sd4520),    //  17.656
        .K_ALD            (17'sd19973),   //  78.02

        // ---- 平衡死区（原始 ADC counts）：带内角度误差视为 0，抑制抖动 ----
        .BAL_DEADBAND     (12'd5),

        // ---- 速度项不做额外低通（实测加低通会失稳）----
        .VEL_FILT_SHIFT   (2),

        // ---- 输出 / 保护 ----
        .PWM_MAX          (16'd5300),
        .MANUAL_WINDOW    (12'd350),      // 手动启动窗口 ≈29°（LQR 有效区小，别设太大）
        .ANGLE_LIMIT      (12'd600),      // 平衡超限 ≈50° 判倒
        .ENC_JUMP_MAX     (16'd30),       // 摆臂 5ms 突变保护（飞车）
        .PROT_DEBOUNCE    (16'd3),        // 连续 4×5ms 异常才判倒
        .ANGLE_JUMP_MAX   (13'd400),      // 角度单拍最大可信跳变

        // ---- 摆臂目标（平衡中 SW3/SW4）----
        .STEP_POS         (32'sd87),      // ±30°（2.889 counts/°）
        .RAMP_STEP        (32'sd1),       // 1 count/5ms ≈ 17°/s

        // ---- 起摆（沿用 PD 版实测参数）----
        .SWING_GAIN       (16'd1000),
        .SWING_DAMP       (16'd10),
        .CATCH_WINDOW     (16'd450),
        .CATCH_VEL        (16'd150),
        .QUARTER          (16'd1070),
        .SWING_VEL_MAX    (16'd100),
        .SWING_TIMEOUT_MAX(32'sd4000),    // 20s
        .SUCCESS_NEED     (16'd10),
        .DOWN_WINDOW      (12'd300),
        .WAIT_VEL_MAX     (16'd20),
        .WAIT_STABLE_CNT  (16'd10)
    ) u_ctrl (
        .clk       (clk_50m),
        .rst_n     (rst_n),
        .tick_5ms  (tick_5ms),

        .angle     (angle),
        .enc       (enc_ctrl),

        .key_start (key_pulse[0]),   // SW1：启动 / 停机
        .key_mode  (1'b0),           // 保留（手动/自动）
        .key_inc   (key_pulse[2]),   // SW3：摆臂目标 +30°
        .key_dec   (key_pulse[3]),   // SW4：摆臂目标 -30°
        .dir_inv   (dir_sw),
        .preset    (lqr_preset),     // SW2：调试档位（分级限幅 / 诊断）

        .pwm_cmd      (pwm_cmd),
        .state_dbg    (state_dbg),
        .led_run      (led_run),
        .led_manual   (led_manual),
        .led_auto     (led_auto),
        .led_err      (led_err),
        .angle_mid_dbg(angle_mid_dbg),
        .db_flag      (db_flag)
    );

    // ========================================================================
    // 电机驱动：A/B 两路同时输出同一 PWM
    // ========================================================================
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

    // B 通道（受 MOTOR_SEL 控制）
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
        .UART_BPS(9600)
    ) u_uart (
        .clk      (clk_50m),
        .rst_n    (rst_n),
        .tick_20ms(tick_20ms),
        .angle    (angle),
        .enc1     (enc1),
        .pwm_cmd  (pwm_cmd),
        .state_dbg(state_dbg),
        .angle_mid(angle_mid_dbg),
        .db_flag  (db_flag),
        .uart_tx  (uart_tx)
    );

    // ========================================================================
    // LED
    //   运行中：LED4 = 错误（亮 1s）  LED3 = 平衡中  LED2 = 起摆中  LED1 = 已启动
    //   停止时：LED2/LED1 以二进制显示当前档位（00/01/10/11）
    // ========================================================================
    assign led = led_run ? { led_err, (state_dbg == 4'd2), (state_dbg == 4'd1), 1'b1 }
                         : { 2'b00, lqr_preset };

endmodule

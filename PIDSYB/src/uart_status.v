// ============================================================================
// uart_status.v  —  周期性串口状态上报（含麦克风音频时域采样流）
// 每 20ms 发送一帧（115200 8N1），共 102 字节：
//   AA 55 | angle(2B) | enc1(2B) | enc2(2B) | pwm(2B) | state(1B)
//         | rawAngle(2B) | posTgt(2B) | posPwm(2B) | micLvl(2B) | abV(2B,s)
//         | audio(39 × 2B, 有符号大端) | chk(1B) | 0D 0A
//
//   angle   : 控制器【实际采信】的 12-bit 角度（α-β 滤波后）
//   enc1    : 16-bit 编码器 1（A 通道 / XH1）
//   enc2    : 16-bit 编码器 2（B 通道 / XH2）
//   pwm     : 16-bit 有符号 PWM 输出（-PWM_MAX ~ +PWM_MAX）
//   state   : 该字节低 4 位 = 状态机（0=IDLE, 1=SWING, 2=BALANCE）；
//             高 4 位 = 模式标志 {2'b00, bit5=手轮模式选中, bit4=模式已确认(arm)}
//             旧版高 4 位恒 0，兼容只取 &0x0F 的旧上位机。
//   rawAngle: 原始 ADC 角度（不滤波），用于和 angle 对比验证滤波器
//             该字节（索引 11）高 4 位改为【当前预选模式号】（0~5），
//             低 4 位仍是角度高 4 位；上位机取 &0x0F 不受影响
//   posTgt  : 16-bit 有符号位置环目标（斜坡跟随后的值）
//   posPwm  : 16-bit 有符号位置环输出（限幅后）
//   micLvl  : 麦克风单频 DFT 幅值（满幅≈31000）
//   abV     : 16-bit 有符号 α-β 速度估计（计数/5ms）
//   audio   : 麦克风【时域】交流采样，39 点/帧：
//             15.625 kSPS ÷ 8 = 1953.125 Hz（Nyquist ≈ 976 Hz），
//             每 20ms 恰好 ~39 个点，每帧发「最近 39 点」（≈20ms 音频），
//             上位机可拼成连续波形。静音时以 0 为中心；暖机 2.1s 内输出 0。
//             ★ 想要更高保真：把 AUD_DEC 改 4（3906 Hz，78 点/帧），
//               同时把 FRAME_LEN 改 180、校验/帧尾索引后移 78，
//               并把上位机的 AUD_FS / FRAME_LEN / AUD_N 一起改。
//               代价：UART 占空比从 44% 升到 78%，寄存器多用 ~1250 个。
//   chk     : 字节 2~98 的异或校验和（不含帧头/帧尾）
//
//   所有多字节字段均为大端。
//   115200 下 102 字节 = 8.9ms，仍小于 20ms 帧周期（占空比 44%）。
// ============================================================================
module uart_status #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer UART_BPS = 115200
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick_20ms,
    input  wire [11:0]        angle,       // 控制器实际采信的角度
    input  wire [11:0]        angle_raw,   // 原始 ADC 值（含尖峰）
    input  wire [15:0]        enc1,
    input  wire [15:0]        enc2,
    input  wire signed [15:0] pwm_cmd,    // 16-bit 有符号 PWM 输出
    input  wire [3:0]         state_dbg,  // 4-bit 状态机状态
    input  wire [1:0]         mode_flag,  // bit0 = 模式已确认(arm)，bit1 = 手轮模式选中
    input  wire [2:0]         mode_dbg,   // 当前预选模式号（0~5）
    input  wire signed [15:0] pos_target, // 位置环目标位置
    input  wire signed [15:0] pos_pwm,    // 位置环输出
    input  wire [15:0]        mic_level,  // 麦克风单频幅值
    input  wire signed [15:0] ab_v,       // α-β 速度估计（计数/5ms）
    input  wire signed [15:0] mic_smp,    // 麦克风时域采样（15.625 kSPS，已去直流）
    input  wire               mic_vld,    // mic_smp 有效脉冲（1 个 clk）
    output wire               uart_tx
);

    // ⚠ 位宽必须够：FRAME_LEN=102 需要 7 位（5 位最大 31）。
    localparam [6:0] FRAME_LEN = 7'd102;
    localparam integer AUD_N   = 39;         // 每帧音频样点数
    localparam integer AUD_DEC = 8;          // 15.625k ÷ 8 = 1953.125 Hz

    // 发送前锁存，保证一帧数据一致
    reg [11:0]        angle_lat;
    reg [11:0]        angle_raw_lat;
    reg [15:0]        enc1_lat;
    reg [15:0]        enc2_lat;
    reg signed [15:0] pwm_lat;
    reg [3:0]         state_lat;
    reg [1:0]         mode_flag_lat;
    reg [2:0]         mode_dbg_lat;
    reg signed [15:0] pos_tgt_lat;
    reg signed [15:0] pos_pwm_lat;
    reg [15:0]        mic_lat;
    reg signed [15:0] abv_lat;

    // ---------------- 麦克风音频采样：÷8 抽取 + 39 级移位寄存器 ----------------
    //   用移位寄存器而不是 RAM：每帧要一次读出 39 个样点，
    //   单端口 RAM 做不到（工具会复制 RAM 或直接报错）。
    reg [2:0]         adec;               // 抽取计数
    reg signed [15:0] aud_sr [0:AUD_N-1]; // aud_sr[0] = 最新
    reg signed [15:0] aud_lat[0:AUD_N-1]; // aud_lat[0] = 最旧（发送顺序）
    reg [7:0]         aud_chk_lat;
    integer           m, n, p;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            adec <= 3'd0;
            for(m = 0; m < AUD_N; m = m + 1) aud_sr[m] <= 16'sd0;
        end else if(mic_vld) begin
            adec <= adec + 3'd1;
            if(adec == (AUD_DEC-1)) begin
                aud_sr[0] <= mic_smp;
                for(m = 1; m < AUD_N; m = m + 1) aud_sr[m] <= aud_sr[m-1];
            end
        end
    end

    // 音频字节的异或校验（顺序无关，直接对移位寄存器求和）
    reg [7:0] aud_chk_c;
    always @(*) begin
        aud_chk_c = 8'd0;
        for(n = 0; n < AUD_N; n = n + 1)
            aud_chk_c = aud_chk_c ^ aud_sr[n][15:8] ^ aud_sr[n][7:0];
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            angle_lat <= 12'd0;
            angle_raw_lat <= 12'd0;
            enc1_lat  <= 16'd0;
            enc2_lat  <= 16'd0;
            pwm_lat   <= 16'sd0;
            state_lat <= 4'd0;
            mode_flag_lat <= 2'd0;
            mode_dbg_lat  <= 3'd0;
            pos_tgt_lat <= 16'sd0;
            pos_pwm_lat <= 16'sd0;
            mic_lat     <= 16'd0;
            abv_lat     <= 16'sd0;
            aud_chk_lat <= 8'd0;
            for(p = 0; p < AUD_N; p = p + 1) aud_lat[p] <= 16'sd0;
        end else if(tick_20ms) begin
            angle_lat <= angle;
            angle_raw_lat <= angle_raw;
            enc1_lat  <= enc1;
            enc2_lat  <= enc2;
            pwm_lat   <= pwm_cmd;
            state_lat <= state_dbg;
            mode_flag_lat <= mode_flag;
            mode_dbg_lat  <= mode_dbg;
            pos_tgt_lat <= pos_target;
            pos_pwm_lat <= pos_pwm;
            mic_lat     <= mic_level;
            abv_lat     <= ab_v;
            // 按「最旧 → 最新」排好，发送时上位机从前往后拼就是连续波形
            for(p = 0; p < AUD_N; p = p + 1)
                aud_lat[p] <= aud_sr[AUD_N-1-p];
            aud_chk_lat <= aud_chk_c;
        end
    end

    // 直接输出 ADC 原值：串口读数 == ADC 值 == top.v 的 ANGLE_MID_DEFAULT，
    // 不再像原工程那样做 theta_out_r = theta_r + 135，避免与设定值对不上。
    wire [15:0] angle_tx = {4'h0, angle_lat};

    // 校验和：字节 2~98 的异或（不含帧头 AA/55 与帧尾 0D/0A）
    //   ⚠ 第 11 字节（索引 10）含 mode_flag，第 12 字节（索引 11）含模式号，
    //     第 22~99 字节是 39 个音频样点，校验必须覆盖同一批字节
    wire [7:0] state_byte = {2'b00, mode_flag_lat, state_lat};
    wire [7:0] raw_hi_byte = {1'b0, mode_dbg_lat, angle_raw_lat[11:8]};
    wire [7:0] chk_sum = angle_tx[15:8] ^ angle_tx[7:0]
                       ^ enc1_lat[15:8] ^ enc1_lat[7:0]
                       ^ enc2_lat[15:8] ^ enc2_lat[7:0]
                       ^ pwm_lat[15:8]  ^ pwm_lat[7:0]
                       ^ state_byte
                       ^ raw_hi_byte ^ angle_raw_lat[7:0]
                       ^ pos_tgt_lat[15:8] ^ pos_tgt_lat[7:0]
                       ^ pos_pwm_lat[15:8] ^ pos_pwm_lat[7:0]
                       ^ mic_lat[15:8]     ^ mic_lat[7:0]
                       ^ abv_lat[15:8]     ^ abv_lat[7:0]
                       ^ aud_chk_lat;

    // ---------------- 发送状态机寄存器 ----------------
    reg [6:0] idx;
    reg [1:0] st;            // 0=等待, 1=装载并启动, 2=等待发送完成

    // ---------------- 帧内容 ----------------
    //   音频区：索引 21~98，共 78 字节 = 39 个 16-bit 大端样点
    wire [6:0] aidx = idx - 7'd21;       // 0 ~ 77
    wire [5:0] a_i  = {1'b0, aidx[6:1]}; // 0 ~ 38

    reg [7:0] frame_byte;
    always @(*) begin
        case(idx)
            7'd0:  frame_byte = 8'hAA;
            7'd1:  frame_byte = 8'h55;
            7'd2:  frame_byte = angle_tx[15:8];
            7'd3:  frame_byte = angle_tx[7:0];
            7'd4:  frame_byte = enc1_lat[15:8];  // 编码器 1（A 通道 / XH1）
            7'd5:  frame_byte = enc1_lat[7:0];
            7'd6:  frame_byte = enc2_lat[15:8];  // 编码器 2（B 通道 / XH2）
            7'd7:  frame_byte = enc2_lat[7:0];
            7'd8:  frame_byte = pwm_lat[15:8];   // PWM 输出（有符号，大端）
            7'd9:  frame_byte = pwm_lat[7:0];
            7'd10: frame_byte = state_byte;  // 状态机状态 + 模式标志
            7'd11: frame_byte = raw_hi_byte;  // 高 4 位 = 模式号，低 4 位 = 原始角度高 4 位
            7'd12: frame_byte = angle_raw_lat[7:0];           // 原始角度低 8 位
            7'd13: frame_byte = pos_tgt_lat[15:8];   // 位置环目标 高字节
            7'd14: frame_byte = pos_tgt_lat[7:0];    // 位置环目标 低字节
            7'd15: frame_byte = pos_pwm_lat[15:8];   // 位置环输出 高字节
            7'd16: frame_byte = pos_pwm_lat[7:0];    // 位置环输出 低字节
            7'd17: frame_byte = mic_lat[15:8];       // 麦克风单频幅值 高字节
            7'd18: frame_byte = mic_lat[7:0];        // 麦克风单频幅值 低字节
            7'd19: frame_byte = abv_lat[15:8];       // α-β 速度 高字节
            7'd20: frame_byte = abv_lat[7:0];        // α-β 速度 低字节
            7'd99: frame_byte = chk_sum;             // 异或校验和
            7'd100: frame_byte = 8'h0D;
            7'd101: frame_byte = 8'h0A;
            default: begin
                if(idx >= 7'd21 && idx <= 7'd98)
                    // 音频：偶数偏移 = 高字节，奇数偏移 = 低字节
                    frame_byte = aidx[0] ? aud_lat[a_i][7:0] : aud_lat[a_i][15:8];
                else
                    frame_byte = 8'h00;
            end
        endcase
    end

    // ---------------- 串口发送 ----------------
    reg [7:0] tx_data;
    reg       tx_start;
    wire      tx_busy;
    wire      tx_done;

    uart_tx #(
        .CLK_FREQ(CLK_FREQ),
        .UART_BPS(UART_BPS)
    ) u_tx (
        .clk(clk),
        .rst_n(rst_n),
        .tx_data(tx_data),
        .tx_start(tx_start),
        .tx(uart_tx),
        .tx_busy(tx_busy),
        .tx_done(tx_done)
    );

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            idx      <= 7'd0;
            st       <= 2'd0;
            tx_data  <= 8'd0;
            tx_start <= 1'b0;
        end else begin
            tx_start <= 1'b0;     // 默认只脉冲一拍
            case(st)
                2'd0: begin
                    if(tick_20ms) begin
                        idx <= 7'd0;
                        st  <= 2'd1;
                    end
                end
                2'd1: begin
                    if(idx < FRAME_LEN) begin
                        tx_data  <= frame_byte;
                        tx_start <= 1'b1;
                        idx      <= idx + 7'd1;
                        st       <= 2'd2;
                    end else begin
                        st <= 2'd0;   // 一帧结束
                    end
                end
                2'd2: begin
                    if(tx_done) st <= 2'd1;
                end
                default: st <= 2'd0;
            endcase
        end
    end

endmodule

// ============================================================================
// mic_fft.v  —  MP34DT01 / PDM 麦克风 + 单频 DFT 幅值检测（FFT 单 bin）
//
// 【为什么不是完整 FFT】
//   512 点 FFT 需要旋转因子 ROM + 蝶形运算 + 位反序 + 双口 BSRAM，约 3000+ LUT，
//   而且要把整块频谱搬出来再挑出某一条谱线。本任务只要「某一个频率的幅值」，
//   等价于只算 FFT 的一条谱线 —— 用正交解调（Goertzel / 锁相放大器）实现，
//   数学上就是 DFT 的第 k 条谱线：
//
//       X(f0) = Σ x[n]·e^(-j2πf0 n/fs) = I + jQ
//       |X(f0)| = √(I² + Q²)          ← 就是该频率的幅值
//
//   代价只有 2 个乘法器，而且换频率不需要重算任何系数（NCO 相位字直接改）。
//
// 【信号链】
//   PDM 3.125MHz → 3阶CIC(÷200) → 15.625kSPS → 去直流
//   → NCO 产生 sin/cos(f0) → I/Q 各自乘累加 512 点
//   → |X| ≈ max + 0.375·min（免开方近似）→ level
//
// 【频率】
//   f0 = phase_inc × fs / 65536 ，fs = 15.625kHz
//   → 分辨率 0.238Hz，范围 1Hz ~ 7.8kHz（Nyquist）
//   频率字由 freq_hz 换算：phase_inc = freq_hz × 68719 / 2^14 ≈ freq_hz × 4.194304
//
// 【幅值定标】
//   满幅单音（调制度 100%，≈120dBSPL）→ level ≈ 31000
//   等效于 CIC 满量程 4e6 的交流分量
//
// 【参数（串口下发，见 top.v）】
//   0x18 MIC_FREQ  目标频率 Hz
//   0x19 MIC_TH    点亮阈值（与 level 同标度）
//   0x1A MIC_HOLD  保持时间，单位 5ms
// ============================================================================
module mic_fft #(
    parameter integer OSR     = 200,   // PDM 抽取比：3.125MHz/200 = 15.625kSPS
    parameter integer LOG2N   = 9,     // 相干积分点数 2^9 = 512（32.8ms / 块）
    parameter integer AMP_SH  = 14     // 幅值定标右移
)(
    input  wire        clk_50m,
    input  wire        rst_n,

    output reg         mic_clk,        // 3.125MHz PDM 时钟（送麦克风 CLK）
    input  wire        mic_data,       // 麦克风 DOUT

    input  wire [15:0] freq_hz,        // 目标频率（Hz）
    input  wire [15:0] th,             // 点亮阈值
    input  wire [15:0] hold_ticks,     // 保持时间（×5ms）
    input  wire        tick_5ms,

    output reg         level_led,      // 幅值指示（接 LED）
    output reg  [15:0] level,          // 该频率的幅值（可读回上位机调阈值）
    // ---- 时域采样输出（供上位机画音频波形）----
    output wire signed [15:0] sample,  // 15.625 kSPS 交流采样（已去直流，暖机期间为 0）
    output wire               sample_vld // sample 有效脉冲（与 sample 同拍）
);

    localparam integer NF     = (1 << LOG2N);
    localparam [15:0]  N_LAST = NF - 1;

    // ========================================================================
    // 1) PDM 时钟：50MHz / 16 = 3.125MHz
    // ========================================================================
    reg [3:0] div16;
    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n) div16 <= 4'd0;
        else       div16 <= div16 + 4'd1;

    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n) mic_clk <= 1'b0;
        else       mic_clk <= div16[3];

    // ========================================================================
    // 2) DOUT 同步
    //    LR=GND（左声道）：数据在 CLK 下降沿更新。
    //    div16 回 0 时出下降沿，取 div16==12（240ns 后）采样，裕量充足。
    // ========================================================================
    reg mic_d1, mic_d2;
    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n) begin
            mic_d1 <= 1'b0;
            mic_d2 <= 1'b0;
        end else begin
            mic_d1 <= mic_data;
            mic_d2 <= mic_d1;
        end

    wire bit_ce = (div16 == 4'd12);

    // ========================================================================
    // 3) 三阶 CIC 积分器
    //    积分器允许模 2^26 回绕，梳状差分会把回绕抵消（CIC 的正常行为）
    // ========================================================================
    reg [25:0] i1, i2, i3;
    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n) begin
            i1 <= 26'd0;
            i2 <= 26'd0;
            i3 <= 26'd0;
        end else if(bit_ce) begin
            i1 <= i1 + {25'd0, mic_d2};
            i2 <= i2 + i1;
            i3 <= i3 + i2;
        end

    wire [25:0] i3_new = i3 + i2;

    // ========================================================================
    // 4) 抽取计数
    // ========================================================================
    reg [15:0] dec_cnt;
    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n) dec_cnt <= 16'd0;
        else if(bit_ce)
            dec_cnt <= (dec_cnt == OSR - 1) ? 16'd0 : dec_cnt + 16'd1;

    wire dec_ce = bit_ce && (dec_cnt == OSR - 1);

    // ========================================================================
    // 5) 三阶梳状（在 15.625kSPS 上运行）
    // ========================================================================
    reg signed [25:0] c1, c2, c3;
    reg signed [25:0] d1, d2, d3;
    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n) begin
            c1 <= 26'sd0; c2 <= 26'sd0; c3 <= 26'sd0;
            d1 <= 26'sd0; d2 <= 26'sd0; d3 <= 26'sd0;
        end else if(dec_ce) begin
            c1 <= $signed(i3_new) - d1;
            d1 <= $signed(i3_new);
            c2 <= c1 - d2;
            d2 <= c1;
            c3 <= c2 - d3;
            d3 <= c2;
        end

    reg comb_vld;
    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n) comb_vld <= 1'b0;
        else       comb_vld <= dec_ce;

    // ========================================================================
    // 6) 直流跟踪（CIC 输出直流 ≈ 增益的一半，必须扣掉才能看交流）
    //    前 2.1s 用 τ≈262ms 快速收敛，之后用 τ≈8.4s 慢跟踪
    // ========================================================================
    reg signed [26:0] dc;
    reg [15:0]        warm_cnt;

    wire signed [26:0] dc_err = $signed(c3) - dc;

    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n) begin
            dc       <= 27'sd4_000_000;   // 静音时 PDM 密度≈50% → CIC≈增益一半
            warm_cnt <= 16'd32768;        // 2.1s
        end else if(comb_vld) begin
            if(warm_cnt != 16'd0) begin
                warm_cnt <= warm_cnt - 16'd1;
                dc <= dc + (dc_err >>> 12);
            end else begin
                dc <= dc + (dc_err >>> 17);
            end
        end

    // 交流分量缩到 16-bit（满量程 ±4e6 → ±15625）
    //   直接写 dc_err[23:8] 等价于 (dc_err >>> 8) 的低 16 位，
    //   既避免了位宽截断告警，也保证符号位不丢（|值| < 2^15）。
    wire signed [15:0] ac16 = $signed(dc_err[23:8]);

    // ========================================================================
    // 7) NCO：相位累加器 + sin/cos ROM
    //    f0 = phase_inc × 15.625kHz / 65536
    //    cos(θ) = sin(θ + 90°) → 只需一张 256 项正弦表
    // ========================================================================
    wire [33:0] fw_tmp    = freq_hz * 32'd68719;   // ≈ freq_hz × 4.194304 × 2^14
    wire [15:0] phase_inc = fw_tmp[29:14];

    // 暖机期间不累加：此时 dc 未收敛，算出来的电压全是垃圾
    wire samp_vld = comb_vld && (warm_cnt == 16'd0);

    // 时域采样输出：暖机结束后直接就是去直流后的交流分量
    assign sample     = samp_vld ? ac16 : 16'sd0;
    assign sample_vld = samp_vld;

    reg [15:0] phase_acc;
    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n) phase_acc <= 16'd0;
        else if(samp_vld) phase_acc <= phase_acc + phase_inc;

    wire [7:0] nco_i = phase_acc[15:8];
    wire [7:0] nco_q = nco_i + 8'd64;              // +90°

    reg signed [7:0] sin_rom [0:255];
    initial begin
        `include "sin256.vh"
    end

    reg signed [7:0] sin_r, cos_r;
    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n) begin
            sin_r <= 8'sd0;
            cos_r <= 8'sd127;
        end else begin
            sin_r <= sin_rom[nco_i];
            cos_r <= sin_rom[nco_q];
        end

    // ========================================================================
    // 8) 正交解调流水线
    //    拍 T   : ac16 / sin_r / cos_r 就绪
    //    拍 T+1 : I = ac16·sin ，Q = ac16·cos
    //    拍 T+2 : 累加进块和
    // ========================================================================
    reg               s1_vld;
    reg signed [15:0] s1_ac;
    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n) begin
            s1_vld <= 1'b0;
            s1_ac  <= 16'sd0;
        end else begin
            s1_vld <= samp_vld;
            if(samp_vld) s1_ac <= ac16;
        end

    reg               s2_vld;
    reg signed [23:0] s2_pi, s2_pq;
    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n) begin
            s2_vld <= 1'b0;
            s2_pi  <= 24'sd0;
            s2_pq  <= 24'sd0;
        end else begin
            s2_vld <= s1_vld;
            // 赋值目标 24-bit 会把两个操作数一起扩展到 24-bit，得完整乘积
            s2_pi  <= s1_ac * sin_r;
            s2_pq  <= s1_ac * cos_r;
        end

    // ========================================================================
    // 9) 块累加（每 NF 个样本输出一次）
    //    单样本乘积 ≤ 15625×127 ≈ 2e6，512 点累加 ≤ 1e9 → 34-bit 足够
    // ========================================================================
    reg signed [33:0] acc_i, acc_q;
    reg signed [33:0] blk_i, blk_q;
    reg [15:0]        blk_cnt;
    reg               blk_done;

    wire blk_end = (blk_cnt == N_LAST);

    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n) begin
            acc_i    <= 34'sd0;
            acc_q    <= 34'sd0;
            blk_i    <= 34'sd0;
            blk_q    <= 34'sd0;
            blk_cnt  <= 16'd0;
            blk_done <= 1'b0;
        end else begin
            blk_done <= 1'b0;
            if(s2_vld) begin
                if(blk_end) begin
                    blk_i    <= acc_i + s2_pi;
                    blk_q    <= acc_q + s2_pq;
                    acc_i    <= 34'sd0;
                    acc_q    <= 34'sd0;
                    blk_cnt  <= 16'd0;
                    blk_done <= 1'b1;
                end else begin
                    acc_i   <= acc_i + s2_pi;
                    acc_q   <= acc_q + s2_pq;
                    blk_cnt <= blk_cnt + 16'd1;
                end
            end
        end

    // ========================================================================
    // 10) 幅值：|X| = √(I²+Q²) 的免开方近似
    //     |X| ≈ max + 0.375·min ，误差 0 ~ -3%（够做阈值判定与读数）
    //     避免了 34×34 乘法器和开方电路
    // ========================================================================
    wire signed [33:0] abs_i = blk_i[33] ? (~blk_i + 34'sd1) : blk_i;
    wire signed [33:0] abs_q = blk_q[33] ? (~blk_q + 34'sd1) : blk_q;
    wire        [33:0] hi    = (abs_i > abs_q) ? abs_i : abs_q;
    wire        [33:0] lo    = (abs_i > abs_q) ? abs_q : abs_i;
    wire        [36:0] mag   = {3'd0, hi}
                             + {5'd0, lo[33:2]}      // + 0.25·lo
                             + {6'd0, lo[33:3]};     // + 0.125·lo

    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n)           level <= 16'd0;
        else if(blk_done) begin
            if(|mag[36:30])  level <= 16'hFFFF;      // 饱和
            else             level <= mag[29:14];    // >>AMP_SH(14)
        end

    // ========================================================================
    // 11) LED：幅值 ≥ 阈值点亮，并保持 hold_ticks 个 5ms 拍
    // ========================================================================
    reg [15:0] hold_cnt;
    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n) begin
            hold_cnt  <= 16'd0;
            level_led <= 1'b0;
        end else if(tick_5ms) begin
            if(warm_cnt != 16'd0) begin
                hold_cnt  <= 16'd0;          // 上电收敛期强制熄灭
                level_led <= 1'b0;
            end else if(level >= th) begin
                hold_cnt  <= hold_ticks;
                level_led <= 1'b1;
            end else if(hold_cnt != 16'd0) begin
                hold_cnt  <= hold_cnt - 16'd1;
                level_led <= 1'b1;
            end else begin
                level_led <= 1'b0;
            end
        end

endmodule

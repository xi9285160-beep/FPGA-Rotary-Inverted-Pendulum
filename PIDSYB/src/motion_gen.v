// ============================================================================
// motion_gen.v  —  固件内建的摆臂运动曲线发生器（脱机演示用）
//
// 【为什么需要】网页控制台里的「运动曲线」是浏览器每 50ms 下发一次目标值算出来的，
//   断掉串口就没有曲线了。本模块把发生器搬进 FPGA，按键即可脱机演示。
//
// 【模式】（对应按键 SW2 轮换）
//   0 = 纯稳摆    ：offset 恒为 0（只保持平衡，目标由 SW3/SW4 另行设定）
//   1 = 正弦      ：offset = amp · sin(2π·t/T)
//   2 = 三角      ：offset = amp · tri(2π·t/T)      匀速往复
//   3 = 线性      ：offset = amp · t/T              单向斜坡，到 +amp 后停住
//   4 = 四四拍    ：4/4 拍短距离往复。每拍一个半正弦鼓包，拍间回零。
//                   方向：左 左 右 右（第1、2拍同向，第3、4拍反向）
//                   幅度：强 弱 次强 弱（1.00 / 0.40 / 0.70 / 0.40）
//                   幅度由【独立参数 beat_amp】控制（不再借用 amp，也不再减半）
//
// 【相位】phase 16-bit，65536 = 一个完整周期。
//   每 tick_5ms 累加 step = 65536 / period，所以周期 = period × 5ms。
//   除法只在 period 变化时做一次（17 次迭代的移位减法），不需要常驻除法器。
//
// 【正弦表】复用 src/sin256.vh（256 项，幅值 ±127），与 u_mic 各自独立实例化。
//
// 【enable】为 0 时相位归零、输出 0。进入平衡才使能，保证每次起摆后曲线都从
//   0 开始，不会有跳变。
// ============================================================================
module motion_gen #(
    parameter integer MAX_AMP  = 520,      // 幅值上限（计数），520 ≈ 180°
    parameter integer DEF_PER  = 800       // 默认周期（×5ms）= 4 秒
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick_5ms,    // 5ms 节拍

    input  wire               enable,      // 1 = 输出曲线；0 = 归零
    input  wire [2:0]         mode,        // 0~4
    input  wire signed [15:0] amp,         // 曲线幅值（模式 1~3 用，计数）
    input  wire signed [15:0] beat_amp,    // 四四拍单拍峰值（模式 4 用，计数）
    input  wire [15:0]        period,      // 周期（×5ms）

    output reg  signed [15:0] offset       // 相对起点的位置偏移（计数）
);

    // ========================================================================
    // 1) period 变化时重算相位步长  step = 65536 / period
    //    17 次移位减法，period 稳定后完全不占时间
    // ========================================================================
    wire [15:0] per = (period < 16'd16) ? 16'd16 : period;   // 防除零

    reg [15:0] period_d;
    reg        dv_busy;
    reg [5:0]  dv_cnt;
    reg [31:0] dv_rem;
    reg [17:0] dv_quo;
    reg [17:0] dv_num;
    reg [15:0] phase_step;

    wire [31:0] dv_sh = {dv_rem[30:0], dv_num[17]};
    wire        dv_ge = (dv_sh >= {16'd0, per});

    always @(posedge clk or negedge rst_n)
        if(!rst_n) begin
            period_d   <= 16'd0;
            dv_busy    <= 1'b0;
            dv_cnt     <= 6'd0;
            dv_rem     <= 32'd0;
            dv_quo     <= 18'd0;
            dv_num     <= 18'd0;
            phase_step <= 16'd0;
        end else if(per != period_d) begin
            // 启动一次除法
            period_d <= per;
            dv_busy  <= 1'b1;
            dv_cnt   <= 6'd18;
            dv_rem   <= 32'd0;
            dv_quo   <= 18'd0;
            dv_num   <= 18'h10000;          // 被除数 = 65536
        end else if(dv_busy) begin
            if(dv_cnt != 6'd0) begin
                dv_rem <= dv_ge ? (dv_sh - {16'd0, per}) : dv_sh;
                dv_quo <= {dv_quo[16:0], dv_ge};
                dv_num <= {dv_num[16:0], 1'b0};
                dv_cnt <= dv_cnt - 6'd1;
            end else begin
                dv_busy    <= 1'b0;
                phase_step <= (dv_quo > 18'd65535) ? 16'd65535 : dv_quo[15:0];
            end
        end

    // ========================================================================
    // 2) 相位累加 + 线性模式的「到终点保持」标志
    // ========================================================================
    reg [15:0] phase;
    reg        lin_hold;

    // 本拍累加后的相位（用于同拍生成输出，避免多一拍延迟）
    wire [16:0] phase_sum = {1'b0, phase} + {1'b0, phase_step};
    wire [15:0] phase_nxt = phase_sum[15:0];

    always @(posedge clk or negedge rst_n)
        if(!rst_n) begin
            phase    <= 16'd0;
            lin_hold <= 1'b0;
        end else if(tick_5ms) begin
            if(!enable) begin
                phase    <= 16'd0;
                lin_hold <= 1'b0;
            end else begin
                phase <= phase_nxt;
                // 线性模式：第一次回绕就到终点，之后一直保持
                if(phase_sum[16]) lin_hold <= 1'b1;
            end
        end

    // ========================================================================
    // 3) 波形表
    // ========================================================================
    reg signed [7:0] sin_rom [0:255];
    initial begin
        `include "sin256.vh"
    end

    wire [7:0]        idx  = phase[15:8];       // 0~255 对应 0~360°
    wire signed [7:0] sval = sin_rom[idx];      // 正弦

    // ---- 三角：-127 → +127 → -127，匀速往复 ----
    //   idx < 128 : 2·idx - 127      (0→-127, 127→127)
    //   idx >=128 : 381 - 2·idx      (128→125, 255→-129 略微越界，下面钳位)
    wire signed [10:0] tri_w = (idx < 8'd128)
                             ? (($signed({3'b0, idx}) <<< 1) - 11'sd127)
                             : (11'sd381 - ($signed({3'b0, idx}) <<< 1));
    wire signed [7:0] tri_v = (tri_w >  11'sd127) ?  8'sd127 :
                              (tri_w < -11'sd127) ? -8'sd127 : tri_w[7:0];

    // ---- 四四拍：4/4 拍短距离往复，每拍一个半正弦鼓包 ----
    //   4/4 拍 = 每周期 4 拍，每拍时长相同（周期/4）。
    //   每拍是一个 0→峰值→0 的半正弦鼓包，拍与拍之间回零。
    //
    //   【节奏】左 左 右 右（对应「强 弱 次强 弱」）：
    //     第1拍 强拍  → 左
    //     第2拍 弱拍  → 左
    //     第3拍 次强拍 → 右
    //     第4拍 弱拍  → 右
    //   方向由 beat[1] 决定（0,1 拍同向，2,3 拍反向），
    //   幅度比例 Q8：256 / 102 / 179 / 102（即 1.00 / 0.40 / 0.70 / 0.40）。
    wire [1:0]        beat   = idx[7:6];              // 0~3
    wire [5:0]        beat_u = idx[5:0];              // 拍内相位 0~63
    wire signed [7:0] half   = sin_rom[{beat_u, 1'b0}];  // 0→127→0 的半正弦
    wire              beat_dir_neg = beat[1];         // 第 3、4 拍取反 → 右

    reg [15:0] beat_ratio;                            // 本拍幅度比例（Q8，256 = 1.00）
    always @(*) begin
        case(beat)
            2'd0:    beat_ratio = 16'd256;   // 强
            2'd1:    beat_ratio = 16'd102;   // 弱（≈0.40）
            2'd2:    beat_ratio = 16'd179;   // 次强（≈0.70）
            default: beat_ratio = 16'd102;   // 弱（≈0.40）
        endcase
    end

    // ========================================================================
    // 4) 幅值钳位 + 波形合成
    //    sin_rom 幅值是 ±127，所以统一用 >>>7 换算到 amp 的量纲。
    // ========================================================================
    // 幅值钳位：先扩到 24-bit 再比较，避免 16-bit 常量与参数比较时的位宽告警
    localparam signed [23:0] AMP_MAX24 = MAX_AMP;
    wire signed [23:0] amp24 = $signed(amp);
    wire signed [23:0] amp_c = (amp24 >  AMP_MAX24) ?  AMP_MAX24 :
                               (amp24 < -AMP_MAX24) ? -AMP_MAX24 : amp24;

    // 正弦 / 三角：amp × 波形值 / 127
    wire signed [23:0] off_sin = ($signed(amp_c) * $signed(sval)) >>> 7;
    wire signed [23:0] off_tri = ($signed(amp_c) * $signed(tri_v)) >>> 7;

    // 线性：amp × idx / 256，回绕后保持 amp
    wire signed [23:0] off_lin_ramp = ($signed(amp_c) * $signed({8'd0, idx})) >>> 8;
    wire signed [23:0] off_lin = lin_hold ? amp_c : off_lin_ramp;

    // 四四拍：独立幅度 beat_amp × 本拍比例 × 半正弦鼓包 × 方向
    //   幅度不再借用 amp，也不再固定减半 —— 上位机可直接设「单拍峰值」。
    localparam signed [23:0] BEAT_MAX24 = MAX_AMP;
    wire signed [23:0] beat24 = $signed(beat_amp);
    wire signed [23:0] beat_c = (beat24 >  BEAT_MAX24) ?  BEAT_MAX24 :
                                (beat24 < -BEAT_MAX24) ? -BEAT_MAX24 : beat24;

    // 本拍峰值 = beat_amp × 比例 / 256（强拍 ratio=256 → 正好等于 beat_amp）
    wire signed [23:0] beat_peak = ($signed(beat_c) * $signed({1'b0, beat_ratio})) >>> 8;
    // 波形：峰值 × 半正弦 / 127（与其它模式同一标度）
    wire signed [23:0] off_half  = ($signed(beat_peak) * $signed(half)) >>> 7;
    wire signed [23:0] off_beat  = beat_dir_neg ? -off_half : off_half;

    // ========================================================================
    // 5) 模式选择
    // ========================================================================
    reg signed [23:0] off_sel;
    always @(*) begin
        case(mode)
            3'd1:    off_sel = off_sin;
            3'd2:    off_sel = off_tri;
            3'd3:    off_sel = off_lin;
            3'd4:    off_sel = off_beat;
            default: off_sel = 24'sd0;      // 0 = 纯稳摆
        endcase
    end

    // 饱和到 16-bit 有符号
    wire signed [23:0] off_fin = enable ? off_sel : 24'sd0;

    always @(posedge clk or negedge rst_n)
        if(!rst_n)                  offset <= 16'sd0;
        else if(tick_5ms) begin
            if(off_fin > 24'sd32767)      offset <=  16'sd32767;
            else if(off_fin < -24'sd32768) offset <= -16'sd32768;
            else                           offset <= off_fin[15:0];
        end

endmodule

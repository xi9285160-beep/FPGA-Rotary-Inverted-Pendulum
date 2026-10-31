// ============================================================================
// traj_gen.v  —  摆臂位置曲线发生器
//
// 四种曲线（mode）：
//   0 = 位置正弦   pos = A·sin(2πt/T)         速度连续
//   1 = 位置三角   三角波                      速度方波（换向时加速度冲击）
//   2 = S 形往复   三角波经一阶低通           速度梯形，启停平滑
//   3 = 匀速旋转   pos = v·t                   单向持续，配合复位点使用
//
// 相位累加器 16 位：PHASE_STEP=82 → 65536/82 ≈ 799 个 5ms ≈ 4s 一个周期
// 幅度 amp 单位是编码器计数：1040 counts/圈 → 100 counts ≈ 34.6°
//
// en 的上升沿会自动把相位/积分清零，所以起摆或保护重起后曲线总是从头开始，
// 不需要额外的复位信号。
// ============================================================================
module traj_gen #(
    parameter signed [31:0] AMP_INIT  = 32'sd100,   // 初始幅度（counts），约 34.6°
    parameter signed [31:0] AMP_MIN   = 32'sd20,    // 幅度下限
    parameter signed [31:0] AMP_MAX   = 32'sd300,   // 幅度上限
    parameter signed [31:0] AMP_STEP  = 32'sd29,    // 每次按键 ±29 counts ≈ ±10°
    parameter        [15:0] PHASE_STEP = 16'd82,    // 相位步长（≈4s 周期）
    parameter signed [31:0] V_CONST    = 32'sd1,    // 匀速曲线速度：1 count/5ms ≈ 69°/s
    parameter signed [31:0] POS_LIMIT  = 32'sd20000 // 匀速曲线位置限幅（约 ±19 圈）
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick_5ms,

    input  wire               en,          // 1 = 曲线运行（en 上升沿自动复位相位）
    input  wire [1:0]         mode,        // 0=正弦 1=三角 2=S形 3=匀速
    input  wire               amp_up,      // 幅度 +（单脉冲）
    input  wire               amp_dn,      // 幅度 −（单脉冲）

    output reg  signed [31:0] pos_ref,     // 位置给定（编码器计数，相对复位点）
    output wire signed [31:0] amp_dbg      // 当前幅度（调试用）
);

    // ---------------- 寄存器 ----------------
    // 注意：phase 必须在下面的正弦表 always 块之前声明（Verilog 要求先声明后引用）
    reg [15:0]        phase;        // 相位累加器
    reg signed [31:0] amp;          // 当前幅度
    reg signed [31:0] pos_lin;      // 匀速曲线的位置积分
    reg signed [31:0] tri_filt;     // S 形曲线：三角波的一阶低通结果
    reg               en_d;         // en 上一拍，用于取上升沿

    assign amp_dbg = amp;

    // ---------------- 64 点正弦表（幅度 ±127） ----------------
    reg signed [7:0] sin_val;
    always @(*) begin
        case(phase[15:10])
            6'd0 : sin_val = 8'sd0  ;
            6'd1 : sin_val = 8'sd12 ;
            6'd2 : sin_val = 8'sd25 ;
            6'd3 : sin_val = 8'sd37 ;
            6'd4 : sin_val = 8'sd49 ;
            6'd5 : sin_val = 8'sd60 ;
            6'd6 : sin_val = 8'sd71 ;
            6'd7 : sin_val = 8'sd81 ;
            6'd8 : sin_val = 8'sd90 ;
            6'd9 : sin_val = 8'sd98 ;
            6'd10: sin_val = 8'sd106;
            6'd11: sin_val = 8'sd112;
            6'd12: sin_val = 8'sd117;
            6'd13: sin_val = 8'sd122;
            6'd14: sin_val = 8'sd125;
            6'd15: sin_val = 8'sd126;
            6'd16: sin_val = 8'sd127;
            6'd17: sin_val = 8'sd126;
            6'd18: sin_val = 8'sd125;
            6'd19: sin_val = 8'sd122;
            6'd20: sin_val = 8'sd117;
            6'd21: sin_val = 8'sd112;
            6'd22: sin_val = 8'sd106;
            6'd23: sin_val = 8'sd98 ;
            6'd24: sin_val = 8'sd90 ;
            6'd25: sin_val = 8'sd81 ;
            6'd26: sin_val = 8'sd71 ;
            6'd27: sin_val = 8'sd60 ;
            6'd28: sin_val = 8'sd49 ;
            6'd29: sin_val = 8'sd37 ;
            6'd30: sin_val = 8'sd25 ;
            6'd31: sin_val = 8'sd12 ;
            6'd32: sin_val = 8'sd0  ;
            6'd33: sin_val = -8'sd12 ;
            6'd34: sin_val = -8'sd25 ;
            6'd35: sin_val = -8'sd37 ;
            6'd36: sin_val = -8'sd49 ;
            6'd37: sin_val = -8'sd60 ;
            6'd38: sin_val = -8'sd71 ;
            6'd39: sin_val = -8'sd81 ;
            6'd40: sin_val = -8'sd90 ;
            6'd41: sin_val = -8'sd98 ;
            6'd42: sin_val = -8'sd106;
            6'd43: sin_val = -8'sd112;
            6'd44: sin_val = -8'sd117;
            6'd45: sin_val = -8'sd122;
            6'd46: sin_val = -8'sd125;
            6'd47: sin_val = -8'sd126;
            6'd48: sin_val = -8'sd127;
            6'd49: sin_val = -8'sd126;
            6'd50: sin_val = -8'sd125;
            6'd51: sin_val = -8'sd122;
            6'd52: sin_val = -8'sd117;
            6'd53: sin_val = -8'sd112;
            6'd54: sin_val = -8'sd106;
            6'd55: sin_val = -8'sd98 ;
            6'd56: sin_val = -8'sd90 ;
            6'd57: sin_val = -8'sd81 ;
            6'd58: sin_val = -8'sd71 ;
            6'd59: sin_val = -8'sd60 ;
            6'd60: sin_val = -8'sd49 ;
            6'd61: sin_val = -8'sd37 ;
            6'd62: sin_val = -8'sd25 ;
            default: sin_val = -8'sd12;
        endcase
    end

    // ---------------- 各曲线位置 ----------------
    // 正弦：sin_val(-127..127) × amp / 128
    wire signed [31:0] pos_sin = ($signed(sin_val) * amp) >>> 7;

    // 三角：phase 高段折返成 0..32768，再以 16384 为中心映射到 ±amp
    wire        [15:0] tri_u   = phase[15] ? (16'd65535 - phase + 16'd1) : phase;
    wire signed [31:0] tri_s   = $signed({1'b0, tri_u}) - 32'sd16384;
    wire signed [31:0] pos_tri = ($signed(tri_s) * amp) >>> 14;

    // S 形：三角波一阶低通（1/8），把速度的方波磨成梯形
    wire signed [31:0] tri_filt_next = tri_filt + ((pos_tri - tri_filt) >>> 3);

    // 匀速：单向线性累加
    wire signed [31:0] pos_lin_next =
        (pos_lin > POS_LIMIT) ? pos_lin :
        (pos_lin < -POS_LIMIT) ? pos_lin : (pos_lin + V_CONST);

    // ---------------- 曲线选择 ----------------
    reg signed [31:0] pos_sel;
    always @(*) begin
        case(mode)
            2'd0:    pos_sel = pos_sin;
            2'd1:    pos_sel = pos_tri;
            2'd2:    pos_sel = tri_filt_next;
            default: pos_sel = pos_lin_next;
        endcase
    end

    // ---------------- 时序 ----------------
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            phase     <= 16'd0;
            amp       <= AMP_INIT;
            pos_lin   <= 32'sd0;
            tri_filt  <= 32'sd0;
            pos_ref   <= 32'sd0;
            en_d      <= 1'b0;
        end else begin
            en_d <= en;

            // 幅度调节（不受 en 限制，停止时也能调）
            if(amp_up && (amp < AMP_MAX))
                amp <= amp + AMP_STEP;
            if(amp_dn && (amp > AMP_MIN))
                amp <= amp - AMP_STEP;

            if(tick_5ms) begin
                if(!en) begin
                    // 未运行：清零内部状态，下次 en 上升沿从头开始
                    phase    <= 16'd0;
                    pos_lin  <= 32'sd0;
                    tri_filt <= 32'sd0;
                    pos_ref  <= 32'sd0;
                end else begin
                    // 运行：相位推进 + 各曲线状态更新
                    phase    <= phase + PHASE_STEP;
                    pos_lin  <= pos_lin_next;
                    tri_filt <= tri_filt_next;
                    pos_ref  <= pos_sel;
                end
            end
        end
    end

endmodule

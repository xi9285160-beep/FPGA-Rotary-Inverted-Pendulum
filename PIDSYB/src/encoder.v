// ============================================================================
// encoder.v  —  正交编码器四倍频解码
// 与 J280 Interface_test_new/src/encoder.v 完全一致：
//   - A/B 相先打两拍同步
//   - state_last 在第二个 always 块中更新
//   - 四倍频判方向，16-bit 位置计数
//   - 初值 10000（与 IP570 STM32 源码一致）
// ============================================================================
module encoder(
    input  wire        clk_50m,
    input  wire        rst_n,
    input  wire        enc_a,
    input  wire        enc_b,
    output wire [15:0] pos
);

    assign pos = pos_reg;

    reg [1:0]  enc_sync_a;
    reg [1:0]  enc_sync_b;
    reg [1:0]  state_now;
    reg [1:0]  state_last;
    reg [15:0] pos_reg;

    // ---------------- A/B 相两级同步 ----------------
    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n) begin
            enc_sync_a <= 2'd0;
            enc_sync_b <= 2'd0;
            state_now  <= 2'd0;
        end else begin
            enc_sync_a <= {enc_sync_a[0], enc_a};
            enc_sync_b <= {enc_sync_b[0], enc_b};
            state_now  <= {enc_sync_a[1], enc_sync_b[1]};
        end
    end

    // ---------------- 四倍频方向判断 + 位置累加 ----------------
    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n) begin
            pos_reg    <= 16'd10000;
            state_last <= 2'b00;
        end else begin
            state_last <= state_now;

            case({state_last, state_now})
                // 正转（计数减小）
                4'b00_01: pos_reg <= pos_reg - 16'd1;
                4'b01_11: pos_reg <= pos_reg - 16'd1;
                4'b11_10: pos_reg <= pos_reg - 16'd1;
                4'b10_00: pos_reg <= pos_reg - 16'd1;
                // 反转（计数增大）
                4'b00_10: pos_reg <= pos_reg + 16'd1;
                4'b10_11: pos_reg <= pos_reg + 16'd1;
                4'b11_01: pos_reg <= pos_reg + 16'd1;
                4'b01_00: pos_reg <= pos_reg + 16'd1;
                default:  pos_reg <= pos_reg;
            endcase
        end
    end

endmodule

// ============================================================================
// uart_cmd.v  —  串口命令帧解析（只解析「写参数」）
//
// 帧格式（9 字节）：
//   AA 55 01 | ADDR | DATA_H DATA_L | CHK | 0D 0A
//     AA 55  : 帧头
//     01     : 命令码（写参数）
//     ADDR   : 8-bit 参数地址（见 pendulum_ctrl.v 的地址表）
//     DATA   : 16-bit 参数值（大端）
//     CHK    : 字节 2..5 的异或和（即 01^ADDR^DATA_H^DATA_L）
//     0D 0A  : 帧尾
//
// 出错处理：
//   - 帧头不同步 -> 逐字节滑动重新搜索 AA 55
//   - 校验/帧尾错误 -> 丢弃该帧，err_cnt 加 1（用于网页端显示）
//   - 未知地址 -> 不输出 we（由 pendulum_ctrl 自行忽略）
//
// 输出：param_we 为单周期脉冲，此时 param_addr/param_data 有效。
// ============================================================================
module uart_cmd (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [7:0]  rx_byte,
    input  wire        rx_byte_valid,

    output reg  [7:0]  param_addr,
    output reg  [15:0] param_data,
    output reg         param_we,

    output reg  [7:0]  err_cnt,      // 帧错误计数（饱和）
    output reg  [7:0]  ok_cnt        // 成功写入计数（饱和）
);

    localparam [7:0] HDR0 = 8'hAA;
    localparam [7:0] HDR1 = 8'h55;
    localparam [7:0] CMD_WRITE = 8'h01;
    localparam [7:0] TAIL0 = 8'h0D;
    localparam [7:0] TAIL1 = 8'h0A;

    // 帧内已收字节数（0..8）
    reg [3:0] idx;

    // 暂存
    reg [7:0]  b_cmd;
    reg [7:0]  b_addr;
    reg [7:0]  b_dh;
    reg [7:0]  b_dl;
    reg [7:0]  b_chk;

    wire [7:0] chk_calc = b_cmd ^ b_addr ^ b_dh ^ b_dl;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            idx        <= 4'd0;
            b_cmd      <= 8'd0;
            b_addr     <= 8'd0;
            b_dh       <= 8'd0;
            b_dl       <= 8'd0;
            b_chk      <= 8'd0;
            param_addr <= 8'd0;
            param_data <= 16'd0;
            param_we   <= 1'b0;
            err_cnt    <= 8'd0;
            ok_cnt     <= 8'd0;
        end else begin
            param_we <= 1'b0;

            if(rx_byte_valid) begin
                case(idx)
                    // ---------- 帧头 0 ----------
                    4'd0: begin
                        if(rx_byte == HDR0) idx <= 4'd1;
                    end
                    // ---------- 帧头 1 ----------
                    4'd1: begin
                        if(rx_byte == HDR1) idx <= 4'd2;
                        else if(rx_byte == HDR0) idx <= 4'd1;   // AA AA 55 也能对齐
                        else idx <= 4'd0;
                    end
                    // ---------- 命令码 ----------
                    4'd2: begin
                        if(rx_byte == CMD_WRITE) begin
                            b_cmd <= rx_byte;
                            idx   <= 4'd3;
                        end else begin
                            idx <= 4'd0;
                        end
                    end
                    // ---------- 地址 ----------
                    4'd3: begin b_addr <= rx_byte; idx <= 4'd4; end
                    // ---------- 数据高字节 ----------
                    4'd4: begin b_dh   <= rx_byte; idx <= 4'd5; end
                    // ---------- 数据低字节 ----------
                    4'd5: begin b_dl   <= rx_byte; idx <= 4'd6; end
                    // ---------- 校验和 ----------
                    4'd6: begin b_chk  <= rx_byte; idx <= 4'd7; end
                    // ---------- 帧尾 0D ----------
                    4'd7: begin
                        if(rx_byte == TAIL0) idx <= 4'd8;
                        else begin
                            if(err_cnt != 8'hFF) err_cnt <= err_cnt + 8'd1;
                            idx <= 4'd0;
                        end
                    end
                    // ---------- 帧尾 0A -> 提交 ----------
                    4'd8: begin
                        idx <= 4'd0;
                        if(rx_byte == TAIL1) begin
                            if(b_chk == chk_calc) begin
                                param_addr <= b_addr;
                                param_data <= {b_dh, b_dl};
                                param_we   <= 1'b1;
                                if(ok_cnt != 8'hFF) ok_cnt <= ok_cnt + 8'd1;
                            end else begin
                                if(err_cnt != 8'hFF) err_cnt <= err_cnt + 8'd1;
                            end
                        end else begin
                            if(err_cnt != 8'hFF) err_cnt <= err_cnt + 8'd1;
                        end
                    end
                    default: idx <= 4'd0;
                endcase
            end
        end
    end

endmodule

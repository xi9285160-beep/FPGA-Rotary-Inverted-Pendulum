// ============================================================================
// uart_tx.v  —  8N1 串口发送（参数化波特率）
// 与 J280 Interface_test_new/src/uart_tx.v 功能一致，改为 start/done 握手，
// 便于上层逐字节发送。
// ============================================================================
module uart_tx #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer UART_BPS = 115200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] tx_data,
    input  wire       tx_start,   // 单周期脉冲
    output reg        tx,
    output reg        tx_busy,
    output reg        tx_done     // 发送完一个字节的单周期脉冲
);

    localparam integer BAUD_CNT = CLK_FREQ / UART_BPS;

    reg [15:0] baud_cnt;
    reg [3:0]  bit_state;   // 0=空闲, 1=起始位, 2..9=数据位, 10=停止位
    reg [7:0]  data_reg;
    reg [3:0]  bit_idx;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            tx        <= 1'b1;
            tx_busy   <= 1'b0;
            tx_done   <= 1'b0;
            baud_cnt  <= 16'd0;
            bit_state <= 4'd0;
            data_reg  <= 8'd0;
            bit_idx   <= 4'd0;
        end else begin
            tx_done <= 1'b0;

            if(bit_state == 4'd0) begin
                // 空闲
                tx      <= 1'b1;
                tx_busy <= 1'b0;
                if(tx_start) begin
                    data_reg  <= tx_data;
                    bit_idx   <= 4'd0;
                    tx        <= 1'b0;       // 起始位
                    baud_cnt  <= 16'd0;
                    bit_state <= 4'd1;
                    tx_busy   <= 1'b1;
                end
            end else begin
                tx_busy <= 1'b1;
                if(baud_cnt >= BAUD_CNT - 1) begin
                    baud_cnt <= 16'd0;
                    if(bit_idx < 4'd8) begin
                        tx      <= data_reg[bit_idx[2:0]];
                        bit_idx <= bit_idx + 4'd1;
                    end else if(bit_idx == 4'd8) begin
                        tx      <= 1'b1;     // 停止位
                        bit_idx <= 4'd9;
                    end else begin
                        // 停止位结束
                        tx        <= 1'b1;
                        bit_state <= 4'd0;
                        tx_busy   <= 1'b0;
                        tx_done   <= 1'b1;
                    end
                end else begin
                    baud_cnt <= baud_cnt + 16'd1;
                end
            end
        end
    end

endmodule

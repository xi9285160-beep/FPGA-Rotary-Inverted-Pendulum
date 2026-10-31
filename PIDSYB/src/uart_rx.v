// ============================================================================
// uart_rx.v  —  8N1 串口接收（逐字节输出）
//
// 采样策略：不做过采样累加，直接用「位中点对齐」的精确计数。
//   BAUD_CNT = CLK_FREQ / UART_BPS（115200 @50MHz -> 434）
//   起始位下降沿 -> 等 BAUD_CNT/2 到起始位中点 -> 校验为低
//   之后每 BAUD_CNT 个时钟采一位，共 8 个数据位 + 1 个停止位
//   每位误差为 0（整数分频），10 位累计偏移 < 0.5 位，采样点始终在中心附近。
//
// 说明：115200 时 BAUD_CNT=434，实际波特率 50e6/434 = 115207，误差 0.006%，可忽略。
// ============================================================================
module uart_rx #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer UART_BPS = 115200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,

    output reg  [7:0] rx_data,     // 收到的字节
    output reg        rx_valid     // 单周期有效脉冲
);

    localparam integer BAUD_CNT = CLK_FREQ / UART_BPS;
    localparam integer HALF_CNT = BAUD_CNT / 2;

    // ---------------- 输入同步（异步串口必须打两拍）----------------
    reg [2:0] rx_sync;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) rx_sync <= 3'b111;
        else       rx_sync <= {rx_sync[1:0], rx};
    end
    wire rx_d = rx_sync[2];

    // ---------------- 接收状态机 ----------------
    localparam [1:0] S_IDLE  = 2'd0;   // 等起始位
    localparam [1:0] S_START = 2'd1;   // 等到起始位中点
    localparam [1:0] S_DATA  = 2'd2;   // 采 8 数据位 + 1 停止位

    reg [1:0]  st;
    reg [15:0] cnt;
    reg [3:0]  bit_idx;
    reg [7:0]  sh;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            st       <= S_IDLE;
            cnt      <= 16'd0;
            bit_idx  <= 4'd0;
            sh       <= 8'd0;
            rx_data  <= 8'd0;
            rx_valid <= 1'b0;
        end else begin
            rx_valid <= 1'b0;

            case(st)
                // ---------- 等起始位下降沿 ----------
                S_IDLE: begin
                    if(!rx_d) begin
                        cnt <= HALF_CNT[15:0];   // 到起始位中点
                        st  <= S_START;
                    end
                end

                // ---------- 确认起始位仍为低，开始采数据 ----------
                S_START: begin
                    if(cnt == 16'd0) begin
                        if(!rx_d) begin           // 确认是真起始位
                            cnt     <= BAUD_CNT[15:0];
                            bit_idx <= 4'd0;
                            st      <= S_DATA;
                        end else begin
                            st <= S_IDLE;         // 毛刺，放弃
                        end
                    end else begin
                        cnt <= cnt - 16'd1;
                    end
                end

                // ---------- 采 8 数据位 + 停止位 ----------
                S_DATA: begin
                    if(cnt == 16'd0) begin
                        cnt <= BAUD_CNT[15:0];
                        if(bit_idx == 4'd8) begin
                            // 停止位已采到，输出整字节
                            rx_data  <= sh;
                            rx_valid <= 1'b1;
                            st       <= S_IDLE;
                        end else begin
                            sh      <= {rx_d, sh[7:1]};   // LSB 先到
                            bit_idx <= bit_idx + 4'd1;
                        end
                    end else begin
                        cnt <= cnt - 16'd1;
                    end
                end

                default: st <= S_IDLE;
            endcase
        end
    end

endmodule

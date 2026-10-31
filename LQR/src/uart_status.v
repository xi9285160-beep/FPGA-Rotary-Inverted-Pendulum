// ============================================================================
// uart_status.v  —  周期性串口状态上报
// 每 20ms 发送一帧（9600 8N1，12 字节 = 12.5ms，留 7.5ms 空闲）：
//   AA 55 | {state,angle} | enc | pwm | {flag,angle_mid} | 0D 0A
//   bytes 2-3 : 高 4 位 = 状态机状态(0~3)，低 12 位 = ADC 角度原值
//   bytes 4-5 : 16-bit 摆臂编码器
//   bytes 6-7 : 16-bit 有符号控制输出 pwm_cmd
//   bytes 8-9 : 高 4 位 = {3'b0, db_flag}，低 12 位 = 标定后的竖直点 angle_mid
//               db_flag = 1 表示当前落在平衡死区内
// ============================================================================
module uart_status #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer UART_BPS = 115200
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick_20ms,
    input  wire [11:0]        angle,
    input  wire [15:0]        enc1,
    input  wire signed [15:0] pwm_cmd,
    input  wire [3:0]         state_dbg,
    input  wire [11:0]        angle_mid,
    input  wire               db_flag,
    output wire               uart_tx
);

    localparam [3:0] FRAME_LEN = 4'd12;

    // 发送前锁存，保证一帧数据一致
    reg [11:0]        angle_lat;
    reg [15:0]        enc1_lat;
    reg signed [15:0] pwm_lat;
    reg [3:0]         state_lat;
    reg [11:0]        mid_lat;
    reg               dbf_lat;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            angle_lat <= 12'd0;
            enc1_lat  <= 16'd0;
            pwm_lat   <= 16'sd0;
            state_lat <= 4'd0;
            mid_lat   <= 12'd0;
            dbf_lat   <= 1'b0;
        end else if(tick_20ms) begin
            angle_lat <= angle;
            enc1_lat  <= enc1;
            pwm_lat   <= pwm_cmd;
            state_lat <= state_dbg;
            mid_lat   <= angle_mid;
            dbf_lat   <= db_flag;
        end
    end

    // 高 4 位放状态机状态（angle 只占 12 位），低 12 位是 ADC 原值
    wire [15:0] angle_tx = {state_lat, angle_lat};
    // 高 4 位放死区标志，低 12 位是标定后的竖直点
    wire [15:0] mid_tx   = {3'b000, dbf_lat, mid_lat};

    // ---------------- 发送状态机寄存器 ----------------
    reg [3:0] idx;
    reg [1:0] st;            // 0=等待, 1=装载并启动, 2=等待发送完成

    // ---------------- 帧内容 ----------------
    reg [7:0] frame_byte;
    always @(*) begin
        case(idx)
            4'd0:  frame_byte = 8'hAA;
            4'd1:  frame_byte = 8'h55;
            4'd2:  frame_byte = angle_tx[15:8];
            4'd3:  frame_byte = angle_tx[7:0];
            4'd4:  frame_byte = enc1_lat[15:8];  // 摆臂编码器
            4'd5:  frame_byte = enc1_lat[7:0];
            4'd6:  frame_byte = pwm_lat[15:8];   // 控制输出（有符号 16-bit）
            4'd7:  frame_byte = pwm_lat[7:0];
            4'd8:  frame_byte = mid_tx[15:8];    // {db_flag, angle_mid}
            4'd9:  frame_byte = mid_tx[7:0];
            4'd10: frame_byte = 8'h0D;
            4'd11: frame_byte = 8'h0A;
            default: frame_byte = 8'h00;
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
            idx      <= 4'd0;
            st       <= 2'd0;
            tx_data  <= 8'd0;
            tx_start <= 1'b0;
        end else begin
            tx_start <= 1'b0;     // 默认只脉冲一拍
            case(st)
                2'd0: begin
                    if(tick_20ms) begin
                        idx <= 4'd0;
                        st  <= 2'd1;
                    end
                end
                2'd1: begin
                    if(idx < FRAME_LEN) begin
                        tx_data  <= frame_byte;
                        tx_start <= 1'b1;
                        idx      <= idx + 4'd1;
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

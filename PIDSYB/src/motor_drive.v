// ============================================================================
// motor_drive.v  —  TB6612 单通道电机驱动（带符号 PWM）
// 输入 pwm_cmd：有符号 16-bit，正 = 正转，负 = 反转，0 = 停止
// 输出 ain1/ain2/pwm_out 接 TB6612 的 AIN1/AIN2/PWMA
//
// TB6612 逻辑：
//   AIN1 AIN2
//    0    0   停止
//    1    0   正转 (CW)
//    0    1   反转 (CCW)
//    1    1   短路刹车
//
// PWM 频率：50MHz / 5400 ≈ 9.26kHz
// PWM_INV = 1 时交换正反转方向（上机调试用）
// ============================================================================
module motor_drive #(
    parameter integer PWM_INV = 0,
    parameter [12:0]  PWM_PERIOD = 13'd5400
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire signed [15:0] pwm_cmd,
    output reg                ain1,
    output reg                ain2,
    output wire               pwm_out
);

    // ---------------- PWM 载波计数器 ----------------
    reg [12:0] pwm_cnt;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)
            pwm_cnt <= 13'd0;
        else if(pwm_cnt >= PWM_PERIOD - 13'd1)
            pwm_cnt <= 13'd0;
        else
            pwm_cnt <= pwm_cnt + 13'd1;
    end

    // ---------------- 绝对值（占空比） ----------------
    wire [15:0] mag = pwm_cmd[15] ? (~pwm_cmd + 16'd1) : pwm_cmd;

    // ---------------- 方向逻辑 ----------------
    wire dir_pos = (pwm_cmd > 0);
    wire dir_neg = (pwm_cmd < 0);

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            ain1 <= 1'b0;
            ain2 <= 1'b0;
        end else begin
            if(PWM_INV != 0) begin
                ain1 <= dir_neg;
                ain2 <= dir_pos;
            end else begin
                ain1 <= dir_pos;
                ain2 <= dir_neg;
            end
        end
    end

    // ---------------- PWM 输出 ----------------
    assign pwm_out = (pwm_cnt < mag) ? 1'b1 : 1'b0;

endmodule

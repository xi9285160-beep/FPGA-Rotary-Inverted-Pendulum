// ============================================================================
// key_filter.v  —  按键消抖
// 移植自 J280 Interface_test_new/src/key_filter.v
//    - 按键低有效（按下 = 0）
//    - key_flag ：连续保持低电平 CNT_MAX 个时钟后，输出一个时钟周期的高脉冲（用于单次触发）
//    - key_level：消抖后的电平，0 = 按住，1 = 松开（用于“按住持续动作”）
//    - 默认 CNT_MAX = 1_000_000 -> 50MHz 下约 20ms
// ============================================================================
module key_filter #(
    parameter [19:0] CNT_MAX = 20'd1_000_000
)(
    input  wire sys_clk,
    input  wire sys_rst_n,
    input  wire key_in,
    output reg  key_flag,     // 按下瞬间的单脉冲
    output reg  key_level     // 消抖后的电平：0 = 按住，1 = 松开
);

    reg [19:0] cnt;

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if(!sys_rst_n)
            cnt <= 20'd0;
        else if(key_in == 1'b1)          // 松开，计数清零
            cnt <= 20'd0;
        else if(cnt == CNT_MAX)          // 保持按下，计数饱和
            cnt <= cnt;
        else
            cnt <= cnt + 20'd1;
    end

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if(!sys_rst_n) begin
            key_flag  <= 1'b0;
            key_level <= 1'b1;
        end else begin
            // 单脉冲：只在消抖计满的前一拍输出一次
            if(cnt == CNT_MAX - 20'd1)
                key_flag <= 1'b1;
            else
                key_flag <= 1'b0;

            // 电平：稳定按住后为 0（cnt 饱和在 CNT_MAX），松开立即回 1
            if(key_in == 1'b1)
                key_level <= 1'b1;
            else if(cnt == CNT_MAX)
                key_level <= 1'b0;
        end
    end

endmodule

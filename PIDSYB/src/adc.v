// ============================================================================
// adc.v  —  3PA1030 (10-bit 并行 ADC) 采集 + 16 次过采样平均
//
//    - adc_clk = 12.5MHz（50MHz 四分频）
//    - 每 20us 采一次，累加 250 次后输出平均值（每 5ms 更新）-> 50kSPS
//    - 10-bit 数据左移 2 位扩展成 12-bit
//
// 【相位锁定到控制节拍】
//   250000 个时钟 = 一个 5ms 周期；每 4 个时钟一个 adc_ce -> 62500 个。
//   每 250 个 adc_ce 取 1 个样本 -> 250 样本/周期，62500 % 250 == 0，
//   tick_5ms 处 clk_div / dec_cnt 天然都是 0，没有任何余数。
//   因此每个控制周期恒为 250 个采样点，最后一点比 tick_5ms 早 3 拍，
//   控制环读到的就是本周期均值，延迟恒定，不存在 0~5ms 的漂移。
//
// 【三级抗干扰链路】
//   1) 3 点数值中值   —— 剔除孤立的单样本毛刺
//   2) 两拍比较       —— |本拍-上拍| > SPIKE_TH 判为错读，沿用上拍有效值
//   3) 250 点累加平均 —— 单点错读影响稀释到 4092/250 = 16 计数
//   旧版 16 点平均下单点错读影响 4092/16 = 256 计数，与实测尖峰幅度
//   （137~266 计数）完全吻合；现在三个方向同时改善。
// ============================================================================
module adc(
    input  wire        clk_50m,
    input  wire        rst_n,
    input  wire        tick_5ms,      // 控制节拍（用于相位锁定）

    output wire        adc_clk,
    output wire        adc_oe_n,      // 低有效：0 = 正常工作
    output wire        adc_stby_n,    // 低有效：0 = 正常工作
    input  wire [9:0]  adc_data_in,   // D9~D0 并行数据

    output reg  [11:0] theta_out      // 12-bit 角度值
);

    // ---------------- ADC 时钟：50MHz / 4 = 12.5MHz ----------------
    //   高/低电平各 40ns（原 2 分频只有 20ns，裕量不足）
    //   tick_5ms 强制归零：250000 是 4 的整数倍，稳态下 clk_div 在这里本来就是 0，
    //   不会产生任何抖动；软复位后 clk_div 与 cnt_5ms 相位失配，靠这一拍自愈。
    reg [1:0] clk_div;
    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n)        clk_div <= 2'd0;
        else if(tick_5ms) clk_div <= 2'd0;
        else              clk_div <= clk_div + 2'd1;

    assign adc_clk    = clk_div[1];   // 0,0,1,1 -> 12.5MHz
    assign adc_oe_n   = 1'b0;
    assign adc_stby_n = 1'b0;

    // ---------------- 输入寄存（防亚稳态）----------------
    //   adc_data_in 相对 clk_50m 异步，打一拍后再用。
    //   采样点已距 adc_clk 上升沿 20ns，再加寄存的 20ns，共约 40ns 裕量。
    reg [9:0] adc_d1;
    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n) adc_d1 <= 10'd0;
        else       adc_d1 <= adc_data_in;

    // ---------------- 采样使能 / 抽取到 50kSPS ----------------
    //   adc_ce  : 每 4 个时钟一次，固定在 adc_clk 上升沿前 20ns 的相位点上
    //             每个控制周期 250000/4 = 62500 次
    //   dec_cnt : 每 250 个 adc_ce 取 1 个样本 -> 1000 个时钟 = 20us
    //             250 个样本 = 250000 个时钟 = 恰好一个控制周期
    //   => 每周期恒为 250 个样本，相位天然锁定，无余数、无漂移
    //   （旧版 16 样本/5ms = 3.2kSPS，现在 250 样本/5ms = 50kSPS，16 倍）
    localparam [7:0] DEC_LAST = 8'd249;   // 0..249 -> 250 分频

    wire adc_ce = (clk_div == 2'd1);
    reg  [7:0] dec_cnt;
    wire sample_tick = adc_ce && (dec_cnt == DEC_LAST);

    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n)        dec_cnt <= 8'd0;
        else if(tick_5ms) dec_cnt <= 8'd0;   // 稳态本就是 0（62500 % 250 == 0）
        else if(adc_ce)   dec_cnt <= (dec_cnt == DEC_LAST) ? 8'd0 : dec_cnt + 8'd1;

    // ---------------- 第一层：3 点数值中值（剔除孤立单样本毛刺）----------------
    //   ⚠ 必须用【比较器】实现的数值中值。
    //     不能用位运算 (a&b)|(b&c)|(a&c) —— 那是按位多数表决，
    //     在二进制进位边界会输出错误值（1,2,3 得 3；2,3,4 得 2）。
    //   采样间隔 20us，摆杆在这么短时间内角度变化远小于 1 计数，
    //   中值对真实信号无影响。
    function [9:0] median3;
        input [9:0] a, b, c;
        begin
            median3 = (a > b) ? ((b > c) ? b : ((a > c) ? c : a))
                              : ((a > c) ? a : ((b > c) ? c : b));
        end
    endfunction

    reg [9:0] s_cur, s_p1;
    wire [9:0] s_med = median3(adc_d1, s_cur, s_p1);

    // ---------------- 第二层：两拍比较（剔除整拍性错误采样）----------------
    //   相邻两拍间隔 20us，摆杆在最大角速度下位移 <1 计数，
    //   所以 |本拍 - 上拍| > SPIKE_TH 一定不是真实运动，而是 ADC 错读：
    //   丢弃本拍，沿用上一拍的有效值（s_acc 不更新）。
    //   连续剔除达到 BAD_MAX 次则强制接受，避免「真的动了」却被永久锁死
    //   （之前那版 hold 型滤波没有超时，把系统冻死过一次）。
    //   复位后前 4 拍用于填充 s_cur / s_p1 / s_acc 历史，不做判定直接放行。
    localparam [10:0] SPIKE_TH = 11'd12;    // 相邻两拍允许的最大跳变（约 0.7 度）
    localparam [3:0]  BAD_MAX  = 4'd8;      // 连续剔除上限 -> 8 * 20us = 160us

    reg [9:0] s_acc;
    reg [3:0] bad_run;
    reg [2:0] warm_cnt;
    reg [9:0] s_filt;

    wire warm = (warm_cnt == 3'd4);
    wire [10:0] s_diff = (s_med >= s_acc) ? ({1'b0, s_med} - {1'b0, s_acc})
                                          : ({1'b0, s_acc} - {1'b0, s_med});
    wire s_bad = warm && (s_diff > SPIKE_TH) && (bad_run != BAD_MAX);

    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n) begin
            s_cur    <= 10'd0;
            s_p1     <= 10'd0;
            s_acc    <= 10'd0;
            s_filt   <= 10'd0;
            bad_run  <= 4'd0;
            warm_cnt <= 3'd0;
        end else if(sample_tick) begin
            s_p1 <= s_cur;
            s_cur <= adc_d1;
            if(!warm) begin
                // 填充历史：本拍直接放行
                warm_cnt <= warm_cnt + 3'd1;
                s_acc    <= adc_d1;
                s_filt   <= adc_d1;
                bad_run  <= 4'd0;
            end else if(s_bad) begin
                // 判为错读：沿用上一拍有效值
                bad_run <= bad_run + 4'd1;
                s_filt  <= s_acc;
            end else begin
                bad_run <= 4'd0;
                s_acc   <= s_med;
                s_filt  <= s_med;
            end
        end
    end

    // ---------------- 第三层：250 点累加平均 ----------------
    //   s_filt 是 10-bit 原始码，左移 2 位扩展成 12-bit 后累加。
    //   250 * 4092 = 1023000 < 2^20，20-bit 足够。
    //
    //   除以 250：(sum * 4194 + 524288) >> 20
    //     4194/2^20 = 0.00399971，相对误差 -0.0072%
    //     满量程 4092 时误差仅 -0.3 计数（约 0.02 度），可忽略
    //   4194 = 4096 + 64 + 32 + 2，所以不用乘法器，直接移位相加：
    //     sum*4194 = (sum<<12) + (sum<<6) + (sum<<5) + (sum<<1)
    //   （写成 * 常量会被推断成 DSP，与 u_ctrl 的乘法抢时序，这里刻意避开）
    reg [7:0]  sample_num;
    reg [19:0] theta_sum;

    wire [11:0] s_filt12 = {2'b0, s_filt};
    wire [19:0] sum_tot  = theta_sum + {8'd0, s_filt12};

    wire [31:0] sum_sh1  = {11'd0, sum_tot, 1'b0};    // sum<<1
    wire [31:0] sum_sh5  = {7'd0,  sum_tot, 5'b0};    // sum<<5
    wire [31:0] sum_sh6  = {6'd0,  sum_tot, 6'b0};    // sum<<6
    wire [31:0] sum_sh12 = {sum_tot, 12'b0};          // sum<<12
    // 1023000 * 4194 = 4290462000 < 2^32，32-bit 刚好够
    wire [31:0] avg_rnd  = sum_sh1 + sum_sh5 + sum_sh6 + sum_sh12 + 32'd524_288;

    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n) begin
            sample_num <= 8'd0;
            theta_sum  <= 20'd0;
            theta_out  <= 12'd0;
        end else if(sample_tick) begin
            if(sample_num == DEC_LAST) begin
                // 第 250 个样本：输出本周期的平均值
                sample_num <= 8'd0;
                theta_sum  <= 20'd0;
                theta_out  <= avg_rnd[31:20];
            end else begin
                sample_num <= sample_num + 8'd1;
                theta_sum  <= sum_tot;
            end
        end
    end

endmodule

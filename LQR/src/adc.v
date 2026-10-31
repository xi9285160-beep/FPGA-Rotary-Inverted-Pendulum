// ============================================================================
// adc.v  —  3PA1030 (10-bit 并行 ADC) 采集 + 16 次过采样平均
// 移植自 J280 Interface_test_new/src/adc.v
//    - adc_clk = 25MHz（50MHz 二分频）
//    - 每 187.52us 采一次（9376 个 50MHz 时钟 = 4688 个 ADC 周期，整数倍），
//      累加 16 次后输出平均值（每 3.0ms 更新）
//    - 10-bit 数据左移 2 位扩展成 12-bit：theta_out = sum[16] >> 4
//
// 为什么采样间隔必须是 ADC 周期的整数倍，见下面 SAMPLE_CNT_MAX 的注释。
// ============================================================================
module adc #(
    // 采样间隔：50MHz 时钟个数 - 1。必须是 ADC 时钟周期（2 个 50MHz 时钟）的整数倍，
    // 否则采样相位会每拍漂移（历史 bug，见下方 SAMPLE_CNT_MAX 处注释）。
    //   14'd1951 -> 1952 个时钟 = 976 个 ADC 周期（偶数）
    //   128 次 × 1952 = 249856 个时钟 = 4.997ms，刚好在一个 5ms 控制周期内跑完
    parameter [13:0]  SAMPLE_CNT_MAX = 14'd1_951,
    // 采样相位：0 = 在 adc_clk 上升沿（ADC 更新输出的那一拍）取样，读到的是上一次转换结果
    //           1 = adc_clk 反相，采样点后移 20ns，采到刚转换完的新值
    // 两个相位哪个噪声小取决于 ADC 的 t_OD，实测看哪个好就用哪个。
    parameter integer PHASE_INV = 0
)(
    input  wire        clk_50m,
    input  wire        rst_n,
    input  wire        sync,          // 控制周期同步信号（5ms），用于重启累加器

    output wire        adc_clk,
    output wire        adc_oe_n,      // 低有效：0 = 正常工作
    output wire        adc_stby_n,    // 低有效：0 = 正常工作
    input  wire [9:0]  adc_data_in,   // D9~D0 并行数据

    output reg  [11:0] theta_out      // 12-bit 角度值
);

    localparam [7:0] NSAMP = 8'd128;   // 每个控制周期平均的样本数

    // ---------------- ADC 时钟：50MHz / 2 = 25MHz ----------------
    reg clk_div;
    always @(posedge clk_50m or negedge rst_n)
        if(!rst_n) clk_div <= 1'b0;
        else       clk_div <= ~clk_div;

    assign adc_clk    = PHASE_INV ? ~clk_div : clk_div;
    assign adc_oe_n   = 1'b0;
    assign adc_stby_n = 1'b0;

    // ---- 10-bit 并行总线两级同步（打两拍）----
    //   ADC 的输出数据在 adc_clk 之后 t_OD 才更新，相对 clk_50m 是异步的，
    //   直接进累加器有亚稳态风险（偶发错位/野值）。打两拍可以把亚稳态
    //   隔离在第一级，第二级拿到的必是稳定值。
    //   代价：采样点提前 2 个时钟 = 40ns = 恰好 1 个 ADC 周期，
    //   相位不变，所以不影响“周期是偶数个 ADC 周期”的约束。
    reg [9:0] adc_s1;
    reg [9:0] adc_s2;
    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n) begin
            adc_s1 <= 10'd0;
            adc_s2 <= 10'd0;
        end else begin
            adc_s1 <= adc_data_in;   // 第一级：可能亚稳态
            adc_s2 <= adc_s1;        // 第二级：已稳定
        end
    end

    // 10-bit -> 12-bit（用同步后的值）
    wire [11:0] theta_12bit = {adc_s2, 2'b0};

    // ---------------- 16 次过采样平均 ----------------
    //
    // 【关键】采样间隔必须是 ADC 时钟周期的整数倍，否则采样相位会每拍漂移。
    //   ADC 时钟 = 25MHz（周期 40ns = 2 个 50MHz 时钟）
    //   旧值 9375 个 50MHz 时钟 = 4687.5 个 ADC 周期 —— 带半拍！
    //     → 采样点在「ADC 正在更新输出」和「输出已稳定」两个相位之间来回切换，
    //       一半的采样落在并行数据总线翻转的瞬间，读回来的值忽大忽小。
    //   新值 9376 个 50MHz 时钟 = 4688 个 ADC 周期（偶数）
    //     → 相位固定，每次都在同一个 adc_clk 上升沿相位取样，
    //       读到的是稳定的上一次转换结果（数据已保持 40ns - t_OD）。
    //   187.52us × 16 = 3.0003ms，与原来基本一致。
    //   （SAMPLE_CNT_MAX 实际是“最大计数值”，周期 = SAMPLE_CNT_MAX + 1 个时钟）

    reg [13:0] cnt_sample;
    reg [7:0]  sample_num;
    reg [19:0] theta_sum;      // 128 * 4092 = 523776，需要 20-bit

    // 累加和（含最后一拍），最大 128*4*1023 = 523776，21-bit 足够
    wire [20:0] theta_sum_all = theta_sum + theta_12bit;

    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n) begin
            cnt_sample <= 14'd0;
            sample_num <= 8'd0;
            theta_sum  <= 20'd0;
            theta_out  <= 12'd0;
        end else if(sync) begin
            // 每个 5ms 控制周期重新开一个窗口，保证一周期恰好 NSAMP 个样本，
            // 且输出就在控制节拍前 144 个时钟更新（无随机相位差）。
            cnt_sample <= 14'd0;
            sample_num <= 8'd0;
            theta_sum  <= 20'd0;
        end else if(cnt_sample == SAMPLE_CNT_MAX) begin
            cnt_sample <= 14'd0;
            theta_sum  <= theta_sum + theta_12bit;

            if(sample_num == NSAMP - 8'd1) begin
                // 第 NSAMP 个采样：输出平均值
                //   平均值 = sum / 128 = sum >> 7，最大值 523776>>7 = 4092，必在 12 位内
                theta_out  <= theta_sum_all[18:7];
                theta_sum  <= 20'd0;
                sample_num <= 8'd0;
            end else begin
                sample_num <= sample_num + 8'd1;
            end
        end else begin
            cnt_sample <= cnt_sample + 14'd1;
        end
    end

endmodule

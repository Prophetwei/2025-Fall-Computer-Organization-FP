
`timescale 1ns/10ps
`include "SP_pipeline.v"
`include "PATTERN.v"
`include "DATA_MEM.v"
`include "INST_MEM.v"

module TESTBED;

// wire connection
wire clk,rst_n,in_valid,out_valid,mem_wen;
wire [11:0] mem_addr,inst_addr;
wire [31:0] inst_wo_delay,mem_dout_wo_delay,mem_din;
reg [31:0] inst_delay, mem_dout_delay;
wire [31:0] inst_delay_w, mem_dout_delay_w;
assign inst_delay_w = inst_delay;
assign mem_dout_delay_w = mem_dout_delay;
integer i;     
//wire real \aaa_[0] = My_SP.r[0];


initial begin
    // $fsdbDumpfile("SP_pipeline.fsdb");
    // $fsdbDumpvars(0,"+mda");
    $dumpfile("SP_pipeline.vcd");
    $dumpvars(0,My_SP);
	for(i = 0;i < 32;i = i + 1) begin
        $dumpvars(0, My_SP.r[i]);
	end
end

SP_pipeline My_SP(
    .clk(clk),
    .rst_n(rst_n),
    .in_valid(in_valid),
    .out_valid(out_valid),
    .mem_wen(mem_wen),
    .inst(inst_delay_w),
    .mem_dout(mem_dout_delay_w),
    .inst_addr(inst_addr),
    .mem_addr(mem_addr),
    .mem_din(mem_din)
);

DATA_MEM My_MEM(
    .Q(mem_dout_wo_delay),
    .CLK(clk),
    .CEN(1'b0),
    .WEN(mem_wen),
    .A(mem_addr),
    .D(mem_din),
    .OEN(1'b0)
);

INST_MEM My_INST(
    .Q(inst_wo_delay),
    .CLK(clk),
    .CEN(~in_valid),
    .WEN(1'b1),
    .A(inst_addr),
    .D(),
    .OEN(1'b0)
);

PATTERN My_PATTERN(
    .clk(clk),
    .rst_n(rst_n),
    .in_valid(in_valid),
    .out_valid(out_valid)
);

always @(posedge clk) begin
    inst_delay <= inst_wo_delay;
    mem_dout_delay <= mem_dout_wo_delay;
end



endmodule
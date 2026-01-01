module SP_pipeline(
    // INPUT SIGNAL
	clk,
	rst_n,
	in_valid,
	inst,
	mem_dout,
	// OUTPUT SIGNAL
	out_valid,
	inst_addr,
	mem_wen,
	mem_addr,
	mem_din
);

//------------------------------------------------------------------------
//   INPUT AND OUTPUT DECLARATION                         
//------------------------------------------------------------------------

input                    clk, rst_n, in_valid;
input             [31:0] inst;
input  signed     [31:0] mem_dout;
output reg               out_valid;
output reg        [11:0] inst_addr;
output reg               mem_wen;
output reg        [11:0] mem_addr;
output reg signed [31:0] mem_din;


//==============================================//
//                 parameter                    //
//==============================================//
parameter inst_width = 32;
 
//==============================================//
//    REGISTER FILE, DO NOT EDIT THE NAME.      //
//==============================================//
reg	signed [31:0] r [0:31]; 

//==============================================//
//           reg & wire declaration             //
//==============================================//

reg in_valid_IF, in_valid_ID, in_valid_EX, in_valid_MEM, in_valid_WB;

//==============================================//
//                  design                      //
//==============================================//
always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		in_valid_IF <= 1'b0;
		in_valid_ID <= 1'b0;
		in_valid_EX <= 1'b0;
		in_valid_MEM <= 1'b0;
		in_valid_WB <= 1'b0;
		out_valid <= 1'b0;
	end
	else begin
		in_valid_IF <= in_valid;
		in_valid_ID <= in_valid_IF;
		in_valid_EX <= in_valid_ID;
		in_valid_MEM <= in_valid_EX;
		in_valid_WB <= in_valid_MEM;
		out_valid <= in_valid_WB;
	end
end
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		inst_addr <= 0;
	end
	else if (in_valid) begin
		inst_addr <= inst_addr + 1;
	end
end


//IF stage
wire [6:0] opcode_reg, funct_reg;
wire [4:0] rs_reg, rt_reg, rd_reg, shamt_reg;
wire [15:0] immediate_reg;
assign opcode_reg = inst[31:26];
assign rs_reg = inst[25:21];
assign rt_reg =  inst[20:16];
assign rd_reg = inst[15:11];
assign shamt_reg = inst[10:6];
assign funct_reg = inst[5:0];
assign immediate_reg = inst[15:0];

//IF/ID register
reg [6:0] opcode, funct;
reg [4:0] rs, rt, rd, shamt;
reg [15:0] immediate;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		opcode <= 0;
		funct <= 0;
		rs <= 0;
		rt <= 0;
		rd <= 0;
		shamt <= 0;
		immediate <= 0;
	end
	else if (in_valid_IF) begin
		opcode <= opcode_reg;
		funct <= funct_reg;
		rs <= rs_reg;
		rt <= rt_reg;
		rd <= rd_reg;
		shamt <= shamt_reg;
		immediate <= immediate_reg;
	end
end

//ID stage
reg signed [31:0] op1_reg, op2_reg;
wire signed [31:0] sign_ex_imm_reg;
wire        [31:0] zero_ex_imm_reg;
wire        [31:0] upper_ex_imm_reg;
wire signed [31:0] mem_addr_reg;

always @(*) begin
	case(rs)
		5'b00000: op1_reg = r[0];
		5'b00001: op1_reg = r[1];
		5'b00010: op1_reg = r[2];
		5'b00011: op1_reg = r[3];
		5'b00100: op1_reg = r[4];
		5'b00101: op1_reg = r[5];
		5'b00110: op1_reg = r[6];
		5'b00111: op1_reg = r[7];
		5'b01000: op1_reg = r[8];
		5'b01001: op1_reg = r[9];
		5'b01010: op1_reg = r[10];
		5'b01011: op1_reg = r[11];
		5'b01100: op1_reg = r[12];
		5'b01101: op1_reg = r[13];
		5'b01110: op1_reg = r[14];
		5'b01111: op1_reg = r[15];
		5'b10000: op1_reg = r[16];
		5'b10001: op1_reg = r[17];
		5'b10010: op1_reg = r[18];
		5'b10011: op1_reg = r[19];
		5'b10100: op1_reg = r[20];
		5'b10101: op1_reg = r[21];
		5'b10110: op1_reg = r[22];
		5'b10111: op1_reg = r[23];
		5'b11000: op1_reg = r[24];
		5'b11001: op1_reg = r[25];
		5'b11010: op1_reg = r[26];
		5'b11011: op1_reg = r[27];
		5'b11100: op1_reg = r[28];
		5'b11101: op1_reg = r[29];
		5'b11110: op1_reg = r[30];
		5'b11111: op1_reg = r[31];
	endcase
	case(rt)
		5'b00000: op2_reg = r[0];
		5'b00001: op2_reg = r[1];
		5'b00010: op2_reg = r[2];
		5'b00011: op2_reg = r[3];
		5'b00100: op2_reg = r[4];
		5'b00101: op2_reg = r[5];
		5'b00110: op2_reg = r[6];
		5'b00111: op2_reg = r[7];
		5'b01000: op2_reg = r[8];
		5'b01001: op2_reg = r[9];
		5'b01010: op2_reg = r[10];
		5'b01011: op2_reg = r[11];
		5'b01100: op2_reg = r[12];
		5'b01101: op2_reg = r[13];
		5'b01110: op2_reg = r[14];
		5'b01111: op2_reg = r[15];
		5'b10000: op2_reg = r[16];
		5'b10001: op2_reg = r[17];
		5'b10010: op2_reg = r[18];
		5'b10011: op2_reg = r[19];
		5'b10100: op2_reg = r[20];
		5'b10101: op2_reg = r[21];
		5'b10110: op2_reg = r[22];
		5'b10111: op2_reg = r[23];
		5'b11000: op2_reg = r[24];
		5'b11001: op2_reg = r[25];
		5'b11010: op2_reg = r[26];
		5'b11011: op2_reg = r[27];
		5'b11100: op2_reg = r[28];
		5'b11101: op2_reg = r[29];
		5'b11110: op2_reg = r[30];
		5'b11111: op2_reg = r[31];
	endcase
end

assign sign_ex_imm_reg = {{16{immediate[15]}}, immediate};
assign zero_ex_imm_reg = {16'b0, immediate};
assign upper_ex_imm_reg = {immediate, 16'b0};
assign mem_addr_reg = op1_reg + sign_ex_imm_reg;

//ID/EX register
reg signed [31:0] op1, op2;
reg        [5:0]  opcode_ex, funct_ex;
reg signed [31:0] sign_ex_imm;
reg        [31:0] zero_ex_imm;
reg        [31:0] upper_ex_imm;
reg        [4:0]  rt_ex, rd_ex;
reg        [4:0]  shamt_ex;

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		op1 <= 0;
		op2 <= 0;
		opcode_ex <= 0;
		funct_ex <= 0;
		sign_ex_imm <= 0;
		zero_ex_imm <= 0;
		upper_ex_imm <= 0;
		rt_ex <= 0;
		rd_ex <= 0;
		shamt_ex <= 0;
		mem_addr <= 0;
	end
	else if (in_valid_ID) begin
		op1 <= op1_reg;
		op2 <= op2_reg;
		opcode_ex <= opcode;
		funct_ex <= funct;
		sign_ex_imm <= sign_ex_imm_reg;
		zero_ex_imm <= zero_ex_imm_reg;
		upper_ex_imm <= upper_ex_imm_reg;
		rt_ex <= rt;
		rd_ex <= rd;
		shamt_ex <= shamt;
		mem_addr <= mem_addr_reg[11:0];
	end
end

//EX stage
reg signed [31:0] ALUresult;
wire       [4:0]  dst_reg;
wire              readmem_reg;
wire              writemem_reg;
wire              writereg_reg;

assign dst_reg = (opcode_ex == 0) ? rd_ex : rt_ex;
assign readmem_reg = (opcode_ex == 6'd5);
assign writemem_reg = !(opcode_ex == 6'd6);
assign writereg_reg = (opcode_ex != 6'd6);

always @(*) begin
	if (opcode_ex == 0) begin
		case(funct)
			6'd0: ALUresult = op1 & op2;
			6'd1: ALUresult = op1 | op2;
			6'd2: ALUresult = op1 + op2;
			6'd3: ALUresult = op1 - op2;
			6'd4: ALUresult = (op1 < op2) ? 1 : 0;
			6'd5: ALUresult = op1 << shamt_ex;
			6'd6: ALUresult = ~(op1 | op2);
			default: ALUresult = 0;
		endcase
	end
	else begin
		case(opcode_ex)
			6'd1: ALUresult = op1 & zero_ex_imm;
			6'd2: ALUresult = op1 | zero_ex_imm;
			6'd3: ALUresult = op1 + sign_ex_imm;
			6'd4: ALUresult = op1 - sign_ex_imm;
			6'd5: ALUresult = op1 + sign_ex_imm;
			6'd6: ALUresult = op1 + sign_ex_imm;
			6'd9: ALUresult = upper_ex_imm;
			default: ALUresult = 0;
		endcase
	end
end

//EX/MEM register
reg signed [31:0] ALUresult_mem;
reg        [4:0]  dst_mem;
reg               readmem;
reg               writereg;

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		ALUresult_mem <= 0;
		dst_mem <= 0;
		readmem <= 0;
		writereg <= 0;
		mem_din <= 0;
		mem_wen <= 1;
	end
	else if (in_valid_EX) begin
		ALUresult_mem <= ALUresult;
		dst_mem <= dst_reg;
		readmem <= readmem_reg;
		writereg <= writereg_reg;
		mem_din <= ALUresult;
		mem_wen <= writemem_reg;
	end
end

//MEM stage
wire signed [31:0] read_data_reg;

assign read_data_reg = (readmem) ? mem_dout : 0;

//MEM/WB register
reg signed [31:0] read_data;
reg signed [31:0] ALUresult_wb;
reg        [4:0]  dst_wb;
reg               writereg_wb;
reg	   			  readmem_wb;

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		read_data <= 0;
		ALUresult_wb <= 0;
		dst_wb <= 0;
		writereg_wb <= 0;
		readmem_wb <= 0;
	end
	else if (in_valid_MEM) begin
		read_data <= read_data_reg;
		ALUresult_wb <= ALUresult_mem;
		dst_wb <= dst_mem;
		writereg_wb <= writereg;
		readmem_wb <= readmem;
	end
end

//WB stage
wire signed [31:0] write_data;

assign write_data = (readmem_wb) ? read_data : ALUresult_wb;
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		for (integer i = 0; i < 32; i = i + 1) begin
			r[i] <= 32'b0;
		end
	end
	else if (writereg_wb) begin
		case(dst_wb)
			5'b00000: r[0] = write_data;
			5'b00001: r[1] = write_data;
			5'b00010: r[2] = write_data;
			5'b00011: r[3] = write_data;
			5'b00100: r[4] = write_data;
			5'b00101: r[5] = write_data;
			5'b00110: r[6] = write_data;
			5'b00111: r[7] = write_data;
			5'b01000: r[8] = write_data;
			5'b01001: r[9] = write_data;
			5'b01010: r[10] = write_data;
			5'b01011: r[11] = write_data;
			5'b01100: r[12] = write_data;
			5'b01101: r[13] = write_data;
			5'b01110: r[14] = write_data;
			5'b01111: r[15] = write_data;
			5'b10000: r[16] = write_data;
			5'b10001: r[17] = write_data;
			5'b10010: r[18] = write_data;
			5'b10011: r[19] = write_data;
			5'b10100: r[20] = write_data;
			5'b10101: r[21] = write_data;
			5'b10110: r[22] = write_data;
			5'b10111: r[23] = write_data;
			5'b11000: r[24] = write_data;
			5'b11001: r[25] = write_data;
			5'b11010: r[26] = write_data;
			5'b11011: r[27] = write_data;
			5'b11100: r[28] = write_data;
			5'b11101: r[29] = write_data;
			5'b11110: r[30] = write_data;
			5'b11111: r[31] = write_data;
		endcase
	end
end

endmodule
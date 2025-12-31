`define CYCLE_TIME 10
`timescale 1ns/10ps
module PATTERN(
    // Output Signals
    clk,
    rst_n,
    in_valid,
    // Input Signals
    out_valid
);

//================================================================
//   Input and Output Declaration                         
//================================================================

output reg clk,rst_n,in_valid;
input wire out_valid;

//================================================================
// parameters & integer
//================================================================

integer execution_num=3700,out_max_latency=10,seed=64;
integer i,t,latency,out_valid_counter,in_valid_counter,cur_inst_addr,golden_inst_addr_out;
integer opcode,rs,rt,rd,shamt,func,immediate, address, temp_addr;
integer instruction [4095:0];
integer golden_r [31:0];
integer mem [4095:0];
integer file_r_log;

//================================================================
// clock setting
//================================================================

real CYCLE = `CYCLE_TIME;

always #(CYCLE/2.0) clk = ~clk;

//================================================================
// initial
//================================================================

initial begin
    // file_r_log = $fopen("debug_output_reg.log", "r");
    // read data mem & instrction
    $readmemh("instruction.txt",instruction);
    $readmemh("data.txt",mem);

    // initialize control signal 
    rst_n=1'b1;
    in_valid=1'b0;

    // initial variable
    cur_inst_addr = 0;
    golden_inst_addr_out = 0;
    in_valid_counter = 0;
    out_valid_counter = 0;
    latency = -1;
    for(i=0;i<32;i=i+1)begin
        golden_r[i]=0;
    end

    // reset check task
    reset_check_task;

    // generate random idle clk
	t=$random(seed)%3+1'b1;
	repeat(t) @(negedge clk);

    // main pattern
	while(out_valid_counter<execution_num)begin
		input_task;
        check_ans_task;
        @(negedge clk);
	end

    // check memory and out_valid
    check_memory_and_out_valid;
    display_pass_task;

end
//================================================================
// task
//================================================================

// reset check task
task reset_check_task; begin
    // force clk
    force clk=0;

    // generate reset signal
    #CYCLE; rst_n=1'b0;
    #CYCLE; rst_n=1'b1;

    // check output signal=0
    if(out_valid!==1'b0)begin
        $display("************************************************************");     
        $display("*  Output signal should be 0 after initial RESET  at %8t   *",$time);
        $display("************************************************************");
        repeat(2) #CYCLE;
        $finish;

    end

    // check r
    for(i=0;i<32;i=i+1)begin
        if(My_SP.r[i]!==32'd0)begin
            $display("************************************************************");     
            $display("  Register r should be 0 after initial RESET  at %8t  ",$time);
            $display("************************************************************");
            repeat(2) #CYCLE;
            $finish;
        end
    end

    // release clk
    #CYCLE; release clk;

end
endtask

// input task
task input_task;
begin
    if(in_valid_counter < execution_num)begin
        in_valid = 1'b1;   
        in_valid_counter = in_valid_counter + 1;
    end
    else begin
        in_valid = 1'b0;
    end
end
endtask

// check_ans_task
task check_ans_task; begin
    // check out_valid
    if(out_valid)begin
        // pc update
        cur_inst_addr = golden_inst_addr_out;
        golden_inst_addr_out = golden_inst_addr_out + 1;

        // answer calculate
        opcode      = instruction[cur_inst_addr][31:26];
        rs          = instruction[cur_inst_addr][25:21];
        rt          = instruction[cur_inst_addr][20:16];
        rd          = instruction[cur_inst_addr][15:11];
        shamt       = instruction[cur_inst_addr][10:6];
        func        = instruction[cur_inst_addr][5:0];
        immediate   = instruction[cur_inst_addr][15:0];
        address     = instruction[cur_inst_addr][25:0];
        temp_addr   = golden_r[rs]+immediate;

        // sign extension (except andi, ori)
        if(immediate[15]==1'b1&&opcode!=6'd1&&opcode!=6'd2)begin
            immediate={16'hffff,immediate[15:0]};
        end

        // execute instruction
        if(opcode==6'd0)begin
            // R-type
            if(func==6'd0)begin
                // and
                golden_r[rd]=golden_r[rs]&golden_r[rt];
            end
            else if(func==6'd1)begin
                // or
                golden_r[rd]=golden_r[rs]|golden_r[rt];
            end
            else if(func==6'd2)begin
                // add
                golden_r[rd]=golden_r[rs]+golden_r[rt];
            end
            else if(func==6'd3)begin
                // sub
                golden_r[rd]=golden_r[rs]-golden_r[rt];
            end
            else if(func==6'd4)begin
                // slt
                golden_r[rd]=(golden_r[rs]<golden_r[rt])?32'd1:32'd0;
            end
            else if(func==6'd5)begin
                // sll
                golden_r[rd]=golden_r[rs]<<shamt;
            end
			else if(func==6'd6)begin
				//nor
				golden_r[rd]=~(golden_r[rs]|golden_r[rt]);
			end
            else begin
                $display("Unknown R-type func code @%d: %h",out_valid_counter,instruction[cur_inst_addr]);
            end
        end
        else begin
            // I-type
            if(opcode==6'd1)begin
                // andi
                golden_r[rt]=golden_r[rs]&immediate;
            end
            else if(opcode==6'd2)begin
                // ori
                golden_r[rt]=golden_r[rs]|immediate;
            end
            else if(opcode==6'd3)begin
                // addi
                golden_r[rt]=golden_r[rs]+immediate;
            end
            else if(opcode==6'd4)begin
                // subi
                golden_r[rt]=golden_r[rs]-immediate;
            end
            else if(opcode==6'd5)begin
                // lw
                if(golden_r[rs]+immediate<0 || golden_r[rs]+immediate >= 4096)begin
                    $display("Mem_Addr overflow @%d",out_valid_counter);
                    $finish;
                end
                golden_r[rt]=mem[golden_r[rs]+immediate];
                
            end
            else if(opcode==6'd6)begin
                // sw
                mem[golden_r[rs]+immediate]=golden_r[rt];
				if(golden_r[rs]+immediate<0 || golden_r[rs]+immediate >= 4096)begin
                    $display("Mem_Addr overflow @%d",out_valid_counter);
                    $finish;
				end
            end
			else if(opcode==6'd9)begin
                // lui
                golden_r[rt]={immediate,16'd0};
			end     
            else begin
                $display("Unknown I-type opcode code @%d: %h",out_valid_counter,instruction[cur_inst_addr]);
            end
        end


        check_register_task;
        // check_log_task;
        $display("\033[0;34m*                        Pass PATTERN NO.%4d 	                  *\033[0m",out_valid_counter);

        out_valid_counter = out_valid_counter + 1;
    end
    else begin
        // check execution cycle
        if(out_valid_counter==0)begin
            latency=latency+1;
            if(latency==out_max_latency)begin
                $display("***************************************************");     
                $display("   the execution cycles are more than 10 cycles  ",$time);
                $display("***************************************************");
                repeat(2) @(negedge clk);
                $finish;
            end
        end
        // check out_valid pulled down
        else begin
            $display("************************************************************");     
            $display("  out_valid should not fall when executing  at %8t  ",$time);
            $display("************************************************************");
            repeat(2) #CYCLE;
            $finish;
        end
    end

end
endtask

// check register task
task check_register_task; 
begin
    for(i=0;i<32;i=i+1)begin
        if(My_SP.r[i]!==golden_r[i])begin
            display_fail_task;
            $display("Instrucion@%4d: %h",cur_inst_addr, instruction[cur_inst_addr]);
            $display("      curr_pc  = %4d", cur_inst_addr);
            $display("      opcode   = %4d", opcode       );
            $display("      rs       = %4d", rs           );
            $display("      rt       = %4d", rt           );
            $display("      rd       = %4d", rd           );
            $display("      func     = %4d", func         );
            $display("      imm      = %4d", immediate    );
            // $display("      addr     = %4h", address      );
            $display("      load from / store to    = %4d", temp_addr);
            $display("-------------------------------------------------------------------");
            $display("                     Register r[%2d]  error                        ",i);
            $display("          answer should be : %0d , your answer is : %0d            ",golden_r[i],My_SP.r[i]);
            $display("-------------------------------------------------------------------");
            print_reg_task;
            repeat(2) @(negedge clk);
            $finish;
        end
    end
end
endtask

// print register task
task print_reg_task; begin
    $display ("     **********************  Register File ****************************");
    $display ("      r0 = %12d,  golden_r[ 0] = %12d |        r16 = %12d,  golden_r[16] = %12d", My_SP.r[0], golden_r[0], My_SP.r[16], golden_r[16]);
    $display ("      r1 = %12d,  golden_r[ 1] = %12d |        r17 = %12d,  golden_r[17] = %12d", My_SP.r[1], golden_r[1], My_SP.r[17], golden_r[17]);
    $display ("      r2 = %12d,  golden_r[ 2] = %12d |        r18 = %12d,  golden_r[18] = %12d", My_SP.r[2], golden_r[2], My_SP.r[18], golden_r[18]);
    $display ("      r3 = %12d,  golden_r[ 3] = %12d |        r19 = %12d,  golden_r[19] = %12d", My_SP.r[3], golden_r[3], My_SP.r[19], golden_r[19]);
    $display ("      r4 = %12d,  golden_r[ 4] = %12d |        r20 = %12d,  golden_r[20] = %12d", My_SP.r[4], golden_r[4], My_SP.r[20], golden_r[20]);
    $display ("      r5 = %12d,  golden_r[ 5] = %12d |        r21 = %12d,  golden_r[21] = %12d", My_SP.r[5], golden_r[5], My_SP.r[21], golden_r[21]);
    $display ("      r6 = %12d,  golden_r[ 6] = %12d |        r22 = %12d,  golden_r[22] = %12d", My_SP.r[6], golden_r[6], My_SP.r[22], golden_r[22]);
    $display ("      r7 = %12d,  golden_r[ 7] = %12d |        r23 = %12d,  golden_r[23] = %12d", My_SP.r[7], golden_r[7], My_SP.r[23], golden_r[23]);
    $display ("      r8 = %12d,  golden_r[ 8] = %12d |        r24 = %12d,  golden_r[24] = %12d", My_SP.r[8], golden_r[8], My_SP.r[24], golden_r[24]);
    $display ("      r9 = %12d,  golden_r[ 9] = %12d |        r25 = %12d,  golden_r[25] = %12d", My_SP.r[9], golden_r[9], My_SP.r[25], golden_r[25]);
    $display ("     r10 = %12d,  golden_r[10] = %12d |        r26 = %12d,  golden_r[26] = %12d", My_SP.r[10], golden_r[10], My_SP.r[26], golden_r[26]);
    $display ("     r11 = %12d,  golden_r[11] = %12d |        r27 = %12d,  golden_r[27] = %12d", My_SP.r[11], golden_r[11], My_SP.r[27], golden_r[27]);
    $display ("     r12 = %12d,  golden_r[12] = %12d |        r28 = %12d,  golden_r[28] = %12d", My_SP.r[12], golden_r[12], My_SP.r[28], golden_r[28]);
    $display ("     r13 = %12d,  golden_r[13] = %12d |        r29 = %12d,  golden_r[29] = %12d", My_SP.r[13], golden_r[13], My_SP.r[29], golden_r[29]);
    $display ("     r14 = %12d,  golden_r[14] = %12d |        r30 = %12d,  golden_r[30] = %12d", My_SP.r[14], golden_r[14], My_SP.r[30], golden_r[30]);
    $display ("     r15 = %12d,  golden_r[15] = %12d |        r31 = %12d,  golden_r[31] = %12d", My_SP.r[15], golden_r[15], My_SP.r[31], golden_r[31]);
end
endtask


// check_memory_and_out_valid
task check_memory_and_out_valid; begin
    // check memory
    for(i=0;i<4096;i=i+1)begin
        if(My_MEM.mem[i]!==mem[i])begin
            display_fail_task;
            $display("-------------------------------------------------------------------");
            $display("                      MEM [%4d]  error                   ",i);
            $display("           answer should be : %d , your answer is : %d        ",mem[i],My_MEM.mem[i]);
            $display("-------------------------------------------------------------------");
            repeat(2) @(negedge clk);
            $finish;
        end
    end

    // check out_valid
    if(out_valid==1'b1)begin
        $display("************************************************************");     
        $display("*  out_valid should be low after finish execute at %8t  *",$time);
        $display("************************************************************");
        repeat(2) #CYCLE;
        $finish;
    end

end
endtask

// display fail task
task display_fail_task; begin

        $display("\n");
        $display("        ----------------------------               ");
        $display("        --                        --       |\__||  ");
        $display("        --  OOPS!!                --      / X,X  | ");
        $display("        --                        --    /_____   | ");
        $display("        --  \033[0;31mSimulation Failed!!\033[m   --   /^ ^ ^ \\  |");
        $display("        --                        --  |^ ^ ^ ^ |w| ");
        $display("        ----------------------------   \\m___m__|_|");
        $display("\n");
end 
endtask

// display pass task
task display_pass_task; begin

        $display("\n");
        $display("        ----------------------------               ");
        $display("        --                        --       |\__||  ");
        $display("        --  Congratulations !!    --      / O.O  | ");
        $display("        --                        --    /_____   | ");
        $display("        --  \033[0;32mSimulation PASS!!\033[m     --   /^ ^ ^ \\  |");
        $display("        --                        --  |^ ^ ^ ^ |w| ");
        $display("        ----------------------------   \\m___m__|_|");
        $display("\n");
		repeat(2) @(negedge clk);
		$finish;

end 
endtask

endmodule
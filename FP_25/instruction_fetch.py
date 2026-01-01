def decode_to_file(input_path, output_path):
    # Mapping based on Project Specification Table 
    r_type_funct = {
        0x00: "and", 0x01: "or", 0x02: "add", 
        0x03: "sub", 0x04: "slt", 0x05: "sll", 0x06: "nor"
    }
    i_type_opcode = {
        0x01: "andi", 0x02: "ori", 0x03: "addi", 
        0x04: "subi", 0x05: "lw", 0x06: "sw", 0x09: "lui"
    }

    try:
        with open(input_path, 'r') as f_in, open(output_path, 'w') as f_out:
            # Header for the output file
            f_out.write(f"{'Addr':<6} {'Hex':<10} {'Inst':<6} {'Op':<4} {'rs':<4} {'rt':<4} {'rd':<4} {'funct':<6} {'imm':<8}\n")
            f_out.write("-" * 65 + "\n")
            
            for addr, line in enumerate(f_in):
                hex_str = line.strip()
                if not hex_str: continue
                
                # Convert hex to integer
                instr = int(hex_str, 16)
                
                # Bit field extraction 
                opcode = (instr >> 26) & 0x3F  # 
                rs     = (instr >> 21) & 0x1F  # 
                rt     = (instr >> 16) & 0x1F  # 
                rd     = (instr >> 11) & 0x1F  # 
                shamt  = (instr >> 6)  & 0x1F  # 
                funct  = instr & 0x3F         # 
                imm    = instr & 0xFFFF       # 
                
                # Handle Sign Extension (SE) for display [cite: 15]
                signed_imm = imm if imm < 0x8000 else imm - 0x10000

                if opcode == 0:
                    name = r_type_funct.get(funct, "???")
                    # R-type format 
                    f_out.write(f"{addr:<6} {hex_str:<10} {name:<6} {opcode:<4} {rs:<4} {rt:<4} {rd:<4} {hex(funct):<6} {'n/a':<8}\n")
                else:
                    name = i_type_opcode.get(opcode, "???")
                    # I-type format 
                    f_out.write(f"{addr:<6} {hex_str:<10} {name:<6} {opcode:<4} {rs:<4} {rt:<4} {'n/a':<4} {'n/a':<6} {signed_imm:<8}\n")

        print(f"Successfully decoded instructions to {output_path}")

    except FileNotFoundError:
        print(f"Error: {input_path} not found.")

# Run the decoder
decode_to_file("instruction.txt", "decoded_results.txt")
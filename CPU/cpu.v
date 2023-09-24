//Author: Saravnan S V (EE20B117)
//Module: Single Cycle CPU (ALU, LOAD, STORE and BRANCH Instuction Compatible) based on RISC V ISA
//Course: EE2003 Computer Organization
//Last Modified: 08-10-2022

module cpu (
    input clk, 
    input reset,
    output reg [31:0] iaddr,
  input [31:0] idata, // comes from the imem module
    output reg [31:0] daddr,
  input [31:0] drdata, //comes from the dmem module 
    output reg [31:0] dwdata,
    output reg [3:0] dwe
);
  /* instantiations - needed only for synthesis; not for simulation as testbench instantiates by itself
	imem im(
        .iaddr(iaddr),
        .idata(idata)
    );

    dmem dm(
        .clk(clk),
        .daddr(daddr),
        .drdata(drdata),
        .dwdata(dwdata),
        .dwe(dwe)
    );*/ 
  
  reg [31:0] int_reg [0:31]; //Register File (RF) consisting of 32 reigsters (each 32 bit long)

  //to handle jump offset
  reg [31:0] jmp_off;
  reg is_jmp;
  reg [20:0] offset;

  //to decode the instruction
  reg [6:0] opcode;
  reg [2:0] funct3;
  reg [3:0] funct;
  reg [1:0] quad;
  reg [4:0] rd_addr;
  reg [4:0] rs1_addr;
  reg [31:0] rs1_data;
  reg [4:0] rs2_addr;
  reg [31:0] rs2_data;
  
  integer i;
  
  //to handle signed operations; it is unsigned by default
  reg signed [31:0] rs1_sdata;
  reg signed [31:0] rs2_sdata; 
  reg signed [4:0] scomp;

  reg [31:0] write_reg;
  reg [7:0] byte_reg;
  reg [15:0] half_reg;
   
  always @(posedge clk) begin //to write, write-back at clk edges
    if (reset) begin
      iaddr <= 0; //moving the instruction pointer to the beginning of the instruction memory
      for (i=0;i<32;i=i+1) begin //Initialin=sing the Register File to zero
	      int_reg[i] <= 0;
	    end
    end 
    else begin 
      if (!is_jmp)  //Non-Branching 
        iaddr <= iaddr + 4; //Increment by 4 bytes (A word)
      else //Branching instruction
        iaddr <= iaddr + jmp_off; //Increment by the offset provided by the branch instruction
      if (rd_addr) //to ensure that register ;0' is always hard coded to 0 and does not change during the course of the program
        int_reg[rd_addr] <= write_reg; // Writing into the registeres in the next clock cycle
    end
  end
  
  always @* begin //to decode, perform ALU operations and keep data ready for write and writ-back
    //default statements 
	dwe = 4'b0;
	daddr = 32'b0;
	dwdata = 32'b0;
    opcode = idata[6:0];
    rd_addr = idata[11:7];
    write_reg = int_reg[rd_addr];
    byte_reg = 8'b0;
    half_reg = 16'b0;
    jmp_off = 32'b100;
    is_jmp = 1'b0;
    //decoding and performing correspoding operations
    case (opcode)
        7'b0110111: begin //Load Upper Immediate (LUI)
            write_reg = {{idata[31:12]},{12{1'b0}}};
        end
        7'b0010111: begin //Add Upper Immediate to PC (AUIPC)
            write_reg = {{idata[31:12]},{12{1'b0}}};
            write_reg = write_reg + iaddr;
        end
        7'b1101111: begin //Jump And Link (JAL)
            is_jmp = 1'b1;
            //offset = {idata[31],idata[19:12],idata[20],idata[30:21],1'b0};
            //jmp_off = {{11{offset[20]}},offset};
            jmp_off = {{12{idata[31]}},idata[19:12],idata[20],idata[30:21],1'b0};
            write_reg = iaddr + 4;
        end
        7'b1100111: begin //Jump and Link Register (JALR)
            is_jmp = 1'b1;
            jmp_off = {{20{idata[31]}},idata[31:20]};
            jmp_off = jmp_off + int_reg[idata[19:15]];
            jmp_off = jmp_off & 32'hfffffffe;
            jmp_off = jmp_off - iaddr;
            write_reg = iaddr + 4;
        end
        7'b1100011: begin //Branch Instrucion Set
            is_jmp = 1'b1;
            funct3 = idata[14:12];
            rs1_addr = idata[19:15];
            rs2_addr = idata[24:20];
            rs1_data = int_reg[rs1_addr];
            rs2_data = int_reg[rs2_addr];
            rs1_sdata = rs1_data;
            rs2_sdata = rs2_data;
            case (funct3)
                3'b000: begin //Branch if equal (BEQ)
                    if (rs1_data==rs2_data)
                    jmp_off = {{20{idata[31]}},idata[7],idata[30:25],idata[11:8],1'b0};
                end
                3'b001: begin //Branch if NOT equal (BEN)
                    if (rs1_data!=rs2_data)
                    jmp_off = {{20{idata[31]}},idata[7],idata[30:25],idata[11:8],1'b0};
                end
                3'b100: begin //Branch if LESS than (BLT)
                    if (rs1_data<rs2_data)
                    jmp_off = {{20{idata[31]}},idata[7],idata[30:25],idata[11:8],1'b0};
                end
                3'b101: begin //Branch if Greater than or equal to (BGE)
                    if (rs1_data>=rs2_data)
                    jmp_off = {{20{idata[31]}},idata[7],idata[30:25],idata[11:8],1'b0};
                end
                3'b110: begin //Branch if LESS than Unsigned (BLTU)
                    if (rs1_sdata<rs2_sdata)
                    jmp_off = {{20{idata[31]}},idata[7],idata[30:25],idata[11:8],1'b0};
                end
                3'b111: begin //Branch if Greater than or equal to Unsigned (BGEU)
                    if (rs1_sdata>=rs2_sdata)
                    jmp_off = {{20{idata[31]}},idata[7],idata[30:25],idata[11:8],1'b0};
                end
            endcase
        end
        7'b0000011: begin // load instruction set
            funct3 = idata[14:12];
            rs1_addr = idata[19:15];
            daddr = {{20{idata[31]}},idata[31:20]};
            daddr = (daddr + int_reg[rs1_addr]); //data[19:15]);        
            rd_addr = idata[11:7]; //idata[7]+2*idata[8]+4*idata[9]+8*idata[10]+16*idata[11];
            quad = daddr[1:0]; 
            case (funct3) // Load
                3'b100: begin // Load Byte Unsigned (LBU)
                    case (quad) 
                        2'b00: begin 
                        write_reg = drdata[7:0];
                        end
                        2'b01: begin 
                        write_reg = drdata[15:8];
                        end
                        2'b10: begin 
                        write_reg = drdata[23:16];
                        end
                        2'b11: begin 
                        write_reg = drdata[31:24];
                        end
                    endcase
                end 
                3'b000: begin // Load Byte (LB)
                    case (quad) 
                        2'b00: begin 
                        byte_reg = drdata[7:0];
                        end
                        2'b01: begin 
                        byte_reg = drdata[15:8];
                        end
                        2'b10: begin 
                        byte_reg = drdata[23:16];
                        end
                        2'b11: begin 
                        byte_reg = drdata[31:24];
                        end
                    endcase
                    write_reg = {{24{byte_reg[7]}},byte_reg}; 
                end 
                3'b101: begin //Load Half Word Unsigned (LHU)
                    case (quad)
                        2'b00: begin //Lower Half of the word   
                            write_reg = drdata[15:0];
                        end
                        2'b10: begin //Upper Half of the word
                            write_reg = drdata[31:16];
                        end
                    endcase
                end 
                3'b001: begin //Load Half Word (LH)
                    case (quad)
                        2'b00: begin //Lower Half of the word   
                            half_reg = drdata[15:0];
                        end
                        2'b10: begin //Upper Half of the word
                            half_reg = drdata[31:16];
                        end
                    endcase
                    write_reg = {{16{half_reg[15]}},half_reg};
                end 
                3'b010: begin //Load Word (LW)
                    write_reg = drdata;
                end
            endcase
        end
        7'b0100011: begin //Store Instruction set
            daddr = {{20{idata[31]}},{idata[31:25],idata[11:7]}};
            daddr = (daddr + int_reg[idata[19:15]]);        
            rs1_addr = idata[24:20]; 
            dwdata = int_reg[rs1_addr];
            quad = daddr[1:0];
	    funct3 = idata[14:12];
            case (funct3)
                3'b000: begin //Store Byte (SB)
                    case (quad)
                        2'b00: begin 
                            dwe = 4'b0001;
                        end
                        2'b01: begin 
                            dwe = 4'b0010;
                        end
                        2'b10: begin 
                            dwe = 4'b0100;
                        end
                        2'b11: begin 
                            dwe = 4'b1000;
                        end
                    endcase
                end
                3'b001: begin //Store Half Word (SH)
                    case (quad)
                        2'b00: begin 
                            dwe = 4'b0011;
                        end
                        2'b10: begin 
                            dwe = 4'b1100;
                        end
                    endcase
                end
                3'b010: begin //Store Word (SW)
                    dwe = 4'b1111;
                end
            endcase
        end
        7'b0010011: begin //Immediate ALU operations
            funct3 = idata[14:12];
            rd_addr = idata[11:7];
            rs1_addr = idata[19:15];
            rs1_data = int_reg[rs1_addr];
            rs1_sdata = rs1_data;
            scomp = idata[24:20];
            case (funct3)
                3'b000: begin //Add Immediate (ADDI)
                    write_reg = {{20{idata[31]}},idata[31:20]};  
                    write_reg = (write_reg + rs1_data); 
                end
                3'b100: begin //(XORI)
                    write_reg = {{20{idata[31]}},idata[31:20]}; 
                    write_reg = ((write_reg) ^ (rs1_data)); 
                end
                3'b110: begin //(ORI)
                    write_reg = {{20{idata[31]}},idata[31:20]};
                    write_reg = ((write_reg) | (rs1_data)); 
                end
                3'b111: begin //(ANDI)
                    write_reg = {{20{idata[31]}},idata[31:20]}; 
                    write_reg = ((write_reg) & (rs1_data)); 
                end
                3'b001: begin //Shift Left Logaical Immediate (SLLI)
                    write_reg = ((rs1_data)<<(idata[24:20]));
                end
                3'b101: begin //Shift Right Logical Immediate (SRLI) and Shift Right Arithmetic Immediate (SRAI)
                    if (!idata[30]) //SRLI
                        write_reg = ((rs1_data)>>(idata[24:20]));
                    else //SRAI
                        write_reg = (rs1_sdata>>>scomp); 
                end
                3'b010: begin //Set Less than Immediate (SLTI)
                    rs1_sdata = rs1_data;
                    rs2_sdata = rs2_data;
                    if (rs1_sdata<scomp) 
                        write_reg = 1;
                    else 
                        write_reg = 0;
                end
                3'b011: begin //Set Less than Immediate Unsigned (SLTIU)                    
                    if (rs1_data<idata[24:20]) 
                        write_reg = 1;
                    else 
                        write_reg = 0;
                end
            endcase
        end
        7'b0110011: begin //Register ALU operations
            funct3 = idata[14:12];
            funct = {funct3,idata[30]};
            rd_addr = idata[11:7];
            rs1_addr = idata[19:15];
            rs2_addr = idata[24:20];
            rs1_data = int_reg[rs1_addr];
            rs2_data = int_reg[rs2_addr];
            rs1_sdata = rs1_data;
            rs2_sdata = rs2_data;
            case (funct) 
                4'b0000: begin //(ADD)
                    write_reg = rs1_data + rs2_data;
                end
                4'b0001: begin //(SUB)
                    write_reg = rs1_data - rs2_data;
                end
                4'b1000: begin //(XOR)
                    write_reg = ((rs1_data) ^ (rs2_data)); 
                end
                4'b1100: begin //(OR)
                    write_reg = ((rs1_data) | (rs2_data));
                end
                4'b1110: begin //(AND)
                    write_reg = ((rs1_data) & (rs2_data));
                end
                4'b0010: begin //Shift Left Logical (SLL)
                    write_reg = ((rs1_data)<<(rs2_data[24:20]));
                end
                4'b1010: begin //Shift Right Logical (SRL)
                    write_reg = ((rs1_data)>>(rs2_data[24:20]));
                end
                4'b1011: begin //Shift Right Arithmetic (SRA)
                    write_reg = (rs1_sdata>>>rs2_sdata[24:20]);
                end
                4'b0100: begin //Set Less than (SLT)
                    if (rs1_sdata<rs2_sdata) 
                        write_reg = 1;
                    else 
                        write_reg = 0;
                end
                4'b0110: begin //Set Less than Unsigned(SlTU)                    
                    if (rs1_data<rs2_data) 
                        write_reg = 1;
                    else 
                        write_reg = 0;
                end
            endcase
        end
    endcase
  end
endmodule

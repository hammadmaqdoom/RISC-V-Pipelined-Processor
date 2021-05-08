// Code your design here

module Adder(
    input [63:0] a, b,
    output reg [63:0] out
);
always@(*)
    out = a + b;
endmodule

module registerFile(
    input [63:0] WriteData,
    input [4:0] RS1,
    input [4:0] RS2,
    input [4:0] RD,
    input RegWrite, clk, reset,
    output reg [63:0] ReadData1,
    output reg [63:0] ReadData2
);
reg [63:0] Registers [31:0];
initial
    begin
    Registers[0] = 64'd 0;
    Registers[1] = 64'd 0;
    Registers[2] = 64'd 0;
    Registers[3] = 64'd 0;
    Registers[4] = 64'd 0;
    Registers[5] = 64'd 0;
    Registers[6] = 64'd 0;
    Registers[7] = 64'd 0;
    Registers[8] = 64'd 0;
    Registers[9] = 64'd 0;
    Registers[10] = 64'd 0;
    Registers[11] = 64'd 0;
    Registers[12] = 64'd 0;
    Registers[13] = 64'd 0;
    Registers[14] = 64'd 0;
    Registers[15] = 64'd 0;
    Registers[16] = 64'd 0;
    Registers[17] = 64'd 0;
    Registers[18] = 64'd 0;
    Registers[19] = 64'd 0;
    Registers[20] = 64'd 0;
    Registers[21] = 64'd 0;
    Registers[22] = 64'd 0;
    Registers[23] = 64'd 0;
    Registers[24] = 64'd 0;
    Registers[25] = 64'd 0;
    Registers[26] = 64'd 0;
    Registers[27] = 64'd 0;
    Registers[28] = 64'd 0;
    Registers[29] = 64'd 0;
    Registers[30] = 64'd 0;
    Registers[31] = 64'd 0;
    end
  always @(posedge clk)
    if(RegWrite)
        begin
        Registers[RD] = WriteData;
        end
  always @(*)
    if(reset)
        begin
        ReadData1 = 64'b0;
        ReadData2 = 64'b0;
        end
    else
        begin
        ReadData1 = Registers[RS1];
        ReadData2 = Registers[RS2];
        end
endmodule

module Instruction_Parser(
    input [31:0] instruction,
    output [6:0] opcode, funct7,
    output [4:0] rd , rs1 , rs2,
    output [2:0] funct3 

);

assign opcode = instruction[6:0];
assign rd = instruction[11:7];
assign funct3 = instruction[14:12];
assign rs1 = instruction[19:15];
assign rs2 = instruction[24:20];
assign funct7 = instruction[31:25];

endmodule

module mux2x1
(
    input [63:0] a,b,
    input s ,
    output [63:0] data_out
);

assign data_out = s ? b : a;

endmodule

module data_generator
(
input [31:0] instruction,
output reg [63:0] imm_data

);

wire [6:0] opcode;
assign opcode = instruction[6:0];

always @(*)
begin
    case (opcode)
        7'b0000011: imm_data =  {{52{instruction[31]}}, instruction [31:20]};
        7'b0100011: imm_data = {{52{instruction[31]}}, instruction [31:25], instruction [11:7]};
        7'b1100011: imm_data = {{52{instruction[31]}}, instruction [31] , instruction [7], instruction [30:25], instruction [11:8]};
        7'b0010011: imm_data = {{52{instruction[31]}}, instruction[31:20]};
        default : imm_data = 64'd0;
    endcase
end

endmodule

// reg [63:0] immediate;
// wire [6:0] opcode;
// assign opcode = instruction[6:0];

// always @(instruction)
// begin
//     if(instruction[6]==0) //data transfer
// 	      if(instruction[5]==0) //load
// 	              immediate[11:0] = instruction [31:20];
// 	      else if(instruction[5]==1) //store
// 	              immediate[11:0]= {instruction[31:25],instruction[11:7]};
//     else if(instruction[6]==1) //conditional branches
//         immediate[11:0] = {instruction[31],instruction[7],instruction[30:25] , instruction[11:8]};
// end

// assign imm_data[11:0]= immediate[11:0];
// assign imm_data[63:12] = {52{instruction[31]}};


// endmodule

module selector(
    input branch, ZERO,
    input [63:0] a, b,
    input [2:0] funct3,
    output reg sel
);

always@(*)
begin
    if (branch == 1) 
        begin
            case(funct3)
            3'b001: //bne
            begin
                if(branch == 1 & ZERO == 0)
                    sel = 1;
                else
                    sel = 0;
            end
            3'b000: //beq
            begin
                if(branch == 1 & ZERO == 1)
                    sel = 1;
                else
                    sel = 0;
            end
            3'b101: //bge
            begin
                if (a >= b)
                    sel = 1;
                else
                    sel = 0;
            end
          endcase
        end
   else
	    sel = 0;
end
endmodule




module Program_Counter
(
    input clk, reset,
    input [63:0] PC_In,
    output reg [63:0] PC_Out
);

reg reset_force; // variable to force 0th value after reset

initial 
PC_Out <= 64'd0;


always @(posedge clk or posedge reset) begin
    if (reset || reset_force) begin
        PC_Out = 64'd0;
        reset_force <= 0;
        end
    
    // else if (!PCWrite) begin
    //     PC_Out = PC_Out;
    // end
    else
    PC_Out = PC_In;

end

always @(negedge reset) reset_force <= 1;

endmodule // Program_Counter

module Data_Memory(
    input [63:0] mem_addr,
    input [63:0] write_data,
    input clk, mem_write, mem_read,
  output reg [63:0] read_data,
  output [63:0] element1,
  output [63:0] element2 ,
  output [63:0] element3,
  output [63:0] element4,
  output [63:0] element5,
  output [63:0] element6,
  output [63:0] element7,
  output [63:0] element8);
reg [0:7] data_mem[63:0];
initial
begin
    data_mem[0] = 64'd0;
    data_mem[1] = 64'd0;
    data_mem[2] = 64'd0;
    data_mem[3] = 64'd0;
    data_mem[4] = 64'd0;
    data_mem[5] = 64'd0;
    data_mem[6] = 64'd0;
    data_mem[7] = 64'd0;
    data_mem[8] = 64'd0;
    data_mem[9] = 64'd0;
    data_mem[10] = 64'd0;
    data_mem[11] = 64'd0;
    data_mem[12] = 64'd0;
    data_mem[13] = 64'd0;
    data_mem[14] = 64'd0;
    data_mem[15] = 64'd0;
    data_mem[16] = 64'd0;
    data_mem[17] = 64'd0;
    data_mem[18] = 64'd0;
    data_mem[19] = 64'd0;
    data_mem[20] = 64'd0;
    data_mem[21] = 64'd0;
    data_mem[22] = 64'd0;
    data_mem[23] = 64'd0;
    data_mem[24] = 64'd0;
    data_mem[25] = 64'd0;
    data_mem[26] = 64'd0;
    data_mem[27] = 64'd0;
    data_mem[28] = 64'd0;
    data_mem[29] = 64'd0;
    data_mem[30] = 64'd0;
    data_mem[31] = 64'd0;
    data_mem[32] = 64'd0;
    data_mem[33] = 64'd0;
    data_mem[34] = 64'd0;
    data_mem[35] = 64'd0;
    data_mem[36] = 64'd0;
    data_mem[37] = 64'd0;
    data_mem[38] = 64'd0;
    data_mem[39] = 64'd0;
    data_mem[40] = 64'd0;
    data_mem[41] = 64'd0;
    data_mem[42] = 64'd0;
    data_mem[43] = 64'd0;
    data_mem[44] = 64'd0;
    data_mem[45] = 64'd0;
    data_mem[46] = 64'd0;
    data_mem[47] = 64'd0;
    data_mem[48] = 64'd0;
    data_mem[49] = 64'd0;
    data_mem[50] = 64'd0;
    data_mem[51] = 64'd0;
    data_mem[52] = 64'd0;
    data_mem[53] = 64'd0;
    data_mem[54] = 64'd0;
    data_mem[55] = 64'd0;
    data_mem[56] = 64'd0;
    data_mem[57] = 64'd0;
    data_mem[58] = 64'd0;
    data_mem[59] = 64'd0;
    data_mem[60] = 64'd0;
    data_mem[61] = 64'd0;
    data_mem[62] = 64'd0;
    data_mem[63] = 64'd0;
end
always @(negedge clk)
begin
    if (mem_write)
        begin
            data_mem[mem_addr] = write_data[7:0];
            data_mem[mem_addr+1] = write_data[15:8];
            data_mem[mem_addr+2] = write_data[23:16];
            data_mem[mem_addr+3] = write_data[31:24];
            data_mem[mem_addr+4] = write_data[39:32];
            data_mem[mem_addr+5] = write_data[47:40];
            data_mem[mem_addr+6] = write_data[55:48];
            data_mem[mem_addr+7] = write_data[63:56];
        end
end
  always @(*)
    begin
        if(mem_read)
        begin
            read_data = {data_mem[mem_addr+7],data_mem[mem_addr+6], data_mem[mem_addr+5], data_mem[mem_addr+4], data_mem[mem_addr+3], data_mem[mem_addr+2], data_mem[mem_addr+1], data_mem[mem_addr]};
        end
    end
  assign element1= {data_mem[7],data_mem[6],data_mem[5], data_mem[4], data_mem[3], data_mem[2], data_mem[1], data_mem[0]};
  assign element2= {data_mem[15],data_mem[14],data_mem[13], data_mem[12], data_mem[11], data_mem[10], data_mem[9], data_mem[8]};
  assign element3= {data_mem[23],data_mem[22],data_mem[21], data_mem[20], data_mem[19], data_mem[18], data_mem[17], data_mem[16]};
  assign element4= {data_mem[31],data_mem[30],data_mem[29], data_mem[28], data_mem[27], data_mem[26], data_mem[25], data_mem[24]};
  assign element5= {data_mem[39],data_mem[38],data_mem[37],data_mem[36], data_mem[35], data_mem[34], data_mem[33], data_mem[32]};
  assign element6= {data_mem[47], data_mem[46],data_mem[45],data_mem[44],data_mem[43], data_mem[42], data_mem[41], data_mem[40]};
  assign element7= {data_mem[55], data_mem[54],data_mem[53],data_mem[52],data_mem[51], data_mem[50], data_mem[49], data_mem[48]};
  assign element8= {data_mem[63], data_mem[62],data_mem[61],data_mem[60],data_mem[59], data_mem[58], data_mem[57], data_mem[56]};
endmodule

module Instruction_Memory(
    input [63:0] Inst_Address,
    output reg [31:0] Instruction
);
reg [7:0] inst_memory [131:0];

initial
begin
    inst_memory[0] = 8'b10010011;
    inst_memory[1] = 8'b00000010;
    inst_memory[2] = 8'b00110000;
    inst_memory[3] = 8'b00000000;
    
    inst_memory[4] = 8'b00100011;
    inst_memory[5] = 8'b00110010;
    inst_memory[6] = 8'b01010000;
    inst_memory[7] = 8'b00000000;

    inst_memory[8] = 8'b10010011;
    inst_memory[9] = 8'b00000010;
    inst_memory[10] = 8'b00100000;
    inst_memory[11] = 8'b00000000;
    
    inst_memory[12] = 8'b00100011;
    inst_memory[13] = 8'b00110110;
    inst_memory[14] = 8'b01010000;
    inst_memory[15] = 8'b00000000;
    
    inst_memory[16] = 8'b10010011;
    inst_memory[17] = 8'b00000010;
    inst_memory[18] = 8'b10100000;
    inst_memory[19] = 8'b00000000;
    
    inst_memory[20] = 8'b00100011;
    inst_memory[21] = 8'b00111010;
    inst_memory[22] = 8'b01010000;
    inst_memory[23] = 8'b00000000;
    
    inst_memory[24] = 8'b00010011;
    inst_memory[25] = 8'b00000101;
    inst_memory[26] = 8'b01000000;
    inst_memory[27] = 8'b00000000;
    
    inst_memory[28] = 8'b10010011;
    inst_memory[29] = 8'b00000101;
    inst_memory[30] = 8'b00110000;
    inst_memory[31] = 8'b00000000;
    
    inst_memory[32] = 8'b01100011;
    inst_memory[33] = 8'b00010110;
    inst_memory[34] = 8'b00000101;
    inst_memory[35] = 8'b00000000;
    
    //bne 101
    inst_memory[36] = 8'b01100011;
    inst_memory[37] = 8'b10010100;
    inst_memory[38] = 8'b00000101;
    inst_memory[39] = 8'b00000000;

    //beq 011
    inst_memory[40] = 8'b01100011;
    inst_memory[41] = 8'b00001100;
    inst_memory[42] = 8'b00000000;
    inst_memory[43] = 8'b00000100;
    
    inst_memory[44] = 8'b00010011;
    inst_memory[45] = 8'b00001001;
    inst_memory[46] = 8'b00000000;
    inst_memory[47] = 8'b00000000;
    
    inst_memory[48] = 8'b01100011;
    inst_memory[49] = 8'b00000110;
    inst_memory[50] = 8'b10111001;
    inst_memory[51] = 8'b00000100;
    
    inst_memory[52] = 8'b10110011;
    inst_memory[53] = 8'b00001001;
    inst_memory[54] = 8'b00100000;
    inst_memory[55] = 8'b00000001;
    
    inst_memory[56] = 8'b01100011;
    inst_memory[57] = 8'b10001110;
    inst_memory[58] = 8'b10111001;
    inst_memory[59] = 8'b00000010;
    
    inst_memory[60] = 8'b10010011;
    inst_memory[61] = 8'b00010010;
    inst_memory[62] = 8'b00111001;
    inst_memory[63] = 8'b00000000;
    
    inst_memory[64] = 8'b00010011;
    inst_memory[65] = 8'b10010011;
    inst_memory[66] = 8'b00111001;
    inst_memory[67] = 8'b00000000;
    
    inst_memory[68] = 8'b10110011;
    inst_memory[69] = 8'b10000010;
    inst_memory[70] = 8'b10100010;
    inst_memory[71] = 8'b00000000;
    
    inst_memory[72] = 8'b00110011;
    inst_memory[73] = 8'b00000011;
    inst_memory[74] = 8'b10100011;
    inst_memory[75] = 8'b00000000;
    
    inst_memory[76] = 8'b00000011;
    inst_memory[77] = 8'b10111110;
    inst_memory[78] = 8'b00000010;
    inst_memory[79] = 8'b00000000;
    
    inst_memory[80] = 8'b10000011;
    inst_memory[81] = 8'b00111110;
    inst_memory[82] = 8'b00000011;
    inst_memory[83] = 8'b00000000;
    //bge 111
    inst_memory[84] = 8'b01100011;
    inst_memory[85] = 8'b01011100;
    inst_memory[86] = 8'b11011110;
    inst_memory[87] = 8'b00000001;
    
    inst_memory[88] = 8'b00110011;
    inst_memory[89] = 8'b00001111;
    inst_memory[90] = 8'b11000000;
    inst_memory[91] = 8'b00000001;
    
    inst_memory[92] = 8'b00110011;
    inst_memory[93] = 8'b00001110;
    inst_memory[94] = 8'b11010000;
    inst_memory[95] = 8'b00000001;
    
    inst_memory[96] = 8'b10110011;
    inst_memory[97] = 8'b00001110;
    inst_memory[98] = 8'b11100000;
    inst_memory[99] = 8'b00000001;
    
    inst_memory[100] =8'b00100011;
    inst_memory[101] =8'b10110000;
    inst_memory[102] =8'b11000010;
    inst_memory[103] =8'b00000001;
    
    inst_memory[104] =8'b00100011;
    inst_memory[105] =8'b00110000;
    inst_memory[106] =8'b11010011;
    inst_memory[107] =8'b00000001;
    
    inst_memory[108] =8'b10010011;
    inst_memory[109] =8'b10001001;
    inst_memory[110] =8'b00011001;
    inst_memory[111] =8'b00000000;
    
    inst_memory[112] =8'b11100011;
    inst_memory[113] =8'b00000100;
    inst_memory[114] =8'b00000000;
    inst_memory[115] =8'b11111100;
    
    inst_memory[116] =8'b00010011; 
    inst_memory[117] =8'b00001001;
    inst_memory[118] =8'b00011001;
    inst_memory[119] =8'b00000000;

    inst_memory[120] =8'b11100011;
    inst_memory[121] =8'b00001100;
    inst_memory[122] =8'b00000000;
    inst_memory[123] =8'b11111010;
    
    inst_memory[124] =8'b01100011;
    inst_memory[125] =8'b00000010;   
    inst_memory[126] =8'b00000000;
    inst_memory[127] =8'b00000000;
    
    inst_memory[128] =8'b00010011;
    inst_memory[129] =8'b00000000;
    inst_memory[130] =8'b00000000;
    inst_memory[131] =8'b00000000;
    
end
always@(Inst_Address)
    Instruction = {inst_memory[Inst_Address+3],inst_memory[Inst_Address+2],inst_memory[Inst_Address+1], inst_memory[Inst_Address]};
endmodule

module Control_Unit
(
    input [6:0] Opcode,
    output reg [1:0] ALUOp,
    output reg Branch, MemRead, MemtoReg, MemWrite, ALUSrc, Regwrite
);
always @(*)
begin
    case (Opcode)
    7'b0110011: // R-type (add/sub)
        begin
            ALUSrc = 1'b0;
            MemtoReg = 1'b0;
            Regwrite = 1'b1;
            MemRead = 1'b0;
            MemWrite = 1'b0;
            Branch = 1'b0;
            ALUOp = 2'b10;
        end
    7'b0000011: // I-type (ld)
        begin
            ALUSrc = 1'b1;
            MemtoReg = 1'b1;
            Regwrite = 1'b1;
            MemRead = 1'b1;
            MemWrite = 1'b0;
            Branch = 1'b0;
            ALUOp = 2'b00;
        end
    7'b0100011: // S-type(sd)
        begin
            ALUSrc = 1'b1;
            MemtoReg = 1'bx;
            Regwrite = 1'b0;
            MemRead = 1'b0;
            MemWrite = 1'b1;
            Branch = 1'b0;
            ALUOp = 2'b00;
        end
    7'b0010011: // I-type (addi)
        begin
            ALUSrc = 1'b1;
            MemtoReg = 1'b0;
            Regwrite = 1'b1;
            MemRead = 1'b1;
            MemWrite = 1'b0;
            Branch = 1'b0;
            ALUOp = 2'b00;
        end
    7'b1100011: // SB-type (beq/bne/bge)
        begin
            ALUSrc = 1'b0;
            MemtoReg = 1'bx;
            Regwrite = 1'b0;
            MemRead = 1'b0;
            MemWrite = 1'b0;
            Branch = 1'b1;
            ALUOp = 2'b01;
        end
    default: begin
            ALUSrc = 1'b0;
            MemtoReg = 1'b0;
            Regwrite = 1'b0;
            MemRead = 1'b0;
            MemWrite = 1'b0;
            Branch = 1'b0;
            ALUOp = 2'b00;
    end
    endcase
end
endmodule

module ALU_Control
(
    input [1:0] ALUOp,
    input [3:0] Funct,
    output reg [3:0] Operation
);
always@(*)
begin
    case(ALUOp)
        2'b00: //for both addi and slli
            begin
                case(Funct[2:0])
                3'b000: //addi
                begin
                    Operation = 4'b0010;
                end
                3'b001: //slli
                begin
                    Operation = 4'b1000;
                end
              endcase
            end
        2'b01:
            begin
            Operation = 4'b0110;
            end
        2'b10:
            begin
            case(Funct)
            4'b0000:
                begin
                Operation = 4'b0010;
                end
            4'b1000:
                begin
                Operation = 4'b0110;
                end
            4'b0111:
                begin
                Operation = 4'b0000;
                end
            4'b0110:
                begin
                Operation = 4'b0001;
                end
            endcase
            end
    endcase
end
endmodule

module alu_64(
	input [63:0] a, 
    input [63:0] b,
    input [3:0] ALUOp,
	output reg [63:0] Result,
    output reg ZERO
);
always @ (*)
begin
    case(ALUOp)
    4'b0000 :
        begin
        Result = a&b;
        end
    4'b0001 :
        begin
        Result = a|b;
        end
    4'b0010 :
        begin
        Result = a+b;
        end
    4'b0110:
        begin
        Result = a-b;
        end
    4'b1100:
        begin
        Result = ~(a|b);
        end
    4'b1000:
        begin
            Result = a << b;
        end
    default : Result = 0;
endcase

if (Result == 64'b0)
    ZERO = 1'b1;
else
    ZERO = 1'b0;

end
endmodule



module RISC_V_Processor(
    input clk, reset
);

//Mux output
wire [63:0] PC_In_from_mux;
//Program counter output
wire [63:0] PC_Out;
//Adders outputs
wire [63:0] a1_out;
wire [63:0] a2_out;
//Input to Adder a1
wire [63:0] b_in = 64'd4;
//Output from IM
wire [31:0] Instruction;
//Output from IP
wire [4:0] rd; 
wire [4:0] rs1;
wire [4:0] rs2;
wire [6:0] opcode;
wire [6:0] funct7;
wire [2:0] funct3;
//Outputs from RegisterFile
wire [63:0] ReadData1;
wire [63:0] ReadData2;
//Outputs from Control Unit
wire [1:0] ALUOp;
wire Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite;
//Outputs from ALU Control
wire [3:0] Operation;
//Funct input to ALU_Control
wire [3:0] Funct;
assign Funct = {Instruction[30], Instruction[14:12]};
//Output from ALU
wire [63:0] Result_from_alu;
wire zero_output;
//Output from Data generator
wire [63:0] imm_data;
//Output from mux2
wire [63:0] out_from_mux2;
//sel for mux1
wire sel;
//Output from Data memory
wire [63:0] out_from_DM;
//Output from mux3
wire [63:0] out_from_mux3;
//Input to Adder a2
wire [63:0] b_adder2;
assign b_adder2 = imm_data << 1;

Program_Counter pc (.clk(clk), .reset(reset), .PC_In(PC_In_from_mux), .PC_Out(PC_Out));

Adder a1 (.a(PC_Out), .b(b_in), .out(a1_out));

Adder a2(.a(PC_Out), .b(b_adder2), .out(a2_out));

mux2x1 mux1(.a(a1_out), .b(a2_out), .s(sel), .data_out(PC_In_from_mux));

Instruction_Memory im(.Inst_Address(PC_Out), .Instruction(Instruction));

Instruction_Parser ip(.instruction(Instruction), .rd(rd), .rs1(rs1), .rs2(rs2), .funct3(funct3), .funct7(funct7), .opcode(opcode));

registerFile rf(.WriteData(out_from_mux3), .RS1(rs1), .RS2(rs2), .RD(rd), .clk(clk), .reset(reset), .RegWrite(RegWrite), .ReadData1(ReadData1), .ReadData2(ReadData2));

Control_Unit cu(
    .Opcode(opcode), 
    .ALUOp(ALUOp), 
    .Branch(Branch), 
    .MemRead(MemRead), 
    .MemtoReg(MemtoReg), 
    .MemWrite(MemWrite), 
    .ALUSrc(ALUSrc), 
    .Regwrite(RegWrite));

ALU_Control aluc(
    .ALUOp(ALUOp), 
    .Funct(Funct), 
    .Operation(Operation));

data_generator dg(
    .instruction(Instruction), 
    .imm_data(imm_data));

mux2x1 mux2(
    .a(ReadData2), 
    .b(imm_data), 
    .s(ALUSrc), 
    .data_out(out_from_mux2));

alu_64 alu(
    .a(ReadData1), 
    .b(out_from_mux2), 
    .ALUOp(Operation), 
    .Result(Result_from_alu), 
    .ZERO(zero_output));

selector s(
    .branch(Branch), 
    .ZERO(zero_output), 
    .a(ReadData1), 
    .b(out_from_mux2), 
    .funct3(funct3), 
    .sel(sel));

  Data_Memory dm(.mem_addr(Result_from_alu), .write_data(ReadData2), .clk(clk), .mem_write(MemWrite), .mem_read(MemRead), .read_data(out_from_DM), .element1(el1), .element2(el2), .element3(el3), .element4(el4), .element5(el5), .element6(el6), .element7(el7), .element8(el8));

mux2x1 mux3(
    .b(out_from_DM), 
    .a(Result_from_alu), 
    .s(MemtoReg), 
    .data_out(out_from_mux3));

always @(posedge clk)
    begin
        $monitor(
        "PC_In = ", PC_In_from_mux, 
        ", PC_Out = ", PC_Out, 
        ", Instruction = %b", Instruction,
        ", Opcode = %b", opcode, 
        ", Funct3 = %b", funct3, 
        ", Zero = %b", zero_output,
        ", Branch = %d", Branch,
        ", sel = %d", sel,
        ", rs1 = %d", rs1, 
        ", rs2 = %d", rs2, 
        ", rd = %d", rd,
        ", funct7 = %b", funct7, 
        ", ALUOp = %b", ALUOp,
        ", imm_data = %d", imm_data,
        ", Operation = %b", Operation 
        );
    end
endmodule



`timescale 1ns/1ps

module top(input  logic        clk, reset, 
           output logic [31:0] WriteDataM, DataAdrM, 
           output logic        MemWriteM);

  logic [31:0] PCF, InstrF, ReadDataM;
  
  // instantiate processor and memories
  riscv riscv(clk, reset, PCF, InstrF, MemWriteM, DataAdrM, 
              WriteDataM, ReadDataM);
  imem imem(PCF, InstrF);
  dmem dmem(clk, MemWriteM, DataAdrM, WriteDataM, ReadDataM);
endmodule

module riscv(input  logic        clk, reset,
             output logic [31:0] PCF,
             input  logic [31:0] InstrF,
             output logic        MemWriteM,
             output logic [31:0] ALUResultM, WriteDataM,
             input  logic [31:0] ReadDataM);

  logic [6:0]  opD;
  logic [2:0]  funct3D;
  logic        funct7b5D;
  logic [1:0]  ImmSrcD;
  logic        ZeroE;
  logic        PCSrcE;
  logic [2:0]  ALUControlE;
  logic        ALUSrcE;
  logic        ResultSrcEb0;
  logic        RegWriteM;
  logic [1:0]  ResultSrcW;
  logic        RegWriteW;

  logic [1:0]  ForwardAE, ForwardBE;
  logic        StallF, StallD, FlushD, FlushE;

  logic [4:0]  Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW;
  
  // Hazard Unit
  rase hazard_unit(
    .clk(clk),
    .reset(reset),
    .Rs1D(Rs1D),
    .Rs2D(Rs2D),
    .Rs1E(Rs1E),
    .Rs2E(Rs2E),
    .RdE(RdE),
    .RdM(RdM),
    .RdW(RdW),
    .RegWriteM(RegWriteM),
    .RegWriteW(RegWriteW),
    .MemWriteM(MemWriteM),
    .PCSrcE(PCSrcE),
    .ResultSrcEb0(ResultSrcEb0),
    .StallF(StallF),
    .StallD(StallD),
    .FlushD(FlushD),
    .FlushE(FlushE),
    .ForwardAE(ForwardAE),
    .ForwardBE(ForwardBE)
  );

  controller c(
    .clk(clk),
    .reset(reset),
    .opD(opD),
    .funct3D(funct3D),
    .funct7b5D(funct7b5D),
    .ImmSrcD(ImmSrcD),
    .FlushE(FlushE),
    .ZeroE(ZeroE),
    .PCSrcE(PCSrcE),
    .ALUControlE(ALUControlE),
    .ALUSrcE(ALUSrcE),
    .ResultSrcEb0(ResultSrcEb0),
    .MemWriteM(MemWriteM),
    .RegWriteM(RegWriteM),
    .RegWriteW(RegWriteW),
    .ResultSrcW(ResultSrcW)
  );

  datapath dp(
    .clk(clk),
    .reset(reset),
    .StallF(StallF),
    .PCF(PCF),
    .InstrF(InstrF),
    .opD(opD),
    .funct3D(funct3D),
    .funct7b5D(funct7b5D),
    .StallD(StallD),
    .FlushD(FlushD),
    .ImmSrcD(ImmSrcD),
    .FlushE(FlushE),
    .ForwardAE(ForwardAE),
    .ForwardBE(ForwardBE),
    .PCSrcE(PCSrcE),
    .ALUControlE(ALUControlE),
    .ALUSrcE(ALUSrcE),
    .ZeroE(ZeroE),
    .MemWriteM(MemWriteM),
    .WriteDataM(WriteDataM),
    .ALUResultM(ALUResultM),
    .ReadDataM(ReadDataM),
    .RegWriteW(RegWriteW),
    .ResultSrcW(ResultSrcW),
    .Rs1D(Rs1D),
    .Rs2D(Rs2D),
    .Rs1E(Rs1E),
    .Rs2E(Rs2E),
    .RdE(RdE),
    .RdM(RdM),
    .RdW(RdW)
  );
endmodule

module rase(
    input  logic        clk, reset,
    // Entradas para detecção de hazards
    input  logic [4:0]  Rs1D, Rs2D,
    input  logic [4:0]  Rs1E, Rs2E,
    input  logic [4:0]  RdE, RdM, RdW,
    input  logic        RegWriteM, RegWriteW,
    input  logic        MemWriteM,
    input  logic        PCSrcE,
    input  logic        ResultSrcEb0,  // lw seguido por uso
    // Saídas de controle
    output logic        StallF, StallD,
    output logic        FlushD, FlushE,
    output logic [1:0]  ForwardAE, ForwardBE
    );

    // Lógica de forwarding para estágio Execute
    always_comb begin
        // ForwardAE (para Rs1E)
        if ((Rs1E != 0) && (Rs1E == RdM) && RegWriteM)
            ForwardAE = 2'b10;  // Forward de Memory stage
        else if ((Rs1E != 0) && (Rs1E == RdW) && RegWriteW)
            ForwardAE = 2'b01;  // Forward de Writeback stage
        else
            ForwardAE = 2'b00;  // Valor do registrador

        // ForwardBE (para Rs2E)
        if ((Rs2E != 0) && (Rs2E == RdM) && RegWriteM)
            ForwardBE = 2'b10;
        else if ((Rs2E != 0) && (Rs2E == RdW) && RegWriteW)
            ForwardBE = 2'b01;
        else
            ForwardBE = 2'b00;
    end

    // Lógica de stall para load-use hazard
    always_comb begin
        StallF = 0;
        StallD = 0;
        FlushD = 0;
        FlushE = 0;
        
        // Stall para load-use hazard
        if (ResultSrcEb0 && ((Rs1D == RdE) || (Rs2D == RdE))) begin
            StallF = 1;
            StallD = 1;
            FlushE = 1;
        end
        
        // Flush para branch/jump
        if (PCSrcE) begin
            FlushD = 1;
            FlushE = 1;
        end
    end
endmodule

module controller(input  logic        clk, reset,
                  // Decode stage control signals
                  input logic [6:0]  opD,
                  input logic [2:0]  funct3D,
                  input logic        funct7b5D,
                  output logic [1:0] ImmSrcD,
                  // Execute stage control signals
                  input logic        FlushE, 
                  input logic        ZeroE, 
                  output logic       PCSrcE,
                  output logic [2:0] ALUControlE, 
                  output logic       ALUSrcE,
                  output logic       ResultSrcEb0,
                  // Memory stage control signals
                  output logic       MemWriteM,
                  output logic       RegWriteM,
                  // Writeback stage control signals
                  output logic       RegWriteW,
                  output logic [1:0] ResultSrcW);

  logic        RegWriteD, RegWriteE;
  logic [1:0]  ResultSrcD, ResultSrcE, ResultSrcM;
  logic        MemWriteD, MemWriteE;
  logic        JumpD, JumpE;
  logic        BranchD, BranchE;
  logic [1:0]  ALUOpD;
  logic [2:0]  ALUControlD;
  logic        ALUSrcD;
  
  // Decode stage logic
  maindec md(
    .op(opD),
    .ResultSrc(ResultSrcD),
    .MemWrite(MemWriteD),
    .Branch(BranchD),
    .ALUSrc(ALUSrcD),
    .RegWrite(RegWriteD),
    .Jump(JumpD),
    .ImmSrc(ImmSrcD),
    .ALUOp(ALUOpD)
  );
  
  aludec ad(
    .opb5(opD[5]),
    .funct3(funct3D),
    .funct7b5(funct7b5D),
    .ALUOp(ALUOpD),
    .ALUControl(ALUControlD)
  );
  
  // Execute stage pipeline control register and logic
  floprc #(10) controlregE(
    .clk(clk),
    .reset(reset),
    .clear(FlushE),
    .d({RegWriteD, ResultSrcD, MemWriteD, JumpD, BranchD, ALUControlD, ALUSrcD}),
    .q({RegWriteE, ResultSrcE, MemWriteE, JumpE, BranchE, ALUControlE, ALUSrcE})
  );

  assign PCSrcE = (BranchE & ZeroE) | JumpE;
  assign ResultSrcEb0 = ResultSrcE[0];
  
  // Memory stage pipeline control register
  flopr #(4) controlregM(
    .clk(clk),
    .reset(reset),
    .d({RegWriteE, ResultSrcE, MemWriteE}),
    .q({RegWriteM, ResultSrcM, MemWriteM})
  );
  
  // Writeback stage pipeline control register
  flopr #(3) controlregW(
    .clk(clk),
    .reset(reset),
    .d({RegWriteM, ResultSrcM}),
    .q({RegWriteW, ResultSrcW})
  );     
endmodule

module maindec(input  logic [6:0] op,
               output logic [1:0] ResultSrc,
               output logic       MemWrite,
               output logic       Branch, ALUSrc,
               output logic       RegWrite, Jump,
               output logic [1:0] ImmSrc,
               output logic [1:0] ALUOp);

  logic [10:0] controls;

  assign {RegWrite, ImmSrc, ALUSrc, MemWrite,
          ResultSrc, Branch, ALUOp, Jump} = controls;

  always_comb
    case(op)
      7'b0000011: controls = 11'b1_00_1_0_01_0_00_0; // lw
      7'b0100011: controls = 11'b0_01_1_1_00_0_00_0; // sw
      7'b0110011: controls = 11'b1_xx_0_0_00_0_10_0; // R-type 
      7'b1100011: controls = 11'b0_10_0_0_00_1_01_0; // beq
      7'b0010011: controls = 11'b1_00_1_0_00_0_10_0; // I-type ALU
      7'b1101111: controls = 11'b1_11_0_0_10_0_00_1; // jal
      7'b0000000: controls = 11'b0_00_0_0_00_0_00_0; // reset
      default:    controls = 11'bx_xx_x_x_xx_x_xx_x; // undefined
    endcase
endmodule

module aludec(input  logic       opb5,
              input  logic [2:0] funct3,
              input  logic       funct7b5, 
              input  logic [1:0] ALUOp,
              output logic [2:0] ALUControl);

  logic  RtypeSub;
  assign RtypeSub = funct7b5 & opb5;

  always_comb
    case(ALUOp)
      2'b00:                ALUControl = 3'b000; // add
      2'b01:                ALUControl = 3'b001; // sub
      default: case(funct3) // R-type or I-type ALU
                 3'b000:  ALUControl = RtypeSub ? 3'b001 : 3'b000; // add/sub
                 3'b010:  ALUControl = 3'b101; // slt
                 3'b110:  ALUControl = 3'b011; // or
                 3'b111:  ALUControl = 3'b010; // and
                 default: ALUControl = 3'bxxx; // undefined
               endcase
    endcase
endmodule

module datapath(input  logic        clk, reset,
                // Fetch stage signals
                input  logic        StallF,
                output logic [31:0] PCF,
                input  logic [31:0] InstrF,
                // Decode stage signals
                output logic [6:0]  opD,
                output logic [2:0] funct3D, 
                output logic        funct7b5D,
                input  logic        StallD, FlushD,
                input  logic [1:0]  ImmSrcD,
                // Execute stage signals
                input  logic        FlushE,
                input  logic [1:0]  ForwardAE, ForwardBE,
                input  logic        PCSrcE,
                input  logic [2:0]  ALUControlE,
                input  logic        ALUSrcE,
                output logic        ZeroE,
                // Memory stage signals
                input  logic        MemWriteM, 
                output logic [31:0] WriteDataM, ALUResultM,
                input  logic [31:0] ReadDataM,
                // Writeback stage signals
                input  logic        RegWriteW, 
                input  logic [1:0]  ResultSrcW,
                // Hazard Unit signals 
                output logic [4:0]  Rs1D, Rs2D, Rs1E, Rs2E,
                output logic [4:0]  RdE, RdM, RdW);

  // Fetch stage signals
  logic [31:0] PCNextF, PCPlus4F;
  // Decode stage signals
  logic [31:0] InstrD;
  logic [31:0] PCD, PCPlus4D;
  logic [31:0] RD1D, RD2D;
  logic [31:0] ImmExtD;
  logic [4:0]  RdD;
  // Execute stage signals
  logic [31:0] RD1E, RD2E;
  logic [31:0] PCE, ImmExtE;
  logic [31:0] SrcAE, SrcBE;
  logic [31:0] ALUResultE;
  logic [31:0] WriteDataE;
  logic [31:0] PCPlus4E;
  logic [31:0] PCTargetE;
  // Memory stage signals
  logic [31:0] PCPlus4M;
  // Writeback stage signals
  logic [31:0] ALUResultW;
  logic [31:0] ReadDataW;
  logic [31:0] PCPlus4W;
  logic [31:0] ResultW;

  // Fetch stage pipeline register and logic
  mux2 #(32) pcmux(
    .d0(PCPlus4F),
    .d1(PCTargetE),
    .s(PCSrcE),
    .y(PCNextF)
  );
  
  flopenr #(32) pcreg(
    .clk(clk),
    .reset(reset),
    .en(~StallF),
    .d(PCNextF),
    .q(PCF)
  );
  
  adder pcadd(
    .a(PCF),
    .b(32'h4),
    .y(PCPlus4F)
  );

  // Decode stage pipeline register and logic
  flopenrc #(96) regD(
    .clk(clk),
    .reset(reset),
    .clear(FlushD),
    .en(~StallD),
    .d({InstrF, PCF, PCPlus4F}),
    .q({InstrD, PCD, PCPlus4D})
  );
  
  assign opD       = InstrD[6:0];
  assign funct3D   = InstrD[14:12];
  assign funct7b5D = InstrD[30];
  assign Rs1D      = InstrD[19:15];
  assign Rs2D      = InstrD[24:20];
  assign RdD       = InstrD[11:7];
  
  regfile rf(
    .clk(clk),
    .we3(RegWriteW),
    .a1(Rs1D),
    .a2(Rs2D),
    .a3(RdW),
    .wd3(ResultW),
    .rd1(RD1D),
    .rd2(RD2D)
  );
  
  extend ext(
    .instr(InstrD[31:7]),
    .immsrc(ImmSrcD),
    .immext(ImmExtD)
  );
 
  // Execute stage pipeline register and logic
  floprc #(175) regE(
    .clk(clk),
    .reset(reset),
    .clear(FlushE),
    .d({RD1D, RD2D, PCD, Rs1D, Rs2D, RdD, ImmExtD, PCPlus4D}),
    .q({RD1E, RD2E, PCE, Rs1E, Rs2E, RdE, ImmExtE, PCPlus4E})
  );
  
  mux3 #(32) faemux(
    .d0(RD1E),
    .d1(ResultW),
    .d2(ALUResultM),
    .s(ForwardAE),
    .y(SrcAE)
  );
  
  mux3 #(32) fbemux(
    .d0(RD2E),
    .d1(ResultW),
    .d2(ALUResultM),
    .s(ForwardBE),
    .y(WriteDataE)
  );
  
  mux2 #(32) srcbmux(
    .d0(WriteDataE),
    .d1(ImmExtE),
    .s(ALUSrcE),
    .y(SrcBE)
  );
  
  alu alu(
    .a(SrcAE),
    .b(SrcBE),
    .alucontrol(ALUControlE),
    .result(ALUResultE),
    .zero(ZeroE)
  );
  
  adder branchadd(
    .a(ImmExtE),
    .b(PCE),
    .y(PCTargetE)
  );

  // Memory stage pipeline register
  flopr #(101) regM(
    .clk(clk),
    .reset(reset),
    .d({ALUResultE, WriteDataE, RdE, PCPlus4E}),
    .q({ALUResultM, WriteDataM, RdM, PCPlus4M})
  );
  
  // Writeback stage pipeline register and logic
  flopr #(101) regW(
    .clk(clk),
    .reset(reset),
    .d({ALUResultM, ReadDataM, RdM, PCPlus4M}),
    .q({ALUResultW, ReadDataW, RdW, PCPlus4W})
  );
  
  mux3 #(32) resultmux(
    .d0(ALUResultW),
    .d1(ReadDataW),
    .d2(PCPlus4W),
    .s(ResultSrcW),
    .y(ResultW)
  );
endmodule

module regfile(input  logic        clk, 
               input  logic        we3, 
               input  logic [4:0] a1, a2, a3, 
               input  logic [31:0] wd3, 
               output logic [31:0] rd1, rd2);

  logic [31:0] rf[31:0];

  always_ff @(negedge clk)
    if (we3) rf[a3] <= wd3;

  assign rd1 = (a1 != 0) ? rf[a1] : 0;
  assign rd2 = (a2 != 0) ? rf[a2] : 0;
endmodule

module adder(input  [31:0] a, b,
             output [31:0] y);

  assign y = a + b;
endmodule

module extend(input  logic [31:7] instr,
              input  logic [1:0]  immsrc,
              output logic [31:0] immext);
 
  always_comb
    case(immsrc) 
      2'b00: immext = {{20{instr[31]}}, instr[31:20]};  // I-type
      2'b01: immext = {{20{instr[31]}}, instr[31:25], instr[11:7]}; // S-type
      2'b10: immext = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0}; // B-type
      2'b11: immext = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0}; // J-type
      default: immext = 32'bx;
    endcase             
endmodule

module flopr #(parameter WIDTH = 8)
              (input  logic             clk, reset,
               input  logic [WIDTH-1:0] d, 
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 0;
    else       q <= d;
endmodule

module flopenr #(parameter WIDTH = 8)
                (input  logic             clk, reset, en,
                 input  logic [WIDTH-1:0] d, 
                 output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset)   q <= 0;
    else if (en) q <= d;
endmodule

module flopenrc #(parameter WIDTH = 8)
                (input  logic             clk, reset, clear, en,
                 input  logic [WIDTH-1:0] d, 
                 output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset)   q <= 0;
    else if (en) 
      if (clear) q <= 0;
      else       q <= d;
endmodule

module floprc #(parameter WIDTH = 8)
              (input  logic clk,
               input  logic reset,
               input  logic clear,
               input  logic [WIDTH-1:0] d, 
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 0;
    else       
      if (clear) q <= 0;
      else       q <= d;
endmodule

module mux2 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, 
              input  logic             s, 
              output logic [WIDTH-1:0] y);

  assign y = s ? d1 : d0; 
endmodule

module mux3 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, d2,
              input  logic [1:0]       s, 
              output logic [WIDTH-1:0] y);

  assign y = s[1] ? d2 : (s[0] ? d1 : d0); 
endmodule

module imem(input  logic [31:0] a,
            output logic [31:0] rd);

  logic [31:0] RAM[63:0];

  initial
      $readmemh("riscvtest.txt",RAM);

  assign rd = RAM[a[31:2]]; // word aligned
endmodule

module dmem(input  logic        clk, we,
            input  logic [31:0] a, wd,
            output logic [31:0] rd);

  logic [31:0] RAM[63:0];

  assign rd = RAM[a[31:2]];

  always_ff @(posedge clk)
    if (we) RAM[a[31:2]] <= wd;
endmodule

module alu(input  logic [31:0] a, b,
           input  logic [2:0]  alucontrol,
           output logic [31:0] result,
           output logic        zero);

  logic [31:0] condinvb, sum;
  logic        v;              // overflow
  logic        isAddSub;       // true when is add or subtract operation

  assign condinvb = alucontrol[0] ? ~b : b;
  assign sum = a + condinvb + alucontrol[0];
  assign isAddSub = ~alucontrol[2] & ~alucontrol[1] |
                    ~alucontrol[1] &  alucontrol[0];

  always_comb
    case (alucontrol)
      3'b000:  result = sum;         // add
      3'b001:  result = sum;         // subtract
      3'b010:  result = a & b;       // and
      3'b011:  result = a | b;       // or
      3'b100:  result = a ^ b;       // xor
      3'b101:  result = sum[31] ^ v; // slt
      3'b110:  result = a << b[4:0]; // sll
      3'b111:  result = a >> b[4:0]; // srl
      default: result = 32'bx;
    endcase

  assign zero = (result == 32'b0);
  assign v = ~(alucontrol[0] ^ a[31] ^ b[31]) & (a[31] ^ sum[31]) & isAddSub;
endmodule
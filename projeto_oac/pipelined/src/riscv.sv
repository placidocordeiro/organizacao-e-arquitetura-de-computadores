module riscv(input  logic        clk, reset,
             output logic [31:0] PCF,
             input  logic [31:0] InstrF,
             output logic        MemWriteM,
             output logic [31:0] ALUResultM, WriteDataM,
             input  logic [31:0] ReadDataM);

  // Pipeline control signals
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

  // Hazard signals
  logic [1:0]  ForwardAE, ForwardBE;
  logic        StallF, StallD, FlushD, FlushE;

  // Register addresses
  logic [4:0]  Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW;
  
  // Pipeline registers for control signals
  logic        RegWriteD, RegWriteE;
  logic        MemWriteD, MemWriteE;
  logic [1:0]  ResultSrcD, ResultSrcE, ResultSrcM;
  logic        BranchD, BranchE;
  logic        JumpD, JumpE;

  controller c(clk, reset,
               opD, funct3D, funct7b5D, ImmSrcD,
               FlushE, ZeroE, PCSrcE, ALUControlE, ALUSrcE, ResultSrcEb0,
               MemWriteM, RegWriteM, 
               RegWriteW, ResultSrcW);

  datapath dp(clk, reset,
              StallF, PCF, InstrF,
              opD, funct3D, funct7b5D, StallD, FlushD, ImmSrcD,
              FlushE, ForwardAE, ForwardBE, PCSrcE, ALUControlE, ALUSrcE, ZeroE,
              MemWriteM, WriteDataM, ALUResultM, ReadDataM,
              RegWriteW, ResultSrcW,
              Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW);

  // Hazard Unit
  always_comb begin
    // Forwarding Unit
    ForwardAE = (Rs1E != 0 && Rs1E == RdM && RegWriteM) ? 2'b10 :
               (Rs1E != 0 && Rs1E == RdW && RegWriteW) ? 2'b01 : 2'b00;
               
    ForwardBE = (Rs2E != 0 && Rs2E == RdM && RegWriteM) ? 2'b10 :
               (Rs2E != 0 && Rs2E == RdW && RegWriteW) ? 2'b01 : 2'b00;
    
    // Stall detection (load-use hazard)
    StallF = (ResultSrcEb0 && (Rs1D == RdE || Rs2D == RdE)); // Fixed stall condition
    StallD = StallF;
    FlushD = PCSrcE;
    FlushE = StallD || PCSrcE;
  end

  // Pipeline registers for control signals
  always_ff @(posedge clk) begin
    if (reset || FlushE) begin
      RegWriteE <= 0;
      MemWriteE <= 0;
      ResultSrcE <= 0;
      BranchE <= 0;
      JumpE <= 0;
    end else if (!StallD) begin
      RegWriteE <= c.RegWriteD;
      MemWriteE <= c.MemWriteD;
      ResultSrcE <= c.ResultSrcD;
      BranchE <= c.BranchD;
      JumpE <= c.JumpD;
    end
  end
endmodule

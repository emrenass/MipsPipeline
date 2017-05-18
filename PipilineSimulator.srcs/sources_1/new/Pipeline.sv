module top  (input   logic 	 clkIn,
         input logic button, clear,
         output logic [3:0] AN,
         output logic [6:0] C,
         output logic DP,
         output logic led);  

   logic [31:0] readdata;
   logic clk;    
    pulse_controller pls(clkIn, button, clear, clk );
   // instantiate processor and memories
     
    logic[31:0] writedata, dataadr;
    logic memwrite;  
    logic[31:0] tempwrite;
    logic[31:0] tempdata;
    assign tempwrite = writedata;
    assign tempdata = dataadr;
   mips mips (clk, reset, pc, instr, memwrite, dataadrreaddata);  
   imem imem (pc >> 2, instr); 
   display_controller dis(clkIn, clear, tempdata[3:0],tempdata[7:4],tempwrite[3:0],tempwrite[7:4], AN, C, DP);
   

endmodule



// External data memory used by MIPS single-cycle processor

module dmem (input  logic        clk, we,
             input  logic[31:0]  a, wd,
             output logic[31:0]  rd);

   logic  [31:0] RAM[63:0];
  
   assign rd = RAM[a[31:2]];    // word-aligned  read (for lw)

   always_ff @(posedge clk)
     if (we)
       RAM[a[31:2]] <= wd;      // word-aligned write (for sw)

endmodule



// External instruction memory used by MIPS single-cycle
// processor. It models instruction memory as a stored-program 
// ROM, with address as input, and instruction as output


module imem ( input logic [5:0] addr, output logic [31:0] instr);

// imem is modeled as a lookup table, a stored-program byte-addressable ROM
	always_comb
	   case ({addr,2'b00})		   	// word-aligned fetch
//		address		instruction
//		-------		-----------
		8'h00: instr = 32'h20020005;  	// disassemble, by hand 
		8'h04: instr = 32'h2003000c;  	// or with a program,
		8'h08: instr = 32'h2067fff7;  	// to find out what
		8'h0c: instr = 32'h00e22025;  	// this program does!
		8'h10: instr = 32'h00642824;
		8'h14: instr = 32'h00a42820;
		8'h18: instr = 32'h10a7000a;
		8'h1c: instr = 32'h0064202a;
		8'h20: instr = 32'h10800001;
		8'h24: instr = 32'h20050000;
		8'h28: instr = 32'h00e2202a;
		8'h2c: instr = 32'h00853820;
		8'h30: instr = 32'h00e23822;
		8'h34: instr = 32'hac670044;
		8'h38: instr = 32'h8c020050;
		8'h3c: instr = 32'h08000011;
		8'h40: instr = 32'h20020001;
		8'h44: instr = 32'hac020054;
		8'h48: instr = 32'h08000012;	// j 48, so it will loop here
	     default:  instr = {32{1'bx}};	// unknown address
	   endcase
endmodule


// single-cycle MIPS processor, with controller and datapath

module mips (input  logic        clk, reset,
             output logic[31:0]  pc,
             input  logic[31:0]  instr,
             output logic        memwrite,
             output logic[31:0]  aluout, resultW,
             output logic[31:0] instrOut,
             input  logic[31:0]  readdata);

  logic        memtoreg, pcsrc, zero, alusrc, regdst, regwrite, jump;
  logic [2:0]  alucontrol;
  assign instrOut = instr;
  controller c (instr[31:26], instr[5:0], zero, memtoreg, memwrite, pcsrc,
                        alusrc, regdst, regwrite, jump, alucontrol);

  datapath dp (clk, reset, memtoreg, pcsrc, alusrc, regdst, regwrite, jump,
                          alucontrol, zero, pc, instr, aluout, resultW, readdata);

endmodule
module controller(input  logic[5:0] op, funct,
                  input  logic     zero,
                  output logic     memtoreg, memwrite,
                  output logic     pcsrc, alusrc,
                  output logic     regdst, regwrite,
                  output logic     jump,
                  output logic[2:0] alucontrol);

   logic [1:0] aluop;
   logic       branch;

   maindec md (op, memtoreg, memwrite, branch, alusrc, regdst, regwrite, 
		 jump, aluop);

   aludec  ad (funct, aluop, alucontrol);

   assign pcsrc = branch & zero;

endmodule

module maindec (input logic[5:0] op, 
	              output logic memtoreg, memwrite, branch,
	              output logic alusrc, regdst, regwrite, jump,
	              output logic[1:0] aluop );
   logic [8:0] controls;

   assign {regwrite, regdst, alusrc, branch, memwrite,
                memtoreg,  aluop, jump} = controls;

  always_comb
    case(op)
      6'b000000: controls <= 9'b110000100; // R-type
      6'b100011: controls <= 9'b101001000; // LW
      6'b101011: controls <= 9'b001010000; // SW
      6'b000100: controls <= 9'b000100010; // BEQ
      6'b001000: controls <= 9'b101000000; // ADDI
      6'b000010: controls <= 9'b000000001; // J
      default:   controls <= 9'bxxxxxxxxx; // illegal op
    endcase
endmodule

module aludec (input    logic[5:0] funct,
               input    logic[1:0] aluop,
               output   logic[2:0] alucontrol);
  always_comb
    case(aluop)
      2'b00: alucontrol  = 3'b010;  // add  (for lw/sw/addi)
      2'b01: alucontrol  = 3'b110;  // sub   (for beq)
      default: case(funct)          // R-TYPE instructions
          6'b100000: alucontrol  = 3'b010; // ADD
          6'b100010: alucontrol  = 3'b110; // SUB
          6'b100100: alucontrol  = 3'b000; // AND
          6'b100101: alucontrol  = 3'b001; // OR
          6'b101010: alucontrol  = 3'b111; // SLT
          default:   alucontrol  = 3'bxxx; // ???
        endcase
    endcase
endmodule

module datapath (input  logic clk, reset, memtoreg, pcsrc, alusrc, regdst,
                 input  logic regwrite, jump, 
		 input  logic[2:0]  alucontrol, 
                 output logic zero, 
		 output logic[31:0] pc, 
	         input  logic[31:0] instr,
                 output logic[31:0] aluout, resultW, 
	         input  logic[31:0] readdata);
  logic stallF, stallD,  ForwardAD, ForwardBD,  FlushE, ForwardAE, ForwardBE;
  logic [4:0]  writereg;
  logic [31:0] pcnext, pcnextbr, pcplus4, pcbranch;
  logic [31:0] signimm, signimmsh, srca, srcb, result;
  logic regwriteE, memtoregE,memwriteE,alusrcE, regdstE, branchE;
  logic [2:0] alucontrolE;
  // next PC logic
  flopr #(32) pcreg(clk, reset, pcnext, pc);
  adder       pcadd1(pc, 32'b100, pcplus4);
  sl2         immsh(signimm, signimmsh);
  adder       pcadd2(pcplus4, signimmsh, pcbranch);
  mux2 #(32)  pcbrmux(pcplus4, pcbranch, pcsrc,
                      pcnextbr);
  mux2 #(32)  pcmux(pcnextbr, {pcplus4[31:28], 
                    instr[25:0], 2'b00}, jump, pcnext);
  
  //IMDE Pipeline register
  logic [31:0] instOut, pcPlusOut,pcPlusOutE;
  pipeIMID imid(instr, pcplus4,StallD, clk,instOut, pcPlusOut);
  
  
// register file logic
   regfile     rf (clk, regwrite, instr[25:21], instr[20:16], writeregW,
                   result, srca, writedata);
   signext         se (instr[15:0], signimm);
    //diexe Pipeline register
    logic [31:0] rsOut, rdOut;
    logic [5:0] rsE, rdE, rtE;
    logic [15:0] signExtE;
    logic regwriteE, memtoregE,memwriteE,alusrcE, regdstE, branchE;
    logic [2:0] alucontrolE;
    logic [31:0] srcbE;
     pipeDIEXE diexe(instr, pcplus4, srca, writedata, FlushE, clk,
                     signimm,
                     regwrite, memtoreg,memwrite,alusrc, regdst, pcbranch,
                     alucontrol,
                     regwriteE, memtoregE,memwriteE,alusrcE, regdstE, branchE,
                     alucontrolE,
                     pcPlusOutE, rsOut, rdOut,
                     rsE, rdE, rtE,
                     signExtE);
   
   

  logic[31:0] ALUResultM, WriteDataM;
  logic regwriteM, memtoregM, memwriteM;
  
  // ALU logic
  mux3 m1(rsOut, resultW, ALUoutM, forwardAE, srcaE);
  mux3 m2(rdOut, resultW, ALUoutM, forwardBE, srcbFirst);
  
   mux2 #(32)  srcbmux (srcbFirst, signExtE, alusrc, srcbE);
   alu         alu (srcaE, srcbE, alucontrol, aluout, zero);
   pipeEXEDM(srcb, aluout,
                     clk, regwriteE, memtoregE, memwriteE,
                     ALUResultM, WriteDataM,
                     regwriteM, memtoregM, memwriteM);
                     
   mux2 #(5)    wrmux (rdE, rtE, regdstE, writeregE);
   dmem dmem (clk, memwriteM, ALUResultM, WriteDataM, readdataM);
   logic [31:0] readDataW, AluResultW;
   logic regwriteW, memtoregW;
   pipeDMWB(readdataM, ALUResultM,
                   clk, regwriteM, memtoregM,
                   readDataW, AluResultW,
                   regwriteW, memtoregW);
   
   mux2 #(32)  resmux (AluResultW, readDataW, memtoregW, resultW);
   
   hazardUnit(rsD, rtD, rsE, rtE,
                     WriteRegM, WriteRegE, WriteRegW,
                     BranchD,MemToRegE, MemToRegM, RegWriteE, RegWriteM, regWriteW, RegWriteW,
                     stallF, stallD,  ForwardAD, ForwardBD,  FlushE, 
                     ForwardAE, ForwardBE);
endmodule


module regfile (input    logic clk, we3, 
                input    logic[4:0]  ra1, ra2, wa3, 
                input    logic[31:0] wd3, 
                output   logic[31:0] rd1, rd2);

  logic [31:0] rf [31:0];

  // three ported register file: read two ports combinationally
  // write third port on rising edge of clock. Register0 hardwired to 0.

  always_ff  @(posedge clk)
     if (we3) 
         rf [wa3] <= wd3;	

  assign rd1 = (ra1 != 0) ? rf [ra1] : 0;
  assign rd2 = (ra2 != 0) ? rf[ ra2] : 0;

endmodule


module alu(input  logic [31:0] a, b, 
           input  logic [2:0]  alucont, 
           output logic [31:0] result,
           output logic zero);

    always_comb
        begin
        case(alucont)
                3'b000 : result = a&b;
                3'b001 : result = a | b;
                3'b010 : result = a + b;
                3'b100 : result = a&~b;
                3'b101 : result = a|~b;
                3'b110 : begin
                    result = a-b;
                    zero = a-b==0 ? 1:0;
                    end
                3'b111 : result = a<b ? 1:0;
           endcase
        end
endmodule



module adder (input  logic[31:0] a, b,
              output logic[31:0] y);
     
     assign y = a + b;
endmodule

module sl2 (input  logic[31:0] a,
            output logic[31:0] y);
     
     assign y = {a[29:0], 2'b00}; // shifts left by 2
endmodule

module signext (input  logic[15:0] a,
                output logic[31:0] y);
              
  assign y = {{16{a[15]}}, a};    // sign-extends 16-bit a
endmodule

// parameterized register
module flopr #(parameter WIDTH = 8)
              (input logic clk, reset, 
	       input logic[WIDTH-1:0] d, 
               output logic[WIDTH-1:0] q);

  always_ff@(posedge clk, posedge reset)
    if (reset) q <= 0; 
    else       q <= d;
endmodule


// paramaterized 2-to-1 MUX
module mux2 #(parameter WIDTH = 8)
             (input  logic[WIDTH-1:0] d0, d1,  
              input  logic s, 
              output logic[WIDTH-1:0] y);
  
   assign y = s ? d1 : d0; 
endmodule



module pipeIMID(input logic[31:0] instr, pcPlus4,
                input logic EN, clk,
                output logic[31:0] instOut, pcPlusOut);
                always_ff @(posedge clk)
                    if(EN)
                        begin
                        instOut<=instr;
                        pcPlusOut<=pcPlus4;
                        end
                
endmodule

module pipeDIEXE(input logic[31:0] instr, pcPlus4, rs, rd,
                input logic clr, clk,
                input logic [15:0] signExtD,
                input logic regwrite, memtoreg,memwrite,alusrc, regdst, branch,
                input logic [2:0]alucontroller,
                output logic regwriteE, memtoregE,memwriteE,alusrcE, regdstE, branchE,
                output logic [2:0]alucontrollerE,
                output logic[31:0] pcPlusOut, rsOut, rdOut,
                output logic [5:0] rsE, rdE, rtE,
                output logic [15:0] signExtE);
                always_ff @(posedge clk)
                    if(clr)
                        begin
                        pcPlusOut<=32'b0;
                        rsE <= 6'b0;
                        rdE <= 6'b0;
                        rtE <= 6'b0;
                        rsOut<=32'b0;
                        rdOut<=32'b0;
                        signExtE <=16'b0;
                        regwriteE<=1'b0;
                        memtoregE<=1'b0;
                        memwriteE<=1'b0;
                        alusrcE<=1'b0;
                        regdstE<=1'b0;
                        branchE<=1'b0;
                        alucontrollerE<=2'b0;
                        end
                    else
                        begin
                        pcPlusOut<=pcPlus4;
                        rsE <= instr[25:21];
                        rdE <= instr[20:16];
                        rtE <= instr[15:11];
                        rsOut<=rs;
                        rdOut<=rd;
                        signExtE <=signExtD;
                        regwriteE<=regwrite;
                        memtoregE<=memtoreg;
                        memwriteE<=memwrite;
                        alusrcE<=alusrc;
                        regdstE<=regdst;
                        branchE<=branch;
                        alucontrollerE<=alucontroller;
                        end
                
endmodule

module pipeIMEXE(input logic[31:0] rs, rd, rt,
                input logic clr, clk, regwrite, memtoreg, memwrite,ALUSrc,REGDst, 
                input logic [2:0]ALUControl, 
                output logic[31:0] rsOut, rdOut, rtOut,
                output logic regwriteOut, memtoregOut, memwriteOut,ALUSrcOut,REGDstOut,
                output logic[2:0]ALUControlOut);
                always_ff @(posedge clk)
                    if(clr)
                        begin
                        rsOut <= 32'b0;
                        rdOut <= 32'b0;
                        rtOut <= 32'b0;
                        regwriteOut <= 1'b0;
                        memtoregOut <= 1'b0;
                        memwriteOut <= 1'b0;
                        REGDstOut <= 1'b0;
                        ALUControlOut <= 2'b0;
                        end
                    else
                        begin
                        rsOut <= rs;
                        rdOut <= rd;
                        rtOut <= rt;
                        regwriteOut <= regwrite;
                        memtoregOut <= memtoreg;
                        memwriteOut <= memwrite;
                        REGDstOut <= ALUSrc;
                        ALUControlOut <= REGDst;
                        end
endmodule

module pipeEXEDM(input logic[31:0] WriteData, ALUResult,
                input logic clk, regwrite, memtoreg, memwrite,
                output logic[31:0] ALUResultOut, WriteDataOut,
                output logic regwriteOut, memtoregOut, memwriteOut);
                always_ff @(posedge clk)
                        begin
                        WriteDataOut <= WriteData;
                        regwriteOut <= regwrite;
                        memtoregOut <= memtoreg;
                        memwriteOut <= memwrite;
                         ALUResultOut <= ALUResult;
                        end
endmodule

module pipeDMWB(input logic[31:0] readData, AluResult,
                input logic clk, regwrite, memtoreg,
                output logic[31:0] readDataOut, AluResultOut,
                output logic regwriteOut, memtoregOut);
                always_ff @(posedge clk)
                        begin
                        readDataOut <= readData;
                        AluResultOut <= AluResult;
                        regwriteOut <= regwrite;
                        memtoregOut <= memtoreg;
                        end
endmodule

module equalizer(input logic[31:0] a,b,
                 output logic c);
    assign c = (a==b) ? 1 : 0;
endmodule

module mux3(input logic [31:0]a, b, c,
            input logic [1:0]s,
            output logic [31:0]d);
        always_comb
            begin
            case(s)
            2'b00 : d = a;
            2'b01 : d = b;
            2'b10 : d = c;
            endcase
            end
endmodule

module zeroExt(input logic [15:0] a,
                output logic [31:0] b);
        logic zero = 1'b0;
        assign b = {{16{zero}}, a};
endmodule

module floprPC #(parameter WIDTH = 8)
              (input logic clk, reset, EN, 
	       input logic[WIDTH-1:0] d, 
               output logic[WIDTH-1:0] q);

  always_ff@(posedge clk, posedge reset)
    if (reset) q <= 0; 
    else if(EN) q <= d;
endmodule
module hazardUnit(input logic [31:0] rsD, rtD, rsE, rtE,
                  input logic [5:0] WriteRegM, WriteRegE, WriteRegW,
                  input logic BranchD,MemToRegE, MemToRegM, RegWriteE, RegWriteM, regWriteW, RegWriteW,
                  output logic stallF, stallD,  ForwardAD, ForwardBD,  FlushE, 
                  output logic [1:0] ForwardAE, ForwardBE);
        logic branchstall;
        assign ForwardAD = ((rsD !=0) && (rsD == WriteRegM) && RegWriteM) ? 1:0;
        assign ForwardBD = ((rtD !=0) && (rtD == WriteRegM) && RegWriteM) ? 1:0;
        assign branchstall = (BranchD && RegWriteE  &&  (WriteRegE == rsD  ||  WriteRegE == rtD)  ||  BranchD  &&  MemToRegM  &&  (WriteRegM == rsD || WriteRegM == rtD)) ? 1:0;
        assign stallF = branchstall;
        assign stallD = branchstall;
        assign FlushE = branchstall;
        
        always_comb
            begin
            if  ((rsE != 0) && (rsE == WriteRegM) && RegWriteM)     
                    ForwardAE = 2'b10;
            else begin
                if ((rsE != 0) && (rsE == WriteRegW) && RegWriteW) 
                 ForwardAE = 2'b01;
                else
                ForwardAE = 2'b00;
                end
            if  ((rtE != 0) && (rtE == WriteRegM) && RegWriteM)     
                    ForwardBE = 2'b10;
            else begin
                if ((rtE != 0) && (rtE == WriteRegW) && RegWriteW) 
                 ForwardBE = 2'b01;
                else
                ForwardBE = 2'b00;
                end
            end                   
endmodule

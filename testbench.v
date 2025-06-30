`timescale 1ns / 1ps

module MIPS_Processor_TB();
    
    reg clk;
    reg reset;
    
    
    reg instr_write_enable;
    reg [7:0] instr_write_addr;
    reg [31:0] instr_write_data;
    
    reg data_init_write_enable;
    reg [7:0] data_init_addr;
    reg [31:0] data_init_data;
    
    
    wire [31:0] RegValue0, RegValue1, RegValue2, RegValue3;
    wire [31:0] RegValue4, RegValue5, RegValue6, RegValue7;
    wire [31:0] RegValue8, RegValue9, RegValue10, RegValue11;
    wire [31:0] RegValue12, RegValue13, RegValue14, RegValue15;
    wire [31:0] RegValue16, RegValue17, RegValue18, RegValue19;
    wire [31:0] RegValue20, RegValue21, RegValue22, RegValue23;
    wire [31:0] RegValue24, RegValue25, RegValue26, RegValue27;
    wire [31:0] RegValue28, RegValue29, RegValue30, RegValue31;
    
    
    MIPS_Processor test (
        .clk(clk),
        .reset(reset),
        .instr_write_enable(instr_write_enable),
        .instr_write_addr(instr_write_addr),
        .instr_write_data(instr_write_data),
        .data_init_write_enable(data_init_write_enable),
        .data_init_addr(data_init_addr),
        .data_init_data(data_init_data),
        .RegValue0(RegValue0), .RegValue1(RegValue1), .RegValue2(RegValue2), .RegValue3(RegValue3),
        .RegValue4(RegValue4), .RegValue5(RegValue5), .RegValue6(RegValue6), .RegValue7(RegValue7),
        .RegValue8(RegValue8), .RegValue9(RegValue9), .RegValue10(RegValue10), .RegValue11(RegValue11),
        .RegValue12(RegValue12), .RegValue13(RegValue13), .RegValue14(RegValue14), .RegValue15(RegValue15),
        .RegValue16(RegValue16), .RegValue17(RegValue17), .RegValue18(RegValue18), .RegValue19(RegValue19),
        .RegValue20(RegValue20), .RegValue21(RegValue21), .RegValue22(RegValue22), .RegValue23(RegValue23),
        .RegValue24(RegValue24), .RegValue25(RegValue25), .RegValue26(RegValue26), .RegValue27(RegValue27),
        .RegValue28(RegValue28), .RegValue29(RegValue29), .RegValue30(RegValue30), .RegValue31(RegValue31)
    );
    
    
    wire [31:0] PC = test.datapath.PC;
    wire [31:0] Instruction = test.datapath.Instruction;
    wire [3:0] ALUControl = test.datapath.ALUControl;
    wire Zero = test.datapath.Zero;
    wire [31:0] ALUResult = test.datapath.ALUResult;
    wire [31:0] ReadData = test.datapath.ReadData;
    wire RegDst = test.control.RegDst;
    wire Jump = test.control.Jump;
    wire Branch = test.control.Branch;
    wire MemRead = test.control.MemRead;
    wire MemtoReg = test.control.MemtoReg;
    wire [1:0] ALUOp = test.control.ALUOp;
    wire MemWrite = test.control.MemWrite;
    wire ALUSrc = test.control.ALUSrc;
    wire RegWrite = test.datapath.RegWrite;
    wire [4:0] WriteReg = test.datapath.WriteReg;
    wire [31:0] WriteData = test.datapath.WriteData;
    
    
    initial begin
        clk = 0;
        forever #5 clk = ~clk; 
    end
    
    
    initial begin
        

        instr_write_enable = 0;
        data_init_write_enable = 0;
        
        
        #20;
        reset = 0;
        
     
        
      
        
        
        #1500;
        
        
        $display("Simulation Complete - Fibonacci Sequence Calculation");
        $display("------------------------------------------------");
        $finish;
    end

    
    always @(posedge clk) begin
        if (!reset) begin
            $display("Time: %t", $time);
            $display("PC: %h", PC);
            $display("Instruction: %h", Instruction);
            
            
            $display("Control Signals - RegDst: %b, Jump: %b, Branch: %b, MemRead: %b, MemtoReg: %b", 
                RegDst, Jump, Branch, MemRead, MemtoReg);
            $display("ALUOp: %b, MemWrite: %b, ALUSrc: %b, RegWrite: %b", 
                ALUOp, MemWrite, ALUSrc, RegWrite);
            
            
            $display("ALU Control: %b, Zero: %b, Result: %h", 
                ALUControl, Zero, ALUResult);
            
          
            $display("Registers - $1: %d, $2: %d, $3: %d, $4: %d, $5: %d, $6: %d, $7: %d",
                RegValue1, // Constant 1
                RegValue2, // n
                RegValue3, // first
                RegValue4, // second/current fibonacci
                RegValue5, // next
                RegValue6, // counter
                RegValue7  // temp for branch condition
            );
            
            
            if (PC == 32'd20) begin 
                $display("Storing Fibonacci Number: %d at Data Memory Address 400", RegValue4);
            end
            
            $display("------------------------------------------------");
        end
    end
endmodule

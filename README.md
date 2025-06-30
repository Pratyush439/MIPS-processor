# MIPS-processor

This project implements a simple RISC (Reduced Instruction Set Computing) processor using Verilog. The processor supports basic instructions such as add, sub, store, load, add constant, and jump to an address. The design includes a datapath and a controller, with an ALU (Arithmetic Logic Unit) implemented using gate-level multiplexer logic.

# Introduction

The Simple RISC Processor is a basic educational project designed to demonstrate the implementation of a processor using Verilog HDL (Hardware Description Language). The processor executes a set of predefined instructions and can perform arithmetic operations, memory operations, and control flow operations

# Instruction Set

The processor supports the following instructions:

    add : Adds two registers and stores the result in a register.
    sub : Subtracts one register from another and stores the result in a register.
    store : Stores data from a register to a memory address.
    load : Loads data from a memory address into a register.
    add constant : Adds a constant value to a register and stores the result in the same register.
    jump : Jumps to a specified address.

# Architecture

The processor architecture consists of the following key components:

    Datapath: Responsible for data processing and includes the ALU, registers, and memory.
    Controller: Manages the control signals to direct the operations of the datapath.
    ALU (Arithmetic Logic Unit): Performs arithmetic operations and is implemented using gate-level multiplexer logic.


Components
Datapath

The datapath includes the following modules:

    Registers: Store intermediate data and results.
    Memory: Holds instructions and data.
    ALU: Performs arithmetic operations (add, sub, add constant).

 Controller

The controller generates control signals based on the current instruction to direct the operations of the datapath. It includes:

    Instruction Decoder: Decodes the current instruction.
    Control Logic: Generates appropriate control signals for the datapath components.

 ALU (Arithmetic Logic Unit)
The ALU is implemented using gate-level multiplexer logic to perform arithmetic operations. It supports the following functions:

    Addition
    Subtraction
    Add constant

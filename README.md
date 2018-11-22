SERV
====

Prerequisites
-------------

Create a directory to keep all the different parts of the project together. We
will refer to this directory as `$SERV from now on`

Download the main serv repo

`cd $SERV && git clone https://github.com/olofk/serv`

Install FuseSoC

`pip install fusesoc`

Initialize the FuseSoC standard libraries

`fusesoc init`

Create a workspace directory for FuseSoC

`mkdir $SERV/workspace`

Register the serv repo as a core library

`cd $SERV/workspace && fusesoc library add serv ../serv`

Check that the CPU passes the linter

`cd $SERV/workspace && fusesoc run --target=lint serv`

Running test software
---------------------

Build and run the zephyr hello world example with verilator

    cd $SERV/workspace
    fusesoc run	--target=verilator_tb serv --firmware=../serv/hellomin.hex

Other applications can be tested by compiling and converting to bin and then hex e.g. with makehex.py found in $SERV/serv/riscv-target/serv

Run the compliance tests
------------------------

Build the verilator model (if not already done)

`cd $SERV/workspace && fusesoc run --target=verilator_tb --setup --build serv`

Download the tests repo

`cd $SERV && git clone https://github.com/riscv/riscv-compliance`

Run the compliance tests

`cd $SERV/riscv-compliance && make TARGETDIR=$SERV/serv/riscv-target RISCV_TARGET=serv RISCV_DECICE=rv32i RISCV_ISA=rv32i TARGET_SIM=$SERV/workspace/build/serv_0/verilator_tb-verilator/Vserv_wrapper`

Run on hardware
---------------

Only supported so far is a single threaded hello world on TinyFPGA BX

Pin B3 is used for UART output with 57600 baud rate.

    cd $SERV/workspace
    fusesoc run --target=tinyfpga_bx serv
    tinyprog --program build/serv_0/tinyfpga_bx-icestorm/serv_0.bin



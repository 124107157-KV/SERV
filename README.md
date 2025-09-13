<img align="right" src="https://svg.wavedrom.com/{signal:[{wave:'0.P...'},{wave:'023450',data:'S E R V'}]}"/>

# SERV

[![Join the chat at https://gitter.im/librecores/serv](https://badges.gitter.im/librecores/serv.svg)](https://gitter.im/librecores/serv?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)
[![Compliance tests](https://github.com/olofk/serv/actions/workflows/ci.yml/badge.svg)](https://github.com/olofk/serv/actions/workflows/ci.yml)
[![Documentation Status](https://readthedocs.org/projects/serv/badge/?version=latest)](https://serv.readthedocs.io/en/latest/?badge=latest)

SERV is an award-winning bit-serial RISC-V core

In fact, the award-winning SERV is the world's smallest RISC-V CPU. It's the perfect companion whenever you need a bit of computation and silicon real estate is at a premium.

How small is it then? Synthesizing the latest version of SERV in its most minimal form, yields the following results for some popular FPGA architectures and a typical CMOS process.

| Lattice iCE40 | Intel Cyclone 10LP | AMD Artix-7 | CMOS   |
| ------------- | ------------------ | ----------- | ------ |
| 198 LUT       | 239 LUT            | 125 LUT     | 2.1kGE |
| 164 FF        | 164 FF             | 164 FF      |        |


If you want to know more about SERV, what a bit-serial CPU is and what it's good for, I recommend starting out by watching the fantastic short SERV movies
* [introduction to SERV](https://www.award-winning.me/serv-introduction/)
* [SERV : RISC-V for a fistful of gates](https://www.award-winning.me/serv-for-a-fistful-of-gates/)
* [SERV: 32-bit is the New 8-bit](https://www.award-winning.me/serv-32-bit-is-the-new-8-bit/)
* [Bit by bit - How to fit 8 RISC V cores in a $38 FPGA board (presentation from the Zürich 2019 RISC-V workshop)](https://www.youtube.com/watch?v=xjIxORBRaeQ)

All SERV videos and more can also be found [here](https://www.award-winning.me/videos/).

Apart from being the world's smallest RISC-V CPU, SERV also aims at being the best documented RISC-V CPU. For this there is an official [SERV user manual](https://serv.readthedocs.io/en/latest/#) with block diagrams that are correct to the gate-level, cycle-accurate timing diagrams and an in-depth description of how things work.

###
SERV-Stochastic extends the award-winning, world’s-smallest RISC-V core (SERV) with a dual-mode ALU that runs both deterministic (binary) and stochastic arithmetic on the same bit-serial datapath—plus a lightweight serial MAC path for neural-network workloads. The design targets tiny area & power, preserves RV32I transparency in normal mode, and adds runtime-selectable SC/NN execution.

# can_axi_mapper
CPP Software module to interface the CAN Bus peripherial on the processing system of the Zynq7000 SoC and further
processing to a custom AXI periherial.

# Prerequistes

- Installed Embedded Linux on Zynq7000
- Corresponding ARM cross-compiler

## can4linux 

- Download can4linux source: https://sourceforge.net/projects/can4linux/
- Adjust Makefile with folder of can4linux source
- Build can4linux
- Copy into kernel modules folder
- Add entry to kernel modules list

@TODO: More detailed instructions

# How To

## Prepare on Zynq console

- Unlooad xilinx_can device driver
- Load can4linux device driver
- Activate CAN peripherial if not done by the hardware description

``activate_can4linux_driver.sh``

## Build 

``make``

## Deploy

``scp can_axi_mappper  root@<zynq-ip>:<target-folder>``

# Misc

@TODO Cleanup .cpp source code.

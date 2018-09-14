@echo off
pushd build
STM32_Programmer_CLI -c port=swd -w hal_new_test.bin 0x08000000 -rst -q
popd





-Run tracing of program in simulator

simulavr -F 16000000 -d atmega328 -c vcd:trace.txt:xxx.vcd -m 1000000000 --file .pio/build/nanoatmega328/firmware.elf

-View the trace
gtkwave

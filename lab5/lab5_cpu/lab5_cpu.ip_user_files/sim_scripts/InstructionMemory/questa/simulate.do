onbreak {quit -f}
onerror {quit -f}

vsim -t 1ps -lib xil_defaultlib InstructionMemory_opt

do {wave.do}

view wave
view structure
view signals

do {InstructionMemory.udo}

run -all

quit -force

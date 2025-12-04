Peter - Tested Oct 31st, 2024 on DE1-SoC in lab

What this design does!

This is a step towards building a framebuffer.  The problem is the memory on chip is too small to hold the entire frame buffer.  In this case, I 
build a 4x4 = 16 location memory.  Then in the write phase (which is executed after reset) the memory is written to with some values.

To execute the code - turn on SW[0], SW[1], and SW[2] (these are used in lines 304 to 306 of vga_driver_memory_2.v)
Next, reset the circuit ("rst") by pressing key[0]

The code after rst executes an FSM that does the writing to the memory and then goes into read cycles continuously
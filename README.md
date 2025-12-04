# ECE-287 Final
This is the final project for our Digital Systems Design Class.

For this project we are tasked with creating something complex on the DE1-SoC FPGA boards we are given. Project ideas must be approved by the professor.

Our Proposal was to create a grid based Frogger game system. 

Main Idea:
create a single level that has some obstacles and a player controllable frog(Green square, as pixel art using Verilog would be EXTREMELY difficult). Goal is to use an already pre-existing VGA module that divides the screen into a grid of squares that contain a certain number of pixels. This allows us to essentially store the position of the frog and obstacles(cars) as a singular data point. This should in theory make collision detection a little simpler and moving around the screen appear snappy and make logical sense. 

### Overview

This project implements a simplified, hardware-accelerated version of Frogger on the Intel/Altera DE1-SoC FPGA board using Verilog HDL. The game is fully rendered using a VGA driver, and all gameplay logic (movement, collisions, win conditions, grid updates) runs entirely in hardware.

Our final design features:

- 8×8 Grid Game World

- Real-time VGA output at 640×480

- Hardware frog movement (up, down, left, right)

- Win detection (reaching top row)

- Configurable obstacles (cars/logs)

- Debounced push-button controls

- Cleanly structured modules (VGA, memory, FSM, debounce)

- Designed for actual hardware timing on the DE1-SoC

This project builds on the VGA memory example provided by the instructor

### Player Controls

| DE1-SoC Input | Function                  |
| ------------- | ------------------------- |
| **KEY0**      | Move Down                 |
| **KEY1**      | Move Right                |
| **KEY2**      | Move Up                   |
| **KEY3**      | Move Left                 |
| **SW0**       | Reset Game                |


### Hardware Architecture

1. VGA Display Pipeline

The vga_driver.v module generates:

- Pixel clock

- Horizontal & vertical sync

- Visible region detection

The game world memory feeds pixel color data into the VGA module on each frame.

2. Memory-Backed Renderer

vga_driver_memory_2.v stores an array of game-world values (color identifiers).
The game writes:

 - frog position

 - obstacle positions

 - empty grid

 - win indicator

This module handles translating game state into RGB values on the monitor.

3. Game Logic FSM

 - Input.v Handles:

  - Movement updates

 - vga_driver_memory_2.v handles: 

  - Boundary checking

  - Collision detection

  - Win condition detection

  - Memory writes to update VGA output

This FSM runs synchronously with the 50 MHz system clock.


### How To Play

1. Open .qsf project file into Quartus Prime

2. Open the following files to insure everything runs smoothly:
 - vga_driver_memory_2.v
 - input.v

3. Program onto a DE1_SoC board

4. Switch Monitor to VGA input and Enjoy!
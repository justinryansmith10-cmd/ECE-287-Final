# ECE-287 Final
This is the final project for our Digital Systems Design Class.

For this project we are tasked with creating something complex on the DE1-SoC FPGA boards we are given. Project ideas must be approved by the professor.

Our Proposal was to create a grid based Frogger game system. 

Main Idea:
create a single level that has some obstacles and a player controllable frog(Green square, as pixel art using Verilog would be EXTREMELY difficult). Goal is to use an already pre-existing VGA module that divides the screen into a grid of squares that contain a certain number of pixels. This allows us to essentially store the position of the frog and obstacles(cars) as a singular data point. This should in theory make collision detection a little simpler and moving around the screen appear snappy and make logical sense. 


# Overview

This project implements a simplified, hardware-accelerated version of Frogger on the Intel/Altera DE1-SoC FPGA board using Verilog HDL. The game is fully rendered using a VGA driver, and all gameplay logic (movement, collisions, win conditions, grid updates) runs entirely in hardware.

Our final design features:

8×8 Grid Game World

Real-time VGA output at 640×480

Hardware frog movement (up, down, left, right)

Win detection (reaching top row)

Configurable obstacles (cars/logs)

Debounced push-button controls

Cleanly structured modules (VGA, memory, FSM, debounce)

Designed for actual hardware timing on the DE1-SoC

This project builds on the VGA memory example provided by the instructor





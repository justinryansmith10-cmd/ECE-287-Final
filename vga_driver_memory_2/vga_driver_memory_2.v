module vga_driver_memory_2	(
    	//////////// ADC //////////
	//output		          		ADC_CONVST,
	//output		          		ADC_DIN,
	//input 		          		ADC_DOUT,
	//output		          		ADC_SCLK,

	//////////// Audio //////////
	//input 		          		AUD_ADCDAT,
	//inout 		          		AUD_ADCLRCK,
	//inout 		          		AUD_BCLK,
	//output		          		AUD_DACDAT,
	//inout 		          		AUD_DACLRCK,
	//output		          		AUD_XCK,

	//////////// CLOCK //////////
	//input 		          		CLOCK2_50,
	//input 		          		CLOCK3_50,
	//input 		          		CLOCK4_50,
	input 		          		CLOCK_50,

	//////////// SDRAM //////////
	//output		    [12:0]		DRAM_ADDR,
	//output		     [1:0]		DRAM_BA,
	//output		          		DRAM_CAS_N,
	//output		          		DRAM_CKE,
	//output		          		DRAM_CLK,
	//output		          		DRAM_CS_N,
	//inout 		    [15:0]		DRAM_DQ,
	//output		          		DRAM_LDQM,
	//output		          		DRAM_RAS_N,
	//output		          		DRAM_UDQM,
	//output		          		DRAM_WE_N,

	//////////// I2C for Audio and Video-In //////////
	//output		          		FPGA_I2C_SCLK,
	//inout 		          		FPGA_I2C_SDAT,

	//////////// SEG7 //////////
	output		     [6:0]		HEX0,
	output		     [6:0]		HEX1,
	output		     [6:0]		HEX2,
	output		     [6:0]		HEX3,
	//output		     [6:0]		HEX4,
	//output		     [6:0]		HEX5,

	//////////// IR //////////
	//input 		          		IRDA_RXD,
	//output		          		IRDA_TXD,

	//////////// KEY //////////
	input 		     [3:0]		KEY,

	//////////// LED //////////
	output		     [9:0]		LEDR,

	//////////// PS2 //////////
	//inout 		          		PS2_CLK,
	//inout 		          		PS2_CLK2,
	//inout 		          		PS2_DAT,
	//inout 		          		PS2_DAT2,

	//////////// SW //////////
	input 		     [9:0]		SW,

	//////////// Video-In //////////
	//input 		          		TD_CLK27,
	//input 		     [7:0]		TD_DATA,
	//input 		          		TD_HS,
	//output		          		TD_RESET_N,
	//input 		          		TD_VS,

	//////////// VGA //////////
	output		          		VGA_BLANK_N,
	output reg	     [7:0]		VGA_B,
	output		          		VGA_CLK,
	output reg	     [7:0]		VGA_G,
	output		          		VGA_HS,
	output reg	     [7:0]		VGA_R,
	output		          		VGA_SYNC_N,
	output		          		VGA_VS

	//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
	//inout 		    [35:0]		GPIO_0,

	//////////// GPIO_1, GPIO_1 connect to GPIO Default //////////
	//inout 		    [35:0]		GPIO_1

);

  // Turn off all displays.
	assign	HEX0		=	7'h00;
	assign	HEX1		=	7'h00;
	assign	HEX2		=	7'h00;
	assign	HEX3		=	7'h00;

// DONE STANDARD PORT DECLARATION ABOVE

/* HANDLE SIGNALS FOR CIRCUIT */
wire clk;
wire rst;

assign clk = CLOCK_50;
assign rst = KEY[0];

wire [17:0]SW_db;

debounce_switches db(
.clk(clk),
.rst(rst),
.SW(SW), 
.SW_db(SW_db)
);
/* -------------------------------- */

/* DEBUG SIGNALS */
//assign LEDR[0] = active_pixels;

/* -------------------------------- */
// VGA DRIVER
wire active_pixels; // is on when we're in the active draw space
wire frame_done;

wire [9:0]x; // current x
wire [9:0]y; // current y - 10 bits = 1024 ... a little bit more than we need

vga_driver the_vga(
.clk(clk),
.rst(rst),

.vga_clk(VGA_CLK),

.hsync(VGA_HS),
.vsync(VGA_VS),

.active_pixels(active_pixels),
.frame_done(frame_done),

.xPixel(x),
.yPixel(y),

.VGA_BLANK_N(VGA_BLANK_N),
.VGA_SYNC_N(VGA_SYNC_N)
);

/* -------------------------------- */
/* MEMORY to STORE a MINI frambuffer.  Problem is the FPGA's on-chip memory can't hold an entire frame, so some
form of compression is needed.  I show a simple compress the image to 16 pixels or a 4 by 4, but this memory
could handle more */
reg [14:0] frame_buf_mem_address;
reg [23:0] frame_buf_mem_data;
reg frame_buf_mem_wren;
wire [23:0]frame_buf_mem_q;

/* This memory is 
vga_frame vga_memory(
	frame_buf_mem_address,
	clk,
	frame_buf_mem_data,
	frame_buf_mem_wren,
	frame_buf_mem_q);



/* -------------------------------- */
/* 	FSM to control the writing to the framebuffer and the reading of it.
	I make a 4x4 pixel map in memory.  Then as I read this info I display it 
	noting that the VGA draws in rows, so I have to make sure the right data
	is loaded.  Note, that some of these parameters can be increased. */
reg [15:0]i;
reg [7:0]S;
reg [7:0]NS;
parameter 
	START = 8'd0,
	W2M_INIT = 8'd1, // Write 2 Memory init - this is a FOR loop
	W2M_COND = 8'd2, // Write 2 Memory condion
	W2M_INC = 8'd3, // Write 2 Memory incrementer
	RFM_INIT = 8'd4, // Read From Memory init
	RFM_DRAWING = 8'd5, // Read From Memory draw step
	ERROR = 8'hFF;

parameter LOOP_SIZE = 16'd16;
parameter LOOP_I_SIZE = 16'd4;
parameter WIDTH = 16'd640;
parameter HEIGHT = 16'd480;
parameter PIXELS_IN_WIDTH = WIDTH/LOOP_I_SIZE; // 160
parameter PIXELS_IN_HEIGHT = HEIGHT/LOOP_I_SIZE; // 120

/* Calculate NS */
always @(*)
	case (S)
		START: NS = W2M_INIT;
		W2M_INIT: NS = W2M_COND;
		W2M_COND:
			if (i < LOOP_SIZE)
				NS = W2M_INC;
			else
				NS = RFM_INIT;
		W2M_INC: NS = W2M_COND;
		RFM_INIT: 
			if (frame_done == 1'b0)
				NS = RFM_DRAWING;
			else	
				NS = RFM_INIT;
		RFM_DRAWING:
			if (frame_done == 1'b1)
				NS = RFM_INIT;
			else
				NS = RFM_DRAWING;
		default:	NS = ERROR;
	endcase

	
always @(posedge clk or negedge rst)
begin
	if (rst == 1'b0)
	begin
			S <= START;
	end
	else
	begin
			S <= NS;
	end
end

/* 
The code goes through a write phase (after reset) and an endless read phase once writing is done.

The W2M (write to memory) code is roughly:
for (i = 0; i < 16; i++)
	mem[i] = color // where color is a shade of FF/16 * i if switch is on SW[2:0] for {R, G, B}

The RFM (read from memory) is synced with the VGA display which goes row by row
for (i = 0; i < 480; i++) // height
	for (j = 0; j < 640; j++) // width
		color = mem[(i/120 * 4) + j/160] OR just use x, y coming from vga_driver
		
I later simplified and just used the x and y coming from the vga_driver and used it to calculate the memory load.
		
*/

always @(posedge clk or negedge rst)
begin
	if (rst == 1'b0)
	begin
		frame_buf_mem_address <= 14'd0;
		frame_buf_mem_data <= 24'd0;
		frame_buf_mem_wren <= 1'd0;
		i <= 16'd0;
	end
	else
	begin
		case (S)
			START:
			begin
				frame_buf_mem_address <= 14'd0;
				frame_buf_mem_data <= 24'd0;
				frame_buf_mem_wren <= 1'd0;
				i <= 16'd0;
			end
			W2M_INIT:
			begin
				frame_buf_mem_address <= 14'd0;
				frame_buf_mem_data <= 24'd0;
				frame_buf_mem_wren <= 1'd1;
				i <= 16'd0;
			end
			W2M_COND:
			begin
			end
			W2M_INC: 
			begin
				i <= i + 1'b1;
				frame_buf_mem_address <= frame_buf_mem_address + 1'b1;
				frame_buf_mem_data <= {red, green, blue}; // done in the combinational part below
			end
			RFM_INIT: 
			begin
				frame_buf_mem_wren <= 1'd0; // turn off writing to memory
				// y and x come from the vga_driver module as it progresses through the drawing of the page
				if (y < HEIGHT && x < WIDTH)
					frame_buf_mem_address <= (y/PIXELS_IN_HEIGHT) * LOOP_I_SIZE + (x/PIXELS_IN_WIDTH);
			end
			RFM_DRAWING:
			begin
				// y and x come from the vga_driver module as it progresses through the drawing of the page
				if (y < HEIGHT && x < WIDTH)
					frame_buf_mem_address <= (y/PIXELS_IN_HEIGHT) * LOOP_I_SIZE + (x/PIXELS_IN_WIDTH);
			end	
		endcase
	end
end

reg [7:0]red;
reg [7:0]green;
reg [7:0]blue;

always @(*)
begin
	/* This part is for taking the memory value read out from memory and sending to the VGA */
	if (S == RFM_INIT || S == RFM_DRAWING)
		{VGA_R, VGA_G, VGA_B} = frame_buf_mem_q;
	else
		{VGA_R, VGA_G, VGA_B} = 24'hFFFFFF;
	
	/* this does calculations for R, G, and B in the writing phase */
	red = 8'h0F * (i+1) * SW[2];	
	green = 8'h0F * (i+1) * SW[1];	
	blue = 8'h0F * (i+1) * SW[0];
	
end

endmodule



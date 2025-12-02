module vga_driver_memory_2 (
    input           CLOCK_50,
    input   [3:0]   KEY,
    input   [9:0]   SW,
    output  [6:0]   HEX0,
    output  [6:0]   HEX1,
    output  [6:0]   HEX2,
    output  [6:0]   HEX3,
    output  [9:0]   LEDR,
    output          VGA_BLANK_N,
    output reg [7:0] VGA_B,
    output          VGA_CLK,
    output reg [7:0] VGA_G,
    output          VGA_HS,
    output reg [7:0] VGA_R,
    output          VGA_SYNC_N,
    output          VGA_VS
);

// ---------------------------
// HEX displays off
// ---------------------------
assign HEX0 = 7'h00;
assign HEX1 = 7'h00;
assign HEX2 = 7'h00;
assign HEX3 = 7'h00;

// ---------------------------
// Clock and reset
// ---------------------------
wire clk = CLOCK_50;
wire rst = SW[1];

// ---------------------------
// Debounce switches
// ---------------------------
wire [17:0] SW_db;
debounce_switches db(
    .clk(clk),
    .rst(rst),
    .SW(SW), 
    .SW_db(SW_db)
);

// ---------------------------
// VGA driver
// ---------------------------
wire active_pixels;
wire frame_done;
wire [9:0] x;
wire [9:0] y;

vga_driver the_vga(
    .clk(clk),
    .rst(1'b0),
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

// ---------------------------
// Frog movement
// ---------------------------
parameter LOOP_I_SIZE = 8;
parameter WIDTH = 640;
parameter HEIGHT = 480;
parameter PIXELS_IN_WIDTH = WIDTH / LOOP_I_SIZE;   // 160
parameter PIXELS_IN_HEIGHT = HEIGHT / LOOP_I_SIZE; // 120

reg [31:0] frog_tile = 32'd59; // starting tile
wire at_top = frog_tile < LOOP_I_SIZE;
reg game_won = 1'b0;
reg game_lost = 1'b0;

reg [31:0] obstacle_tile = 32'd23; // starting tile (row 2, rightmost)
reg [31:0] obstacle_tile_2 = 32'd32;

wire up, right, left, down;
input_fsm frog_input(.clk(clk), .rst(rst), .KEY(KEY), .up(up), .right(right), .left(left), .down(down));


always @(posedge clk or posedge rst) begin
    if (rst) begin
        frog_tile <= 32'd59;
		  game_won <= 1'b0;
		  game_lost <= 1'b0;
    end else begin
		if (!game_won || !game_lost) begin 
			 // move on pulses
        if (up && frog_tile >= 8)             frog_tile <= frog_tile - 8;
		  else if (down && frog_tile <= 55)     frog_tile <= frog_tile + 8;
        else if (left && frog_tile % 8 != 0)  frog_tile <= frog_tile - 1;
        else if (right && frog_tile % 8 != 7) frog_tile <= frog_tile + 1;
		  if (at_top && up) begin
				game_won <= 1'b1;
		  end
		  if (frog_tile == obstacle_tile || frog_tile == obstacle_tile_2) begin
				game_lost <= 1'b1;
		  end
    end
end
end

parameter CLOCK_FREQ = 50_000_000;
parameter HALF_SECOND = CLOCK_FREQ / 2; // 25,000,000 cycles


reg [31:0] obstacle_counter = 32'd0; 

always @(posedge clk or posedge rst) begin
    if (rst) begin
        obstacle_tile <= 32'd23;
		  obstacle_tile_2 <= 32'd32;
        obstacle_counter <= 32'd0;
    end
    else if (!game_won || !game_lost) begin
        if (obstacle_counter >= HALF_SECOND - 1) begin
            obstacle_counter <= 32'd0;
            
            // Move obstacle left
            if (obstacle_tile % 8 == 0) begin
                // Reached leftmost position, wrap to right
                obstacle_tile <= 32'd23;
            end
				if ((obstacle_tile_2 + 1) % 8 == 0) begin
					 obstacle_tile_2 <= 32'd32;
				end
            else begin
                // Move one tile left
                obstacle_tile <= obstacle_tile - 1;
					 obstacle_tile_2 <= obstacle_tile_2 + 1;
            end
        end
        else begin
            obstacle_counter <= obstacle_counter + 1;
        end
    end
end

// ---------------------------
// Framebuffer memory (optional, can be simplified for frog demo)
// ---------------------------
reg [14:0] frame_buf_mem_address;
reg [23:0] frame_buf_mem_data;
reg frame_buf_mem_wren;
wire [23:0] frame_buf_mem_q;

vga_frame vga_memory_2(
    .address(frame_buf_mem_address),
    .clock(clk),
    .data(frame_buf_mem_data),
    .wren(frame_buf_mem_wren),
    .q(frame_buf_mem_q)
);


// ---------------------------
// RGB background color
// ---------------------------
reg [7:0] red   = 8'd173;
reg [7:0] green = 8'd216;
reg [7:0] blue  = 8'd230;

wire [9:0] tile_x = x / 40;  // 16 tiles wide
wire [9:0] tile_y = y / 40;  // 12 tiles tall
wire checkered = (tile_x + tile_y) % 2;

// Simple "YOU WIN" text pattern (centered region)
wire in_text_region = (x >= 220 && x < 420) && (y >= 200 && y < 280);
wire [9:0] text_x = x - 220;
wire [9:0] text_y = y - 200;

// Very simple letter patterns (8x10 pixels per character, scaled up)
wire win_letter_pixel;
win_text_rom text_rom(
    .x(text_x / 20),      // 10 characters wide
    .y(text_y / 8),       // 10 rows
    .pixel(win_letter_pixel)
);

wire lose_letter_pixel;
lose_text_rom lose_text(
    .x(text_x / 20),      // 10 characters wide
    .y(text_y / 8),       // 10 rows
    .pixel(lose_letter_pixel)
);

// ---------------------------
// Compute current pixel color
// ---------------------------
wire [31:0] current_tile = (y / PIXELS_IN_HEIGHT) * LOOP_I_SIZE + (x / PIXELS_IN_WIDTH);

wire [23:0] game_pixel_color = 
    (current_tile == frog_tile) ? 24'h00FF00 :
	 (current_tile == obstacle_tile) ? 24'hFF0000 :
	 (current_tile == obstacle_tile_2) ? 24'hFF0000 : {red, green, blue};

wire [23:0] win_pixel_color = 
    in_text_region ? (win_letter_pixel ? 24'hFFFF00 : 24'h0000FF) :
    (checkered ? 24'hFFD700 : 24'hFFA500);
	 
wire [23:0] lost_pixel_color = 
    in_text_region ? (lose_letter_pixel ? 24'hFF0000 : 24'h0000FF) :
    (checkered ? 24'hFF0000 : 24'h000000);

wire [23:0] current_pixel_color = game_won ? win_pixel_color :
											 game_lost ? lost_pixel_color : 
											 game_pixel_color;

// ---------------------------
// VGA output
// ---------------------------
always @(*) begin
    {VGA_R, VGA_G, VGA_B} = current_pixel_color;
end
endmodule

// ---------------------------
// Simple text ROM for "YOU WIN"
// ---------------------------
module win_text_rom(
    input [9:0] x,
    input [9:0] y,
    output reg pixel
);

always @(*) begin
    pixel = 1'b0;
    
    // Simple block letters "WIN"
    if (y >= 1 && y <= 8) begin
        case (x)
            // W - columns 0-4
            0: pixel = (y >= 2);
            1: pixel = (y >= 5);
            2: pixel = (y >= 3);
            3: pixel = (y >= 5);
            4: pixel = (y >= 2);
            // space
            // I - columns 6-7
            6: pixel = (y >= 2 && y <= 7);
            7: pixel = (y >= 2 && y <= 7);
            // space
            // N - columns 9-12
            9: pixel = (y >= 2 && y <= 7);
            10: pixel = (y == 3 || y == 4);
            11: pixel = (y == 5 || y == 6);
            12: pixel = (y >= 2 && y <= 7);
            default: pixel = 1'b0;
        endcase
    end
end
endmodule

module lose_text_rom(
    input [9:0] x,
    input [9:0] y,
    output reg pixel
);

always @(*) begin
    pixel = 1'b0;
    
    // Simple block letters "LOSE"
    if (y >= 1 && y <= 8) begin
        case (x)
            // L - columns 0-2
            0: pixel = (y >= 2 && y <= 7);
            1: pixel = (y == 7);
            2: pixel = (y == 7);
            // space
            // O - columns 4-6
            4: pixel = (y >= 2 && y <= 7);
            5: pixel = (y == 2 || y == 7);
            6: pixel = (y >= 2 && y <= 7);
            // space
            // S - columns 8-10
            8: pixel = (y >= 2 && y <= 7);
            9: pixel = (y == 2 || y == 4 || y == 7);
            10: pixel = (y >= 2 && y <= 7);
            // space
            // E - columns 12-14
            12: pixel = (y >= 2 && y <= 7);
            13: pixel = (y == 2 || y == 4 || y == 7);
            14: pixel = (y == 2 || y == 4 || y == 7);
            default: pixel = 1'b0;
        endcase
    end
end
endmodule
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

// (unused LEDs for now)
assign LEDR = 10'd0;

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
// Frog movement / game state
// ---------------------------
parameter LOOP_I_SIZE       = 8;
parameter WIDTH             = 640;
parameter HEIGHT            = 480;
parameter PIXELS_IN_WIDTH   = WIDTH  / LOOP_I_SIZE;   // 80
parameter PIXELS_IN_HEIGHT  = HEIGHT / LOOP_I_SIZE;   // 60

reg  [5:0] frog_tile = 6'd59; // starting tile
reg  [5:0] next_frog_tile;
wire       at_top = frog_tile < LOOP_I_SIZE;

reg        game_won  = 1'b0;
reg        game_lost = 1'b0;
reg  [1:0] level     = 2'd0;   // 0 = first level, 1 = second level, 2 = third

// obstacle tiles (0..63)
reg [5:0] obstacle_tile   = 6'd23;
reg [5:0] obstacle_tile_2 = 6'd32;
reg [5:0] obstacle_tile_3 = 6'd1;
reg [5:0] obstacle_tile_4 = 6'd62;
reg [5:0] obstacle_tile_5 = 6'd27; // static
reg [5:0] obstacle_tile_6 = 6'd28; // static

wire up, right, left, down;
input_fsm frog_input(
    .clk(clk),
    .rst(rst),
    .KEY(KEY),
    .up(up),
    .right(right),
    .left(left),
    .down(down)
);

// ---------------------------
// Next frog tile (combinational)
// ---------------------------
always @(*) begin
    next_frog_tile = frog_tile;

    if (!game_won && !game_lost) begin
        // move on pulses
        if (up && frog_tile >= 8)
            next_frog_tile = frog_tile - 8;
        else if (down && frog_tile <= 55)
            next_frog_tile = frog_tile + 8;
        else if (left && frog_tile % 8 != 0)
            next_frog_tile = frog_tile - 1;
        else if (right && frog_tile % 8 != 7)
            next_frog_tile = frog_tile + 1;
    end
end

// ---------------------------
// Frog + level + collision (sequential)
// ---------------------------
always @(posedge clk or posedge rst) begin
    if (rst) begin
        frog_tile  <= 6'd59;
        game_won   <= 1'b0;
        game_lost  <= 1'b0;
        level      <= 2'd0;
    end else begin
        if (!game_won && !game_lost) begin
            // apply movement
            frog_tile <= next_frog_tile;

            // check if we reached the top row on an up move
            if (up && (next_frog_tile < LOOP_I_SIZE)) begin
                if (level == 2'd0) begin
                    // advance to level 1
                    level      <= 2'd1;
                    frog_tile  <= 6'd59; // restart frog at bottom
                    game_lost  <= 1'b0;
                end else if (level == 2'd1) begin
                    // advance to level 2
                    level      <= 2'd2;
                    frog_tile  <= 6'd59;
                    game_lost  <= 1'b0;
                end else begin
                    // already on level 2 and reached top -> win
                    game_won <= 1'b1;
                end
            end

            // collision check using the NEW tile
            if (next_frog_tile == obstacle_tile   ||
                next_frog_tile == obstacle_tile_2 ||
                next_frog_tile == obstacle_tile_3 ||
                next_frog_tile == obstacle_tile_4 ||
                next_frog_tile == obstacle_tile_5 ||
                next_frog_tile == obstacle_tile_6) begin
                game_lost <= 1'b1;
            end
        end
    end
end

// ---------------------------
// Obstacle movement + level-based patterns
// ---------------------------
parameter CLOCK_FREQ  = 50_000_000;
parameter HALF_SECOND = CLOCK_FREQ / 2; // 25,000,000 cycles

reg [31:0] obstacle_counter = 32'd0;
reg [1:0]  level_prev       = 2'd0;    // track last level to detect changes

always @(posedge clk or posedge rst) begin
    if (rst) begin
        // initial LEVEL 0 positions
        obstacle_tile   <= 6'd23;
        obstacle_tile_2 <= 6'd32;
        obstacle_tile_3 <= 6'd1;
        obstacle_tile_4 <= 6'd62;
        obstacle_tile_5 <= 6'd27;
        obstacle_tile_6 <= 6'd28;

        obstacle_counter <= 32'd0;
        level_prev       <= 2'd0;
    end else begin
        // detect level change and reset obstacles immediately
        if (level != level_prev) begin
            level_prev       <= level;
            obstacle_counter <= 32'd0;

            if (level == 2'd0) begin
                // back to LEVEL 0 pattern
                obstacle_tile   <= 6'd23;
                obstacle_tile_2 <= 6'd32;
                obstacle_tile_3 <= 6'd1;
                obstacle_tile_4 <= 6'd62;
                obstacle_tile_5 <= 6'd27;
                obstacle_tile_6 <= 6'd28;
            end else if (level == 2'd1) begin
                // initial LEVEL 1 positions (example pattern)
                obstacle_tile   <= 6'd47;
                obstacle_tile_2 <= 6'd32;
                obstacle_tile_3 <= 6'd23;
                obstacle_tile_4 <= 6'd8;
                obstacle_tile_5 <= 6'd27; // still static if you want
                obstacle_tile_6 <= 6'd28;
            end else if (level == 2'd2) begin
                // initial LEVEL 2 positions
                obstacle_tile   <= 6'd7;
                obstacle_tile_2 <= 6'd0;
                obstacle_tile_3 <= 6'd24;
                obstacle_tile_4 <= 6'd31;
                obstacle_tile_5 <= 6'd25;
                obstacle_tile_6 <= 6'd30;
            end
        end else if (!game_won && !game_lost) begin
            // normal obstacle movement
            if (obstacle_counter >= HALF_SECOND - 1) begin
                obstacle_counter <= 32'd0;

                if (level == 2'd0) begin
                    // ----- LEVEL 0 PATTERN -----
                    // Obstacle 1: move left, wrap to right
                    if (obstacle_tile % 8 == 0)
                        obstacle_tile <= 6'd23;
                    else
                        obstacle_tile <= obstacle_tile - 1;

                    // Obstacle 2: move right, wrap to left
                    if ((obstacle_tile_2 + 1) % 8 == 0)
                        obstacle_tile_2 <= 6'd32;
                    else
                        obstacle_tile_2 <= obstacle_tile_2 + 1;

                    // Obstacle 3: move down
                    if (obstacle_tile_3 > 54)
                        obstacle_tile_3 <= 6'd1;
                    else
                        obstacle_tile_3 <= obstacle_tile_3 + 8;

                    // Obstacle 4: move up
                    if (obstacle_tile_4 < 8)
                        obstacle_tile_4 <= 6'd62;
                    else
                        obstacle_tile_4 <= obstacle_tile_4 - 8;

                    // 5 & 6 are static here

                end else if (level == 2'd1) begin
                    // ----- LEVEL 1 PATTERN -----
                    // Obstacle 1: move left, wrap
                    if (obstacle_tile % 8 == 0)
                        obstacle_tile <= 6'd47;
                    else
                        obstacle_tile <= obstacle_tile - 1;

                    // Obstacle 2: move right, wrap
                    if ((obstacle_tile_2 + 1) % 8 == 0)
                        obstacle_tile_2 <= 6'd32;
                    else
                        obstacle_tile_2 <= obstacle_tile_2 + 1;

                    // Obstacle 3: move left, wrap
                    if (obstacle_tile_3 % 8 == 0)
                        obstacle_tile_3 <= 6'd23;
                    else
                        obstacle_tile_3 <= obstacle_tile_3 - 1;

                    // Obstacle 4: move right, wrap
                    if ((obstacle_tile_4 + 1) % 8 == 0)
                        obstacle_tile_4 <= 6'd8;
                    else
                        obstacle_tile_4 <= obstacle_tile_4 + 1;

                    // 5 & 6 static

                end else if (level == 2'd2) begin
                    // ----- LEVEL 2 PATTERN -----
                    if (obstacle_tile == 6'd56)
                        obstacle_tile <= 6'd7;
                    else
                        obstacle_tile <= obstacle_tile + 7;

                    if (obstacle_tile_2 == 6'd63)
                        obstacle_tile_2 <= 6'd0;
                    else
                        obstacle_tile_2 <= obstacle_tile_2 + 9;
                    // 5 & 6 static for now
                end
            end else begin
                obstacle_counter <= obstacle_counter + 1;
            end
        end
    end
end

// ---------------------------
// Framebuffer memory (unused here but kept)
// ---------------------------
reg [14:0] frame_buf_mem_address;
reg [23:0] frame_buf_mem_data;
reg        frame_buf_mem_wren;
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
wire       checkered = (tile_x + tile_y) % 2;

// Simple "YOU WIN" / "YOU LOSE" text region
wire in_text_region = (x >= 220 && x < 420) && (y >= 200 && y < 280);
wire [9:0] text_x = x - 220;
wire [9:0] text_y = y - 200;

wire win_letter_pixel;
win_text_rom text_rom(
    .x(text_x / 20),      // 10 characters wide
    .y(text_y / 8),       // 10 rows
    .pixel(win_letter_pixel)
);

wire lose_letter_pixel;
lose_text_rom lose_text(
    .x(text_x / 20),
    .y(text_y / 8),
    .pixel(lose_letter_pixel)
);

// ---------------------------
// Compute current pixel color
// ---------------------------
wire [31:0] current_tile =
    (y / PIXELS_IN_HEIGHT) * LOOP_I_SIZE +
    (x / PIXELS_IN_WIDTH);

wire [23:0] level_0_color = 24'hFF0000;
wire [23:0] level_1_color = 24'h0000FF;
wire [23:0] level_2_color = 24'h000000;

// Choose obstacle color based on level
wire [23:0] obstacle_color =
    (level == 2'd0) ? level_0_color :
    (level == 2'd1) ? level_1_color :
                      level_2_color;  // default for level 2 and others

// Is this tile any obstacle?
wire is_obstacle_tile =
       (current_tile == obstacle_tile)
    || (current_tile == obstacle_tile_2)
    || (current_tile == obstacle_tile_3)
    || (current_tile == obstacle_tile_4)
    || (current_tile == obstacle_tile_5)
    || (current_tile == obstacle_tile_6);

// Final pixel color
wire [23:0] game_pixel_color =
    (current_tile == frog_tile) ? 24'h00FF00 :         // frog = green
    (is_obstacle_tile)          ? obstacle_color :     // obstacles = level-based color
                                  {red, green, blue};  // background

wire [23:0] win_pixel_color = 
    in_text_region ? (win_letter_pixel ? 24'hFFFF00 : 24'h0000FF) :
    (checkered ? 24'hFFD700 : 24'hFFA500);
	 
wire [23:0] lost_pixel_color = 
    in_text_region ? (lose_letter_pixel ? 24'hFF0000 : 24'h0000FF) :
    (checkered ? 24'hFF0000 : 24'h000000);

wire [23:0] current_pixel_color =
    game_won  ? win_pixel_color  :
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
// Simple text ROM for "WIN"
// ---------------------------
module win_text_rom(
    input  [9:0] x,
    input  [9:0] y,
    output reg   pixel
);
always @(*) begin
    pixel = 1'b0;
    
    // Simple block letters "WIN"
    if (y >= 1 && y <= 8) begin
        case (x)
            // W - columns 0-4
            0:  pixel = (y >= 2);
            1:  pixel = (y >= 5);
            2:  pixel = (y >= 3);
            3:  pixel = (y >= 5);
            4:  pixel = (y >= 2);
            // space
            // I - columns 6-7
            6:  pixel = (y >= 2 && y <= 7);
            7:  pixel = (y >= 2 && y <= 7);
            // space
            // N - columns 9-12
            9:  pixel = (y >= 2 && y <= 7);
            10: pixel = (y == 3 || y == 4);
            11: pixel = (y == 5 || y == 6);
            12: pixel = (y >= 2 && y <= 7);
            default: pixel = 1'b0;
        endcase
    end
end
endmodule

// ---------------------------
// Simple text ROM for "LOSE"
// ---------------------------
module lose_text_rom(
    input  [9:0] x,
    input  [9:0] y,
    output reg   pixel
);
always @(*) begin
    pixel = 1'b0;
    
    // Simple block letters "LOSE"
    if (y >= 1 && y <= 8) begin
        case (x)
            // L - columns 0-2
            0:  pixel = (y >= 2 && y <= 7);
            1:  pixel = (y == 7);
            2:  pixel = (y == 7);
            // space
            // O - columns 4-6
            4:  pixel = (y >= 2 && y <= 7);
            5:  pixel = (y == 2 || y == 7);
            6:  pixel = (y >= 2 && y <= 7);
            // space
            // S - columns 8-10
            8:  pixel = (y >= 2 && y <= 7);
            9:  pixel = (y == 2 || y == 4 || y == 7);
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

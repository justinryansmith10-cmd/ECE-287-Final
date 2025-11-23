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
wire rst = KEY[0];

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

// ---------------------------
// Frog movement
// ---------------------------
reg [5:0] frog_tile = 6'd14; // starting tile

wire up, right, left;
input_fsm frog_input(.clk(clk), .rst(rst), .KEY(KEY), .up(up), .right(right), .left(left));

always @(posedge clk or negedge rst) begin
    if (!rst)
        frog_tile <= 6'd14;
    else begin
        // move on pulses
        if (up && frog_tile >= 4)             frog_tile <= frog_tile - 4;
        else if (left && frog_tile % 4 != 0)  frog_tile <= frog_tile - 1;
        else if (right && frog_tile % 4 != 3) frog_tile <= frog_tile + 1;
    end
end

// ---------------------------
// Framebuffer memory (optional, can be simplified for frog demo)
// ---------------------------
reg [14:0] frame_buf_mem_address;
reg [23:0] frame_buf_mem_data;
reg frame_buf_mem_wren;
wire [23:0] frame_buf_mem_q;

vga_frame vga_memory(
    frame_buf_mem_address,
    clk,
    frame_buf_mem_data,
    frame_buf_mem_wren,
    frame_buf_mem_q
);

parameter LOOP_I_SIZE = 4;
parameter WIDTH = 640;
parameter HEIGHT = 480;
parameter PIXELS_IN_WIDTH = WIDTH / LOOP_I_SIZE;   // 160
parameter PIXELS_IN_HEIGHT = HEIGHT / LOOP_I_SIZE; // 120

// ---------------------------
// RGB background color
// ---------------------------
reg [7:0] red   = 8'd173;
reg [7:0] green = 8'd216;
reg [7:0] blue  = 8'd230;

// ---------------------------
// Compute current pixel color
// ---------------------------
wire [5:0] current_tile = (y / PIXELS_IN_HEIGHT) * LOOP_I_SIZE + (x / PIXELS_IN_WIDTH);

wire [23:0] current_pixel_color = 
    (current_tile == frog_tile) ? 24'h00FF00 : {red, green, blue};

// ---------------------------
// VGA output
// ---------------------------
always @(*) begin
    {VGA_R, VGA_G, VGA_B} = current_pixel_color;
end

endmodule

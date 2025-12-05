module lfsr6(
    input        clk,
    output reg [5:0] rnd,      // full 6-bit LFSR state
    output      [2:0] rnd_3bit // value in range 0–7
);

wire feedback = rnd[5] ^ rnd[4];

// power-on seed (Quartus will treat this as initial value in regs)
initial begin
    rnd = 6'b000001;           // non-zero
end

always @(posedge clk) begin
    rnd <= {rnd[4:0], feedback};
end

assign rnd_3bit = rnd[2:0];

endmodule


module input_fsm(
    input  clk,
    input  rst,
    input  [3:0] KEY,
    output reg up,
    output reg right,
    output reg left,
	 output reg down
);

// invert keys (KEY is active low)
wire [3:0] key_n = ~KEY;

reg [2:0] S, NS;
reg [27:0] count;

parameter START = 3'd0,
          MOVE  = 3'd1,
          WAIT  = 3'd2;

parameter TARGET = 28'd12500000; // 0.25 sec at 50 MHz

// Sync FSM
always @(posedge clk or posedge rst)
    if (rst) S <= START;
    else      S <= NS;

// Next-state logic
always @(*) begin
    case (S)
        START: NS = MOVE;

        MOVE: begin
            if (key_n[2] | key_n[1] | key_n[3] | key_n[0])
                NS = WAIT;
            else
                NS = MOVE;
        end

        WAIT: begin
            if (count < TARGET-1)
                NS = WAIT;
            else
                NS = MOVE;
        end

        default: NS = START;
    endcase
end

// Output logic (generate pulses)
always @(posedge clk or posedge rst) begin
    if (rst) begin
        up <= 0;
        right <= 0;
        left <= 0;
		  down <= 0;
        count <= 0;
    end 
    else begin
        case (S)
            MOVE: begin
                count <= 0;

                // generate ONE-CYCLE pulses
                up    <= key_n[2];
                right <= key_n[1];
                left  <= key_n[3];
					 down <= key_n[0];
            end

            WAIT: begin
                count <= count + 1;
                
                // hold pulses LOW during wait
                up <= 0;
                right <= 0;
                left <= 0;
					 down <= 0;
            end
        endcase
    end
end

endmodule

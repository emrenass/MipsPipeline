////////////////////////////////////////////////////////
//  
//  display_controller.sv
//
//  by Will Sawyer  31 March 2017
//
//  puts 4 hexadecimal values (from O to F) on the 4-digit 7-segment display unit
//     
//
//  the AN, Cx and DP outputs are active-low, for the BASYS3 board
//    AN3 is the left-most digit, AN2 is the second-left-most, etc
//    C[6] is CA for the a segment, c[5] is CB for the b segment, etc
//   
//  Uses the 100 MHz board clock for clk, and uses a clear signal for resetting
//  Takes 4 active-high enable signals, 1 per digit, to enable 
//     or disable display digits
//
//  For correct connections, carefully plan what should be in the .XDC file
//   
//  
////////////////////////////////////////////////////////

module display_controller(
 input logic clk, clear,
 input logic [3:0] in0, in1, in2, in3,        // 4 values for 4 digits (decimal value)
 output logic [3:0] an, // anode: 4-bit enable signal (active low)
 output logic [6:0] C,
 output logic dp //individual LED output for the 7-segment along with the digital point
 );
 
// divide system clock (100Mhz for Basys3) by 2^N using a counter, which allows us to multiplex at lower speed
localparam N = 18;
logic [N-1:0] count = {N{1'b0}}; //initial value
always_ff @(posedge clk, posedge clear)
		if(clear) count <= 0;
		else count <= count + 1;

 
logic [3:0]digit_val; // 7-bit register to hold the current data on output
logic [3:0]digit_en;  //register for enable vector
 
always_comb
 begin
 
 
  case(count[N-1:N-2]) //using only the 2 MSB's of the counter 
    
   2'b00 :  //select first 7Seg.
    begin
     digit_val = in0;
     digit_en = 4'b1110;
    end
    
   2'b01:  //select second 7Seg.
    begin
     digit_val = in1;
     digit_en = 4'b1101;
    end
    
   2'b10:  //select third 7Seg.
    begin
     digit_val = in2;
     digit_en = 4'b1011;
    end
     
   2'b11:  //select forth 7Seg.
    begin
     digit_val = in3;
     digit_en = 4'b0111;
    end
    
    default:
        begin
             digit_en = 4'b1111; //default
             digit_val = in0; //default
         end
  endcase
 end
 

//Convert digit number to LED vector. LEDs are active low.
logic [6:0] sseg_LEDs; 
always_comb
 begin 
  case(digit_val)
   4'd0 : sseg_LEDs = 7'b1000000; //to display 0
   4'd1 : sseg_LEDs = 7'b1111001; //to display 1
   4'd2 : sseg_LEDs = 7'b0100100; //to display 2
   4'd3 : sseg_LEDs = 7'b0110000; //to display 3
   4'd4 : sseg_LEDs = 7'b0011001; //to display 4
   4'd5 : sseg_LEDs = 7'b0010010; //to display 5
   4'd6 : sseg_LEDs = 7'b0000010; //to display 6
   4'd7 : sseg_LEDs = 7'b1111000; //to display 7
   4'd8 : sseg_LEDs = 7'b0000000; //to display 8
   4'd9 : sseg_LEDs = 7'b0010000; //to display 9
   4'b1010: sseg_LEDs = 7'b0001000;  // A
   4'b1011: sseg_LEDs = 7'b0000000;  // b
   4'b1100: sseg_LEDs = 7'b1000110;  // c
   4'b1101: sseg_LEDs = 7'b1000000;  // d
   4'b1110: sseg_LEDs = 7'b0000110;  // E
   4'b1111: sseg_LEDs = 7'b0001110;  // F
   default: sseg_LEDs = 7'bxxxxxxx;
  endcase
 end
 
assign an = digit_en; 
assign C = sseg_LEDs; 
assign dp = 1'b1; //turn dp off
 
 
endmodule
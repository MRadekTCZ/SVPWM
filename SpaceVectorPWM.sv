module SVPWM_gen(CLK, sektor, time_vector1, time_vector2, T_1, T_2, T_3, T_4, T_5, T_6);
  // Input ports:
parameter Timp = 250; 
input CLK;
input [3:0]sektor;
input [7:0]time_vector1;
input [7:0]time_vector2;

// Output ports:
output T_1;
output T_2;
output T_3;
output T_4;
output T_5;
output T_6;

reg [2:0] OUT[5:0] = '{3'b100,3'b110,3'b010,3'b011,3'b001,3'b101};
reg [2:0] OUT0;
reg T_1;
reg T_2;
reg T_3;
reg T_4;
reg T_5;
reg T_6;
reg [8:0] Timp_2 = Timp * 2;
reg [8:0] imp_counter = 1'b0;
// Module logic:


always @(posedge CLK)
begin
  if((T_1+T_2+T_3) == 2)
	  begin
		OUT0 <= 3'b111;
		end
	else 
		begin 
		OUT0 <= 3'b000;
		end
		
		
	if((T_1+T_2+T_3) == 2)
	  begin
		OUT0 <= 3'b111;
		end	
	if ((time_vector1 <=15) && (time_vector2<=15))
		begin
		T_1 <= 0;
		T_4 <= 0;
		T_2 <= 0;
		T_5 <= 0;
		T_3 <= 0;
		T_6 <= 0;
		end

	else if (imp_counter<=time_vector1)
		begin
		T_1 <= OUT[sektor][0];
		T_4 <= !OUT[sektor][0];
		T_2 <= OUT[sektor][1];
		T_5 <= !OUT[sektor][1];
		T_3 <= OUT[sektor][2];
		T_6 <= !OUT[sektor][2];
		end
	else if (imp_counter<=(time_vector1+time_vector2))
		begin
		if (sektor==5)
			begin
			T_1 <= OUT[0][0];
			T_4 <= !OUT[0][0];
			T_2 <= OUT[0][1];
			T_5 <= !OUT[0][1];
			T_3 <= OUT[0][2];
			T_6 <= !OUT[0][2];
			end
		else
			begin
			T_1 <= OUT[sektor+1][0];
			T_4 <= !OUT[sektor+1][0];
			T_2 <= OUT[sektor+1][1];
			T_5 <= !OUT[sektor+1][1];
			T_3 <= OUT[sektor+1][2];
			T_6 <= !OUT[sektor+1][2];
			end
		end
	else if (imp_counter<=Timp)
		begin
		T_1 <= OUT0[0];
		T_4 <= !OUT0[0];
		T_2 <= OUT0[1];
		T_5 <= !OUT0[1];
		T_3 <= OUT0[2];
		T_6 <= !OUT0[2];
		end
	else if (imp_counter<=(Timp+time_vector2))
		begin
		if (sektor==5)
			begin
			T_1 <= OUT[0][0];
			T_4 <= !OUT[0][0];
			T_2 <= OUT[0][1];
			T_5 <= !OUT[0][1];
			T_3 <= OUT[0][2];
			T_6 <= !OUT[0][2];
			end
		else
			begin
			T_1 <= OUT[sektor+1][0];
			T_4 <= !OUT[sektor+1][0];
			T_2 <= OUT[sektor+1][1];
			T_5 <= !OUT[sektor+1][1];
			T_3 <= OUT[sektor+1][2];
			T_6 <= !OUT[sektor+1][2];
			end
		end
		else if (imp_counter<=(Timp+time_vector2+time_vector1))
		begin
		T_1 <= OUT[sektor][0];
		T_4 <= !OUT[sektor][0];
		T_2 <= OUT[sektor][1];
		T_5 <= !OUT[sektor][1];
		T_3 <= OUT[sektor][2];
		T_6 <= !OUT[sektor][2];
		end
		else if (imp_counter<=Timp_2)
		begin
		T_1 = OUT0[0];
		T_4 = !OUT0[0];
		T_2 = OUT0[1];
		T_5 = !OUT0[1];
		T_3 = OUT0[2];
		T_6 = !OUT0[2];
		end

		
	if(imp_counter >= Timp_2)
		begin
			imp_counter <= 0;
		end
	else
		begin
			imp_counter <= imp_counter + 1;
		end
		
	
end

endmodule
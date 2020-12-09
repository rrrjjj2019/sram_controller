`include "para.v"

module fsram(
	input								clk,
	input								CENA,
	input								CENB,
	input	[`SRAM_NUM - 1 : 0]			WENA,
	input	[`SRAM_NUM - 1 : 0]			WENB,
	input	[`SRAM_NUM * 12 - 1 : 0]	AA,
	input	[`SRAM_NUM * 16 - 1 : 0]	DA,
	input	[`SRAM_NUM * 12 - 1 : 0]	AB,
	input	[`SRAM_NUM * 16 - 1 : 0]	DB,
	output	[`SRAM_NUM * 16 - 1 : 0]	QA,
	output	[`SRAM_NUM * 16 - 1 : 0]	QB
);

wire	CLKA;
wire	CLKB;

assign CLKA = clk;
assign CLKB = ~clk;

generate
	genvar i;
	for(i = 0; i < `SRAM_NUM; i = i + 1) begin: mem
		sram_dp_hde sram(
			.CENYA(),
			.WENYA(),
			.AYA(),
			.DYA(),
			.CENYB(),
			.WENYB(),
			.AYB(),
			.DYB(),
			.QA(QA[(i+1) * 16 - 1 -: 16]),
			.QB(QB[(i+1) * 16 - 1 -: 16]),
			.CLKA(CLKA),
			.CENA(CENA),
			.WENA(WENA[i]),
			.AA(AA[(i+1) * 12 - 1 -: 12]),
			.DA(DA[(i+1) * 16 - 1 -: 16]),
			.CLKB(CLKB),
			.CENB(CENB),
			.WENB(WENB[i]),
			.AB(AB[(i+1) * 12 - 1 -: 12]),
			.DB(DB[(i+1) * 16 - 1 -: 16]),
			.EMAA(3'b000),
			.EMAWA(2'b00),
			.EMASA(1'b0),
			.EMAB(3'b000),
			.EMAWB(2'b00),
			.EMASB(1'b0),
			.TENA(~1'b0),
			.BENA(~1'b0),
			.TCENA(),
			.TWENA(),
			.TAA(),
			.TDA(),
			.TQA(),
			.TENB(~1'b0),
			.BENB(~1'b0),
			.TCENB(),
			.TWENB(),
			.TAB(),
			.TDB(),
			.TQB(),
			.RET1N(1'b1),
			.STOVA(1'b0),
			.STOVB(1'b0),
			.COLLDISN()
			);
	end
endgenerate

endmodule
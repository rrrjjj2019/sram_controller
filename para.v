`timescale 1ns/1ps

`define	CHANNEL_IN 	4
`define	CHANNEL_OUT	32
`define	PEA_num		4
`define	filter_num	32
`define	CH_NUM 		32

`define	LAYER_NUM	1
// `define	ROW 		6
// `define	COL 		6
`define TILE_NUM	0
`define SRAM_NUM 	32

`define	ROW 		16
`define	COL 		256

`define ALL_FSM_IDLE   7'b0000000
//TOP SECTION FINITE STATE MACHINES
`define ACTIVATE_TOP_FSM0  7'b0000001
`define ACTIVATE_TOP_FSM1  7'b0000010
`define ACTIVATE_TOP_FSM2  7'b0000100
//MIDDLE SECTION FINITE STATE MACHINES
`define ACTIVATE_MIDDLE_FSM0  7'b0001000
`define ACTIVATE_MIDDLE_FSM1  7'b0010000

// =======================
// Test Bench
// =======================
`define NULL		0
`define	HALF_CLK	5
// `define DATA_ROW	8
// `define DATA_COL	8
`define WEIGHT_ROW	3
`define WEIGHT_COL	3
// `define OUTPUT_ROW	6
// `define OUTPUT_COL	6

`define DATA_ROW	17
`define DATA_COL	258
`define OUTPUT_ROW	16
`define OUTPUT_COL	256

`define CHIP_ENABLE ~1'b1
`define CHIP_DISABLE ~1'b0
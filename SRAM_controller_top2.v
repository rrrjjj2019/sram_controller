module sram_controller_top(

input									clk,
input									rst_n,
input									start,

// ============================================
// FSRAM1 (Real SRAM)
// ============================================
input [`CHANNEL_OUT  * 8 - 1 : 0]	data_in_1,
input [`CHANNEL_OUT * 16 - 1 : 0]	data_in_1_2,

// ============================================
// FSRAM2 (Real SRAM)
// ============================================
input [`CHANNEL_OUT * 8 - 1 : 0]	data_in_2,
input [`CHANNEL_OUT * 16 - 1 : 0]	data_in_2_2
);

    wire								CENA_1;
    wire								CENB_1;
    wire [`SRAM_NUM - 1 : 0]			WENA_1;
    wire [`SRAM_NUM - 1 : 0]			WENB_1;
    wire [11:0]					        AA_1;
    wire [`SRAM_NUM * 16 - 1 : 0]	    DA_1;
    wire [11:0]					        AB_1;
    wire [`SRAM_NUM * 16 - 1 : 0]	    DB_1;


    wire								CENA_2;
    wire								CENB_2;
    wire [`SRAM_NUM - 1 : 0]			WENA_2;
    wire [`SRAM_NUM - 1 : 0]			WENB_2;
    wire [11:0]					        AA_2;
    wire [`SRAM_NUM * 16 - 1 : 0]	    DA_2;
    wire [11:0]					        AB_2;
    wire [`SRAM_NUM * 16 - 1 : 0]	    DB_2;


    wire [`SRAM_NUM * 16 - 1 : 0]	QA_1;
    wire [`SRAM_NUM * 16 - 1 : 0]	QB_1;
    wire [`SRAM_NUM * 16 - 1 : 0]	QA_2;
    wire [`SRAM_NUM * 16 - 1 : 0]	QB_2;

    wire [2:0] curr_state_FSM;


    sram_controller FSM1(
        .clk(clk),
        .rst_n(rst_n),
        .start(start),

        // ============================================
        // FSRAM1 (Real SRAM)
        // ============================================
        .data_in_1(data_in_1),
        .data_in_1_2(data_in_1_2),
        .CENA_1(CENA_1),
        .CENB_1(CENB_1),
        .WENA_1(WENA_1),
        .WENB_1(WENB_1),
        .AA_1(AA_1),
        .DA_1(DA_1),
        .AB_1(AB_1),
        .DB_1(DB_1),

        // ============================================
        // FSRAM2 (Real SRAM)
        // ============================================
        .data_in_2(data_in_2),
        .data_in_2_2(data_in_2_2),
        .CENA_2(CENA_2),
        .CENB_2(CENB_2),
        .WENA_2(WENA_2),
        .WENB_2(WENB_2),
        .AA_2(AA_2),
        .DA_2(DA_2),
        .AB_2(AB_2),
        .DB_2(DB_2),

        // ============================================
        // WSRAM (Real SRAM)
        // ============================================
        .weight_in(),
        .CEN_w(),
        .WEN_w(),
        .A_w(),
        .D_w(),

        // ============================================
        // IRSRAM (Real SRAM)
        // ============================================
        .CEN_ir(),
        .WEN_ir(),
        .A_ir(),
        .D_ir(),

        // ============================================
        // ORSRAM (Real SRAM)
        // ============================================
        .CEN_or(),
        .WEN_or(),
        .A_or(),
        .D_or(),


        // ============================================
        // Data Process
        // 0: idle
        // 1: 3 zeros
        // 2: pad 1 zeros forward
        // 3: pad 1 zeros backward
        // 4: 1 zeros
        // 5: front data ([15 : 8])
        // 6: back data ([7 : 0])
        // ============================================
        // sram_sel1: For data process module to know which sram to be select
        // ============================================
        .sram_sel1(),
        .sram_sel2(),
        .data_process_reg(),

        // ============================================
        // FSRAM ready, tell CCM start to count
        // ============================================
        .CCM_en(),
        .CCM_en_cnt(),
        .Weight_en(),

        //for debug
        .curr_state_FSM(curr_state_FSM)
    );

    fsram fsram1(
        .clk(clk),
        .CENA(CENA_1),
        .CENB(CENB_1),
        .WENA(WENA_1),
        .WENB(WENB_1),
        .AA({`SRAM_NUM{AA_1}}),
        .DA(DA_1),
        .AB({`SRAM_NUM{AB_1}}),
        .DB(DB_1),
        .QA(QA_1),
        .QB(QB_1)
    );

    fsram fsram2(
        .clk(clk),
        .CENA(CENA_2),
        .CENB(CENB_2),
        .WENA(WENA_2),
        .WENB(WENB_2),
        .AA({`SRAM_NUM{AA_2}}),
        .DA(DA_2),
        .AB({`SRAM_NUM{AB_2}}),
        .DB(DB_2),
        .QA(QA_2),
        .QB(QB_2)
    );


endmodule

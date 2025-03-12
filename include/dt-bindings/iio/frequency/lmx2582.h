/*
 * exas Instruments LMX 2582 High Performance, Wideband
 * PLLatinum RF Synthesizer With Integrated VCO
 *
 * Copyright 2025 PrecisionWave AG
 *
 * Licensed under the GPL-2.
 */

#ifndef _DT_BINDINGS_IIO_FREQUENCY_LMX2582_H_
#define _DT_BINDINGS_IIO_FREQUENCY_LMX2582_H_

/* Input signal path multiplier
 * use for lmx,mult */
#define LMX2582_MULT_BYPASS	1
#define LMX2582_MULT_3		3
#define LMX2582_MULT_4		4
#define LMX2582_MULT_5		5
#define LMX2582_MULT_6		6

/* PFD Mode
 * use for lmx,pfd-ctl */
#define LMX2582_PFD_CTL_DUAL	0
#define LMX2582_PFD_CTL_SINGLE	1

/* Charge pump current
 * use for lmx,cp-idn and lmx,cp-iup */
#define LMX2582_CP_I_0MA	0
#define LMX2582_CP_I_0_156MA	1
#define LMX2582_CP_I_0_312MA	2
#define LMX2582_CP_I_0_468MA	3
#define LMX2582_CP_I_0_625MA	4
#define LMX2582_CP_I_0_781MA	5
#define LMX2582_CP_I_0_937MA	6
#define LMX2582_CP_I_1_093MA	7
#define LMX2582_CP_I_2_5MA	8
#define LMX2582_CP_I_2_656MA	9
#define LMX2582_CP_I_2_812MA	10
#define LMX2582_CP_I_2_968MA	11
#define LMX2582_CP_I_3_125MA	12
#define LMX2582_CP_I_3_281MA	13
#define LMX2582_CP_I_3_437MA	14
#define LMX2582_CP_I_3_593MA	15
#define LMX2582_CP_I_1_25MA	16
#define LMX2582_CP_I_1_406MA	17
#define LMX2582_CP_I_1_562MA	18
#define LMX2582_CP_I_1_718MA	19
#define LMX2582_CP_I_1_875MA	20
#define LMX2582_CP_I_2_031MA	21
#define LMX2582_CP_I_2_187MA	22
#define LMX2582_CP_I_2_343MA	23
#define LMX2582_CP_I_3_75MA	24
#define LMX2582_CP_I_3_906MA	25
#define LMX2582_CP_I_4_062MA	26
#define LMX2582_CP_I_4_218MA	27
#define LMX2582_CP_I_4_375MA	28
#define LMX2582_CP_I_4_531MA	29
#define LMX2582_CP_I_4_687MA	30
#define LMX2582_CP_I_4_843MA	31

/* charge pump gain multiplier
 * use for lmx,cp-icoarse */
#define LMX2582_CP_ICOARSE_X1	0
#define LMX2582_CP_ICOARSE_X2	1
#define LMX2582_CP_ICOARSE_X1_5	2
#define LMX2582_CP_ICOARSE_X2_5	3

/* pll N pre divider
 * use for lmx,pll-n-pre */
#define LMX2582_PLL_N_PRE_DIV2	0
#define LMX2582_PLL_N_PRE_DIV4	1

/* channel divider segment 1
 * use for lmx,chdiv-seg1 */
#define LMX2582_CHDIV_SEG1_DIV2	0
#define LMX2582_CHDIV_SEG1_DIV3	1

/* channel divider segment 2
 * use for lmx,chdiv-seg2 */
#define LMX2582_CHDIV_SEG2_PD	0
#define LMX2582_CHDIV_SEG2_DIV2	1
#define LMX2582_CHDIV_SEG2_DIV4	2
#define LMX2582_CHDIV_SEG2_DIV6	4
#define LMX2582_CHDIV_SEG2_DIV8	8

/* channel divider segment 3
 * use for lmx,chdiv-seg3 */
#define LMX2582_CHDIV_SEG3_PD	0
#define LMX2582_CHDIV_SEG3_DIV2	1
#define LMX2582_CHDIV_SEG3_DIV4	2
#define LMX2582_CHDIV_SEG3_DIV6	4
#define LMX2582_CHDIV_SEG3_DIV8	8

/* channel divider mux config
 * use for lmx,chdiv-seg-sel */
#define LMX2582_CHDIV_SEG_SEL_PD	0
#define LMX2582_CHDIV_SEG_SEL_1		1
#define LMX2582_CHDIV_SEG_SEL_12	2
#define LMX2582_CHDIV_SEG_SEL_123	4

#endif /* _DT_BINDINGS_IIO_FREQUENCY_LMX2582_H_ */

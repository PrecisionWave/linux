// SPDX-License-Identifier: GPL-2.0
/*
 * Texas Instruments LMX 2582 High Performance, Wideband
 * PLLatinum RF Synthesizer With Integrated VCO
 *
 * Copyright 2025 PrecisionWave AG
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/module.h>
#include <asm/div64.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/clk-provider.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/clk/clkscale.h>

// #include <dt-bindings/iio/frequency/lmx2582.h>


/* LMX2582_R0 */
#define LMX2582_R0_LD_EN(x)			(((x) & 0x1) << 13)
#define LMX2582_R0_FCAL_HPFD_ADJ(x)		(((x) & 0x3) << 7)
#define LMX2582_R0_FCAL_LPFD_ADJ(x)		(((x) & 0x3) << 5)
#define LMX2582_R0_ACAL_EN(x)			(((x) & 0x1) << 4)
#define LMX2582_R0_FCAL_EN(x)			(((x) & 0x1) << 3)
#define LMX2582_R0_MUXOUT_SEL(x)		(((x) & 0x1) << 2)
#define LMX2582_R0_RESET(x)			(((x) & 0x1) << 1)
#define LMX2582_R0_POWERDOWN(x)			(((x) & 0x1) << 0)

/* LMX2582_R1 */
#define LMX2582_R1_CAL_CLK_DIV(x)		(((x) & 0x7) << 0)

/* LMX2582_R4 */
#define LMX2582_R4_ACAL_CMP_DLY(x)		(((x) & 0xff) << 8)

/* LMX2582_R8 */
#define LMX2582_R8_VCO_IDAC_OVR(x)		(((x) & 0x1) << 13)
#define LMX2582_R8_VCO_CAPCTRL_OVR(x)		(((x) & 0x1) << 10)

/* LMX2582_R9 */
#define LMX2582_R9_OSC_2X(x)			(((x) & 0x1) << 11)
#define LMX2582_R9_REF_EN(x)			(((x) & 0x1) << 9)

/* LMX2582_R10 */
#define LMX2582_R10_MULT(x)			(((x) & 0x1f) << 7)

/* LMX2582_R11 */
#define LMX2582_R11_PLL_R(x)			(((x) & 0xff) << 4)

/* LMX2582_R12 */
#define LMX2582_R12_PLL_R_PRE(x)		(((x) & 0xfff) << 0)

/* LMX2582_R13 */
#define LMX2582_R13_CP_EN(x)			(((x) & 0x1) << 14)
#define LMX2582_R13_PFD_CTL(x)			(((x) & 0x3) << 0)

/* LMX2582_R14 */
#define LMX2582_R14_CP_IDN(x)			(((x) & 0x1f) << 7)
#define LMX2582_R14_CP_IUP(x)			(((x) & 0x1f) << 2)
#define LMX2582_R14_CP_ICOARSE(x)		(((x) & 0x3) << 0)

/* LMX2582_R19 */
#define LMX2582_R19_VCO_IDAC(x)			(((x) & 0x1ff) << 3)

/* LMX2582_R20 */
#define LMX2582_R20_ACAL_VCO_IDAC_STRT(x)	(((x) & 0x1ff) << 0)

/* LMX2582_R22 */
#define LMX2582_R22_VCO_CAPCTRL(x)		(((x) & 0xff) << 0)

/* LMX2582_R23 */
#define LMX2582_R23_FCAL_VCO_SEL_STRT(x)	(((x) & 0x1) << 14)
#define LMX2582_R23_VCO_SEL(x)			(((x) & 0x7) << 11)
#define LMX2582_R23_VCO_SEL_FORCE(x)		(((x) & 0x1) << 10)

/* LMX2582_R30 */
#define LMX2582_R30_MASH_DITHER(x)		(((x) & 0x1) << 10)
#define LMX2582_R30_VTUNE_ADJ(x)		(((x) & 0x3) << 6)

/* LMX2582_R31 */
#define LMX2582_R31_VCO_DISTB_PD(x)		(((x) & 0x1) << 10)
#define LMX2582_R31_VCO_DISTA_PD(x)		(((x) & 0x1) << 9)
#define LMX2582_R31_CHDIV_DIST_PD(x)		(((x) & 0x1) << 7)

/* LMX2582_R34 */
#define LMX2582_R34_CHDIV_EN(x)			(((x) & 0x1) << 5)

/* LMX2582_R35 */
#define LMX2582_R35_CHDIV_SEG2(x)		(((x) & 0xf) << 9)
#define LMX2582_R35_CHDIV_SEG3_EN(x)		(((x) & 0x1) << 8)
#define LMX2582_R35_CHDIV_SEG2_EN(x)		(((x) & 0x1) << 7)
#define LMX2582_R35_CHDIV_SEG1(x)		(((x) & 0x1) << 2)
#define LMX2582_R35_CHDIV_SEG1_EN(x)		(((x) & 0x1) << 1)

/* LMX2582_R36 */
#define LMX2582_R36_CHDIV_DISTB_EN(x)		(((x) & 0x1) << 11)
#define LMX2582_R36_CHDIV_DISTA_EN(x)		(((x) & 0x1) << 10)
#define LMX2582_R36_CHDIV_SEG_SEL(x)		(((x) & 0x7) << 4)
#define LMX2582_R36_CHDIV_SEG3(x)		(((x) & 0xf) << 0)

/* LMX2582_R37 */
#define LMX2582_R37_PLL_N_PRE(x)		(((x) & 0x1) << 12)

/* LMX2582_R38 */
#define LMX2582_R38_PLL_N(x)			(((x) & 0xfff) << 1)

/* LMX2582_R39 */
#define LMX2582_R39_PFD_DLY(x)			(((x) & 0x3f) << 8)

/* LMX2582_R40 */
#define LMX2582_R40_PLL_DEN_31_16(x)		(((x) >> 16) & 0xffff)

/* LMX2582_R41 */
#define LMX2582_R41_PLL_DEN_15_0(x)		((x) & 0xffff)

/* LMX2582_R42 */
#define LMX2582_R42_MASH_SEED_31_16(x)		(((x) >> 16) & 0xffff)

/* LMX2582_R43 */
#define LMX2582_R43_MASH_SEED_15_0(x)		((x) & 0xffff)

/* LMX2582_R44 */
#define LMX2582_R44_PLL_NUM_31_16(x)		(((x) >> 16) & 0xffff)

/* LMX2582_R45 */
#define LMX2582_R45_PLL_NUM_15_0(x)		((x) & 0xffff)

/* LMX2582_R46 */
#define LMX2582_R46_OUTA_POW(x)			(((x) & 0x3f) << 8)
#define LMX2582_R46_OUTB_PD(x)			(((x) & 0x1) << 7)
#define LMX2582_R46_OUTA_PD(x)			(((x) & 0x1) << 6)
#define LMX2582_R46_MASH_ORDER(x)		(((x) & 0x7) << 0)

/* LMX2582_R47 */
#define LMX2582_R47_OUTA_MUX(x)			(((x) & 0x3) << 11)
#define LMX2582_R47_OUTB_POW(x)			(((x) & 0x3f) << 0)

/* LMX2582_R48 */
#define LMX2582_R48_OUTB_MUX(x)			(((x) & 0x3) << 0)

/* LMX2582_R59 */
#define LMX2582_R59_MUXOUT_HDRV(x)		(((x) & 0x1) << 5)

/* LMX2582_R61 */
#define LMX2582_R61_LD_TYPE(x)			(((x) & 0x1) << 0)

/* LMX2582_R64 */
#define LMX2582_R64_ACAL_FAST(x)		(((x) & 0x1) << 9)
#define LMX2582_R64_FCAL_FAST(x)		(((x) & 0x1) << 8)
#define LMX2582_R64_AJUMP_SIZE(x)		(((x) & 0x7) << 5)
#define LMX2582_R64_FJUMP_SIZE(x)		(((x) & 0xf) << 0)

/* LMX2582_R68) (readback) */
#define LMX2582_R68_rb_LD_VTUNE(x)		(((x) >> 9) & 0x3)
#define LMX2582_R68_rb_LVCO_SEL(x)		(((x) >> 5) & 0xf)

/* LMX2582_R69) (readback) */
#define LMX2582_R69_rb_VCO_CAPCTRL(x)		(((x) >> 0) & 0xff)

/* LMX2582_R70) (readback) */
#define LMX2582_R70_rb_VCO_DACISET(x)		(((x) >> 0) & 0x1ff)

/* Specifications */
#define LMX2582_MIN_VCO_FREQ		3550000000ULL /* 3550 MHz */
#define LMX2582_MAX_VCO_FREQ		7100000000ULL /* 7100 MHz */
#define LMX2582_MIN_OUT_FREQ		  20000000ULL /*   20 MHz */
#define LMX2582_MAX_OUT_FREQ		5500000000ULL /* 5500 MHz */
#define LMX2582_MIN_FREQ_REFIN		   5000000ULL /*    5 MHz */
#define LMX2582_MAX_FREQ_REFIN		1400000000ULL /* 1400 MHz */
#define LMX2582_MIN_FREQ_PFD		   5000000ULL /*    5 MHz */
#define LMX2582_MAX_FREQ_PFD		 200000000ULL /*  200 MHz */
#define LMX2582_MAX_CAL_CLK_FREQ	 200000000ULL /*  200 MHz */
#define LMX2582_FVCO_HIGH		6500000000ULL /* 6500 MHz */

#define LMX2582_CHECK_RANGE(freq, range) \
	((freq > LMX2582_MAX_ ## range) || (freq < LMX2582_MIN_ ## range))

#define LMX2582_CLK_COUNT			2

enum {
	LMX2582_FREQ,
	LMX2582_PWRDOWN,
	LMX2582_CHANNEL_NAME,
};

static const char * const lmx2582_ch_names[] = {
	"RFoutA", "RFoutB",
};

enum {
	LMX2582_OUTx_MUX_CHDIV = 0,
	LMX2582_OUTx_MUX_VCO = 1,
};

enum {
	LMX2582_CH_RFOUTA,
	LMX2582_CH_RFOUTB,
};

/* Registers */
enum lmx2582_reg {
	LMX2582_R0 = 0,
	LMX2582_R1 = 1,
	LMX2582_R2 = 2,
	LMX2582_R4 = 4,
	LMX2582_R7 = 7,
	LMX2582_R8 = 8,
	LMX2582_R9 = 9,
	LMX2582_R10 = 10,
	LMX2582_R11 = 11,
	LMX2582_R12 = 12,
	LMX2582_R13 = 13,
	LMX2582_R14 = 14,
	LMX2582_R19 = 19,
	LMX2582_R20 = 20,
	LMX2582_R22 = 22,
	LMX2582_R23 = 23,
	LMX2582_R24 = 24,
	LMX2582_R25 = 25,
	LMX2582_R28 = 28,
	LMX2582_R29 = 29,
	LMX2582_R30 = 30,
	LMX2582_R31 = 31,
	LMX2582_R32 = 32,
	LMX2582_R33 = 33,
	LMX2582_R34 = 34,
	LMX2582_R35 = 35,
	LMX2582_R36 = 36,
	LMX2582_R37 = 37,
	LMX2582_R38 = 38,
	LMX2582_R39 = 39,
	LMX2582_R40 = 40,
	LMX2582_R41 = 41,
	LMX2582_R42 = 42,
	LMX2582_R43 = 43,
	LMX2582_R44 = 44,
	LMX2582_R45 = 45,
	LMX2582_R46 = 46,
	LMX2582_R47 = 47,
	LMX2582_R48 = 48,
	LMX2582_R59 = 59,
	LMX2582_R61 = 61,
	LMX2582_R62 = 62,
	LMX2582_R64 = 64,
	LMX2582_R68 = 68,
	LMX2582_R69 = 69,
	LMX2582_R70 = 70,
	LMX2582_R_NUM,
};

static const u16 lmx2582_reserved_values[LMX2582_R_NUM] = {
	[LMX2582_R0] = 0x0200,
	[LMX2582_R1] = 0x0808,
	[LMX2582_R2] = 0x0500,
	[LMX2582_R4] = 0x0043,
	[LMX2582_R7] = 0x28b2,
	[LMX2582_R8] = 0x1084,
	[LMX2582_R9] = 0x0102,
	[LMX2582_R10] = 0x1058,
	[LMX2582_R11] = 0x0008,
	[LMX2582_R12] = 0x7000,
	[LMX2582_R13] = 0x0000,
	[LMX2582_R14] = 0x0000,
	[LMX2582_R19] = 0x0005,
	[LMX2582_R20] = 0x0000,
	[LMX2582_R22] = 0x2300,
	[LMX2582_R23] = 0x8042,
	[LMX2582_R24] = 0x0509,
	[LMX2582_R25] = 0x0000,
	[LMX2582_R28] = 0x2924,
	[LMX2582_R29] = 0x0084,
	[LMX2582_R30] = 0x0034,
	[LMX2582_R31] = 0x0001,
	[LMX2582_R32] = 0x210a,
	[LMX2582_R33] = 0x2a0a,
	[LMX2582_R34] = 0xc3ca,
	[LMX2582_R35] = 0x0019,
	[LMX2582_R36] = 0x0000,
	[LMX2582_R37] = 0x4000,
	[LMX2582_R38] = 0x0000,
	[LMX2582_R39] = 0x8004,
	[LMX2582_R40] = 0x0000,
	[LMX2582_R41] = 0x0000,
	[LMX2582_R42] = 0x0000,
	[LMX2582_R43] = 0x0000,
	[LMX2582_R44] = 0x0000,
	[LMX2582_R45] = 0x0000,
	[LMX2582_R46] = 0x0020,
	[LMX2582_R47] = 0x00c0,
	[LMX2582_R48] = 0x03fc,
	[LMX2582_R59] = 0x0000,
	[LMX2582_R61] = 0x0000,
	[LMX2582_R62] = 0x0000,
	[LMX2582_R64] = 0x0010,
	[LMX2582_R68] = 0x0010,
	[LMX2582_R69] = 0x0010,
	[LMX2582_R70] = 0x0010,
};
struct lmx2582_config {
	/* REG0 */
	bool LD_EN;
	u32 FCAL_HPFD_ADJ;
	u32 FCAL_LPFD_ADJ;
	bool ACAL_EN;
	bool FCAL_EN;
	bool MUXOUT_SEL;
	bool RESET;
	bool POWERDOWN;

	/* REG1 */
	u32 CAL_CLK_DIV;

	/* REG4 */
	u32 ACAL_CMP_DLY;

	/* REG8 */
	bool VCO_IDAC_OVR;
	bool VCO_CAPCTRL_OVR;

	/* REG9 */
	bool OSC_2X;
	bool REF_EN;

	/* REG10 */
	u32 MULT;

	/* REG11 */
	u32 PLL_R;

	/* REG12 */
	u32 PLL_R_PRE;

	/* REG13 */
	bool CP_EN;
	u32 PFD_CTL;

	/* REG14 */
	u32 CP_IDN;
	u32 CP_IUP;
	u32 CP_ICOARSE;

	/* REG19 */
	u32 VCO_IDAC;

	/* REG20 */
	u32 ACAL_VCO_IDAC_STRT;

	/* REG22 */
	u32 VCO_CAPCTRL;

	/* REG23 */
	bool FCAL_VCO_SEL_STRT;
	u32 VCO_SEL;
	bool VCO_SEL_FORCE;

	/* R30 */
	bool MASH_DITHER;
	u32 VTUNE_ADJ;

	/* R31 */
	bool VCO_DISTB_PD;
	bool VCO_DISTA_PD;
	bool CHDIV_DIST_PD;

	/* R34 */
	bool CHDIV_EN;

	/* R35 */
	u32 CHDIV_SEG2;
	bool CHDIV_SEG3_EN;
	bool CHDIV_SEG2_EN;
	u32 CHDIV_SEG1;
	bool CHDIV_SEG1_EN;

	/* R36 */
	bool CHDIV_DISTB_EN;
	bool CHDIV_DISTA_EN;
	u32 CHDIV_SEG_SEL;
	u32 CHDIV_SEG3;

	/* R37 */
	bool PLL_N_PRE;

	/* R38 */
	u32 PLL_N;

	/* R39 */
	u32 PFD_DLY;

	/* R40/41 */
	u32 PLL_DEN;

	/* R42/43 */
	u32 MASH_SEED;

	/* R44/45 */
	u32 PLL_NUM;

	/* R46 */
	u32 OUTA_POW;
	bool OUTB_PD;
	bool OUTA_PD;
	u32 MASH_ORDER;

	/* R47 */
	u32 OUTA_MUX;
	u32 OUTB_POW;

	/* R48 */
	u32 OUTB_MUX;

	/* R59 */
	bool MUXOUT_HDRV;

	/* R61 */
	bool LD_TYPE;

	/* R64 */
	bool ACAL_FAST;
	bool FCAL_FAST;
	u32 AJUMP_SIZE;
	u32 FJUMP_SIZE;
};
static const struct lmx2582_config lmx2582_default_values = {
	/* REG0 */
	.LD_EN = true,
	.FCAL_HPFD_ADJ = 0,
	.FCAL_LPFD_ADJ = 0,
	.ACAL_EN = 1,
	.FCAL_EN = 1,
	.MUXOUT_SEL = 1,
	.RESET = 0,
	.POWERDOWN = 0,
	/* REG1 */
	.CAL_CLK_DIV = 3,
	/* REG4 */
	.ACAL_CMP_DLY = 25,
	/* REG8 */
	.VCO_IDAC_OVR = false,
	.VCO_CAPCTRL_OVR = false,
	/* REG9 */
	.OSC_2X = false,
	.REF_EN = true,
	/* REG10 */
	.MULT = 1,
	/* REG11 */
	.PLL_R = 1,
	/* REG12 */
	.PLL_R_PRE = 1,
	/* REG13 */
	.CP_EN = true,
	.PFD_CTL = 0,
	/* REG14 */
	.CP_IDN = 3,
	.CP_IUP = 3,
	.CP_ICOARSE = 0,
	/* REG19 */
	.VCO_IDAC = 300,
	/* REG20 */
	.ACAL_VCO_IDAC_STRT = 300,
	/* REG22 */
	.VCO_CAPCTRL = 0,
	/* REG23 */
	.FCAL_VCO_SEL_STRT = false,
	.VCO_SEL = 1,
	.VCO_SEL_FORCE = false,
	/* R30 */
	.MASH_DITHER = false,
	.VTUNE_ADJ  = 3,
	/* R31 */
	.VCO_DISTB_PD = true,
	.VCO_DISTA_PD = false,
	.CHDIV_DIST_PD = false,
	/* R34 */
	.CHDIV_EN = true,
	/* R35 */
	.CHDIV_SEG2 = 1,
	.CHDIV_SEG3_EN = false,
	.CHDIV_SEG2_EN = false,
	.CHDIV_SEG1 = 1,
	.CHDIV_SEG1_EN = false,
	/* R36 */
	.CHDIV_DISTB_EN = false,
	.CHDIV_DISTA_EN = true,
	.CHDIV_SEG_SEL = 1,
	.CHDIV_SEG3 = 1,
	/* R37 */
	.PLL_N_PRE = false,
	/* R38 */
	.PLL_N = 27,
	/* R39 */
	.PFD_DLY = 2,
	/* R40/41 */
	.PLL_DEN = 1000,
	/* R42/43 */
	.MASH_SEED = 0,
	/* R44/45 */
	.PLL_NUM = 0,
	/* R46 */
	.OUTA_POW = 15,
	.OUTB_PD = true,
	.OUTA_PD = false,
	.MASH_ORDER = 3,
	/* R47 */
	.OUTA_MUX = 0,
	.OUTB_POW = 15,
	/* R48 */
	.OUTB_MUX = 0,
	/* R59 */
	.MUXOUT_HDRV = false,
	/* R61 */
	.LD_TYPE = true,
	/* R64 */
	.ACAL_FAST = false,
	.FCAL_FAST = false,
	.AJUMP_SIZE = 3,
	.FJUMP_SIZE = 15,
};

struct lmx2582_output {
	struct clk_hw		hw;
	struct iio_dev		*indio_dev;
	struct lmx2582_state	*st;
	unsigned int		num;
	bool			enabled;
	struct clock_scale	scale;
	unsigned int		power;
	unsigned int		muxsel;
	unsigned long long	frequency;
};

struct lmx2582_state {
	struct spi_device		*spi;
	struct lmx2582_config		*conf;
	struct dentry			*dent;
	struct clk			*clkin;
	struct clk			*clks[LMX2582_CLK_COUNT];
	struct clk_onecell_data		clk_data;

	struct lmx2582_output		outputs[LMX2582_CLK_COUNT];

	bool				has_clk_out_names;
	const char			*lmx2582_clk_names[LMX2582_CLK_COUNT];

	/* Protect against concurrent accesses to the device */
	struct mutex		lock;
	unsigned long		fosc;
	unsigned long long	fvco; /* Phase Frequency Detector */
	unsigned long long	fpd; /* Phase Frequency Detector */

	u32			integer;
	u32			fract;
	u16			regs[LMX2582_R_NUM];

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	u8			wrval[3] ____cacheline_aligned;
	__be16			rdval ____cacheline_aligned;
};



#define to_clk_priv(_hw) container_of(_hw, struct lmx2582_output, hw)

/*
 * Factorize, approximate N = CLK1 * CLK2 where CLK1,2 are 12-bit registers
 * Either CLK1 or CLK2 must be greater than 1,
 * that is, CLK1 = CLK2 = 1 is not allowed.
 */

static int lmx2582_factorize_clk_divs(u32 n, u32 *clk1, u32 *clk2)
{
	int i, c1, c2, n_calc, delta, delta_max = 0xFFFFFFU;

	n = clamp(n, 2U, 0xFFFFFFU);

	for (i = 0; i <= 12; i++) {
		c1 = BIT(i);
		if (c1 > 0xFFF)
			c1--;

		c2 = DIV_ROUND_CLOSEST(n, c1);

		if (c2 > 0xFFF)
			continue;

		n_calc = c1 * c2;

		if (n == n_calc) {
			*clk1 = c1;
			*clk2 = c2;
			return 0;
		}

		delta = abs(n - n_calc);
		if (delta < delta_max) {
			*clk1 = c1;
			*clk2 = c2;
			delta_max = delta;
		}
	}

	return delta_max;
}

static int lmx2582_spi_write(struct lmx2582_state *st, u8 reg, u16 val)
{
	st->wrval[0] = (reg & 0x7f);
	st->wrval[1] = (val >> 8) & 0xff;
	st->wrval[2] = val & 0xff;

	dev_dbg(&st->spi->dev, "write R%d = 0x%.8X\n", reg, val);

	return spi_write(st->spi, &st->wrval[0], 3);
}

static int lmx2582_spi_read(struct lmx2582_state *st, u8 reg, u16* val)
{
	int ret;

	ret = spi_w8r16be(st->spi, reg | 0x80);
	if (ret < 0)
		return ret;
	
	if (val)
		*val = ret;

	dev_dbg(&st->spi->dev, "read R%d -> 0x%.8X\n", reg, ret);

	return 0;
}

static int lmx2582_soft_reset(struct lmx2582_state *st)
{
	return lmx2582_spi_write(st, LMX2582_R0,
				st->regs[LMX2582_R0] |
				lmx2582_reserved_values[LMX2582_R0] |
				LMX2582_R0_RESET(1));
}

static int lmx2582_sync_config(struct lmx2582_state *st)
{
	u16 val;
	int ret, i;

	for (i = LMX2582_R70; i >= LMX2582_R0; i--) {
		val = lmx2582_reserved_values[i];
		val |= st->regs[i];

		switch (i) {
		/* Force update */
		case LMX2582_R0:
		val |= LMX2582_R0_FCAL_EN(1);
			ret = lmx2582_spi_write(st, i, val);
			if (ret < 0)
				return ret;
			break;
		/* Skip non-existent registers */
		case 3:
		case 5:
		case 6:
		case 15:
		case 16:
		case 17:
		case 18:
		case 21:
		case 26:
		case 27:
		case 49:
		case 50:
		case 51:
		case 52:
		case 53:
		case 54:
		case 55:
		case 56:
		case 57:
		case 58:
		case 60:
		case 63:
		case 65:
		case 66:
		case 67:
			break;
		/* Read only registers*/
		case LMX2582_R68:
		case LMX2582_R69:
		case LMX2582_R70:
			break;
		/* "Normal register" */
		default:
			ret = lmx2582_spi_write(st, i, val);
			if (ret < 0)
				return ret;
			break;
		}
	}

	return 0;
}

static int lmx2582_reg_access(struct iio_dev *indio_dev,
			      unsigned int reg, unsigned int writeval,
			      unsigned int *readval)
{
	struct lmx2582_state *st = iio_priv(indio_dev);
	int ret;

	if (reg >= LMX2582_R_NUM)
		return -EINVAL;

	mutex_lock(&st->lock);
	if (readval == NULL) {
		ret = lmx2582_spi_write(st, reg & 0x7f, writeval & 0xffff);
	} else {
		u16 tmp;
		ret = lmx2582_spi_read(st, reg & 0x7f, &tmp);
		*readval = tmp;
	}
	mutex_unlock(&st->lock);

	return ret;
}
static int lmx2582_pll_fract_n_compute(unsigned long long vco,
					unsigned int pfd,
					unsigned int *integer,
					unsigned int *fract)
{
#if 0
	u64 tmp;

	tmp = do_div(vco, pfd);
	tmp = tmp * ADF4159_MODULUS;
	do_div(tmp, pfd);

	*integer = vco;
	*fract = tmp;
#endif
	return 0;
}

static unsigned long long lmx2582_pll_fract_n_get_rate(struct lmx2582_state *st)
{
#if 0
	u64 val, tmp;

	val = (u64)st->integer * st->fpd;
	tmp = (u64)st->fract * st->fpd;
	do_div(tmp, ADF4159_MODULUS);

	val += tmp;

	return val;
#endif
	return 0;
}

static u32 lmx2582_get_chdiv_total(struct lmx2582_config *conf)
{
	u32 divider1;
	u32 divider2;
	u32 divider3;

	/* decode SEG1 */
	if (conf->CHDIV_SEG1 == 0)
		divider1 = 2;
	else
		divider1 = 3;

	/* decode SEG2 */
	switch (conf->CHDIV_SEG2) {
	case 1:
		divider2 = 2;
		break;
	case 2:
		divider2 = 4;
		break;
	case 4:
		divider2 = 6;
		break;
	case 8:
		divider2 = 8;
		break;
	case 0:
	default:
		divider2 = 0;
		break;
	}

	/* decode SEG3 */
	switch (conf->CHDIV_SEG3) {
	case 1:
		divider3 = 2;
		break;
	case 2:
		divider3 = 4;
		break;
	case 4:
		divider3 = 6;
		break;
	case 8:
		divider3 = 8;
		break;
	case 0:
	default:
		divider3 = 0;
		break;
	}
	
	/* decode mux */
	switch (conf->CHDIV_SEG_SEL) {
	case 1: /* only divider 1 active */
		return divider1;
	case 2: /* divider 1 and 2 active */
		return divider1 * divider2;
	case 4: /* divider 1, 2 and 3 active */
		return divider1 * divider2 * divider3;
	case 0: /* power down */
	default:
		break;
	}
	/* power down or invalid setting */
	return 0;
}

static int lmx2582_setup(struct lmx2582_state *st, unsigned long parent_rate)
{
	struct lmx2582_config *conf = st->conf;
	int i;
	unsigned long long cal_clk;
	unsigned long long chdiv_freq;
	u32 chdiv_total;

	if (parent_rate)
		st->fosc = parent_rate;
	else
		st->fosc = clk_get_rate(st->clkin);

	/* Input path to phase comparator */
	st->fpd = st->fosc;
	st->fpd *= conf->OSC_2X ? 2 : 1;
	st->fpd = div_u64(st->fpd, conf->PLL_R_PRE == 0 ? 1 : conf->PLL_R_PRE);
	st->fpd *= conf->MULT;
	st->fpd = div_u64(st->fpd, conf->PLL_R == 0 ? 1 : conf->PLL_R);

	/* VCO feedback path */
	st->fvco = st->fpd * (conf->PLL_N * conf->PLL_DEN + conf->PLL_NUM);
	st->fvco = div_u64(st->fvco, conf->PLL_DEN == 0 ? 1 : conf->PLL_DEN);
	st->fvco *= conf->PLL_N_PRE ? 4 : 2;

	/* state machine clock */
	for (i = 0; i < 8; i++) {
		conf->CAL_CLK_DIV = i;
		if ((st->fosc >> i) < LMX2582_MAX_CAL_CLK_FREQ)
			break;
	}
	cal_clk = st->fosc >> conf->CAL_CLK_DIV;

	/* output clocks */
	chdiv_total = lmx2582_get_chdiv_total(conf);
	if (chdiv_total == 0) 
		chdiv_freq = 0;
	else
		chdiv_freq = div_u64(st->fvco, chdiv_total);

	if (st->outputs[0].muxsel == LMX2582_OUTx_MUX_CHDIV)
		st->outputs[0].frequency =  chdiv_freq;
	else
		st->outputs[0].frequency = st->fvco;
	
	if (st->outputs[1].muxsel == LMX2582_OUTx_MUX_CHDIV)
		st->outputs[1].frequency = chdiv_freq;
	else
		st->outputs[1].frequency = st->fvco;
	

	/* VCO Band selection */
	if (st->fvco >= LMX2582_FVCO_HIGH)
		conf->VTUNE_ADJ = 3;
	else
		conf->VTUNE_ADJ = 0;

	/* Determine if channel divider needs power */
	/* Default: All paths disabled */
	conf->CHDIV_DIST_PD = true;
	conf->CHDIV_DISTA_EN = false;
	conf->CHDIV_DISTB_EN = false;
	conf->VCO_DISTA_PD = true;
	conf->VCO_DISTB_PD = true;
	conf->CHDIV_EN = false;

	if (st->outputs[0].enabled) {
		switch (conf->OUTA_MUX & 3) {
		case LMX2582_OUTx_MUX_CHDIV:
			conf->CHDIV_DIST_PD = false;
			conf->CHDIV_DISTA_EN = true;
			conf->VCO_DISTA_PD = true;
			conf->CHDIV_EN = true;
			break;
		case LMX2582_OUTx_MUX_VCO:
			conf->VCO_DISTA_PD = false;
			break;
		}
	}

	if (st->outputs[1].enabled) {
		switch (conf->OUTB_MUX & 3) {
		case LMX2582_OUTx_MUX_CHDIV:
			conf->CHDIV_DIST_PD = false;
			conf->CHDIV_DISTB_EN = true;
			conf->VCO_DISTB_PD = true;
			conf->CHDIV_EN = true;
			break;
		case LMX2582_OUTx_MUX_VCO:
			conf->VCO_DISTB_PD = false;
			break;
		}
	}

	/* Channel divider enable control */
	switch (conf->CHDIV_SEG_SEL) {
	case 1: /* only divider 1 active */
		conf->CHDIV_SEG1_EN = true;
		conf->CHDIV_SEG2_EN = false;
		conf->CHDIV_SEG3_EN = false;
		break;
	case 2: /* divider 1 and 2 active */
		conf->CHDIV_SEG1_EN = true;
		conf->CHDIV_SEG2_EN = true;
		conf->CHDIV_SEG3_EN = false;
		break;
	case 4: /* divider 1, 2 and 3 active */
		conf->CHDIV_SEG1_EN = true;
		conf->CHDIV_SEG2_EN = true;
		conf->CHDIV_SEG3_EN = true;
		break;
	case 0: /* power down */
	default:
		conf->CHDIV_SEG1_EN = false;
		conf->CHDIV_SEG2_EN = false;
		conf->CHDIV_SEG3_EN = false;
		break;
	}

	/* R0 Bit Definitions */
	st->regs[LMX2582_R0] =
		LMX2582_R0_LD_EN(conf->LD_EN) |
		LMX2582_R0_FCAL_HPFD_ADJ(conf->FCAL_HPFD_ADJ) |
		LMX2582_R0_FCAL_LPFD_ADJ(conf->FCAL_LPFD_ADJ) |
		LMX2582_R0_ACAL_EN(conf->ACAL_EN) |
		LMX2582_R0_FCAL_EN(conf->FCAL_EN) |
		LMX2582_R0_MUXOUT_SEL(conf->MUXOUT_SEL) |
		LMX2582_R0_RESET(conf->RESET) |
		LMX2582_R0_POWERDOWN(conf->POWERDOWN);

	/* R1 Bit Definitions */
	st->regs[LMX2582_R1] = LMX2582_R1_CAL_CLK_DIV(conf->CAL_CLK_DIV);

	/* R4 Bit Definitions */
	st->regs[LMX2582_R4] = LMX2582_R4_ACAL_CMP_DLY(conf->ACAL_CMP_DLY);

	/* R8 Bit Definitions */
	st->regs[LMX2582_R8] =
		LMX2582_R8_VCO_IDAC_OVR(conf->VCO_IDAC_OVR) |
		LMX2582_R8_VCO_CAPCTRL_OVR(conf->VCO_CAPCTRL_OVR);

	/* R9 Bit Definitions */
	st->regs[LMX2582_R9] =
		LMX2582_R9_OSC_2X(conf->OSC_2X) |
		LMX2582_R9_REF_EN(conf->REF_EN);

	/* R10 Bit Definitions */
	st->regs[LMX2582_R10] = LMX2582_R10_MULT(conf->MULT);

	/* R11 Bit Definitions */
	st->regs[LMX2582_R11] = LMX2582_R11_PLL_R(conf->PLL_R);

	/* R12 Bit Definitions */
	st->regs[LMX2582_R12] = LMX2582_R12_PLL_R_PRE(conf->PLL_R_PRE);

	/* R13 Bit Definitions */
	st->regs[LMX2582_R13] =
		LMX2582_R13_CP_EN(conf->CP_EN) |
		LMX2582_R13_PFD_CTL(conf->PFD_CTL);

	/* R14 Bit Definitions */
	st->regs[LMX2582_R14] =
		LMX2582_R14_CP_IDN(conf->CP_IDN) |
		LMX2582_R14_CP_IUP(conf->CP_IUP) |
		LMX2582_R14_CP_ICOARSE(conf->CP_ICOARSE);

	/* R19 Bit Definitions */
	st->regs[LMX2582_R19] = LMX2582_R19_VCO_IDAC(conf->VCO_IDAC);

	/* R20 Bit Definitions */
	st->regs[LMX2582_R20] =
		LMX2582_R20_ACAL_VCO_IDAC_STRT(conf->ACAL_VCO_IDAC_STRT);

	/* R22 Bit Definitions */
	st->regs[LMX2582_R22] = LMX2582_R22_VCO_CAPCTRL(conf->VCO_CAPCTRL);

	/* R23 Bit Definitions */
	st->regs[LMX2582_R23] =
		LMX2582_R23_FCAL_VCO_SEL_STRT(conf->FCAL_VCO_SEL_STRT) |
		LMX2582_R23_VCO_SEL(conf->VCO_SEL) |
		LMX2582_R23_VCO_SEL_FORCE(conf->VCO_SEL_FORCE);

	/* R30 Bit Definitions */
	st->regs[LMX2582_R30] =
		LMX2582_R30_MASH_DITHER(conf->MASH_DITHER) |
		LMX2582_R30_VTUNE_ADJ(conf->VTUNE_ADJ);

	/* R31 Bit Definitions */
	st->regs[LMX2582_R31] =
		LMX2582_R31_VCO_DISTB_PD(conf->VCO_DISTB_PD) |
		LMX2582_R31_VCO_DISTA_PD(conf->VCO_DISTA_PD) |
		LMX2582_R31_CHDIV_DIST_PD(conf->CHDIV_DIST_PD);

	/* R34 Bit Definitions */
	st->regs[LMX2582_R34] = LMX2582_R34_CHDIV_EN(conf->CHDIV_EN);

	/* R35 Bit Definitions */
	st->regs[LMX2582_R35] =
		LMX2582_R35_CHDIV_SEG2(conf->CHDIV_SEG2) |
		LMX2582_R35_CHDIV_SEG3_EN(conf->CHDIV_SEG3_EN) |
		LMX2582_R35_CHDIV_SEG2_EN(conf->CHDIV_SEG2_EN) |
		LMX2582_R35_CHDIV_SEG1(conf->CHDIV_SEG1) |
		LMX2582_R35_CHDIV_SEG1_EN(conf->CHDIV_SEG1_EN);

	/* R36 Bit Definitions */
	st->regs[LMX2582_R36] =
		LMX2582_R36_CHDIV_DISTB_EN(conf->CHDIV_DISTB_EN) |
		LMX2582_R36_CHDIV_DISTA_EN(conf->CHDIV_DISTA_EN) |
		LMX2582_R36_CHDIV_SEG_SEL(conf->CHDIV_SEG_SEL) |
		LMX2582_R36_CHDIV_SEG3(conf->CHDIV_SEG3);

	/* R37 Bit Definitions */
	st->regs[LMX2582_R37] = LMX2582_R37_PLL_N_PRE(conf->PLL_N_PRE);

	/* R38 Bit Definitions */
	st->regs[LMX2582_R38] = LMX2582_R38_PLL_N(conf->PLL_N);

	/* R39 Bit Definitions */
	st->regs[LMX2582_R39] = LMX2582_R39_PFD_DLY(conf->PFD_DLY);

	/* R40/41 Bit Definitions */
	st->regs[LMX2582_R40] = LMX2582_R40_PLL_DEN_31_16(conf->PLL_DEN);
	st->regs[LMX2582_R41] = LMX2582_R41_PLL_DEN_15_0(conf->PLL_DEN);
	
	/* R42/43 Bit Definitions */
	st->regs[LMX2582_R42] = LMX2582_R42_MASH_SEED_31_16(conf->MASH_SEED);
	st->regs[LMX2582_R43] = LMX2582_R43_MASH_SEED_15_0(conf->MASH_SEED);

	/* R44/45 Bit Definitions */
	st->regs[LMX2582_R44] = LMX2582_R44_PLL_NUM_31_16(conf->PLL_NUM);
	st->regs[LMX2582_R45] = LMX2582_R45_PLL_NUM_15_0(conf->PLL_NUM);

	/* R46 Bit Definitions */
	st->regs[LMX2582_R46] =
		LMX2582_R46_OUTA_POW(conf->OUTA_POW) |
		LMX2582_R46_OUTB_PD(conf->OUTB_PD) |
		LMX2582_R46_OUTA_PD(conf->OUTA_PD) | 
		LMX2582_R46_MASH_ORDER(conf->MASH_ORDER);

	/* R47 Bit Definitions */
	st->regs[LMX2582_R47] =
		LMX2582_R47_OUTA_MUX(conf->OUTA_MUX) |
		LMX2582_R47_OUTB_POW(conf->OUTB_POW);

	/* R48 Bit Definitions */
	st->regs[LMX2582_R48] = LMX2582_R48_OUTB_MUX(conf->OUTB_MUX);

	/* R59 Bit Definitions */
	st->regs[LMX2582_R59] = LMX2582_R59_MUXOUT_HDRV(conf->MUXOUT_HDRV);

	/* R61 Bit Definitions */
	st->regs[LMX2582_R61] = LMX2582_R61_LD_TYPE(conf->LD_TYPE);

	/* R64 Bit Definitions */
	st->regs[LMX2582_R64] =
		LMX2582_R64_ACAL_FAST(conf->ACAL_FAST) |
		LMX2582_R64_FCAL_FAST(conf->FCAL_FAST) |
		LMX2582_R64_AJUMP_SIZE(conf->AJUMP_SIZE) |
		LMX2582_R64_FJUMP_SIZE(conf->FJUMP_SIZE);

	dev_info(&st->spi->dev,
		 "VCO: %llu Hz, FPD %llu Hz, CAL %llu Hz, "
		 "chdiv total: %u, chdiv freq %lld Hz\n",
		 st->fvco, st->fpd, cal_clk, chdiv_total, chdiv_freq);

	return lmx2582_sync_config(st);
}

static ssize_t lmx2582_write(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan,
				    const char *buf, size_t len)
{
	struct lmx2582_state *st = iio_priv(indio_dev);
	//struct lmx2582_config *conf = st->conf;
	long long readin;
	int ret;

	ret = kstrtoll(buf, 10, &readin);
	if (ret)
		return ret;

	mutex_lock(&st->lock);
	switch ((u32)private) {
	case LMX2582_FREQ:
		st->outputs[chan->channel].frequency = readin;
		break;
	case LMX2582_PWRDOWN:
		st->outputs[chan->channel].enabled = readin == 0;
		break;
	default:
		ret = -EINVAL;
	}

	ret = lmx2582_setup(st, 0);

	mutex_unlock(&st->lock);

	return ret ? ret : len;
}

static ssize_t lmx2582_read(struct iio_dev *indio_dev,
				   uintptr_t private,
				   const struct iio_chan_spec *chan,
				   char *buf)
{
	struct lmx2582_state *st = iio_priv(indio_dev);
	//struct lmx2582_config *conf = st->conf;
	long long val;
	//u64 uval;
	int ret = 0;

	switch ((u32)private) {
	case LMX2582_FREQ:
		val = st->outputs[chan->channel].frequency;
		//lmx2582_pll_fract_n_get_rate(st);
		break;
	case LMX2582_PWRDOWN:
		val = st->outputs[chan->channel].enabled ? 0 : 1;
		break;
	case LMX2582_CHANNEL_NAME:
		return sprintf(buf, "%s\n", lmx2582_ch_names[chan->channel]);
	default:
		ret = -EINVAL;
		val = 0;
	}
	//mutex_lock(&st->lock);
	//mutex_unlock(&st->lock);

	return ret < 0 ? ret : sprintf(buf, "%lld\n", val);
}

static int lmx2582_get_outmux(struct iio_dev *indio_dev,
			      const struct iio_chan_spec *chan)
{
	struct lmx2582_state *st = iio_priv(indio_dev);

	switch(chan->channel) {
	case LMX2582_CH_RFOUTA:
		return st->conf->OUTA_MUX & 0x3;
	case LMX2582_CH_RFOUTB:
		return st->conf->OUTB_MUX & 0x3;
	default:
		break;
	}

	return -EINVAL;
}

static int lmx2582_set_outmux(struct iio_dev *indio_dev,
			      const struct iio_chan_spec *chan,
			      unsigned int mode)
{
	struct lmx2582_state *st = iio_priv(indio_dev);

	/* check mode */
	switch (mode) {
	case LMX2582_OUTx_MUX_VCO:
	case LMX2582_OUTx_MUX_CHDIV:
		break;
	default:
		return -EINVAL;
	}

	switch(chan->channel) {
	case LMX2582_CH_RFOUTA:
		st->conf->OUTA_MUX = mode & 0x3;
		return lmx2582_setup(st, 0);
	case LMX2582_CH_RFOUTB:
		st->conf->OUTB_MUX = mode & 0x3;
		return lmx2582_setup(st, 0);
	default:
		break;
	}

	return -EINVAL;
}

static const char * const lmx2582_out_mux_names[] = {
	[0] = "channel-divider",
	[1] = "vco",
};
static const struct iio_enum lmx2582_out_mux_available = {
	.items = lmx2582_out_mux_names,
	.num_items = ARRAY_SIZE(lmx2582_out_mux_names),
	.get = lmx2582_get_outmux,
	.set = lmx2582_set_outmux,
};


#define _LMX2582_EXT_INFO(_name, _ident, _shared) { \
	.name = _name, \
	.read = lmx2582_read, \
	.write = lmx2582_write, \
	.private = _ident, \
	.shared = _shared, \
}

static const struct iio_chan_spec_ext_info lmx2582_ext_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	_LMX2582_EXT_INFO("frequency", LMX2582_FREQ, IIO_SEPARATE),
	_LMX2582_EXT_INFO("powerdown", LMX2582_PWRDOWN, IIO_SEPARATE),
	_LMX2582_EXT_INFO("name", LMX2582_CHANNEL_NAME, IIO_SEPARATE),
	IIO_ENUM("out_mux", IIO_SEPARATE, &lmx2582_out_mux_available),
	IIO_ENUM_AVAILABLE("out_mux", &lmx2582_out_mux_available),
	{ },
};

#define LMX2582_CHANNEL(index) { \
	.type = IIO_ALTVOLTAGE, \
	.output = 1, \
	.channel = index, \
	.ext_info = lmx2582_ext_info, \
	.indexed = 1, \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_PHASE), \
}

static const struct iio_chan_spec lmx2582_chan[LMX2582_CLK_COUNT] = {
	LMX2582_CHANNEL(LMX2582_CH_RFOUTA),
	LMX2582_CHANNEL(LMX2582_CH_RFOUTB),
};

static int lmx2582_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val,
			int *val2,
			long info)
{
	switch (info) {
	case IIO_CHAN_INFO_PHASE:
		*val = 0;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_PROCESSED:
		*val = 0;
		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static int lmx2582_write_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int val,
			int val2,
			long info)
{
	switch (info) {
	case IIO_CHAN_INFO_PHASE:
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct iio_info lmx2582_info = {
	.debugfs_reg_access = &lmx2582_reg_access,
	.read_raw = &lmx2582_read_raw,
	.write_raw = lmx2582_write_raw,
};

static void lmx2582_property_u32(struct lmx2582_state *st,
	const char *name, u32 *val, u32 min, u32 max)
{
	int ret;
	u32 tmp;
	bool out_of_range;

	ret = device_property_read_u32(&st->spi->dev, name, &tmp);
	if (ret == 0)
		out_of_range = tmp > max || tmp < min;
	else
		out_of_range = true;

	if (!out_of_range)
			*val = tmp;

	if (ret < 0 || out_of_range)
		dev_warn(&st->spi->dev,
			 "using default value for %s: %u",
			 name,
			 *val);

	if (st->dent)
		debugfs_create_u32(name, 0644, st->dent, val);
}

static void lmx2582_property_u64(struct lmx2582_state *st,
	const char *name, u64 *val, u64 min, u64 max)
{
	int ret;
	u64 tmp;
	bool out_of_range;

	ret = device_property_read_u64(&st->spi->dev, name, &tmp);
	if (ret == 0)
		out_of_range = tmp > max || tmp < min;
	else
		out_of_range = true;

	if (!out_of_range)
			*val = tmp;

	if (ret < 0 || out_of_range)
		dev_warn(&st->spi->dev,
			 "using default value for %s: %llu",
			 name,
			 *val);

	if (st->dent)
		debugfs_create_u64(name, 0644, st->dent, val);
}

static void lmx2582_property_bool(struct lmx2582_state *st,
	const char *name, bool *val)
{
	*val = device_property_read_bool(&st->spi->dev, name);

	if (st->dent)
		debugfs_create_bool(name, 0644, st->dent, val);
}

static struct lmx2582_config *lmx2582_parse_dt(struct lmx2582_state *st)
{
	struct lmx2582_config *conf;
	struct fwnode_handle *child;
	unsigned int channel, tmp;
	bool btemp;
	int ret;

	conf = devm_kzalloc(&st->spi->dev, sizeof(*conf), GFP_KERNEL);
	if (!conf)
		return NULL;
	
	/* copy default values */
	memcpy(conf, &lmx2582_default_values, sizeof(*conf));

	lmx2582_property_bool(st, "lmx,muxout-sel-readback", &btemp);
	conf->MUXOUT_SEL = !btemp;

	lmx2582_property_bool(st, "lmx,osc-2x", &conf->OSC_2X);

	lmx2582_property_u32(st, "lmx,mult", &conf->MULT, 1, 6);
	lmx2582_property_u32(st, "lmx,pll-r", &conf->PLL_R, 1, 128);
	lmx2582_property_u32(st, "lmx,pll-r-pre", &conf->PLL_R_PRE, 1, 128);

	lmx2582_property_u32(st, "lmx,pfd-ctl", &conf->PFD_CTL, 0, 3);

	lmx2582_property_bool(st, "lmx,cp-disable", &btemp);
	conf->CP_EN = !btemp;

	lmx2582_property_u32(st, "lmx,cp-idn", &conf->CP_IDN, 0, 31);
	lmx2582_property_u32(st, "lmx,cp-iup", &conf->CP_IUP, 0, 31);
	lmx2582_property_u32(st, "lmx,cp-icoarse", &conf->CP_ICOARSE, 0, 3);

	lmx2582_property_bool(st, "lmx,pll-n-pre-x4", &conf->PLL_N_PRE);
	lmx2582_property_u32(st, "lmx,pll-n", &conf->PLL_N, 0, 4095);
	lmx2582_property_u32(st, "lmx,pll-den", &conf->PLL_DEN, 1, U32_MAX);
	lmx2582_property_u32(st, "lmx,pll-num", &conf->PLL_NUM, 0, U32_MAX);

	lmx2582_property_bool(st, "lmx,muxout-hdrv", &conf->MUXOUT_HDRV);

	lmx2582_property_bool(st, "lmx,ld-type-calstat", &btemp);
	conf->LD_TYPE = !btemp;

	lmx2582_property_u32(st, "lmx,chdiv-seg1", &conf->CHDIV_SEG1, 0, 1);
	lmx2582_property_u32(st, "lmx,chdiv-seg2", &conf->CHDIV_SEG2, 0, 8);
	lmx2582_property_u32(st, "lmx,chdiv-seg3", &conf->CHDIV_SEG3, 0, 8);
	lmx2582_property_u32(st, "lmx,chdiv-seg-sel", &conf->CHDIV_SEG_SEL, 0, 4);

	ret = device_property_read_string_array(&st->spi->dev,
				"clock-output-names",
				st->lmx2582_clk_names,
				LMX2582_CLK_COUNT);

	if (ret < 0) {
		dev_warn(&st->spi->dev, "Using the default clk names");
		st->has_clk_out_names = false;
	} else {
		st->has_clk_out_names = true;
	}

	device_for_each_child_node(&st->spi->dev, child) {
		ret = fwnode_property_read_u32(child, "reg", &channel);
		if (ret)
			continue;
		if (channel >= LMX2582_CLK_COUNT)
			continue;

		ret = fwnode_property_present(child, "lmx,output-enable");
		st->outputs[channel].enabled = ret;

		st->outputs[channel].power = 15;
		ret = fwnode_property_read_u32(child, "lmx,output-power", &tmp);
		if (ret == 0)
			st->outputs[channel].power = tmp;

		/* Default to Divider */
		st->outputs[channel].muxsel = LMX2582_OUTx_MUX_CHDIV;
		if (fwnode_property_present(child, "lmx,mux-sel-vco"))
			st->outputs[channel].muxsel = LMX2582_OUTx_MUX_VCO;
	}

	return conf;
}

static unsigned long lmx2582_clk_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	unsigned long long rate;

	rate = lmx2582_pll_fract_n_get_rate(to_clk_priv(hw)->st);

	return to_ccf_scaled(rate, &to_clk_priv(hw)->scale);
}

static long lmx2582_clk_round_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long *parent_rate)
{
	return rate;
}

static int lmx2582_clk_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	//struct lmx2582_state *st = to_clk_priv(hw)->st;

	/*return lmx2582_setup(st, parent_rate,
		from_ccf_scaled(rate, &to_clk_priv(hw)->scale));*/
	return -EINVAL;
}

static int lmx2582_clk_enable(struct clk_hw *hw)
{
	to_clk_priv(hw)->enabled = true;

	return 0;
}

static void lmx2582_clk_disable(struct clk_hw *hw)
{
	to_clk_priv(hw)->enabled = false;
}

static int lmx2582_clk_is_enabled(struct clk_hw *hw)
{
	return to_clk_priv(hw)->enabled;
}

static const struct clk_ops lmx2582_clock_ops = {
	.recalc_rate = lmx2582_clk_recalc_rate,
	.round_rate = lmx2582_clk_round_rate,
	.set_rate = lmx2582_clk_set_rate,
	.enable = lmx2582_clk_enable,
	.disable = lmx2582_clk_disable,
	.is_enabled = lmx2582_clk_is_enabled,
};

static void lmx2582_of_clk_del_provider(void *data)
{
	struct lmx2582_state *st = data;

	of_clk_del_provider(st->spi->dev.of_node);
}

static void lmx2582_action_clk_disable(void *data)
{
	struct clk *clk = data;

	clk_disable_unprepare(clk);
}

static void lmx2582_powerdown(void *data)
{
	struct lmx2582_state *st = data;

	st->regs[LMX2582_R0] |= LMX2582_R0_POWERDOWN(1);
	lmx2582_sync_config(st);
}

static int lmx2582_clk_register(struct iio_dev *indio_dev,
				unsigned int channel,
				const char *parent_name)
{
	struct lmx2582_state *st = iio_priv(indio_dev);
	struct clk_init_data init;
	struct clk *clk_out;
	char name[128];

	if (!st->has_clk_out_names) {
		snprintf(name, sizeof(name), "%s_RFout%c", indio_dev->name, 'A' + channel);
		st->lmx2582_clk_names[channel] = name;
	}
	init.name = st->lmx2582_clk_names[channel];
	init.ops = &lmx2582_clock_ops;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);
	init.flags = CLK_GET_RATE_NOCACHE;

	st->outputs[channel].hw.init = &init;
	st->outputs[channel].indio_dev = indio_dev;
	st->outputs[channel].num = channel;

	clk_out = devm_clk_register(&st->spi->dev, &st->outputs[channel].hw);
	if (IS_ERR(clk_out))
		return PTR_ERR(clk_out);

	st->clks[channel] = clk_out;

	return 0;
}

static int lmx2582_clks_register(struct iio_dev *indio_dev)
{
	struct lmx2582_state *st = iio_priv(indio_dev);
	int i, ret;

	st->clk_data.clks = devm_kcalloc(&st->spi->dev,
					 LMX2582_CLK_COUNT,
					 sizeof(struct clk *), GFP_KERNEL);
	if (!st->clk_data.clks)
		return -ENOMEM;

	for (i = 0; i < LMX2582_CLK_COUNT; i++) {
		ret = lmx2582_clk_register(indio_dev, i,
					   __clk_get_name(st->clkin));
		if (ret < 0) {
			dev_err(&st->spi->dev,
				"Clock provider register failed\n");
			return ret;
		}
	}

	st->clk_data.clks = st->clks;
	st->clk_data.clk_num = LMX2582_CLK_COUNT;
	ret = of_clk_add_provider(st->spi->dev.of_node,
				  of_clk_src_onecell_get, &st->clk_data);
	if (ret < 0)
		return ret;

	return devm_add_action_or_reset(&st->spi->dev,
					lmx2582_of_clk_del_provider, st);
}

static int lmx2582_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	struct iio_dev *indio_dev;
	struct lmx2582_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->clkin = devm_clk_get(&spi->dev, "clkin");
	if (IS_ERR(st->clkin))
		return -EPROBE_DEFER;

	ret = clk_prepare_enable(st->clkin);
	if (ret < 0)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev,
		lmx2582_action_clk_disable, st->clkin);
	if (ret)
		return ret;

	spi_set_drvdata(spi, indio_dev);
	st->spi = spi;
	mutex_init(&st->lock);

	if (spi->dev.of_node)
		indio_dev->name = spi->dev.of_node->name;
	else
		indio_dev->name = id->name;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->info = &lmx2582_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = &lmx2582_chan[0];
	indio_dev->num_channels = LMX2582_CLK_COUNT;

	ret = devm_iio_device_register(&spi->dev, indio_dev);
	if (ret)
		return ret;

	st->dent = iio_get_debugfs_dentry(indio_dev);

	st->conf = lmx2582_parse_dt(st);
	if (!st->conf)
		return -ENOMEM;

	ret = lmx2582_soft_reset(st);
	if (ret)
		return ret;
	
	ret = lmx2582_setup(st, 0);
	if (ret)
		return ret;

	ret = lmx2582_clks_register(indio_dev);
	if (ret)
		return ret;

	return devm_add_action_or_reset(&spi->dev, lmx2582_powerdown, st);
}

static const struct spi_device_id lmx2582_id[] = {
	{"lmx2582", 0},
	{}
};

static struct spi_driver lmx2582_driver = {
	.driver = {
		.name	= "lmx2582",
	},
	.probe		= lmx2582_probe,
	.id_table	= lmx2582_id,
};
module_spi_driver(lmx2582_driver);

MODULE_AUTHOR("Philipp Diethelm");
MODULE_DESCRIPTION("Texas Instruments LMX2582 SPI PLL");
MODULE_LICENSE("GPL v2");

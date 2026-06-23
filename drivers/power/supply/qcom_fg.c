// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 *
 * Qualcomm PM8150B Fuel Gauge (FG-GEN4) driver.
 */

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/unaligned.h>

/* SOC */
#define BATT_MONOTONIC_SOC 0x009
#define FULL_SOC_RAW 255
#define FULL_SOC_REPORT_THR 250

/* BATT (Gen4 uses ADC_RR for temperature) */
#define ADC_RR_BATT_TEMP_LSB 0x288
#define PARAM_ADDR_BATT_VOLTAGE 0x1a0
#define PARAM_ADDR_BATT_CURRENT 0x1a2

#define BATT_VOLTAGE_NUMR 122070
#define BATT_VOLTAGE_DENR 1000
#define BATT_CURRENT_NUMR 488281
#define BATT_CURRENT_DENR 1000

#define BATT_INFO_BASE 0x100
#define MEM_IF_BASE 0x300
#define MEM_IF_INT_RT_STS (MEM_IF_BASE + 0x10)
#define MEM_INTF_CFG (MEM_IF_BASE + 0x50)
#define MEM_IF_DMA_STS (MEM_IF_BASE + 0x70)
#define MEM_IF_DMA_CTL (MEM_IF_BASE + 0x71)
#define MEM_IF_MEM_ARB_CFG (MEM_IF_BASE + 0x40)
#define BATT_INFO_PEEK_MUX4 (BATT_INFO_BASE + 0xEE)
#define ALG_ACTIVE_PEEK_CFG 0xAC

/* MEMIF bits */
#define RIF_MEM_ACCESS_REQ BIT(7)
#define GEN4_MEM_GNT_BIT BIT(3)
#define MEM_ARB_REQ_BIT BIT(0)
#define MEM_ARB_LO_LATENCY_EN_BIT BIT(1)
#define MEM_CLR_LOG_BIT BIT(2)
#define ADDR_KIND_BIT BIT(1)
#define IACS_SLCT_BIT BIT(5)

/* DMA partition mapping for Gen4 FG (PM8150B) */
#define GEN4_FG_DMA0_BASE 0x4400
#define GEN4_FG_DMA1_BASE 0x4500
#define GEN4_FG_DMA2_BASE 0x4600
#define GEN4_FG_DMA3_BASE 0x4700
#define GEN4_FG_DMA4_BASE 0x4800
#define GEN4_FG_DMA5_BASE 0x4900
#define SRAM_ADDR_OFFSET 0x20
#define FG_GEN4_NUM_PARTITIONS 6
#define FG_GEN4_BYTES_PER_WORD 2

/* SRAM word addresses (PM8150B V2) */
#define PROFILE_LOAD_WORD 65
#define PROFILE_INTEGRITY_WORD 299
#define ESR_CAL_SOC_MIN_WORD 10
#define ESR_CAL_THRESH_WORD 11
#define ESR_PULSE_THRESH_WORD 12
#define ESR_TIMER_FAST_CHG_WORD 15
#define ESR_TIMER_FAST_DISCHG_WORD 16
#define ESR_TIMER_DISCHG_WORD 17
#define ESR_TIMER_CHG_WORD 18
#define CUTOFF_CURR_WORD 19
#define CUTOFF_VOLT_WORD 20
#define SYS_TERM_CURR_WORD 22
#define KI_COEFF_LOW_CHG_WORD 28
#define KI_COEFF_LOW_CHG_OFFSET 0
#define KI_COEFF_MED_CHG_WORD 28
#define KI_COEFF_MED_CHG_OFFSET 1
#define KI_COEFF_HI_CHG_WORD 29
#define KI_COEFF_HI_CHG_OFFSET 0
#define KI_COEFF_LO_MED_CHG_THR_WORD 29
#define KI_COEFF_LO_MED_CHG_THR_OFFSET 1
#define KI_COEFF_MED_HI_CHG_THR_WORD 30
#define KI_COEFF_MED_HI_CHG_THR_OFFSET 0
#define VBATT_LOW_WORD 35
#define NOM_CAP_WORD 271
#define ACT_BATT_CAP_WORD 287
#define CYCLE_COUNT_WORD 291
#define BATT_SOC_WORD 455
#define CC_SOC_SW_WORD 464

/* Profile constants */
#define PROFILE_LOAD_BIT BIT(0)
#define HLOS_RESTART_BIT BIT(3)
#define PROFILE_LEN 416

/* JEITA temperature thresholds (decidegrees C) */
#define BATT_TEMP_JEITA_COLD 100
#define BATT_TEMP_JEITA_HOT 450

/* Capacity learning */
#define CC_SOC_30BIT 0x3fffffff
#define CENTI_FULL_SOC 10000
#define BATT_SOC_32BIT 0xffffffff

/* Nominal voltage floor (μV) for ENERGY_* when DT lacks an explicit value. */
#define QCOM_FG_NOMINAL_VOLTAGE_FLOOR_UV 3700000

struct fg_dma_partition {
	u16 partition_start;
	u16 partition_end;
	u16 spmi_addr_base;
};

static const struct fg_dma_partition fg_gen4_partitions[] = {
	{ 0, 63, GEN4_FG_DMA0_BASE + SRAM_ADDR_OFFSET }, /* system */
	{ 64, 169, GEN4_FG_DMA1_BASE + SRAM_ADDR_OFFSET }, /* battery profile */
	{ 170, 274,
	  GEN4_FG_DMA2_BASE + SRAM_ADDR_OFFSET }, /* battery profile cont. */
	{ 275, 299, GEN4_FG_DMA3_BASE + SRAM_ADDR_OFFSET }, /* dp/SW */
	{ 300, 405, GEN4_FG_DMA4_BASE + SRAM_ADDR_OFFSET }, /* wk/scratch */
	{ 406, 486,
	  GEN4_FG_DMA5_BASE + SRAM_ADDR_OFFSET }, /* wk/scratch cont. */
};

struct qcom_fg_chip {
	struct device *dev;
	unsigned int base;
	struct regmap *regmap;
	struct notifier_block nb;

	struct power_supply *batt_psy;
	struct power_supply_battery_info *batt_info;
	struct power_supply *chg_psy;

	/* Battery identification (set once during init_capacity_learning) */
	const char *manufacturer;
	const char *model_name;
	int nom_voltage_uv;

	/* Capacity learning */
	struct mutex cl_lock;
	bool cl_active;
	int cl_init_cc_soc_sw;
	s64 nom_cap_uah;
	s64 init_cap_uah;
	s64 final_cap_uah;
	int cl_max_start_soc;
	int cl_max_cap_inc;
	int cl_max_cap_dec;

	struct completion mem_attn_done;
	struct mutex dma_lock;

	/* Lock order: cl_lock -> state_lock. dma_lock is independent. */
	struct mutex state_lock;
	int status;
	int batt_soc;
	s64 learned_cap_uah;
};

/* IO FUNCTIONS */

static int qcom_fg_read(struct qcom_fg_chip *chip, u8 *val, u16 addr, int len)
{
	if (((chip->base + addr) & 0xff00) == 0)
		return -EINVAL;

	return regmap_bulk_read(chip->regmap, chip->base + addr, val, len);
}

static int qcom_fg_write(struct qcom_fg_chip *chip, u8 *val, u16 addr, int len)
{
	bool sec_access = (addr & 0xff) > 0xd0;
	u8 sec_addr_val = 0xa5;
	int ret;

	if (((chip->base + addr) & 0xff00) == 0)
		return -EINVAL;

	if (sec_access) {
		ret = regmap_bulk_write(chip->regmap,
					((chip->base + addr) & 0xff00) | 0xd0,
					&sec_addr_val, 1);
		if (ret)
			return ret;
	}

	return regmap_bulk_write(chip->regmap, chip->base + addr, val, len);
}

static int qcom_fg_masked_write(struct qcom_fg_chip *chip, u16 addr, u8 mask,
				u8 val)
{
	u8 reg;
	int ret;

	ret = qcom_fg_read(chip, &reg, addr, 1);
	if (ret)
		return ret;

	reg &= ~mask;
	reg |= val & mask;

	return qcom_fg_write(chip, &reg, addr, 1);
}

/* Gen4 FG SRAM spans 6 DMA partitions; cross-partition transfers are handled here. */

/* Resolve (sram_addr, offset) to SPMI addr + bytes available in partition. */
static int qcom_fg_dma_resolve(struct qcom_fg_chip *chip, u16 sram_addr,
			       u8 offset, int len, u16 *addr, int *avail)
{
	int i;

	for (i = 0; i < FG_GEN4_NUM_PARTITIONS; i++) {
		if (sram_addr >= fg_gen4_partitions[i].partition_start &&
		    sram_addr <= fg_gen4_partitions[i].partition_end) {
			*addr = fg_gen4_partitions[i].spmi_addr_base + offset +
				(sram_addr -
				 fg_gen4_partitions[i].partition_start) *
					FG_GEN4_BYTES_PER_WORD;
			*avail = (fg_gen4_partitions[i].partition_end -
				  sram_addr + 1) *
					 FG_GEN4_BYTES_PER_WORD -
				 offset;
			if (*avail > len)
				*avail = len;
			return 0;
		}
	}

	dev_err(chip->dev, "Couldn't find DMA partition for SRAM word %d\n",
		sram_addr);
	return -ENXIO;
}

static irqreturn_t qcom_fg_mem_attn_irq_handler(int irq, void *data)
{
	struct qcom_fg_chip *chip = data;

	complete_all(&chip->mem_attn_done);
	return IRQ_HANDLED;
}

static int qcom_fg_dma_request(struct qcom_fg_chip *chip)
{
	u8 sts;
	int ret;

	ret = qcom_fg_masked_write(chip, MEM_IF_MEM_ARB_CFG, MEM_ARB_REQ_BIT,
				   MEM_ARB_REQ_BIT);
	if (ret)
		return ret;

	ret = qcom_fg_masked_write(chip, MEM_INTF_CFG,
				   RIF_MEM_ACCESS_REQ | IACS_SLCT_BIT,
				   RIF_MEM_ACCESS_REQ);
	if (ret)
		goto out_release_arb;

	/* Skip wait if MEM_GNT already set. */
	ret = qcom_fg_read(chip, &sts, MEM_IF_INT_RT_STS, 1);
	if (ret)
		goto out_release;

	if (sts & GEN4_MEM_GNT_BIT)
		return 0;

	reinit_completion(&chip->mem_attn_done);
	if (!wait_for_completion_timeout(&chip->mem_attn_done,
					 msecs_to_jiffies(1000))) {
		ret = -ETIMEDOUT;
		goto out_release;
	}

	ret = qcom_fg_read(chip, &sts, MEM_IF_INT_RT_STS, 1);
	if (ret)
		goto out_release;

	if (sts & GEN4_MEM_GNT_BIT)
		return 0;

	ret = -ETIMEDOUT;
out_release:
	qcom_fg_masked_write(chip, MEM_INTF_CFG,
			     RIF_MEM_ACCESS_REQ | IACS_SLCT_BIT, 0);
out_release_arb:
	qcom_fg_masked_write(chip, MEM_IF_MEM_ARB_CFG, MEM_ARB_REQ_BIT, 0);
	return ret;
}

static void qcom_fg_dma_release(struct qcom_fg_chip *chip)
{
	qcom_fg_masked_write(chip, MEM_INTF_CFG,
			     RIF_MEM_ACCESS_REQ | IACS_SLCT_BIT, 0);
	qcom_fg_masked_write(chip, MEM_IF_MEM_ARB_CFG, MEM_ARB_REQ_BIT, 0);
}

/* @offset < FG_GEN4_BYTES_PER_WORD. */
static int qcom_fg_sram_xfer(struct qcom_fg_chip *chip, u16 sram_addr,
			     u8 offset, u8 *val, int len, bool write)
{
	u8 *ptr = val;
	u16 addr;
	int num_bytes, ret;

	WARN_ON(offset >= FG_GEN4_BYTES_PER_WORD);
	WARN_ON(len <= 0);

	mutex_lock(&chip->dma_lock);

	ret = qcom_fg_dma_request(chip);
	if (ret) {
		dev_err(chip->dev, "Failed to request DMA access: %d\n", ret);
		goto out;
	}

	while (len > 0) {
		ret = qcom_fg_dma_resolve(chip, sram_addr, offset, len, &addr,
					  &num_bytes);
		if (ret)
			goto out;

		if (write)
			ret = regmap_bulk_write(chip->regmap, addr, ptr,
						num_bytes);
		else
			ret = regmap_bulk_read(chip->regmap, addr, ptr,
					       num_bytes);
		if (ret) {
			dev_err(chip->dev, "DMA %s failed at word %d: %d\n",
				write ? "write" : "read", sram_addr, ret);
			goto out;
		}

		ptr += num_bytes;
		len -= num_bytes;
		sram_addr += num_bytes / FG_GEN4_BYTES_PER_WORD;
		offset = 0;
	}

out:
	qcom_fg_dma_release(chip);
	mutex_unlock(&chip->dma_lock);
	return ret;
}

static int qcom_fg_sram_write(struct qcom_fg_chip *chip, u16 sram_addr,
			      u8 offset, u8 *val, int len)
{
	return qcom_fg_sram_xfer(chip, sram_addr, offset, val, len, true);
}

static int qcom_fg_sram_read(struct qcom_fg_chip *chip, u16 sram_addr,
			     u8 offset, u8 *val, int len)
{
	return qcom_fg_sram_xfer(chip, sram_addr, offset, val, len, false);
}

static int qcom_fg_dma_init(struct qcom_fg_chip *chip)
{
	int ret;
	u8 val;

	/* Configure the DMA peripheral addressing to partition mode */
	ret = qcom_fg_masked_write(chip, MEM_IF_DMA_CTL, ADDR_KIND_BIT,
				   ADDR_KIND_BIT);
	if (ret) {
		dev_err(chip->dev, "Failed to configure DMA_CTL: %d\n", ret);
		return ret;
	}

	/* Release DMA so that request can happen */
	qcom_fg_dma_release(chip);

	/* Set low latency and clear log bit */
	ret = qcom_fg_masked_write(chip, MEM_IF_MEM_ARB_CFG,
				   MEM_ARB_LO_LATENCY_EN_BIT | MEM_CLR_LOG_BIT,
				   MEM_ARB_LO_LATENCY_EN_BIT);
	if (ret) {
		dev_err(chip->dev, "Failed to configure MEM_ARB_CFG: %d\n",
			ret);
		return ret;
	}

	/* Configure PEEK_MUX4 for ALG active signal */
	val = ALG_ACTIVE_PEEK_CFG;
	ret = qcom_fg_write(chip, &val, BATT_INFO_PEEK_MUX4, 1);
	if (ret) {
		dev_err(chip->dev, "Failed to configure PEEK_MUX4: %d\n", ret);
		return ret;
	}

	return 0;
}

/* CAPACITY LEARNING */

static int qcom_fg_get_cc_soc_sw(struct qcom_fg_chip *chip, int *cc_soc_sw)
{
	u8 buf[4];
	int ret;

	ret = qcom_fg_sram_read(chip, CC_SOC_SW_WORD, 0, buf, 4);
	if (ret)
		return ret;

	*cc_soc_sw = get_unaligned_le32(buf) & CC_SOC_30BIT;
	return 0;
}

static int qcom_fg_prime_cc_soc_sw(struct qcom_fg_chip *chip, u32 cc_soc_sw)
{
	u8 buf[4];

	put_unaligned_le32(cc_soc_sw & CC_SOC_30BIT, buf);

	return qcom_fg_sram_write(chip, CC_SOC_SW_WORD, 0, buf, 4);
}

static int qcom_fg_get_batt_soc_cp(struct qcom_fg_chip *chip, int *batt_soc_cp)
{
	u8 buf[4];
	u32 batt_soc;
	int ret;

	ret = qcom_fg_sram_read(chip, BATT_SOC_WORD, 0, buf, 4);
	if (ret)
		return ret;

	batt_soc = get_unaligned_le32(buf);
	*batt_soc_cp =
		div64_u64((u64)batt_soc * CENTI_FULL_SOC, BATT_SOC_32BIT);
	return 0;
}

static int qcom_fg_get_learned_capacity(struct qcom_fg_chip *chip, s64 *cap_uah)
{
	u8 buf[2];
	int cc_mah, ret;

	ret = qcom_fg_sram_read(chip, ACT_BATT_CAP_WORD, 0, buf, 2);
	if (ret)
		return ret;

	cc_mah = get_unaligned_le16(buf);
	*cap_uah = (s64)cc_mah * 1000;
	return 0;
}

static int qcom_fg_store_learned_capacity(struct qcom_fg_chip *chip,
					  s64 learned_cap_uah)
{
	u8 buf[2];
	int cc_mah;

	cc_mah = (int)div_s64(learned_cap_uah, 1000);
	put_unaligned_le16(cc_mah, buf);

	return qcom_fg_sram_write(chip, ACT_BATT_CAP_WORD, 0, buf, 2);
}

/* Cycle counter mirrored by FG HW (word 291, 2 bytes LE). */
static int qcom_fg_get_cycle_count(struct qcom_fg_chip *chip, int *count)
{
	u8 buf[2];
	int ret;

	ret = qcom_fg_sram_read(chip, CYCLE_COUNT_WORD, 0, buf, 2);
	if (ret)
		return ret;

	*count = get_unaligned_le16(buf);
	return 0;
}

static int qcom_fg_cap_learning_begin(struct qcom_fg_chip *chip,
				      int batt_soc_cp)
{
	int batt_soc_pct, cc_soc_sw, ret;
	s64 learned_cap;
	u32 batt_soc_prime;

	batt_soc_pct = DIV_ROUND_CLOSEST(batt_soc_cp, 100);

	if (chip->cl_max_start_soc > 0 && batt_soc_pct > chip->cl_max_start_soc)
		return -EINVAL;

	mutex_lock(&chip->state_lock);
	learned_cap = chip->learned_cap_uah;
	mutex_unlock(&chip->state_lock);

	chip->init_cap_uah =
		div64_s64(learned_cap * batt_soc_cp, CENTI_FULL_SOC);

	batt_soc_prime =
		div64_u64((u64)batt_soc_cp * CC_SOC_30BIT, CENTI_FULL_SOC);
	ret = qcom_fg_prime_cc_soc_sw(chip, batt_soc_prime);
	if (ret)
		return ret;

	ret = qcom_fg_get_cc_soc_sw(chip, &cc_soc_sw);
	if (ret)
		return ret;

	chip->cl_init_cc_soc_sw = cc_soc_sw;
	chip->cl_active = true;

	dev_dbg(chip->dev,
		"Cap learning started: soc=%d%%, init_cap=%lld uAh\n",
		batt_soc_pct, chip->init_cap_uah);
	return 0;
}

static s64 qcom_fg_cap_learning_post_process(struct qcom_fg_chip *chip)
{
	s64 max_inc_val, min_dec_val;

	max_inc_val = div64_s64(
		chip->learned_cap_uah * (1000 + chip->cl_max_cap_inc), 1000);
	min_dec_val = div64_s64(
		chip->learned_cap_uah * (1000 - chip->cl_max_cap_dec), 1000);

	return clamp_t(s64, chip->final_cap_uah, min_dec_val, max_inc_val);
}

static int qcom_fg_cap_learning_done(struct qcom_fg_chip *chip)
{
	int cc_soc_sw, cc_soc_delta_pct, cc_soc_fraction;
	s64 cc_soc_cap_uah, cc_soc_fraction_uah, delta_cap_uah, new_cap_uah;
	int ret;

	ret = qcom_fg_get_cc_soc_sw(chip, &cc_soc_sw);
	if (ret)
		goto out;

	cc_soc_delta_pct =
		div_s64_rem((s64)(cc_soc_sw - chip->cl_init_cc_soc_sw) * 100,
			    CC_SOC_30BIT, &cc_soc_fraction);

	if (cc_soc_delta_pct < 50) {
		if (cc_soc_delta_pct < 0)
			dev_warn(
				chip->dev,
				"Cap learning: negative delta %d%%, skipping\n",
				cc_soc_delta_pct);
		else
			dev_dbg(chip->dev,
				"Cap learning: delta %d%% too small, skipping\n",
				cc_soc_delta_pct);
		goto out;
	}

	cc_soc_cap_uah =
		div64_s64(chip->learned_cap_uah * cc_soc_delta_pct, 100);
	cc_soc_fraction_uah = div64_s64(chip->learned_cap_uah * cc_soc_fraction,
					(s64)CC_SOC_30BIT * 100);
	delta_cap_uah = cc_soc_cap_uah + cc_soc_fraction_uah;

	chip->final_cap_uah = chip->init_cap_uah + delta_cap_uah;
	new_cap_uah = qcom_fg_cap_learning_post_process(chip);

	ret = qcom_fg_store_learned_capacity(chip, new_cap_uah);
	if (ret)
		goto out;

	ret = qcom_fg_prime_cc_soc_sw(chip, CC_SOC_30BIT);
	if (ret)
		goto out;

	mutex_lock(&chip->state_lock);
	chip->learned_cap_uah = new_cap_uah;
	mutex_unlock(&chip->state_lock);

	dev_info(chip->dev,
		 "Cap learning done: new_cap=%lld uAh (delta=%lld uAh)\n",
		 new_cap_uah, chip->final_cap_uah - chip->init_cap_uah);

out:
	chip->cl_active = false;
	return ret;
}

/* IDLE + CHARGING → begin; LEARNING + FULL → done; LEARNING + DISCHARGING → abort. */
static void qcom_fg_cap_learning_update(struct qcom_fg_chip *chip)
{
	int batt_soc_cp;
	int status;
	bool has_cap;

	mutex_lock(&chip->state_lock);
	has_cap = chip->learned_cap_uah > 0;
	status = chip->status;
	mutex_unlock(&chip->state_lock);

	if (!has_cap)
		return;

	mutex_lock(&chip->cl_lock);

	if (qcom_fg_get_batt_soc_cp(chip, &batt_soc_cp))
		goto unlock;

	if (chip->cl_active) {
		/* LEARNING state: check for termination conditions. */
		switch (status) {
		case POWER_SUPPLY_STATUS_FULL:
			/* Session complete — finalize or retry on failure. */
			if (qcom_fg_cap_learning_done(chip))
				dev_warn(chip->dev,
					 "Cap learning finalization failed\n");
			break;
		case POWER_SUPPLY_STATUS_DISCHARGING:
			/* Discharge invalidates the session — abort. */
			chip->cl_active = false;
			dev_dbg(chip->dev,
				"Cap learning aborted: discharging\n");
			break;
		default:
			/* CHARGING / NOT_CHARGING / UNKNOWN — keep learning. */
			break;
		}
	} else {
		/* IDLE state: start a new session when charging begins. */
		if (status == POWER_SUPPLY_STATUS_CHARGING) {
			if (qcom_fg_cap_learning_begin(chip, batt_soc_cp))
				dev_dbg(chip->dev,
					"Cap learning begin failed\n");
		}
	}

unlock:
	mutex_unlock(&chip->cl_lock);
}

/* BATTERY STATUS */

static int qcom_fg_get_capacity(struct qcom_fg_chip *chip, int *val)
{
	bool report_full;
	u8 cap[2];
	int ret;

	ret = qcom_fg_read(chip, cap, BATT_MONOTONIC_SOC, 2);
	if (ret) {
		dev_err(chip->dev, "Failed to read capacity: %d\n", ret);
		return ret;
	}

	mutex_lock(&chip->state_lock);
	report_full = (chip->status == POWER_SUPPLY_STATUS_FULL);
	mutex_unlock(&chip->state_lock);

	cap[0] = min(cap[0], cap[1]);

	if (cap[0] == 0) {
		*val = 0;
	} else if (cap[0] == FULL_SOC_RAW) {
		*val = 100;
	} else if (report_full && cap[0] >= FULL_SOC_REPORT_THR - 2) {
		int pct = DIV_ROUND_CLOSEST(cap[0] * 100, FULL_SOC_RAW);

		*val = min(100, pct + 1);
	} else if (report_full && cap[0] >= FULL_SOC_REPORT_THR - 4) {
		*val = DIV_ROUND_CLOSEST(cap[0] * 100, FULL_SOC_RAW);
	} else {
		*val = DIV_ROUND_CLOSEST((cap[0] - 1) * 98, FULL_SOC_RAW - 2) +
		       1;
	}

	return 0;
}

static int qcom_fg_get_temperature(struct qcom_fg_chip *chip, int *val)
{
	int temp;
	u8 readval[2];
	int ret;

	ret = qcom_fg_read(chip, readval, ADC_RR_BATT_TEMP_LSB, 2);
	if (ret) {
		dev_err(chip->dev, "Failed to read temperature: %d\n", ret);
		return ret;
	}

	temp = (s16)get_unaligned_le16(readval);
	*val = temp * 10;
	return 0;
}

static int qcom_fg_get_current(struct qcom_fg_chip *chip, int *val)
{
	s16 temp;
	u8 readval[2];
	int ret;

	ret = qcom_fg_read(chip, readval, PARAM_ADDR_BATT_CURRENT, 2);
	if (ret) {
		dev_err(chip->dev, "Failed to read current: %d\n", ret);
		return ret;
	}

	temp = (s16)get_unaligned_le16(readval);

	/* FG: discharge-positive.  Invert to power_supply (charge-positive). */
	*val = -div_s64((s64)temp * BATT_CURRENT_NUMR, BATT_CURRENT_DENR);

	return 0;
}

static int qcom_fg_get_voltage(struct qcom_fg_chip *chip, int *val)
{
	int temp;
	u8 readval[2];
	int ret;

	ret = qcom_fg_read(chip, readval, PARAM_ADDR_BATT_VOLTAGE, 2);
	if (ret) {
		dev_err(chip->dev, "Failed to read voltage: %d\n", ret);
		return ret;
	}

	temp = get_unaligned_le16(readval);
	*val = div_u64((u64)temp * BATT_VOLTAGE_NUMR, BATT_VOLTAGE_DENR);

	return 0;
}

/* BATTERY POWER SUPPLY */

static enum power_supply_property qcom_fg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_FULL,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static int qcom_fg_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct qcom_fg_chip *chip = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		mutex_lock(&chip->state_lock);
		val->intval = chip->status;
		mutex_unlock(&chip->state_lock);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY: {
		mutex_lock(&chip->state_lock);
		val->intval = chip->batt_soc;
		mutex_unlock(&chip->state_lock);
		break;
	}
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL: {
		int soc;

		mutex_lock(&chip->state_lock);
		soc = chip->batt_soc;
		mutex_unlock(&chip->state_lock);

		if (soc >= 100)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		else if (soc >= 90)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
		else if (soc >= 20)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		else if (soc >= 5)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
		else if (soc > 0)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		else
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
		break;
	}
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = qcom_fg_get_current(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = qcom_fg_get_voltage(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_POWER_NOW: {
		int voltage_uv, current_ua;
		s64 power_uw;

		ret = qcom_fg_get_voltage(chip, &voltage_uv);
		if (ret)
			break;
		ret = qcom_fg_get_current(chip, &current_ua);
		if (ret)
			break;

		power_uw = (s64)voltage_uv * current_ua;
		val->intval = (int)div_s64(power_uw < 0 ? -power_uw : power_uw,
					   1000000);
		break;
	}
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = chip->batt_info->voltage_min_design_uv;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = chip->batt_info->voltage_max_design_uv;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = chip->batt_info->charge_full_design_uah;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL: {
		s64 cap;

		mutex_lock(&chip->state_lock);
		cap = chip->learned_cap_uah;
		mutex_unlock(&chip->state_lock);
		val->intval = (int)cap;
		break;
	}
	case POWER_SUPPLY_PROP_CHARGE_NOW: {
		s64 cap;
		int soc;

		mutex_lock(&chip->state_lock);
		cap = chip->learned_cap_uah;
		soc = chip->batt_soc;
		mutex_unlock(&chip->state_lock);
		val->intval = (int)div_s64(cap * soc, 100);
		break;
	}
	case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN: {
		s64 nom;
		int v;

		mutex_lock(&chip->state_lock);
		nom = chip->nom_cap_uah;
		v = chip->nom_voltage_uv;
		mutex_unlock(&chip->state_lock);
		val->intval = (int)div_s64(nom * v, 1000000);
		break;
	}
	case POWER_SUPPLY_PROP_ENERGY_FULL: {
		s64 cap;
		int v;

		mutex_lock(&chip->state_lock);
		cap = chip->learned_cap_uah;
		v = chip->nom_voltage_uv;
		mutex_unlock(&chip->state_lock);
		val->intval = (int)div_s64(cap * v, 1000000);
		break;
	}
	case POWER_SUPPLY_PROP_ENERGY_NOW: {
		s64 cap;
		int soc, v;

		mutex_lock(&chip->state_lock);
		cap = chip->learned_cap_uah;
		soc = chip->batt_soc;
		v = chip->nom_voltage_uv;
		mutex_unlock(&chip->state_lock);
		val->intval = (int)div_s64(soc * cap * v, 100 * 1000000);
		break;
	}
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = qcom_fg_get_cycle_count(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_HEALTH: {
		int batt_temp;

		ret = qcom_fg_get_temperature(chip, &batt_temp);
		if (ret)
			break;

		if (batt_temp < BATT_TEMP_JEITA_COLD)
			val->intval = POWER_SUPPLY_HEALTH_COLD;
		else if (batt_temp > BATT_TEMP_JEITA_HOT)
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	}
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = qcom_fg_get_temperature(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = chip->manufacturer;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = chip->model_name;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int qcom_fg_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	struct qcom_fg_chip *chip = power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_FULL: {
		s64 new_cap;

		if (val->intval <= 0)
			return -EINVAL;

		mutex_lock(&chip->cl_lock);
		ret = qcom_fg_store_learned_capacity(chip, val->intval);
		mutex_unlock(&chip->cl_lock);
		if (ret)
			return ret;

		new_cap = val->intval;
		mutex_lock(&chip->state_lock);
		chip->learned_cap_uah = new_cap;
		mutex_unlock(&chip->state_lock);
		power_supply_changed(chip->batt_psy);
		return 0;
	}
	default:
		return -EINVAL;
	}
}

static const struct power_supply_desc batt_psy_desc = {
	.name = "qcom-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = qcom_fg_props,
	.num_properties = ARRAY_SIZE(qcom_fg_props),
	.get_property = qcom_fg_get_property,
	.set_property = qcom_fg_set_property,
};

/* INIT FUNCTIONS */

static void qcom_fg_read_state(struct qcom_fg_chip *chip)
{
	int val;

	if (qcom_fg_get_capacity(chip, &val) == 0) {
		mutex_lock(&chip->state_lock);
		chip->batt_soc = val;
		mutex_unlock(&chip->state_lock);
	}
}

static irqreturn_t qcom_fg_handle_soc_delta(int irq, void *data)
{
	struct qcom_fg_chip *chip = data;

	qcom_fg_read_state(chip);
	qcom_fg_cap_learning_update(chip);
	power_supply_changed(chip->batt_psy);
	return IRQ_HANDLED;
}

static void qcom_fg_status_changed(struct qcom_fg_chip *chip)
{
	union power_supply_propval propval;
	int status;

	if (power_supply_get_property(chip->chg_psy, POWER_SUPPLY_PROP_STATUS,
				      &propval))
		status = POWER_SUPPLY_STATUS_UNKNOWN;
	else
		status = propval.intval;

	mutex_lock(&chip->state_lock);
	chip->status = status;
	mutex_unlock(&chip->state_lock);

	power_supply_changed(chip->batt_psy);
}

static int qcom_fg_notifier_call(struct notifier_block *nb, unsigned long val,
				 void *v)
{
	struct qcom_fg_chip *chip = container_of(nb, struct qcom_fg_chip, nb);
	struct power_supply *psy = v;

	if (psy == chip->chg_psy)
		qcom_fg_status_changed(chip);

	return NOTIFY_OK;
}

static int qcom_fg_load_profile(struct qcom_fg_chip *chip)
{
	struct device *dev = chip->dev;
	struct device_node *node = dev->of_node;
	const u8 *data;
	int ret, len;
	u8 val;

	data = of_get_property(node, "qcom,fg-profile-data", &len);
	if (!data) {
		dev_info(dev,
			 "No battery profile data, skipping profile load\n");
		return 0;
	}

	if (len != PROFILE_LEN) {
		dev_err(dev,
			"Battery profile incorrect size: %d (expected %d)\n",
			len, PROFILE_LEN);
		return -EINVAL;
	}

	ret = qcom_fg_sram_write(chip, PROFILE_LOAD_WORD, 0, (u8 *)data,
				 PROFILE_LEN);
	if (ret) {
		dev_err(dev, "Failed to write profile data: %d\n", ret);
		return ret;
	}

	val = HLOS_RESTART_BIT | PROFILE_LOAD_BIT;
	ret = qcom_fg_sram_write(chip, PROFILE_INTEGRITY_WORD, 0, &val, 1);
	if (ret)
		return ret;

	dev_info(dev, "Battery profile loaded successfully\n");
	return 0;
}

static void fg_encode_ki_coeff(u8 *buf, int val)
{
	buf[0] = (u8)div_s64((s64)val * 1000, 61035);
}

static void fg_encode_ki_coeff_thresh(u8 *buf, int val)
{
	buf[0] = (u8)div_s64((s64)val * 1000, 15625);
}

static void fg_encode_voltage(u8 *buf, int val_mv)
{
	s64 raw = div_s64((s64)val_mv * 1000000, 244141);

	put_unaligned_le16((u16)raw, buf);
}

static void fg_encode_current(u8 *buf, int val_ma)
{
	s64 raw = div_s64((s64)val_ma * 100000, 48828);

	put_unaligned_le16((u16)raw, buf);
}

static void fg_encode_vbatt_low(u8 *buf, int val_mv)
{
	buf[0] = (u8)div_s64((s64)(val_mv - 2000) * 1000, 15625);
}

static int qcom_fg_sram_write_word(struct qcom_fg_chip *chip, u16 word, u8 lo,
				   u8 hi)
{
	u8 buf[2] = { lo, hi };

	return qcom_fg_sram_write(chip, word, 0, buf, 2);
}

static int qcom_fg_sram_write_byte(struct qcom_fg_chip *chip, u16 word,
				   u8 offset, u8 val)
{
	return qcom_fg_sram_write(chip, word, offset, &val, 1);
}

struct fg_u32_param {
	const char *prop;
	u16 word;
	u8 offset;
	u8 len;
	void (*encode)(u8 *buf, int val);
};

static const struct fg_u32_param fg_u32_params[] = {
	{ "qcom,fg-cutoff-voltage", CUTOFF_VOLT_WORD, 0, 2, fg_encode_voltage },
	{ "qcom,fg-cutoff-current", CUTOFF_CURR_WORD, 0, 2, fg_encode_current },
	{ "qcom,fg-empty-voltage", VBATT_LOW_WORD, 1, 1, fg_encode_vbatt_low },
	{ "qcom,fg-sys-term-current", SYS_TERM_CURR_WORD, 0, 2,
	  fg_encode_current },
	{ "qcom,ki-coeff-low-chg", KI_COEFF_LOW_CHG_WORD,
	  KI_COEFF_LOW_CHG_OFFSET, 1, fg_encode_ki_coeff },
	{ "qcom,ki-coeff-med-chg", KI_COEFF_MED_CHG_WORD,
	  KI_COEFF_MED_CHG_OFFSET, 1, fg_encode_ki_coeff },
	{ "qcom,ki-coeff-hi-chg", KI_COEFF_HI_CHG_WORD, KI_COEFF_HI_CHG_OFFSET,
	  1, fg_encode_ki_coeff },
	{ "qcom,ki-coeff-chg-low-med-thresh-ma", KI_COEFF_LO_MED_CHG_THR_WORD,
	  KI_COEFF_LO_MED_CHG_THR_OFFSET, 1, fg_encode_ki_coeff_thresh },
	{ "qcom,ki-coeff-chg-med-hi-thresh-ma", KI_COEFF_MED_HI_CHG_THR_WORD,
	  KI_COEFF_MED_HI_CHG_THR_OFFSET, 1, fg_encode_ki_coeff_thresh },
};

/* DT array [init, max]; SRAM layout byte0=max, byte1=init. */
static const struct {
	const char *prop;
	u16 word;
} fg_esr_timers[] = {
	{ "qcom,fg-esr-timer-chg-fast", ESR_TIMER_FAST_CHG_WORD },
	{ "qcom,fg-esr-timer-dischg-fast", ESR_TIMER_FAST_DISCHG_WORD },
	{ "qcom,fg-esr-timer-chg-slow", ESR_TIMER_CHG_WORD },
	{ "qcom,fg-esr-timer-dischg-slow", ESR_TIMER_DISCHG_WORD },
};

static int qcom_fg_write_u32_param(struct qcom_fg_chip *chip,
				   struct device_node *node,
				   const struct fg_u32_param *p)
{
	int val;
	u8 buf[2];

	if (of_property_read_u32(node, p->prop, &val))
		return 0;

	p->encode(buf, val);
	return qcom_fg_sram_write(chip, p->word, p->offset, buf, p->len);
}

static int qcom_fg_configure_params(struct qcom_fg_chip *chip)
{
	struct device *dev = chip->dev;
	struct device_node *node = dev->of_node;
	const struct fg_u32_param *p;
	u32 arr[2];
	int ret, i;

	for (i = 0; i < ARRAY_SIZE(fg_u32_params); i++) {
		p = &fg_u32_params[i];
		ret = qcom_fg_write_u32_param(chip, node, p);
		if (ret)
			return ret;
	}

	for (i = 0; i < ARRAY_SIZE(fg_esr_timers); i++) {
		if (of_property_read_u32_array(node, fg_esr_timers[i].prop, arr,
					       2))
			continue;
		ret = qcom_fg_sram_write_word(chip, fg_esr_timers[i].word,
					      (u8)arr[1], (u8)arr[0]);
		if (ret)
			return ret;
	}

	if (!of_property_read_u32_array(node, "qcom,fg-esr-cal-soc-thresh", arr,
					2)) {
		ret = qcom_fg_sram_write_byte(chip, ESR_CAL_SOC_MIN_WORD, 1,
					      arr[0]);
		if (ret)
			return ret;
		ret = qcom_fg_sram_write_byte(chip, ESR_CAL_THRESH_WORD, 0,
					      arr[1]);
		if (ret)
			return ret;
	}

	if (!of_property_read_u32_array(node, "qcom,fg-esr-cal-temp-thresh",
					arr, 2)) {
		ret = qcom_fg_sram_write_byte(chip, ESR_CAL_THRESH_WORD, 1,
					      arr[0]);
		if (ret)
			return ret;
		ret = qcom_fg_sram_write_byte(chip, ESR_PULSE_THRESH_WORD, 0,
					      arr[1]);
		if (ret)
			return ret;
	}

	dev_info(dev, "FG parameters configured successfully\n");
	return 0;
}

static int qcom_fg_init_capacity_learning(struct qcom_fg_chip *chip,
					  struct device_node *node)
{
	s64 nom_cap = 0, learned_cap;
	int nom_cap_mah;
	u8 buf[2];
	int ret;

	ret = qcom_fg_sram_read(chip, NOM_CAP_WORD, 0, buf, 2);
	if (ret) {
		dev_warn(chip->dev, "Failed to read NOM_CAP from SRAM: %d\n",
			 ret);
	} else {
		nom_cap_mah = get_unaligned_le16(buf);
		nom_cap = (s64)nom_cap_mah * 1000;
	}

	if (nom_cap <= 0) {
		if (chip->batt_info->charge_full_design_uah <= 0) {
			dev_err(chip->dev,
				"NOM_CAP missing from both SRAM and DT; cannot determine battery capacity\n");
			return -EINVAL;
		}

		nom_cap = chip->batt_info->charge_full_design_uah;
		dev_info(chip->dev,
			 "NOM_CAP missing from SRAM, using DT: %lld uAh\n",
			 nom_cap);

		nom_cap_mah = (int)div_s64(nom_cap, 1000);
		put_unaligned_le16(nom_cap_mah, buf);
		ret = qcom_fg_sram_write(chip, NOM_CAP_WORD, 0, buf, 2);
		if (ret)
			dev_warn(chip->dev, "Failed to write NOM_CAP: %d\n",
				 ret);
	}

	mutex_lock(&chip->cl_lock);
	ret = qcom_fg_get_learned_capacity(chip, &learned_cap);
	if (ret || learned_cap <= 0) {
		ret = qcom_fg_store_learned_capacity(chip, nom_cap);
		if (ret)
			dev_warn(chip->dev, "Failed to init ACT_BATT_CAP: %d\n",
				 ret);
		learned_cap = nom_cap;
	}
	mutex_unlock(&chip->cl_lock);

	chip->nom_cap_uah = nom_cap;
	mutex_lock(&chip->state_lock);
	chip->learned_cap_uah = (learned_cap > 0) ? learned_cap : nom_cap;
	mutex_unlock(&chip->state_lock);

	if (of_property_read_u32(node, "qcom,battery-nominal-voltage-uv",
				 &chip->nom_voltage_uv)) {
		u32 vmin = chip->batt_info->voltage_min_design_uv;
		u32 vmax = chip->batt_info->voltage_max_design_uv;

		if (vmin > 0 && vmax > 0)
			chip->nom_voltage_uv = (vmin + vmax) / 2;
		else
			chip->nom_voltage_uv = QCOM_FG_NOMINAL_VOLTAGE_FLOOR_UV;
	}

	chip->manufacturer = "Unknown";
	chip->model_name = "Unknown";
	of_property_read_string(node, "qcom,battery-manufacturer",
				&chip->manufacturer);
	of_property_read_string(node, "qcom,battery-model-name",
				&chip->model_name);

	of_property_read_u32(node, "qcom,cl-start-capacity",
			     &chip->cl_max_start_soc);
	if (!chip->cl_max_start_soc)
		chip->cl_max_start_soc = 15;

	if (of_property_read_u32(node, "qcom,cl-max-increment",
				 &chip->cl_max_cap_inc))
		chip->cl_max_cap_inc = 5;

	if (of_property_read_u32(node, "qcom,cl-max-decrement",
				 &chip->cl_max_cap_dec))
		chip->cl_max_cap_dec = 100;

	if (chip->cl_max_cap_dec > 1000) {
		dev_warn(chip->dev,
			 "cl-max-decrement %d exceeds 1000%%, clamping\n",
			 chip->cl_max_cap_dec);
		chip->cl_max_cap_dec = 1000;
	}
	if (chip->cl_max_cap_inc > 1000) {
		dev_warn(chip->dev,
			 "cl-max-increment %d exceeds 1000%%, clamping\n",
			 chip->cl_max_cap_inc);
		chip->cl_max_cap_inc = 1000;
	}

	dev_dbg(chip->dev,
		"Capacity learning initialized: nom=%lld uAh, learned=%lld uAh\n",
		chip->nom_cap_uah, chip->learned_cap_uah);
	return 0;
}

static int qcom_fg_probe(struct platform_device *pdev)
{
	struct power_supply_config supply_config = {};
	struct qcom_fg_chip *chip;
	const __be32 *prop_addr;
	int irq;
	u8 dma_status;
	bool error_present;
	int ret;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &pdev->dev;

	init_completion(&chip->mem_attn_done);
	mutex_init(&chip->dma_lock);
	mutex_init(&chip->cl_lock);
	mutex_init(&chip->state_lock);

	chip->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!chip->regmap) {
		dev_err(chip->dev, "Failed to locate the regmap\n");
		return -ENODEV;
	}

	prop_addr = of_get_address(pdev->dev.of_node, 0, NULL, NULL);
	if (!prop_addr) {
		dev_err(chip->dev, "Failed to read SOC base address from dt\n");
		return -EINVAL;
	}
	chip->base = be32_to_cpu(*prop_addr);

	ret = qcom_fg_read(chip, &dma_status, MEM_IF_DMA_STS, 1);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read dma_status: %d\n", ret);
		return ret;
	}

	error_present = dma_status & (BIT(1) | BIT(2));
	ret = qcom_fg_masked_write(chip, MEM_IF_DMA_CTL, BIT(0),
				   error_present ? BIT(0) : 0);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to write dma_ctl: %d\n", ret);
		return ret;
	}

	ret = qcom_fg_dma_init(chip);
	if (ret) {
		dev_err(chip->dev, "DMA init failed: %d\n", ret);
		return ret;
	}

	supply_config.drv_data = chip;

	chip->batt_psy = devm_power_supply_register(chip->dev, &batt_psy_desc,
						    &supply_config);
	if (IS_ERR(chip->batt_psy)) {
		if (PTR_ERR(chip->batt_psy) != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Failed to register battery\n");
		return PTR_ERR(chip->batt_psy);
	}

	platform_set_drvdata(pdev, chip);

	ret = power_supply_get_battery_info(chip->batt_psy, &chip->batt_info);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get battery info: %d\n", ret);
		return ret;
	}
	/* From this point, error paths must call power_supply_put_battery_info() */

	ret = qcom_fg_load_profile(chip);
	if (ret)
		dev_warn(chip->dev, "Failed to load battery profile: %d\n",
			 ret);

	ret = qcom_fg_configure_params(chip);
	if (ret)
		dev_warn(chip->dev, "Failed to configure FG parameters: %d\n",
			 ret);

	ret = qcom_fg_init_capacity_learning(chip, pdev->dev.of_node);
	if (ret)
		goto err_put_info;

	irq = platform_get_irq_byname(pdev, "soc-delta");
	if (irq < 0) {
		dev_err(&pdev->dev, "Failed to get soc-delta IRQ: %d\n", irq);
		ret = irq;
		goto err_put_info;
	}

	ret = devm_request_threaded_irq(chip->dev, irq, NULL,
					qcom_fg_handle_soc_delta, IRQF_ONESHOT,
					"soc-delta", chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to request soc-delta IRQ: %d\n",
			ret);
		goto err_put_info;
	}

	irq = platform_get_irq_byname(pdev, "mem-attn");
	if (irq <= 0) {
		dev_err(&pdev->dev, "Failed to get mem-attn IRQ: %d\n", irq);
		ret = irq ?: -ENXIO;
		goto err_put_info;
	}

	ret = devm_request_threaded_irq(chip->dev, irq, NULL,
					qcom_fg_mem_attn_irq_handler,
					IRQF_ONESHOT, "mem-attn", chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to request mem-attn IRQ: %d\n",
			ret);
		goto err_put_info;
	}

	chip->chg_psy = power_supply_get_by_reference(dev_fwnode(chip->dev),
						      "power-supplies");
	if (chip->chg_psy == ERR_PTR(-EPROBE_DEFER)) {
		dev_dbg(chip->dev,
			"Charger supply not ready, deferring probe\n");
		ret = -EPROBE_DEFER;
		goto err_put_info;
	}
	if (IS_ERR(chip->chg_psy)) {
		ret = PTR_ERR(chip->chg_psy);
		dev_err(chip->dev, "Failed to get charger supply: %d\n", ret);
		chip->chg_psy = NULL;
		goto err_put_info;
	}
	if (!chip->chg_psy) {
		dev_err(chip->dev, "Charger supply not found\n");
		ret = -ENODEV;
		goto err_put_info;
	}

	chip->nb.notifier_call = qcom_fg_notifier_call;
	ret = power_supply_reg_notifier(&chip->nb);
	if (ret) {
		dev_err(chip->dev, "Failed to register notifier: %d\n", ret);
		goto err_put_chg_psy;
	}

	qcom_fg_status_changed(chip);
	qcom_fg_read_state(chip);

	return 0;

err_put_chg_psy:
	power_supply_put(chip->chg_psy);
err_put_info:
	power_supply_put_battery_info(chip->batt_psy, chip->batt_info);
	return ret;
}

static int __maybe_unused qcom_fg_resume(struct device *dev)
{
	struct qcom_fg_chip *chip = dev_get_drvdata(dev);

	qcom_fg_read_state(chip);

	qcom_fg_cap_learning_update(chip);

	power_supply_changed(chip->batt_psy);

	return 0;
}

static SIMPLE_DEV_PM_OPS(qcom_fg_pm_ops, NULL, qcom_fg_resume);

static void qcom_fg_remove(struct platform_device *pdev)
{
	struct qcom_fg_chip *chip = platform_get_drvdata(pdev);

	power_supply_unreg_notifier(&chip->nb);
	power_supply_put(chip->chg_psy);
	power_supply_put_battery_info(chip->batt_psy, chip->batt_info);
}

static const struct of_device_id fg_match_id_table[] = {
	{ .compatible = "qcom,pm8150b-fg" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fg_match_id_table);

static struct platform_driver qcom_fg_driver = {
	.probe = qcom_fg_probe,
	.remove = qcom_fg_remove,
	.driver = {
		.name = "qcom-fg",
		.of_match_table = fg_match_id_table,
		.pm = &qcom_fg_pm_ops,
	},
};

module_platform_driver(qcom_fg_driver);

MODULE_AUTHOR("Caleb Connolly <caleb@connolly.tech>");
MODULE_AUTHOR("Joel Selvaraj <jo@jsfamily.in>");
MODULE_AUTHOR("Yassine Oudjana <y.oudjana@protonmail.com>");
MODULE_AUTHOR("TwinbornPlate75 <3342733415@qq.com>");
MODULE_DESCRIPTION("Qualcomm PM8150B Fuel Gauge Driver");
MODULE_LICENSE("GPL v2");

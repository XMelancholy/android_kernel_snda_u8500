/*
 * Copyright (C) ST-Ericsson SA 2012
 *
 * ST-Ericsson HVA core
 * Author: <thomas.costis@stericsson.com> for ST-Ericsson.
 *
 * License terms: GNU General Public License (GPL), version 2.
 */

/* power and clock management */
#include <linux/io.h>
#include <linux/regulator/driver.h>
#include <linux/clk.h>
#include <linux/mfd/dbx500-prcmu.h>
#include <linux/pm_runtime.h>

#include <video/hva_regs.h>
#include "hva_internals.h"

#define HVA_ID_MASK                 0xFFFF
#define HVA_HIF_REG_HW_RST_BIT      0x2
#define HVA_HIF_REG_SW_RST_BIT      0x1
#define HVA_HIF_REG_RST_ACK_MASK    0x1

/* bit[2] -> HJE_CLK_EN, bit[1] -> HEC_CLK_EN, bit[0] -> HVC_CLK_EN */
#define HVA_HIF_REG_CLK_GATING_VAL  0x3U
#define HVA_HIF_REG_HEC_MIF_CFG_VAL 0xC4U
/* HVA_HIF_REG_MIF_CFG_VAL=(lmi_opcode_message<<16) | lmi_opcode_message */
#define HVA_HIF_REG_MIF_CFG_VAL     0x4460446U
#define HVA_HIF_REG_BSM_9540        0x3F000U
#define HVA_HIF_REG_BSM_8540        0x3F0000U
#define HVA_HIF_REG_BSM_8680        0x1U

void hva_opp_on(struct hva_core *core, struct file *filp)
{
	char opp_dev_name[64];
	int ape_opp = 50;
	int ddr_opp = 25;

	if (core->instance_nb > 1) {
		ape_opp = 100;
		ddr_opp = 100;
	}

	snprintf(opp_dev_name, sizeof(opp_dev_name), HVA_DEVNAME "_%p_", filp);
	prcmu_qos_add_requirement(PRCMU_QOS_APE_OPP, opp_dev_name, ape_opp);
	prcmu_qos_add_requirement(PRCMU_QOS_DDR_OPP, opp_dev_name, ddr_opp);
	dev_dbg(core->dev, "Add opp requirement for %s\n", opp_dev_name);
}

void hva_opp_off(struct hva_core *core, struct file *filp)
{
	char opp_dev_name[64];
	snprintf(opp_dev_name, sizeof(opp_dev_name), HVA_DEVNAME "_%p_", filp);
	prcmu_qos_remove_requirement(PRCMU_QOS_APE_OPP, opp_dev_name);
	prcmu_qos_remove_requirement(PRCMU_QOS_DDR_OPP, opp_dev_name);
	dev_dbg(core->dev, "Remove opp requirement for %s\n", opp_dev_name);
}

void hva_clock_on(struct hva_core *core)
{
	dev_dbg(core->dev, "Request clock ON\n");
	clk_enable(core->clk);
}

void hva_clock_off(struct hva_core *core)
{
	dev_dbg(core->dev, "Request clock OFF\n");
	clk_disable(core->clk);
}

int hva_get_reg(struct hva_core *core, struct hva_reg *reg)
{
	unsigned long flags;

	dev_dbg(core->dev, "%s\n", __func__);

	if (pm_runtime_get_sync(core->dev) < 0) {
		dev_err(core->dev, "%s regulator_enable failed\n",
				__func__);
		return -EFAULT;
	}
	hva_clock_on(core);

	spin_lock_irqsave(&core->hw_lock, flags);
	reg->val = readl(core->hw_base_addr + reg->offset);
	spin_unlock_irqrestore(&core->hw_lock, flags);

	hva_clock_off(core);
	pm_runtime_put_autosuspend(core->dev);

	return 0;
}

#define RESET_TIMEOUT 100
/*
 * Should NOT sleep (called from pm_runtime framework)
 */
int hva_hw_reset(struct hva_core *core)
{
	int ret = 0;
	int reset_retries = 0;
	__u32 val = 0;

	if (pm_runtime_get_sync(core->dev) < 0) {
		dev_err(core->dev, "%s regulator_enable failed\n",
				__func__);
		return -EFAULT;
	}
	hva_clock_on(core);

	if (core->hw_state == HVA_STUCKED) {
		dev_dbg(core->dev, "%s: HVA hard-reset\n", __func__);
		writel(HVA_HIF_REG_HW_RST_BIT,
		       core->hw_base_addr + HVA_HIF_REG_RST);
		dev_dbg(core->dev, "%s: HVA hard-reset done\n", __func__);
		core->hw_state = HVA_OK;
	}

	writel(HVA_HIF_REG_CLK_GATING_VAL,
			core->hw_base_addr + HVA_HIF_REG_CLK_GATING);

	dev_dbg(core->dev, "%s: HVA soft-reset\n", __func__);
	writel(HVA_HIF_REG_SW_RST_BIT, core->hw_base_addr + HVA_HIF_REG_RST);
	do {
		if (reset_retries++ >= RESET_TIMEOUT) {
			dev_info(core->dev,
				 "%s Unable to soft-reset HW\n", __func__);
			ret = -EFAULT;
			break;
		}
		val = readl(core->hw_base_addr + HVA_HIF_REG_RST_ACK);
	} while ((val & HVA_HIF_REG_RST_ACK_MASK) == 0);
	dev_dbg(core->dev, "%s: HVA soft-reset done\n", __func__);

	hva_clock_off(core);
	pm_runtime_put_autosuspend(core->dev);

	return ret;
}

/*
 * Write HVA configuration and send task to HW
 * Define Max Opcode Size and Max Message Size for LMI and EMI
 * and Byte swapping
 */
void hva_trigger_task(struct hva_core *core, struct hva_task *task)
{
	bool update_BSM = true;
	__u32 expected_BSM = HVA_HIF_REG_BSM_9540;

	unsigned long flags;
	__u32 cmd;

	switch (core->chip_id) {
	case HVA_VERSION_MPE41:
	case HVA_VERSION_MPE42:
	case HVA_VERSION_9540:
	case HVA_VERSION_SVP_9540:
		expected_BSM = HVA_HIF_REG_BSM_9540;
		break;
	case HVA_VERSION_9600:
	case HVA_VERSION_SVP_9600:
		expected_BSM = HVA_HIF_REG_BSM_8680;
		break;
	case HVA_VERSION_8540:
	case HVA_VERSION_8540V2:
	case HVA_VERSION_SVP_8540V2:
	case HVA_VERSION_SVP_8540:
		expected_BSM = HVA_HIF_REG_BSM_8540;
		break;
	case HVA_VERSION_5500V2:
	default:
		update_BSM = false;
		dev_dbg(core->dev, "Chip version unknown, no update BSM register\n");
		break;
	}

	dev_dbg(core->dev, "%s: Write configuration registers", __func__);

	spin_lock_irqsave(&core->hw_lock, flags);
	/* Byte swap config */
	if (update_BSM)
		writel(expected_BSM, core->hw_base_addr + HVA_HIF_REG_BSM);
	/*
	 * Define Max Opcode Size and Max Message Size
	 * for LMI and EMI
	 */
	writel(HVA_HIF_REG_MIF_CFG_VAL,
			core->hw_base_addr + HVA_HIF_REG_MIF_CFG);
	writel(HVA_HIF_REG_HEC_MIF_CFG_VAL,
			core->hw_base_addr + HVA_HIF_REG_HEC_MIF_CFG);
	writel(HVA_HIF_REG_CLK_GATING_VAL,
			core->hw_base_addr + HVA_HIF_REG_CLK_GATING);

	dev_dbg(core->dev, "%s: Send task %d (client:%d, cmd:%d, task_desc:0x%x)\n",
			__func__, task->id, task->client_id,
			task->cmd, task->task_desc);
	cmd = (__u32) (task->cmd & 0xFF);
	cmd |= ((__u32) (task->client_id & 0xFF) << 8);
	cmd |= (__u32) (task->id) << 16;
	writel(cmd, core->hw_base_addr + HVA_HIF_FIFO_CMD);
	writel(task->task_desc, core->hw_base_addr + HVA_HIF_FIFO_CMD);
	spin_unlock_irqrestore(&core->hw_lock, flags);
}

/*
 * core->irq_its_handler() - hva interrupt handler
 */
irqreturn_t hva_irq_its_handler(int irq, void *data)
{
	struct hva_core *core = (struct hva_core *)data;
	__u32 sts = 0;
	unsigned long flags;

	/*
	 * spinlock mandatory after tests campaign.
	 * Avoid concurrency between trigger_task registers
	 * write sequence and SFL/STS access when in it
	 * Problem still to be confirmed by Hardware team
	 */
	spin_lock_irqsave(&core->hw_lock, flags);
	if (0 < (readl(core->hw_base_addr + HVA_HIF_REG_SFL) & 0xF)) {
		sts = readl(core->hw_base_addr + HVA_HIF_FIFO_STS);
		writel(0x1, core->hw_base_addr + HVA_HIF_REG_IT_ACK);
		core->hva_hw_cb(core, sts);
	} else
		dev_err(core->dev, "%s Read status failed\n", __func__);
	spin_unlock_irqrestore(&core->hw_lock, flags);

	return IRQ_HANDLED;
}

/*
 * core->irq_err_handler() - hva error interrupt handler
 */
irqreturn_t hva_irq_err_handler(int irq, void *data)
{
	struct hva_core *core = (struct hva_core *)data;
	__u32 val;
	unsigned long flags;

	dev_err(core->dev, "%s Received error IRQ\n", __func__);

	spin_lock_irqsave(&core->hw_lock, flags);

	val = readl(core->hw_base_addr + HVA_HIF_REG_HEC_MIF_ERR);
	if (val & 0x3) {
		dev_err(core->dev,
			"%s: HVA_HIF_REG_HEC_MIF_ERR returns: 0x%x\n",
			__func__, val);
	}
	val = readl(core->hw_base_addr + HVA_HIF_REG_EMI_ERR);
	if (val & 0x3F) {
		dev_err(core->dev,
			"%s: HVA_HIF_REG_EMI_ERR returns: 0x%x\n",
			__func__, val);
	}
	val = readl(core->hw_base_addr + HVA_HIF_REG_LMI_ERR);
	if (val & 0x3F) {
		dev_err(core->dev,
			"%s: HVA_HIF_REG_LMI_ERR returns: 0x%x\n",
			__func__, val);
	}
	val = readl(core->hw_base_addr + HVA_HIF_REG_SFL);
	dev_info(core->dev,
		"%s: Fifo status level: %d\n",
		__func__, val);

	core->hva_hw_err_cb(core);

	/* ACK resets error msg */
	writel(0x1, core->hw_base_addr + HVA_HIF_REG_ERR_IT_ACK);

	spin_unlock_irqrestore(&core->hw_lock, flags);

	return IRQ_HANDLED;
}

int hva_check_task_errors(struct hva_instance *instance,
		struct hva_task *task, unsigned int *status, unsigned int *info)
{
	struct hva_core *core = (struct hva_core *)instance->data;
	unsigned long flags;
	int ret = 0;
	__u32 val;

	if (core->hw_state == HVA_STUCKED) {
		*status = HVA_READ_ERR;
		return -ETIMEDOUT;
	}

	BUG_ON(task->id < 0 || task->id >= HW_HVA_FIFO_SIZE);

	*status = HVA_ERROR_NONE;

	if (core->hw_state == HVA_RET_ERROR) {
		spin_lock_irqsave(&core->hw_lock, flags);
		val =
			readl(core->hw_base_addr + HVA_HIF_REG_HEC_MIF_ERR);
		if (val & 0x3) {
			*status |= HVA_IT_HIF_HEC_MIF_ERROR;
			dev_err(core->dev,
					"%s: HVA_HIF_REG_HEC_MIF_ERR returns: 0x%x\n",
					__func__, val);
		}
		val = readl(core->hw_base_addr + HVA_HIF_REG_EMI_ERR);
		if (val & 0x3F) {
			*status |= HVA_IT_HIF_EMI_ERROR;
			dev_err(core->dev,
					"%s: HVA_HIF_REG_EMI_ERR returns: 0x%x\n",
					__func__, val);
		}
		val = readl(core->hw_base_addr + HVA_HIF_REG_LMI_ERR);
		if (val & 0x3F) {
			*status |= HVA_IT_HIF_LMI_ERROR;
			dev_err(core->dev,
					"%s: HVA_HIF_REG_LMI_ERR returns: 0x%x\n",
					__func__, val);
		}
		val = readl(core->hw_base_addr + HVA_HIF_REG_SFL);
		spin_unlock_irqrestore(&core->hw_lock, flags);
		if ((val & 0xF) == HW_HVA_FIFO_SIZE) {
			*status |= HVA_IT_FIFO_STS_FULL;
			dev_info(core->dev,
					"%s: Fifo status level: %d\n",
					__func__, val);
		}

		ret = -EFAULT;
	} else
		core->hw_state = HVA_OK;

	*info = task->status_reg & 0xFF;

	return ret;
}

/*
 * check hardware ID
 * return the ID if the value is correct
 * else return 0
 */
unsigned long int hva_check_hardware_id(struct hva_core *core)
{
	unsigned long int id;
	unsigned long ret = 0;
	unsigned long flags;

	/* enable power and clock before read register ID */
	if (pm_runtime_get_sync(core->dev) < 0) {
		dev_err(core->dev, "%s pm_runtime_get_sync failed\n",
				__func__);
		return 0;
	}
	hva_clock_on(core);

	spin_lock_irqsave(&core->hw_lock, flags);
	id = readl(core->hw_base_addr + HVA_HIF_REG_VERSION) & HVA_ID_MASK;
	spin_unlock_irqrestore(&core->hw_lock, flags);
	/* disable clock and power before leave */
	hva_clock_off(core);
	pm_runtime_put_autosuspend(core->dev);

	dev_dbg(core->dev, "ID register 0x%lx\n", id);

	switch (id) {
	case HVA_VERSION_5500V2:
	case HVA_VERSION_MPE41:
	case HVA_VERSION_MPE42:
	case HVA_VERSION_9540:
	case HVA_VERSION_8540:
	case HVA_VERSION_8540V2:
	case HVA_VERSION_9600:
	case HVA_VERSION_SVP_9540:
	case HVA_VERSION_SVP_8540:
	case HVA_VERSION_SVP_8540V2:
	case HVA_VERSION_SVP_9600:
		ret = id;
		break;
	default:
		dev_err(core->dev, "Unknown hardware ID 0x%lx\n", id);
		break;
	}

	return ret;
}

/*
 * Copyright (C) ST-Ericsson SA 2010
 * Author: Pankaj Chauhan <pankaj.chauhan@stericsson.com> for ST-Ericsson.
 * License terms: GNU General Public License (GPL), version 2.
 */
#include <linux/delay.h>
#include <linux/init.h>		/* Initiliasation support */
#include <linux/module.h>	/* Module support */
#include <linux/kernel.h>	/* Kernel support */
#include <linux/version.h>	/* Kernel version */
#include <linux/fs.h>		/* File operations (fops) defines */
#include <linux/errno.h>	/* Defines standard err codes */
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/mfd/dbx500-prcmu.h>
#include <linux/mmio.h>


#define ISP_REGION_IO				(0xE0000000)
#define SIA_ISP_REG_ADDR			(0x521E4)
#define SIA_BASE_ADDR				(0x54000)
#define SIA_ISP_MEM				(0x56000)
#define SIA_TIMER_ITC				(0x5BC00)
#define SIA_ISP_MCU_SYS_SIZE			(0x100000)
#define SIA_ISP_MEM_PAGE_REG			(0x54070)
#define SIA_ISP_MCU_SYS_ADDR0_OFFSET	(SIA_BASE_ADDR + 0x40)
#define SIA_ISP_MCU_SYS_SIZE0_OFFSET	(SIA_BASE_ADDR + 0x42)
#define SIA_ISP_MCU_SYS_ADDR1_OFFSET	(SIA_ISP_MCU_SYS_ADDR0_OFFSET + 0x04)
#define SIA_ISP_MCU_SYS_SIZE1_OFFSET	(SIA_ISP_MCU_SYS_SIZE0_OFFSET + 0x04)
#define SIA_ISP_MCU_IO_ADDR0_HI		(SIA_BASE_ADDR + 0x60)

/* HTimer enable in CR register */
#define CR_REG0_HTIMEN				(1 << 26)
#define PICTOR_IN_XP70_L2_MEM_BASE_ADDR		(0x40000)
#define PICTOR_IN_XP70_TCDM_MEM_BASE_ADDR	(0x60000)
#define L2_PSRAM_MEM_SIZE			(0x10000)

#define FW_TO_HOST_ADDR_MASK		(0x00001FFF)
#define FW_TO_HOST_ADDR_SHIFT		(0xD)
#define FW_TO_HOST_CLR_MASK		(0x3F)
#define PHY_TO_ISP_MCU_IO_ADDR0_HI(x)	(((x) >> 24) << 8)
#define XP70_ADDR_MASK			(0x00FFFFFF)

#define CLOCK_ENABLE_DELAY		(0x2)

#define MAX_PRCMU_QOS_APP		(0x64)

#define ISP_WRITE_DATA_SIZE		(0x4)

#define clrbits32(_addr, _clear) \
	writel(readl(_addr) & ~(u32)(_clear), _addr)
#define setbits32(_addr, _set) \
	writel(readl(_addr) | (u32)(_set), _addr)


#define upper_16_bits(n) ((u16)((u32)(n) >> 16))


struct mmio_info {
	struct mmio_platform_data *pdata; /* Config from board */
	struct device *dev; /* My device */
	/* Runtime variables */
	struct miscdevice misc_dev;
	void __iomem *siabase;
	void __iomem *crbase;
	/* States */
	int xshutdown_enabled;
	int xshutdown_is_active_high;
};

/*
 * The one and only private data holder. Default inited to NULL.
 * Declare it here so no code above can use it directly.
 */
static struct mmio_info *info;

/*
 * static 1K buffer to do I/O write instead of kmalloc,
 * no locking, caller can not have parallel use of
 * MMIO_CAM_LOAD_XP70_FW and MMIO_CAM_ISP_WRITE ioctl's
 */
static u16 copybuff[512];

/*
 * This function converts a given logical memory region size
 * to appropriate ISP_MCU_SYS_SIZEx register value.
 */
static int get_mcu_sys_size(u32 size, u32 *val)
{
	int ret = 0;

	if (size > 0 && size <= SZ_4K)
		*val = 4;
	else if (size > SZ_4K && size <= SZ_8K)
		*val = 5;
	else if (size > SZ_8K && size <= SZ_16K)
		*val = 6;
	else if (size > SZ_16K && size <= SZ_32K)
		*val = 7;
	else if (size > SZ_32K && size <= SZ_64K)
		*val = 0;
	else if (size > SZ_64K && size <= SZ_1M)
		*val = 1;
	else if (size > SZ_1M  && size <= SZ_16M)
		*val = 2;
	else if (size > SZ_16M && size <= SZ_256M)
		*val = 3;
	else
		ret = -EINVAL;

	return ret;
}

static int mmio_cam_pwr_sensor(struct mmio_info *info, int on)
{
	int err = 0;
	
	if (on) {
		err = info->pdata->power_enable(info->pdata);

		if (err)
			dev_err(info->dev,
				"power_enable failed. err = %d\n", err);

		/*
		 * When switching from secondary YUV camera
		 * to primary Raw Bayer Camera, a hang is observed without the
		 * below delay. I2C access failure are observed while
		 * communicating with primary camera sensor indicating camera
		 * sensor was not powered up correctly.
		 */
		mdelay(CLOCK_ENABLE_DELAY);
	} else {
		info->pdata->power_disable(info->pdata);
	}

	return err;
}

static int mmio_cam_control_clocks(struct mmio_info *info,
				   enum mmio_bool_t power_on)
{
	int err = 0;

	if (power_on) {
		err = info->pdata->clock_enable(info->pdata);

		if (err)
			dev_err(info->dev,
				"clock_enable failed, err = %d\n",
				err);

                printk("YJYING debug\n");
		udelay(100);
		gpio_set_value(142,1);
		udelay(100);
		gpio_set_value(142,0);
		udelay(100);
		gpio_set_value(142,1);
		udelay(100);
		if(info->pdata->camera_slot ==0)
		{
			printk("YJYING debug master poweron\n");
			gpio_set_value(141,0);
			gpio_set_value(140,1);
		}
		else
		{
			printk("YJYING debug slave power on\n");
			gpio_set_value(141,1);
			gpio_set_value(140,0);
		}

	} else {
              if(info->pdata->camera_slot ==0)
		{
			printk("YJYING all powerdown\n");
			gpio_set_value(141,1);
			gpio_set_value(140,1);
		}
		else
		{
			printk("YJYING all powerdown\n");
			gpio_set_value(141,1);
			gpio_set_value(140,1);
		}

		udelay(500);
		info->pdata->clock_disable(info->pdata);
	}

	return err;
}

static int mmio_cam_set_pri_hwif(struct mmio_info *info)
{
	if (info->xshutdown_enabled)
		info->pdata->set_xshutdown(info->pdata);

	return 0;
}

static int mmio_cam_set_sec_hwif(struct mmio_info *info)
{
	if (info->xshutdown_enabled)
		info->pdata->set_xshutdown(info->pdata);

	return 0;
}

static int mmio_cam_init_mmdsp_timer(struct mmio_info *info)
{
	/* Disabling Accelerators timers */
	clrbits32(info->crbase, CR_REG0_HTIMEN);
	/* Write MMDSPTimer */
	writel(0, info->siabase + SIA_TIMER_ITC);
	/* Enabling Accelerators timers */
	setbits32(info->crbase, CR_REG0_HTIMEN);
	return 0;
}

static u32 t1_to_arm(u32 t1_addr, void __iomem *smia_base_address,
		     u16 *p_mem_page)
{
	u16 mem_page_update = 0;
	mem_page_update = (t1_addr >> FW_TO_HOST_ADDR_SHIFT) &
			  FW_TO_HOST_CLR_MASK;

	if (mem_page_update != *p_mem_page) {
		/* Update sia_mem_page register */
		dev_dbg(info->dev, "mem_page_update=0x%x, mem_page=0x%x\n",
			mem_page_update, *p_mem_page);
		writew(mem_page_update, smia_base_address +
		       SIA_ISP_MEM_PAGE_REG);
		*p_mem_page = mem_page_update;
	}

	return SIA_ISP_MEM + (t1_addr & FW_TO_HOST_ADDR_MASK);
}

static int write_user_buffer(struct mmio_info *info, u32 ioaddr,
					void __iomem *src_buf, u32 size)
{
	u32 i, count, offset = 0;
	u32 itval = 0;
	u16 mem_page = 0;
	int err = 0;

	if (!src_buf || !ioaddr) {
		dev_err(info->dev, "invalid parameters: %p, 0x%x",
				src_buf, ioaddr);

		return -EINVAL;
	}

	for (offset = 0; offset < size; ) {

		if ((size - offset) > sizeof(copybuff))
			count = sizeof(copybuff);
		else
			count = (size - offset);

		if (copy_from_user(copybuff, src_buf + offset, count)) {

			dev_err(info->dev, "failed to copy user buffer"
				" %p at offset=%d, count=%d\n",
				src_buf, offset, count);

			err = -EFAULT;
			goto cp_failed;
		}

		for (i = 0; i < count; ) {
			itval = t1_to_arm(ioaddr + offset,
					info->siabase, &mem_page);
			itval = ((u32) info->siabase) + itval;

			writew(copybuff[i/2], itval);
			offset += 2;
			i = i + 2;
		}
	}

cp_failed:
	return err;
}

static int mmio_load_xp70_fw(struct mmio_info *info,
			     struct xp70_fw_t *xp70_fw)
{
	u32 itval = 0;
	int err = 0;

	if (xp70_fw->size_split != 0) {
		/* if buff size is not as expected */
		if (xp70_fw->size_split != L2_PSRAM_MEM_SIZE) {
			dev_err(info->dev, "xp70_fw_t.size_split must be "
				"%d bytes!\n", L2_PSRAM_MEM_SIZE);
			err = -EINVAL;
			goto err_exit;
		}

		writel(0x0, info->siabase + SIA_ISP_REG_ADDR);

		/* Put the low 64k IRP firmware in ISP MCU L2 PSRAM */
		err = write_user_buffer(info, PICTOR_IN_XP70_L2_MEM_BASE_ADDR,
					xp70_fw->addr_split,
					L2_PSRAM_MEM_SIZE);
		if (err)
			goto err_exit;
	}

	if (xp70_fw->size_data != 0) {

		writel(0x0, info->siabase + SIA_ISP_REG_ADDR);

		err = write_user_buffer(info, PICTOR_IN_XP70_TCDM_MEM_BASE_ADDR,
					xp70_fw->addr_data,
					xp70_fw->size_data);

		if (err)
			goto err_exit;
	}

	if (xp70_fw->size_esram_ext != 0) {
		/*
		 * ISP_MCU_SYS_ADDRx XP70 register (@ of ESRAM where the
		 * external code has been loaded
		 */
		writew(upper_16_bits(xp70_fw->addr_esram_ext),
		       info->siabase + SIA_ISP_MCU_SYS_ADDR0_OFFSET);
		/* ISP_MCU_SYS_SIZEx XP70 register (size of the code =64KB) */
		writew(0x0, info->siabase + SIA_ISP_MCU_SYS_SIZE0_OFFSET);
	}

	if (xp70_fw->size_sdram_ext != 0) {
		/*
		 * ISP_MCU_SYS_ADDRx XP70 register (@ of SDRAM where the
		 * external code has been loaded
		 */
		writew(upper_16_bits(xp70_fw->addr_sdram_ext),
		       info->siabase + SIA_ISP_MCU_SYS_ADDR1_OFFSET);
		/* ISP_MCU_SYS_SIZEx XP70 register */
		err = get_mcu_sys_size(xp70_fw->size_sdram_ext, &itval);

		if (err)
			goto err_exit;

		writew(itval, info->siabase + SIA_ISP_MCU_SYS_SIZE1_OFFSET);
	}

	return 0;
err_exit:
	dev_err(info->dev, "Loading XP70 fw failed\n");
	return -EFAULT;
}

static int mmio_map_statistics_mem_area(struct mmio_info *info,
					void __iomem *addr_to_map)
{
	u16 value;
	BUG_ON(addr_to_map == NULL);
	/* 16 Mbyte aligned page */
	value = PHY_TO_ISP_MCU_IO_ADDR0_HI(*((u32 *)addr_to_map));
	writew(value, info->siabase + SIA_ISP_MCU_IO_ADDR0_HI);
	/* Return the address in the XP70 address space */
	*((u32 *)addr_to_map) = (*((u32 *)addr_to_map) & XP70_ADDR_MASK) |
				ISP_REGION_IO;
	return 0;
}

static int mmio_activate_i2c2(struct mmio_info *info, unsigned long enable)
{
	int err = 0;

	switch (enable) {
	case MMIO_ACTIVATE_I2C_HOST:
		/* Select I2C-2 */
		err = info->pdata->config_i2c_pins(info->pdata,
						   MMIO_ACTIVATE_I2C_HOST);

		if (err) {
			dev_err(info->dev, "Failed to Enable I2C-2, err %d\n",
				err);
			goto out;
		}

		break;
	case MMIO_ACTIVATE_IPI2C2:
		/* Select IPI2C */
		err = info->pdata->config_i2c_pins(info->pdata,
						   MMIO_ACTIVATE_IPI2C2);

		if (err) {
			dev_err(info->dev, "Failed to Enable IPI2C, err %d\n",
				err);
			goto out;
		}

		break;
	case MMIO_DEACTIVATE_I2C: {
		info->pdata->config_i2c_pins(info->pdata, MMIO_DEACTIVATE_I2C);
	}
	break;
	default:
		dev_warn(info->dev, "Invalid I2C2 config\n");
		err = -EINVAL;
		break;
	}

out:
	return err;
}

static int mmio_enable_xshutdown_from_host(struct mmio_info *info,
		unsigned long enable)
{
	int err = 0;
	info->xshutdown_is_active_high = enable & MMIO_XSHUTDOWN_ACTIVE_HIGH;

	if (enable & MMIO_XSHUTDOWN_ENABLE) {
		err = info->pdata->config_xshutdown_pins(info->pdata,
				MMIO_ENABLE_XSHUTDOWN_HOST, enable &
				MMIO_XSHUTDOWN_ACTIVE_HIGH);
	} else {
		info->pdata->config_xshutdown_pins(info->pdata,
						MMIO_ENABLE_XSHUTDOWN_FW, -1);
		/*
		 * XShutdown is controlled by firmware, initial output value is
		 * provided by firmware
		 */
	}

	info->xshutdown_enabled = enable & MMIO_XSHUTDOWN_ENABLE;
	return 0;
}

static int mmio_cam_initboard(struct mmio_info *info)
{
	int err = 0;

	err = prcmu_qos_add_requirement(PRCMU_QOS_APE_OPP, MMIO_NAME,
					MAX_PRCMU_QOS_APP);

	if (err) {
		dev_err(info->dev, "Error adding PRCMU QoS requirement %d\n",
			err);
		goto out_ape;
	}

	err = prcmu_qos_add_requirement(PRCMU_QOS_DDR_OPP, MMIO_NAME,
					PRCMU_QOS_DDR_OPP_MAX);

	if (err) {
		dev_err(info->dev, "Error adding PRCMU QoS requirement %d\n",
			err);
		goto out_ddr;
	}

	/* Initialize platform specific data */
	err = info->pdata->platform_init(info->pdata);

	if (err) {
		dev_err(info->dev,
			"Failed to execute platform init: %d\n",
			err);
		goto out_plat_init;
	}


	/* Configure xshutdown to be disabled by default */
	err = mmio_enable_xshutdown_from_host(info, 0);

	if (err) {
		dev_err(info->dev,
			"Failed to disable xshutdown: %d\n",
			err);
		goto out_xshutdown;
	}

	/* Enable IPI2C */
	err = mmio_activate_i2c2(info, MMIO_ACTIVATE_IPI2C2);

	if (err) {
		dev_err(info->dev,
			"Failed to enable IPI2C: %d\n",
			err);
		goto out_i2c;
	}

	return 0;
out_i2c:
	info->pdata->config_xshutdown_pins(info->pdata, MMIO_DISABLE_XSHUTDOWN,
		-1);
out_xshutdown:
	info->pdata->platform_exit(info->pdata);
out_plat_init:
	prcmu_qos_remove_requirement(PRCMU_QOS_DDR_OPP, MMIO_NAME);
out_ddr:
	prcmu_qos_remove_requirement(PRCMU_QOS_APE_OPP, MMIO_NAME);
out_ape:
	return err;
}

static int mmio_cam_desinitboard(struct mmio_info *info)
{
	mmio_activate_i2c2(info, MMIO_DEACTIVATE_I2C);

	info->pdata->config_xshutdown_pins(info->pdata, MMIO_DISABLE_XSHUTDOWN,
			-1);

	info->pdata->platform_exit(info->pdata);

	prcmu_qos_remove_requirement(PRCMU_QOS_APE_OPP, MMIO_NAME);
	prcmu_qos_remove_requirement(PRCMU_QOS_DDR_OPP, MMIO_NAME);
	return 0;
}

static int mmio_isp_write(struct mmio_info *info,
			  struct isp_write_t *isp_write_p)
{
	int err = 0, i;
	u32 __iomem *data = NULL;
	void __iomem *addr = NULL;
	u16 mem_page = 0;
	u32 size, count, offset;

	if (!isp_write_p->count) {
		dev_warn(info->dev, "no data to write to isp\n");
		return -EINVAL;
	}

	size = isp_write_p->count * ISP_WRITE_DATA_SIZE;
	data = (u32 *) copybuff;

	for (offset = 0; offset < size; ) {

		/* 'offset' and 'size' and 'count' is in bytes */
		if ((size - offset) > sizeof(copybuff))
			count = sizeof(copybuff);
		else
			count = (size - offset);

		if (copy_from_user(data, ((u8 *)isp_write_p->data) + offset,
		    count)) {
			dev_err(info->dev, "failed to copy user buffer"
				" %p at offset=%d, count=%d\n",
				isp_write_p->data, offset, count);

			err = -EFAULT;
			goto out;
		}

		/* index 'i' and 'offset' is in bytes */
		for (i = 0; i < count; ) {
			addr = (void *)(info->siabase + t1_to_arm(
					isp_write_p->t1_dest
					+ offset,
					info->siabase, &mem_page));

			*((u32 *)addr) = data[i/ISP_WRITE_DATA_SIZE];

			offset += ISP_WRITE_DATA_SIZE;
			i = i + ISP_WRITE_DATA_SIZE;
		}
	}

out:
	return err;
}


static int enable_flash_from_host(unsigned long enable)
{
	printk("  mmio_enable_flash_from_host: %ld\n",	  enable);

	switch(enable)
	{
	case 0://shutdown
		gpio_set_value(152,0);
		gpio_set_value(151,0);
		udelay(100);
		break;
	case 1://torch
		gpio_set_value(152,0);
		gpio_set_value(151,1);
		udelay(100);
		break;
	case 2://indicator
		gpio_set_value(152,1);
		gpio_set_value(151,0);
		udelay(100);
		break;
	case 3://flash
		/* flash light need continuse be on several frames to 
		   get camera sensor AE convenience before take picture,
		   since flash control chip timed out and auto off in 800ms,
		   so have to turn off flash and turn on again. */
		gpio_set_value(152,0);
		gpio_set_value(151,0);
		udelay(100);
		gpio_set_value(152,1);
		gpio_set_value(151,1);
		msleep(400);
		printk(KERN_INFO "	mmio_enable_flash_from_host: re-enable\n");
		gpio_set_value(152,0);
		gpio_set_value(151,0);
		udelay(100);
		gpio_set_value(152,1);
		gpio_set_value(151,1);
		msleep(400);
		printk(KERN_INFO "	mmio_enable_flash_from_host: re-enable\n");
		gpio_set_value(152,0);
		gpio_set_value(151,0);
		udelay(100);
		gpio_set_value(152,1);
		gpio_set_value(151,1);
		udelay(100);
		break;
	default:
		break;
	}

	return 0;
}


static long mmio_ioctl(struct file *filp, u32 cmd,
		      unsigned long arg)
{
	struct mmio_input_output_t data;
	int no_of_bytes;
	int enable;
	int ret = 0;
	struct mmio_info *info = (struct mmio_info *)filp->private_data;
	BUG_ON(info == NULL);

	switch (cmd) {
	case MMIO_CAM_INITBOARD:
		no_of_bytes = sizeof(struct mmio_input_output_t);
		memset(&data, 0, sizeof(struct mmio_input_output_t));

		if (copy_from_user(&data, (struct mmio_input_output_t *)arg,
				   no_of_bytes)) {
			dev_err(info->dev,
				"Copy from userspace failed\n");
			ret = -EFAULT;
			break;
		}

		info->pdata->camera_slot = data.mmio_arg.camera_slot;
		ret = mmio_cam_initboard(info);
		break;
	case MMIO_CAM_FLASH_ENABLE:
		{
			ret = enable_flash_from_host(arg);
			if(ret){
				printk("Unable to %s:  %s camera, flash mode\n",
					(arg ?"Enable":"Disable"));
			}
		}
		break;
	case MMIO_CAM_DESINITBOARD:
		ret = mmio_cam_desinitboard(info);
		info->pdata->camera_slot = -1;
		break;
	case MMIO_CAM_PWR_SENSOR:
		no_of_bytes = sizeof(struct mmio_input_output_t);
		memset(&data, 0, sizeof(struct mmio_input_output_t));

		if (copy_from_user
				(&data, (struct mmio_input_output_t *)arg,
				 no_of_bytes)) {
			dev_err(info->dev,
				"Copy from userspace failed\n");
			ret = -EFAULT;
			break;
		}

		ret = mmio_cam_pwr_sensor(info, data.mmio_arg.power_on);
		break;
	case MMIO_CAM_SET_EXT_CLK:
		no_of_bytes = sizeof(struct mmio_input_output_t);
                memset(&data, 0, sizeof(struct mmio_input_output_t));

		if (copy_from_user
				(&data, (struct mmio_input_output_t *)arg,
				 no_of_bytes)) {
			dev_err(info->dev,
				"Copy from userspace failed\n");
			ret = -EFAULT;
			break;
		}

		ret = mmio_cam_control_clocks(info, data.mmio_arg.power_on);
		break;
	case MMIO_CAM_LOAD_XP70_FW:
		no_of_bytes = sizeof(struct mmio_input_output_t);
		memset(&data, 0, sizeof(struct mmio_input_output_t));

		if (copy_from_user
				(&data, (struct mmio_input_output_t *)arg,
				 no_of_bytes)) {
			dev_err(info->dev,
				"Copy from userspace failed\n");
			ret = -EFAULT;
			break;
		}

		ret = mmio_load_xp70_fw(info, &data.mmio_arg.xp70_fw);
		break;
	case MMIO_CAM_MAP_STATS_AREA:
		no_of_bytes = sizeof(struct mmio_input_output_t);
		memset(&data, 0, sizeof(struct mmio_input_output_t));

		if (copy_from_user
				(&data, (struct mmio_input_output_t *)arg,
				 no_of_bytes)) {
			dev_err(info->dev,
				"Copy from userspace failed\n");
			ret = -EFAULT;
			break;
		}

		ret = mmio_map_statistics_mem_area(info,
						   &data.mmio_arg.addr_to_map);

		if (0 != ret) {
			dev_err(info->dev,
				"Unable to map Statistics Mem area\n");
			break;
		}

		if (copy_to_user((struct mmio_input_output_t *)arg,
				 &data, sizeof(no_of_bytes))) {
			dev_err(info->dev,
				"Copy to userspace failed\n");
			ret = -EFAULT;
			break;
		}

		break;
	case MMIO_CAM_SET_PRI_HWIF:
		ret = mmio_cam_set_pri_hwif(info);
		break;
	case MMIO_CAM_SET_SEC_HWIF:
		ret = mmio_cam_set_sec_hwif(info);
		break;
	case MMIO_CAM_INITMMDSPTIMER:
		ret = mmio_cam_init_mmdsp_timer(info);
		break;
	case MMIO_CAM_ISP_WRITE:
		no_of_bytes = sizeof(struct mmio_input_output_t);
		memset(&data, 0, sizeof(struct mmio_input_output_t));

		if (copy_from_user
				(&data, (struct mmio_input_output_t *)arg,
				 no_of_bytes)) {
			dev_err(info->dev,
				"Copy from userspace failed\n");
			ret = -EFAULT;
			break;
		}

		ret = mmio_isp_write(info, &data.mmio_arg.isp_write);
		break;
	case MMIO_ACTIVATE_I2C2:
		no_of_bytes = sizeof(struct mmio_input_output_t);
		memset(&data, 0, sizeof(struct mmio_input_output_t));

		if (copy_from_user
				(&enable, (int *)arg, sizeof(enable))) {
			dev_err(info->dev,
				"Copy from userspace failed\n");
			ret = -EFAULT;
			break;
		}

		ret = mmio_activate_i2c2(info, enable);
		break;
	case MMIO_ENABLE_XSHUTDOWN_FROM_HOST:
		no_of_bytes = sizeof(struct mmio_input_output_t);
		memset(&data, 0, sizeof(struct mmio_input_output_t));

		if (copy_from_user
				(&enable, (int *)arg, sizeof(enable))) {
			dev_err(info->dev,
				"Copy from userspace failed\n");
			ret = -EFAULT;
			break;
		}

		ret = mmio_enable_xshutdown_from_host(info, enable);
		break;
	case MMIO_CAM_GET_IP_GPIO:
		no_of_bytes = sizeof(struct mmio_input_output_t);
		memset(&data, 0, sizeof(struct mmio_input_output_t));

		if (copy_from_user
				(&data, (struct mmio_input_output_t *)arg,
				 no_of_bytes)) {
			dev_err(info->dev,
				"Copy from userspace failed\n");
			ret = -EFAULT;
			break;
		}

		data.mmio_arg.xshutdown_info.ip_gpio =
			info->pdata->reset_ipgpio
			[data.mmio_arg.xshutdown_info.camera_function];

		if (copy_to_user((struct mmio_input_output_t *)arg,
				 &data, sizeof(no_of_bytes))) {
			dev_err(info->dev,
				"Copy to userspace failed\n");
			ret = -EFAULT;
			break;
		}

		break;

	default:
		dev_err(info->dev, "Not an ioctl for this module\n");
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int mmio_release(struct inode *node, struct file *filp)
{
	struct mmio_info *info = filp->private_data;
	BUG_ON(info == NULL);


	/* If not already done... */
	if (info->pdata->camera_slot != -1) {
		/* ...force camera to power off */
		mmio_cam_pwr_sensor(info, false);
		mmio_cam_control_clocks(info, false);
		/* ...force to desinit board */
		mmio_cam_desinitboard(info);
		info->pdata->camera_slot = -1;
	}

	return 0;
}

static int mmio_open(struct inode *node, struct file *filp)
{
	filp->private_data = info;	/* Hook our mmio info */
	return 0;
}

static const struct file_operations mmio_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = mmio_ioctl,
	.open = mmio_open,
	.release = mmio_release,
};

/**
* mmio_probe() - Initialize MMIO Camera resources.
* @pdev: Platform device.
*
* Initialize the module and register misc device.
*
* Returns:
*	0 if there is no err.
*	-ENOMEM if allocation fails.
*	-EEXIST if device has already been started.
*	Error codes from misc_register.
*/
static int __devinit mmio_probe(struct platform_device *pdev)
{
	int err;

	dev_dbg(&pdev->dev, "%s\n", __func__);
	/* Initialize private data. */
	info = kzalloc(sizeof(struct mmio_info), GFP_KERNEL);

	if (!info) {
		dev_err(&pdev->dev, "Could not alloc info struct\n");
		err = -ENOMEM;
		goto err_alloc;
	}

	/* Fill in private data */
	info->pdata = pdev->dev.platform_data;
	info->dev = &pdev->dev;
	info->pdata->dev = &pdev->dev;
	info->misc_dev.minor = MISC_DYNAMIC_MINOR;
	info->misc_dev.name = MMIO_NAME;
	info->misc_dev.fops = &mmio_fops;
	info->misc_dev.parent = pdev->dev.parent;

	info->xshutdown_enabled = 0;
	info->xshutdown_is_active_high = 0;

	/* Register Misc character device */
	err = misc_register(&(info->misc_dev));

	if (err) {
		dev_err(&pdev->dev, "Error %d registering misc dev!", err);
		goto err_miscreg;
	}

	/* Memory mapping */
	info->siabase = ioremap(info->pdata->sia_base, SIA_ISP_MCU_SYS_SIZE);

	if (!info->siabase) {
		dev_err(info->dev, "Could not ioremap SIA_BASE\n");
		err = -ENOMEM;
		goto err_ioremap_sia_base;
	}

	info->crbase = ioremap(info->pdata->cr_base, PAGE_SIZE);

	if (!info->crbase) {
		dev_err(info->dev, "Could not ioremap CR_BASE\n");
		err = -ENOMEM;
		goto err_ioremap_cr_base;
	}

	dev_info(&pdev->dev, "MMIO driver initialized with minor=%d\n",
		 info->misc_dev.minor);
	return 0;

err_ioremap_cr_base:
	iounmap(info->siabase);
err_ioremap_sia_base:
	misc_deregister(&info->misc_dev);
err_miscreg:
	kfree(info);
	info = NULL;
err_alloc:
	return err;
}

/**
* mmio_remove() - Release MMIO Camera resources.
* @pdev:	Platform device.
*
* Remove misc device and free resources.
*
* Returns:
*	0 if success.
*	Error codes from misc_deregister.
*/
static int __devexit mmio_remove(struct platform_device *pdev)
{
	int err;

	if (!info)
		return 0;

	err = misc_deregister(&info->misc_dev);

	if (err)
		dev_err(&pdev->dev, "Error %d deregistering misc dev", err);

	iounmap(info->siabase);
	iounmap(info->crbase);
	kfree(info);
	info = NULL;
	return 0;
}
static struct platform_driver mmio_driver = {
	.driver = {
		.name = MMIO_NAME,
		.owner = THIS_MODULE,
	},
	.probe = mmio_probe,
	.remove = __devexit_p(mmio_remove)
};

/**
* mmio_init() - Initialize module.
*
* Registers platform driver.
*/
static int __init mmio_init(void)
{
	return platform_driver_register(&mmio_driver);
}

/**
* mmio_exit() - Remove module.
*
* Unregisters platform driver.
*/
static void __exit mmio_exit(void)
{
	platform_driver_unregister(&mmio_driver);
}

module_init(mmio_init);
module_exit(mmio_exit);

MODULE_AUTHOR("Joakim Axelsson ST-Ericsson");
MODULE_AUTHOR("Pankaj Chauhan ST-Ericsson");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MMIO Camera driver");

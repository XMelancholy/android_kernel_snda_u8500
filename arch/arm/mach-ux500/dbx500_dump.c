/*
 * Copyright (C) ST-Ericsson SA 2011
 *
 * License Terms: GNU General Public License v2
 * Author: Johan Bjornstedt <johan.bjornstedt@stericsson.com>
 *
 * Save DBx500 registers and AVS values in case of kernel crash
 */

#define pr_fmt(fmt) "dbx500_dump: " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/kdebug.h>

#include <mach/hardware.h>
#include <mach/db8500-regs.h>

#include <mach/id.h>

#define AVS_SIZE	14
#define PRCM_AVS_BASE  0x2FC
#define AVS_BACKUP     0xC30
#define HEADER_LEN     4
#define MAGIC_HEADER   0x53564100

struct dbx500_dump_info {
	char *name;
	int *data;
	int *io_addr;
	int phy_addr;
	int size;
};

static struct dbx500_dump_info db8500_dump[] = {
	{
		.name     = "prcmu_tcdm",
		.phy_addr = U8500_PRCMU_TCDM_BASE,
		.size     = 0x1000,
	},
	{
		.name     = "prcmu_non_sec_1",
		.phy_addr = U8500_PRCMU_BASE,
		.size     = 0x340,
	},
	{
		.name     = "prcmu_pmb",
		.phy_addr = (U8500_PRCMU_BASE + 0x344),
		.size     = 0xC,
	},
	{
		.name     = "prcmu_thermal",
		.phy_addr = (U8500_PRCMU_BASE + 0x3C0),
		.size     = 0x40,
	},
	{
		.name     = "prcmu_non_sec_2",
		.phy_addr = (U8500_PRCMU_BASE + 0x404),
		.size     = 0x1FC,
	},
	{
		.name     = "prcmu_icn_pmu",
		.phy_addr = (U8500_PRCMU_BASE + 0xE00),
		.size     = 0x90,
	},
	{
		.name	  = "public_backup_ram",
		.phy_addr = (U8500_BACKUPRAM1_BASE + 0xC00),
		.size	  = 0x400,
	},
};

static struct dbx500_dump_info db9540_dump[] = {
	{
		.name     = "prcmu_tcdm",
		.phy_addr = U8500_PRCMU_TCDM_BASE + SZ_8K,
		.size     = 0x1000,
	},
	{
		.name     = "prcmu_non_sec_1",
		.phy_addr = U8500_PRCMU_BASE,
		.size     = 0x340,
	},
	{
		.name     = "prcmu_pmb",
		.phy_addr = (U8500_PRCMU_BASE + 0x344),
		.size     = 0xC,
	},
	{
		.name     = "prcmu_thermal",
		.phy_addr = (U8500_PRCMU_BASE + 0x3C0),
		.size     = 0x40,
	},
	{
		.name     = "prcmu_non_sec_2",
		.phy_addr = (U8500_PRCMU_BASE + 0x404),
		.size     = 0x1FC,
	},
	{
		.name     = "prcmu_icn_pmu",
		.phy_addr = (U8500_PRCMU_BASE + 0xE00),
		.size     = 0x90,
	},
};

static struct dbx500_dump_info *dbx500_dump;
static int dbx500_dump_size;
static bool dbx500_dump_done;
static u8 *prcmu_tcdm;
static u8 *backup_ram;

static DEFINE_SPINLOCK(dbx500_dump_lock);

extern void ab8500_dump_all_banks_to_mem(void);

static void avs_backup(void)
{
	u8 avs[AVS_SIZE];
	static u32 counter;

	/* Backup avs values for 8500 family (8500, 9500, 8520, 8420) */
	if (!cpu_is_u8500_family())
		return;

	if (prcmu_tcdm == NULL || backup_ram == NULL) {
		pr_err("AVS and backup ram not ioremapped\n");
		return;
	}

	/* Write special header */
	writel(MAGIC_HEADER + counter, backup_ram + AVS_BACKUP);

	counter++;

	/* Read AVS data from TCDM of PRCMU */
	memcpy_fromio(avs, prcmu_tcdm + PRCM_AVS_BASE, AVS_SIZE);
	/* Save it to BACKUP_RAM (0x80151C34) */
	memcpy_toio(backup_ram + AVS_BACKUP + HEADER_LEN, avs, AVS_SIZE);
}

static void __init avs_backup_init(void)
{
	if (!cpu_is_u8500_family())
		return;

	/* ioremap these areas at boot, when we know that the system is ok */
	prcmu_tcdm = ioremap(U8500_PRCMU_TCDM_BASE, SZ_4K);
	backup_ram = ioremap(U8500_BACKUPRAM1_BASE, SZ_4K);
}

static int crash_notifier(struct notifier_block *nb, unsigned long val,
		void *data)
{
	int i;
	unsigned long flags;

	/*
	* Since there are two ways into this function (die and panic) we have
	* to make sure we only dump the DB registers once
	*/
	spin_lock_irqsave(&dbx500_dump_lock, flags);
	if (dbx500_dump_done)
		goto out;

	pr_info("Saving registers for crash analyze.\n");

	for (i = 0; i < dbx500_dump_size; i++) {
		memcpy_fromio(dbx500_dump[i].data, dbx500_dump[i].io_addr,
			dbx500_dump[i].size);
		pr_info("Saved %d bytes from %s (0x%x)\n",
			dbx500_dump[i].size,
			dbx500_dump[i].name,
			dbx500_dump[i].phy_addr);
	}

	ab8500_dump_all_banks_to_mem();

	avs_backup();
	pr_info("Saved AVS values.\n");

	dbx500_dump_done = true;
out:
	spin_unlock_irqrestore(&dbx500_dump_lock, flags);

	return NOTIFY_DONE;
}

static void __init io_addresses_init(void)
{
	int i;

	for (i = 0; i < dbx500_dump_size; i++)
		dbx500_dump[i].io_addr = ioremap(dbx500_dump[i].phy_addr,
			dbx500_dump[i].size);
}

static struct notifier_block die_notifier = {
	.notifier_call = crash_notifier,
};

static struct notifier_block panic_notifier = {
	.notifier_call = crash_notifier,
};

int __init dbx500_dump_init(void)
{
	int err, i;

	if (cpu_is_u8500_family()) {
		dbx500_dump = db8500_dump;
		dbx500_dump_size = ARRAY_SIZE(db8500_dump);
	} else if (cpu_is_ux540_family()) {
		dbx500_dump = db9540_dump;
		dbx500_dump_size = ARRAY_SIZE(db9540_dump);
	}  else {
		ux500_unknown_soc();
	}

	for (i = 0; i < dbx500_dump_size; i++) {
		dbx500_dump[i].data = kmalloc(dbx500_dump[i].size, GFP_KERNEL);
		if (!dbx500_dump[i].data) {
			pr_err("Could not allocate memory for %s\n",
			       dbx500_dump[i].name);
			err = -ENOMEM;
			goto free_mem;
		}
	}

	io_addresses_init();
	avs_backup_init();

	avs_backup();

	err = atomic_notifier_chain_register(&panic_notifier_list,
					     &panic_notifier);
	if (err != 0) {
		pr_err("Unable to register a panic notifier %d\n", err);
		goto free_mem;
	}

	err = register_die_notifier(&die_notifier);
	if (err != 0) {
		pr_err("Unable to register a die notifier %d\n", err);
		goto free_panic_notifier;
	}

	pr_info("initialized\n");
	return err;

free_panic_notifier:
	atomic_notifier_chain_unregister(&panic_notifier_list,
					 &panic_notifier);

free_mem:
	for (i = i - 1; i >= 0; i--)
		kfree(dbx500_dump[i].data);

	return err;
}
arch_initcall(dbx500_dump_init);

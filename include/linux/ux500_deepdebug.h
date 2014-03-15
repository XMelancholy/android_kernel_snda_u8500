/*
 * Copyright (C) ST-Ericssson SA 2012
 *
 * Deep Debug Framework interface
 *
 * License terms:  GNU General Public License (GPL), version 2
 */

#ifndef UX500_DEEPDEBUG_H
#define UX500_DEEPDEBUG_H

/* Register access rights */
#define	DDBG_RO		0
#define DDBG_WO		1
#define	DDBG_RW		2

#define DDBG_MAX_REG	0x10000

/**
 * ddbg_register - target IP register structure for deep debug
 *
 * @name:	string for register, nicely displayed in debugfs
 * @offset:	target register offset value, from IP registers base addr
 * @perm:	0, read-only, 1 read-write. more to come?
 */
struct ddbg_register {
	const char *name;
	u32 offset;
	u8 perm;
};

/**
 * ddbg_target
 *
 * @name:	name of the feature
 * @phyaddr:	target IP register physical base address
 * @reg:	pointer to target IP dbbg_register table.
 * @nb_regs:	number of registers defined in dbbg_register table
 * @curr:	pointer to currently selected HW register.
 * @write_reg:	driver specific routine to write to selected HW register
 * @read_reg:	driver specific routine to read selected HW register
 * @dir:	pointer to debugfs folder where debug files will be stored
 * @reg_file:	pointer to debugfs filenode "registers"
 * @val_file:	pointer to debugfs filenode "value"
 * @sel_file:	pointer to debugfs filenode "selected"
 * @dev_data:	pointer to target IP data structure
 *
 * Description:
 *   Addr to this struct must be passed to deep_debug_register arg.
 *   Driver shall have initialized field
 *      name, phyaddr, reg,
 *      write_reg, read_reg
 */
struct ddbg_target {
	char name[128];
	u32 phyaddr; /* physical base addr */
	struct ddbg_register *reg;
	u32 nb_regs; /* num regs accessible */
	struct ddbg_register *curr; /* currently selected */
	int (*write_reg)(struct ddbg_target *ddbg, u32 val);
	int (*read_reg)(struct ddbg_target *ddbg, u32 *val);
	int (*suspend)(void);	/* TODO: to setup */
	int (*resume)(void);	/* TODO: to setup */
	struct dentry *dir;
	struct dentry *reg_file;
	struct dentry *val_file;
	struct dentry *sel_file;
	void *dev_data;
};

/**
 * ddbg_service
 *
 * @name:	name of the service
 * @probe:	function pointer to create service debugfs files
 *
 * Description:
 *
 */
struct ddbg_service {
	const char *name;
	int (*probe) (struct ddbg_service *, struct dentry *);
};


/* Marco for filling ddbg_register tables */
#define DDBG_REG(_label, _offset, _perm) { \
		.name = _label, \
		.offset = _offset, \
		.perm = _perm, \
}

#define DDBG_REG_NULL { .name = NULL }

/**
 * deep_debug_regaccess_register - register to deep_debug
 *
 * @ddbg:	addr of ddbg_target struture for target HW IP.
 *
 * Returns zero on success, and negative upon error.
 */
int deep_debug_regaccess_register(struct ddbg_target *ddbg);

/**
 * deep_debug_regaccess_unregister - unregister from deep_debug
 *
 * @ddbg:	addr of ddbg_target struture for target HW IP.
 *
 * Returns zero on success, and negative upon error.
 */
int deep_debug_regaccess_unregister(struct ddbg_target *ddbg);

/**
 * deep_debug_service_access_register - register to deep_debug
 *
 * @ddbg:	addr of ddbg_service struture for target HW IP.
 *
 * Returns zero on success, and negative upon error.
 */
int deep_debug_service_access_register(struct ddbg_service *ddbg);

/**
 * deep_debug_service_access_unregister - unregister from deep_debug
 *
 * @ddbg:	addr of ddbg_service struture for target HW IP.
 *
 * Returns zero on success, and negative upon error.
 */
int deep_debug_service_access_unregister(struct ddbg_service *ddbg);

#endif /* UX500_DEEPDEBUG_H */

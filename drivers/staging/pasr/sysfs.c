/*
 * Copyright (C) ST-Ericsson SA 2011
 * Author: Julien Delacou <julien.delacou@stericsson.com> for ST-Ericsson.
 * License terms:  GNU General Public License (GPL), version 2
 */

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/sysfs.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/pasr.h>
#include <mach/pasr.h>

#define KOBJ_ATTR(_name, _mode, _show, _store) \
struct kobj_attribute attr_##_name = __ATTR(_name, _mode, _show, _store)

#define SUMMARY_LINE_SZ 128
#define PASR_SUMMARY_SZ (SUMMARY_LINE_SZ * PASR_MAX_DIE_NR)
#define DIE_SUMMARY_SZ (SUMMARY_LINE_SZ * PASR_MAX_SECTION_NR_PER_DIE)

struct die_sysfs {
	struct kobject *die_kobj;
	struct kobject *sec_kobj[PASR_MAX_SECTION_NR_PER_DIE];
};

struct pasr_sysfs {
	struct pasr_map *map;
	struct kobject *root_kobj;
};

static struct pasr_sysfs *pasr_sysfs;

/* utility functions */

static int retrieve_die(const char *die)
{
	int i;

	for (i = 0; i < PASR_MAX_DIE_NR; i++) {
		if (!strcmp(pasr_sysfs->map->die[i].kobj->name, die))
			return i;
	}

	pr_err("%s: no die corresponding to %s\n", __func__, die);
	return -EINVAL;
}

static int retrieve_section(int i, const char *section)
{
	int j;

	for (j = 0; j < PASR_MAX_SECTION_NR_PER_DIE; j++) {
		if (!strcmp(pasr_sysfs->map->die[i].section[j].kobj->name,
					section))
			return j;
	}

	pr_err("%s: no section die corresponding to %s\n", __func__, section);
	return -EINVAL;
}

static int retrieve_die_and_section(const struct kobject *kobj, int *i, int *j)
{
	*i = retrieve_die(kobj->parent->name);
	if (*i < 0)
		return -EINVAL;

	*j = retrieve_section(*i, kobj->name);
	if (*j < 0)
		return -EINVAL;

	return 0;
}

/* show/store callbacks */

static ssize_t summary_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int i, n;
	struct pasr_die *die;
	char *summary;
	char *p;

	summary = kzalloc(PASR_SUMMARY_SZ * sizeof(char), GFP_KERNEL);
	p = summary;

	for (i = 0; i < pasr_sysfs->map->nr_dies; i++) {

		die = &pasr_sysfs->map->die[i];
		p += snprintf(p, SUMMARY_LINE_SZ,
				"PASR mask %#010lx to %s (%#010x)\n",
				die->mem_reg, die->kobj->name, die->start);

		strncat(summary, p, sizeof(summary));
	}

	n = sprintf(buf, "%s", summary);
	kfree(summary);

	return n;
}


static ssize_t base_addr_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int i = retrieve_die(kobj->name);
	if (i < 0)
		return -EINVAL;

	return sprintf(buf, "%#08x\n", pasr_sysfs->map->die[i].start);
}

static ssize_t mask_register_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int i = retrieve_die(kobj->name);
	if (i < 0)
		return -EINVAL;

	return sprintf(buf, "%#lx\n", pasr_sysfs->map->die[i].mem_reg);
}

static ssize_t section_base_addr_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int i, j;
	int err = retrieve_die_and_section(kobj, &i, &j);
	if (err)
		return err;

	return sprintf(buf, "%#08x\n",
			pasr_sysfs->map->die[i].section[j].start);
}

static ssize_t free_size_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int i, j;
	int err = retrieve_die_and_section(kobj, &i, &j);
	if (err)
		return err;

	return sprintf(buf, "%#lx\n",
			pasr_sysfs->map->die[i].section[j].free_size);
}

static ssize_t state_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int i, j;
	int err = retrieve_die_and_section(kobj, &i, &j);
	if (err)
		return err;

	return sprintf(buf, "%s\n",
			pasr_sysfs->map->die[i].section[j].state ==
			PASR_REFRESH ? "refresh" : "no refresh");
}

static ssize_t die_summary_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int i, n;
	char *summary;
	char *p;
	struct pasr_die *die;
	struct pasr_section *section;

	i = retrieve_die(kobj->name);
	if (i < 0)
		return -EINVAL;
	die = &pasr_sysfs->map->die[i];

	summary = kzalloc(DIE_SUMMARY_SZ * sizeof(char), GFP_KERNEL);
	p = summary;

	for (i = 0; i < die->nr_sections; i++) {
		section = &die->section[i];

		p += snprintf(p, SUMMARY_LINE_SZ,
				"section%d\n--------\nbase addr:%#x\n"
				"state:%s\nfree size:%#lx\n"
				, i
				, section->start
				, section->state == PASR_REFRESH ?
					"refresh" : "no refresh"
				, section->free_size);

		strncat(summary, p, sizeof(summary));
	}

	n = sprintf(buf, "%s", summary);
	kfree(summary);

	return n;
}


/* entry related to /sys/power/pasr */
static const KOBJ_ATTR(summary, S_IRUGO,
		summary_show, NULL);

/* entries related to /sys/power/pasr/die0..N */
static const KOBJ_ATTR(base_addr, S_IRUGO,
		base_addr_show, NULL);
static const KOBJ_ATTR(mask_register, S_IRUGO,
		mask_register_show, NULL);
static const KOBJ_ATTR(die_summary, S_IRUGO,
		die_summary_show, NULL);

/* entries related to /sys/power/pasr/die0..N/section0..N */
static const KOBJ_ATTR(section_base_addr, S_IRUGO,
		section_base_addr_show, NULL);
static const KOBJ_ATTR(free_size, S_IRUGO,
		free_size_show, NULL);
static const KOBJ_ATTR(state, S_IRUGO,
		state_show, NULL);

static const struct attribute *pasr_entries[] = {
	&attr_summary.attr,
	NULL,
};

static const struct attribute *die_attrs[] = {
	&attr_mask_register.attr,
	&attr_base_addr.attr,
	&attr_die_summary.attr,
	NULL,
};

static const struct attribute *section_attrs[] = {
	&attr_section_base_addr.attr,
	&attr_free_size.attr,
	&attr_state.attr,
	NULL,
};

static struct attribute_group pasr_attr_group = {
	.attrs = (struct attribute **) pasr_entries,
};

static const struct attribute_group die_attr_group = {
	.attrs = (struct attribute **) die_attrs,
};

static const struct attribute_group section_attr_group = {
	.attrs = (struct attribute **) section_attrs,
};

/* cleanup all entries if exist */
static void pasr_sysfs_remove(void)
{
	int i, j;
	struct pasr_die *die;
	struct pasr_section *section;

	for (i = 0; i < pasr_sysfs->map->nr_dies; i++) {
		die = &pasr_sysfs->map->die[i];
		for (j = 0; j < pasr_sysfs->map->die[i].nr_sections; j++) {
			section = &die->section[j];
			if (section->kobj) {
				if (section->pair)
					sysfs_remove_link(section->kobj,
							"pair");
				sysfs_remove_dir(section->kobj);
				kobject_put(section->kobj);
			}
		}
		if (die->kobj)
			kobject_put(die->kobj);
	}
}

/* once each kobj is created, creates sym links to pairs */
static int pasr_sysfs_create_links(void)
{
	int i, j;
	int err = 0;
	struct pasr_section *section;

	for (i = 0; i < pasr_sysfs->map->nr_dies; i++) {
		for (j = 0; j < pasr_sysfs->map->die[i].nr_sections; j++) {
			section = &pasr_sysfs->map->die[i].section[j];
			if (section->pair) {
				err = sysfs_create_link(section->kobj,
						section->pair->kobj, "pair");
				if (err)
					return err;
			}
		}
	}

	return err;
}

/* creates a section#j entry under given die#i subdir */
static int pasr_sysfs_create_section(int i, int j)
{
	int err = 0;

	struct pasr_section *section = &pasr_sysfs->map->die[i].section[j];

	section->kobj = kobject_create();
	if (!section->kobj)
		return -ENOMEM;

	err = kobject_add(section->kobj,
			pasr_sysfs->map->die[i].kobj, "section%u", j);
	if (err)
		return err;

	err = sysfs_create_group(section->kobj, &section_attr_group);

	return err;
}

/* create a die#i entry under pasr subdir */
static int pasr_sysfs_create_die(int i)
{
	int err = 0;

	pasr_sysfs->map->die[i].kobj = kobject_create();
	if (!pasr_sysfs->map->die[i].kobj)
		return -ENOMEM;

	err = kobject_add(pasr_sysfs->map->die[i].kobj,
			pasr_sysfs->root_kobj, "die%u", i);
	if (err)
		return err;

	err = sysfs_create_group(pasr_sysfs->map->die[i].kobj,
			&die_attr_group);

	return err;
}

/* create the sysfs tree */
int pasr_sysfs_create(struct pasr_map *map)
{
	int err = 0;
	int i, j;

	pasr_sysfs = kzalloc(sizeof(*pasr_sysfs), GFP_KERNEL);
	if (!pasr_sysfs)
		return -ENOMEM;

	pasr_sysfs->map = (struct pasr_map *) map;

	pasr_sysfs->root_kobj = kobject_create_and_add("pasr", power_kobj);
	if (!pasr_sysfs->root_kobj)
		return -ENOMEM;

	err = sysfs_create_group(pasr_sysfs->root_kobj, &pasr_attr_group);
	if (err)
		goto error;

	for (i = 0; i < pasr_sysfs->map->nr_dies; i++) {
		err = pasr_sysfs_create_die(i);
		if (err)
			goto error;

		for (j = 0; j < pasr_sysfs->map->die[i].nr_sections; j++) {
			err = pasr_sysfs_create_section(i, j);
			if (err)
				goto error;
		}
	}

	/* need to iterate once again to create sym links */
	err = pasr_sysfs_create_links();
	if (err)
		goto error;

	return 0;

error:
	pr_err("%s error creating sysfs (%d)\n", __func__, err);
	pasr_sysfs_remove();
	kfree(pasr_sysfs);

	return err;
}

static int __init pasr_init_sysfs(void)
{
	struct pasr_map *m = pasr_get_map();
	if (!m)
		return -ENODEV;

	return pasr_sysfs_create(m);
}
module_init(pasr_init_sysfs);

static void __exit pasr_exit_sysfs(void)
{
	pasr_sysfs_remove();
	kfree(pasr_sysfs);
}
module_exit(pasr_exit_sysfs);



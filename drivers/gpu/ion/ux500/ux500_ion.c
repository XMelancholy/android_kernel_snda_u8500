/*
 * drivers/gpu/ion/ux500/ux500_ion.c
 *
 * Copyright (C) Linaro 2012
 * Author: <benjamin.gaignard@linaro.org> for ST-Ericsson.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/err.h>
#include <linux/ion.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include "../ion_priv.h"

struct ion_ux500_internal {
	struct ion_device *ion_device;
	int num_heaps;
	struct ion_heap **heaps;
};

int ux500_ion_probe(struct platform_device *pdev)
{
	struct ion_platform_data *pdata = pdev->dev.platform_data;
	struct ion_ux500_internal *ion_ux500;
	int ret = 0;
	int i;

	ion_ux500 = devm_kzalloc(&pdev->dev,
			sizeof(struct ion_ux500_internal), GFP_KERNEL);
	if (!ion_ux500) {
		dev_err(&pdev->dev, "can't allocate memory\n");
		return -ENOMEM;
	}

	ion_ux500->num_heaps = pdata->nr;
	if (!ion_ux500->num_heaps) {
		dev_err(&pdev->dev, "no heap defined\n");
		return -ENODEV;
	}

	ion_ux500->heaps = devm_kzalloc(&pdev->dev,
			sizeof(struct ion_heap *) * ion_ux500->num_heaps,
			GFP_KERNEL);
	if (!ion_ux500->heaps) {
		dev_err(&pdev->dev, "no memory for heaps\n");
		return -ENOMEM;
	}

	ion_ux500->ion_device = ion_device_create(NULL);
	if (IS_ERR_OR_NULL(ion_ux500->ion_device)) {
		dev_err(&pdev->dev, "failed to create ion device\n");
		return PTR_ERR(ion_ux500->ion_device);
	}

	/* create the heaps as specified in the board file */
	for (i = 0; i < ion_ux500->num_heaps; i++) {
		struct ion_platform_heap *heap_data = &pdata->heaps[i];

		ion_ux500->heaps[i] = ion_heap_create(heap_data);
		if (IS_ERR_OR_NULL(ion_ux500->heaps[i])) {
			ret = PTR_ERR(ion_ux500->heaps[i]);
			ion_ux500->heaps[i] = NULL;
			dev_err(&pdev->dev,
				"failed to create heap type %d id %d\n",
				heap_data->type, heap_data->id);
			goto release_heaps;
		}
		ion_device_add_heap(ion_ux500->ion_device, ion_ux500->heaps[i]);
	}

	platform_set_drvdata(pdev, ion_ux500);

	return ret;

release_heaps:
	for (i = 0; i < ion_ux500->num_heaps; i++)
		if (ion_ux500->heaps[i])
			ion_heap_destroy(ion_ux500->heaps[i]);

	ion_device_destroy(ion_ux500->ion_device);
	return ret;
}

int ux500_ion_remove(struct platform_device *pdev)
{
	struct ion_ux500_internal *ion_ux500 = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < ion_ux500->num_heaps; i++)
		if (ion_ux500->heaps[i])
			ion_heap_destroy(ion_ux500->heaps[i]);

	ion_device_destroy(ion_ux500->ion_device);
	return 0;
}

static struct platform_driver ux500_ion_driver = {
	.probe = ux500_ion_probe,
	.remove = ux500_ion_remove,
	.driver = {
		   .name = "ion-ux500",
	}
};

static int __init ux500_ion_init(void)
{
	return platform_driver_register(&ux500_ion_driver);
}

static void __exit ux500_ion_exit(void)
{
	platform_driver_unregister(&ux500_ion_driver);
}

module_init(ux500_ion_init);
module_exit(ux500_ion_exit);

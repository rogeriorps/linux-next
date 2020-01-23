/*
 * Samsung MIPI DSI Host Controller on IMX
 *
 * Copyright 2018-2019 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <drm/drm_atomic_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>
#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/gpio/consumer.h>
#include <linux/irq.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/regmap.h>
#include <linux/sys_soc.h>
#include <video/videomode.h>

struct sec_mipi_dsim_plat_data {
	uint32_t version;
	//uint32_t max_data_lanes;
	//uint64_t max_data_rate;
	//const struct sec_mipi_dsim_dphy_timing *dphy_timing;
	//uint32_t num_dphy_timing;
	//const struct sec_mipi_dsim_pll *dphy_pll;
	//int (*dphy_timing_cmp)(const void *key, const void *elt);
	//enum drm_mode_status (*mode_valid)(struct drm_connector *connector,
	//				   struct drm_display_mode *mode);
};

static const struct sec_mipi_dsim_plat_data imx8mm_mipi_dsim_plat_data = {
	.version	= 0x1060200,
	//.max_data_lanes = 4,
	//.max_data_rate  = 1500000000ULL,
	//.dphy_pll	= &pll_1432x,
	//.dphy_timing	= dphy_timing_ln14lpp_v1p2,
	//.num_dphy_timing = ARRAY_SIZE(dphy_timing_ln14lpp_v1p2),
	//.dphy_timing_cmp = dphy_timing_default_cmp,
	//.mode_valid	= NULL,
};

static const struct of_device_id imx_sec_dsim_dt_ids[] = {
	{
		.compatible = "fsl,imx8mm-mipi-dsim",
		.data = &imx8mm_mipi_dsim_plat_data,
	},
	{
		.compatible = "fsl,imx8mn-mipi-dsim",
		.data = &imx8mm_mipi_dsim_plat_data,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_sec_dsim_dt_ids);

static int imx_sec_dsim_bind(struct device *dev, struct device *master,
			     void *data)
{
	int ret, irq;
	struct resource *res;
	struct drm_device *drm_dev = data;
	struct platform_device *pdev = to_platform_device(dev);
	struct device_node *np = dev->of_node;
	const struct of_device_id *of_id = of_match_device(imx_sec_dsim_dt_ids,
							   dev);
	const struct sec_mipi_dsim_plat_data *pdata = of_id->data;
	struct drm_encoder *encoder;

  printk(KERN_INFO "Bind dsim!!!");

/*
	dev_dbg(dev, "%s: dsim bind begin\n", __func__);
	dsim_dev = devm_kzalloc(dev, sizeof(*dsim_dev), GFP_KERNEL);
	if (!dsim_dev) {
		dev_err(dev, "Unable to allocate 'dsim_dev'\n");
		return -ENOMEM;
	}

	dsim_dev->dev = &pdev->dev;
*/
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return -ENODEV;
/*
	dsim_dev->gpr = syscon_regmap_lookup_by_phandle(np, "dsi-gpr");
	if (IS_ERR(dsim_dev->gpr))
		return PTR_ERR(dsim_dev->gpr);

	encoder = &dsim_dev->encoder;
	ret = imx_drm_encoder_parse_of(drm_dev, encoder, np);
	if (ret)
		return ret;

	drm_encoder_helper_add(encoder, &imx_sec_dsim_encoder_helper_funcs);

	ret = drm_encoder_init(drm_dev, encoder,
			       &imx_sec_dsim_encoder_funcs,
			       DRM_MODE_ENCODER_DSI, dev_name(dev));
	if (ret)
		return ret;
*/
	/* bind sec dsim bridge */
/*
  ret = sec_mipi_dsim_bind(dev, master, data, encoder, res, irq, pdata);
	if (ret) {
		dev_err(dev, "failed to bind sec dsim bridge: %d\n", ret);
		drm_encoder_cleanup(encoder);
		return ret;
	}

	atomic_set(&dsim_dev->rpm_suspended, 0);
	pm_runtime_enable(dev);
	atomic_inc(&dsim_dev->rpm_suspended);

	dev_dbg(dev, "%s: dsim bind end\n", __func__);
*/
	return 0;
}

static void imx_sec_dsim_unbind(struct device *dev, struct device *master,
				void *data)
{
  printk (KERN_INFO "Unbind DSIM");
  //pm_runtime_disable(dev);

	//sec_mipi_dsim_unbind(dev, master, data);

	//drm_encoder_cleanup(&dsim_dev->encoder);
}


static const struct component_ops imx_sec_dsim_ops = {
	.bind	= imx_sec_dsim_bind,
	.unbind	= imx_sec_dsim_unbind,
};


static int imx_sec_dsim_probe(struct platform_device *pdev)
{
  int ret;
  dev_dbg(&pdev->dev, "%s: dsim probe begin\n", __func__);
  printk(KERN_INFO "%s: dsim probe begin!\n", __func__);

	ret = component_add(&pdev->dev, &imx_sec_dsim_ops);
  printk(KERN_INFO "dsim component_add: %d", ret);
  return ret;
}

static int imx_sec_dsim_remove(struct platform_device *pdev)
{
	//component_del(&pdev->dev, &imx_sec_dsim_ops);
  printk(KERN_INFO "%s: dsim remove begin!\n", __func__);
	return 0;
}





struct platform_driver imx_sec_dsim_driver = {
	.probe    = imx_sec_dsim_probe,
	.remove   = imx_sec_dsim_remove,
	.driver   = {
		.name = "imx_sec_dsim_drv",
		.of_match_table = imx_sec_dsim_dt_ids,
		//.pm = &imx_sec_dsim_pm_ops,
	},
};

module_platform_driver(imx_sec_dsim_driver);

MODULE_DESCRIPTION("NXP i.MX MIPI DSI Host Controller driver");
MODULE_AUTHOR("Rogerio Silva <rogerio.silva@nxp.com>");
MODULE_LICENSE("GPL");

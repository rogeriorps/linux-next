/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Samsung MIPI DSI host driver
 *
 * Copyright (C) 2020 NXP
 */

#ifndef __NWL_DRV_H__
#define __NWL_DRV_H__

#include <linux/phy/phy.h>

#include <drm/drm_bridge.h>
#include <drm/drm_mipi_dsi.h>

struct samsung_dsi_platform_data;

struct samsung_dsi_plat_clk_config {
	const char *id;
	struct clk *clk;
	bool present;
};

struct sec_dsim_dsi {
	struct drm_bridge bridge;
	struct mipi_dsi_host dsi_host;
	struct drm_bridge *panel_bridge;
	struct device *dev;
	struct phy *phy;
	union phy_configure_opts phy_cfg;
	unsigned int quirks;

	struct regmap *regmap;
	int irq;
	struct reset_control *rstc;

	/* External registers */
	struct regmap *mux_sel;

	/* DSI clocks */
	struct clk *phy_ref_clk;
	struct clk *rx_esc_clk;
	struct clk *tx_esc_clk;
	/* Platform dependent clocks */
//	struct nwl_dsi_plat_clk_config clk_config[NWL_DSI_MAX_PLATFORM_CLOCKS];

	/* dsi lanes */
	u32 lanes;
//	enum mipi_dsi_pixel_format format;
	struct drm_display_mode mode;
	unsigned long dsi_mode_flags;

//	struct nwl_dsi_transfer *xfer;

//	const struct nwl_dsi_platform_data *pdata;
};



#endif /* __NWL_DRV_H__ */

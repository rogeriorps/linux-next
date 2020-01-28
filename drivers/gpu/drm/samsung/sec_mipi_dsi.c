#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/regmap.h>
#include <linux/time64.h>

#include <video/mipi_display.h>
#include <video/videomode.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>

#include "sec_mipi_dsim-imx_drv.h"
#include "sec_mipi_dsi.h"

static int nwl_dsi_host_attach(struct mipi_dsi_host *dsi_host,
			       struct mipi_dsi_device *device)
{
//	struct nwl_dsi *dsi = container_of(dsi_host, struct nwl_dsi, dsi_host);
//	struct device *dev = dsi->dev;
	struct drm_bridge *bridge;
	struct drm_panel *panel;
	int ret;

	return 0;
}

static int nwl_dsi_host_detach(struct mipi_dsi_host *dsi_host,
			       struct mipi_dsi_device *device)
{
//	struct nwl_dsi *dsi = container_of(dsi_host, struct nwl_dsi, dsi_host);

//	drm_of_panel_bridge_remove(dsi->dev->of_node, 1, 0);
//	drm_bridge_remove(&dsi->bridge);

	return 0;
}

static ssize_t nwl_dsi_host_transfer(struct mipi_dsi_host *dsi_host,
				     const struct mipi_dsi_msg *msg)
{
//	struct nwl_dsi *dsi = container_of(dsi_host, struct nwl_dsi, dsi_host);
//	struct nwl_dsi_transfer xfer;
//	ssize_t ret = 0;

	return 0;
}

const struct mipi_dsi_host_ops nwl_dsi_host_ops = {
	.attach = nwl_dsi_host_attach,
	.detach = nwl_dsi_host_detach,
	.transfer = nwl_dsi_host_transfer,
};

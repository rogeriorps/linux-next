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
  struct sec_dsim_dsi *dsi = container_of(dsi_host, struct sec_dsim_dsi, dsi_host);
  struct device *dev = dsi->dev;
  struct drm_bridge *bridge;
  struct drm_panel *panel;
  int ret;

    printk(KERN_INFO "!! func: %s\n", __func__);

    DRM_DEV_INFO(dev, "lanes=%u, format=0x%x flags=0x%lx\n", device->lanes,
  	     device->format, device->mode_flags);
    printk(KERN_INFO "!! lanes=%u, format=0x%x flags=0x%lx\n", device->lanes,
       device->format, device->mode_flags);
/*
  if (device->lanes < 1 || device->lanes > 4)
  	return -EINVAL;

  dsi->lanes = device->lanes;
  dsi->format = device->format;
  dsi->dsi_mode_flags = device->mode_flags;

  ret = drm_of_find_panel_or_bridge(dsi->dev->of_node, 1, 0, &panel,
  				  &bridge);
  if (ret)
  	return ret;

  if (panel) {
  	bridge = drm_panel_bridge_add(panel, DRM_MODE_CONNECTOR_DSI);
  	if (IS_ERR(bridge))
  		return PTR_ERR(bridge);
  }

  dsi->panel_bridge = bridge;
  drm_bridge_add(&dsi->bridge);
*/
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

irqreturn_t nwl_dsi_irq_handler(int irq, void *data)
{
	u32 irq_status;
	struct nwl_dsi *dsi = data;

//	irq_status = nwl_dsi_read(dsi, NWL_DSI_IRQ_STATUS);

//	if (irq_status & NWL_DSI_TX_PKT_DONE ||
//	    irq_status & NWL_DSI_RX_PKT_HDR_RCVD ||
//	    irq_status & NWL_DSI_RX_PKT_PAYLOAD_DATA_RCVD)
//		nwl_dsi_finish_transmission(dsi, irq_status);

	return IRQ_HANDLED;
}

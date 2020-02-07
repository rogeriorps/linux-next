#include <drm/drm_mipi_dsi.h>

extern const struct mipi_dsi_host_ops nwl_dsi_host_ops;

irqreturn_t nwl_dsi_irq_handler(int irq, void *data);

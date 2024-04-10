# Copyright 2023 NXP
#
# SPDX-License-Identifier: Apache-2.0

zephyr_sources_ifdef(CONFIG_MIPI_DBI_SPI mipi_dbi_spi.c)
zephyr_sources_ifdef(CONFIG_MIPI_DBI_SMARTBOND mipi_dbi_smartbond.c)
zephyr_sources_ifdef(CONFIG_MIPI_DBI_NXP_LCDIC mipi_dbi_nxp_lcdic.c)
zephyr_sources_ifdef(CONFIG_MIPI_DBI_NXP_FLEXIO_LCDIF mipi_dbi_nxp_flexio_lcdif.c)
# Data bus width is used by the SDK driver and processes it as a compile time option
if(CONFIG_MIPI_DBI_NXP_FLEXIO_LCDIF)
  dt_nodelabel(flexio0_lcd NODELABEL "nxp_flexio_lcd")
  dt_prop(data_bus_width PATH "${flexio0_lcd}" PROPERTY "data-bus-width")
  zephyr_compile_definitions(FLEXIO_MCULCD_DATA_BUS_WIDTH=${data_bus_width})

  zephyr_compile_definitions(FLEXIO_MCULCD_LEGACY_GPIO_FUNC=0)
endif()

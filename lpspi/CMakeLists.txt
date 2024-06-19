#
# Copyright 2024 NXP
#
# SPDX-License-Identifier: BSD-3-Clause

if (CONFIG_MCUX_COMPONENT_driver.lpspi_freertos)
    mcux_add_source(
        SOURCES
            fsl_lpspi_freertos.h
            fsl_lpspi_freertos.c
    )

    mcux_add_include(
        INCLUDES .
    )
endif()

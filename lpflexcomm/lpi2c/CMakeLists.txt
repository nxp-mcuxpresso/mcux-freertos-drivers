#
# Copyright 2024 NXP
#
# SPDX-License-Identifier: BSD-3-Clause

if (CONFIG_MCUX_COMPONENT_driver.lpflexcomm_lpi2c_freertos)
    mcux_add_source(
        SOURCES
            fsl_lpi2c_freertos.h
            fsl_lpi2c_freertos.c
    )

    mcux_add_include(
        INCLUDES .
    )
endif()

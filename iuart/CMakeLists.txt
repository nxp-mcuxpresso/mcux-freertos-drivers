#
# Copyright 2024 NXP
#
# SPDX-License-Identifier: BSD-3-Clause

if (CONFIG_MCUX_COMPONENT_driver.iuart_freertos)
    mcux_add_source(
        SOURCES
            fsl_uart_freertos.h
            fsl_uart_freertos.c
    )

    mcux_add_include(
        INCLUDES .
    )
endif()

/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __FSL_LPSCI_RTOS_H__
#define __FSL_LPSCI_RTOS_H__

#include "fsl_lpsci.h"
#include <FreeRTOS.h>
#include <event_groups.h>
#include <semphr.h>

/*!
 * @addtogroup lpsci_freertos_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
/*! @brief LPSCI FreeRTOS driver version. */
#define FSL_LPSCI_FREERTOS_DRIVER_VERSION (MAKE_VERSION(2, 1, 0))
/*@{*/

/*! @brief LPSCI RTOS configuration structure. */
typedef struct _lpsci_rtos_config
{
    UART0_Type *base;                /*!< LPSCI base address */
    uint32_t srcclk;                 /*!< LPSCI source clock in Hz*/
    uint32_t baudrate;               /*!< Desired communication speed */
    lpsci_parity_mode_t parity;      /*!< Parity setting */
    lpsci_stop_bit_count_t stopbits; /*!< Number of stop bits to use */
    uint8_t *buffer;                 /*!< Buffer for background reception */
    uint32_t buffer_size;            /*!< Size of buffer for background reception */
} lpsci_rtos_config_t;

/*!
 * @cond RTOS_PRIVATE
 * @name LPSCI event flags
 *
 * This are only valid states for txEvent and rxEvent (lpsci_rtos_handle_t).
 */
/*@{*/
/*! @brief Event flag - transfer complete. */
#define RTOS_LPSCI_COMPLETE 0x1U
/*! @brief Event flag - ring buffer overrun. */
#define RTOS_LPSCI_RING_BUFFER_OVERRUN 0x2U
/*! @brief Event flag - hardware buffer overrun. */
#define RTOS_LPSCI_HARDWARE_BUFFER_OVERRUN 0x4U
/*@}*/

/*! @brief LPSCI FreeRTOS transfer structure. */
typedef struct _lpsci_rtos_handle
{
    UART0_Type *base;              /*!< LPSCI base address */
    lpsci_transfer_t txTransfer;   /*!< TX transfer structure */
    lpsci_transfer_t rxTransfer;   /*!< RX transfer structure */
    SemaphoreHandle_t rxSemaphore; /*!< RX semaphore for resource sharing */
    SemaphoreHandle_t txSemaphore; /*!< TX semaphore for resource sharing */
    EventGroupHandle_t rxEvent;    /*!< RX completion event */
    EventGroupHandle_t txEvent;    /*!< TX completion event */
    void *t_state;                 /*!< Transactional state of the underlying driver */
#if (configSUPPORT_STATIC_ALLOCATION == 1)
    StaticSemaphore_t txSemaphoreBuffer; /*!< Statically allocated memory for txSemaphore */
    StaticSemaphore_t rxSemaphoreBuffer; /*!< Statically allocated memory for rxSemaphore */
    StaticEventGroup_t txEventBuffer;    /*!< Statically allocated memory for txEvent */
    StaticEventGroup_t rxEventBuffer;    /*!< Statically allocated memory for rxEvent */
#endif
} lpsci_rtos_handle_t;
/*! \endcond */

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name LPSCI RTOS Operation
 * @{
 */

/*!
 * @brief Initializes an LPSCI instance for operation in RTOS.
 *
 * @param handle The RTOS LPSCI handle, the pointer to allocated space for RTOS context.
 * @param t_handle The pointer to allocated space where to store transactional layer internal state.
 * @param cfg The pointer to the parameters required to configure the LPSCI after initialization.
 * @return 0 succeed, others failed
 */
int LPSCI_RTOS_Init(lpsci_rtos_handle_t *handle, lpsci_handle_t *t_handle, const lpsci_rtos_config_t *cfg);

/*!
 * @brief Deinitializes an LPSCI instance for operation.
 *
 * This function deinitializes the LPSCI modulem, set all register value to reset value
 * and releases the resources.
 *
 * @param handle The RTOS LPSCI handle.
 */
int LPSCI_RTOS_Deinit(lpsci_rtos_handle_t *handle);

/*!
 * @name LPSCI transactional Operation
 * @{
 */

/*!
 * @brief Send data in background.
 *
 * This function sends data. It is synchronous API.
 * If the HW buffer is full, the task is in the blocked state.
 *
 * @param handle The RTOS LPSCI handle.
 * @param buffer The pointer to buffer to send.
 * @param length The number of bytes to send.
 */
int LPSCI_RTOS_Send(lpsci_rtos_handle_t *handle, const uint8_t *buffer, uint32_t length);

/*!
 * @brief Receives data. It is synchronous API.
 *
 * This function receives data from LPSCI. If any data is immediately available
 * it is returned immediately and the number of bytes received.
 *
 * @param handle The RTOS LPSCI handle.
 * @param buffer The pointer to buffer where to write received data.
 * @param length The number of bytes to receive.
 * @param received The pointer to variable of size_t where the number of received data is filled.
 */
int LPSCI_RTOS_Receive(lpsci_rtos_handle_t *handle, uint8_t *buffer, uint32_t length, size_t *received);

/* @} */

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_LPSCI_RTOS_H__ */

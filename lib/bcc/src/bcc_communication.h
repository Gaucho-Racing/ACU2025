/*
 * Copyright 2016 - 2020 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * @file bcc_communication.h
 *
 * This file provides access to the basic low-level functions for both TPL and
 * SPI communication with MC33771C and MC33772C BCC devices.
 */

#ifndef __BCC_COMMUNICATION_H
#define __BCC_COMMUNICATION_H

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "bcc.h"

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @addtogroup function_group
 * @{
 */

/*!
 * @brief This function reads desired number of registers of the BCC device.
 * Intended for TPL mode only.
 *
 * In case of simultaneous read of more registers, address is incremented
 * in ascending manner.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid       Cluster Identification Address of the BCC device.
 * @param regAddr   Register address. See MC33771C.h and MC33772C.h header files
 *                  for possible values (MC3377*C_*_OFFSET macros).
 * @param regCnt    Number of registers to read.
 * @param regVal    Pointer to memory where content of selected 16 bit registers
 *                  will be stored.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Reg_ReadTpl(bcc_drv_config_t* const drvConfig,
    const bcc_cid_t cid, const uint8_t regAddr, const uint8_t regCnt,
    uint16_t* regVal);

/*!
 * @brief This function reads desired number of registers of the BCC device.
 * Intended for SPI mode only.
 *
 * In case of simultaneous read of more registers, address is incremented
 * in ascending manner.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid       Cluster Identification Address of the BCC device.
 * @param regAddr   Register address. See MC33771C.h and MC33772C.h header files
 *                  for possible values (MC3377*C_*_OFFSET macros).
 * @param regCnt    Number of registers to read.
 * @param regVal    Pointer to memory where content of selected 16 bit registers
 *                  will be stored.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Reg_ReadSpi(bcc_drv_config_t* const drvConfig,
    const bcc_cid_t cid, uint8_t regAddr, const uint8_t regCnt,
    uint16_t* regVal);

/*!
 * @brief This function writes a value to addressed register of the BCC device.
 * Intended for TPL mode only.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid       Cluster Identification Address of the BCC device.
 * @param regAddr   Register address. See MC33771C.h and MC33772C.h header files
 *                  for possible values (MC3377*C_*_OFFSET macros).
 * @param regVal    New value of selected register.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Reg_WriteTpl(bcc_drv_config_t* const drvConfig,
    const bcc_cid_t cid, const uint8_t regAddr, const uint16_t regVal);

/*!
 * @brief This function writes a value to addressed register of the BCC device.
 * Intended for SPI mode only.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid       Cluster Identification Address of the BCC device.
 * @param regAddr   Register address. See MC33771C.h and MC33772C.h header files
 *                  for possible values (MC3377*C_*_OFFSET macros).
 * @param regVal    New value of selected register.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Reg_WriteSpi(bcc_drv_config_t* const drvConfig,
    const bcc_cid_t cid, const uint8_t regAddr, const uint16_t regVal);

/*!
 * @brief This function writes a value to addressed register of all configured
 * BCC devices in the chain. Intended for TPL mode only.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param regAddr   Register address. See MC33771C.h and MC33772C.h header files
 *                  for possible values (MC3377*C_*_OFFSET macros).
 * @param regVal    New value of selected register.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Reg_WriteGlobalTpl(bcc_drv_config_t* const drvConfig,
    const uint8_t regAddr, const uint16_t regVal);

/*!
 * @brief This function sends a No Operation command to the BCC device.
 * Intended for TPL mode only.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid       Cluster Identification Address of the BCC device.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_SendNopTpl(bcc_drv_config_t* const drvConfig,
    const bcc_cid_t cid);

/*!
 * @brief This function sends a No Operation command to the BCC device.
 * Intended for SPI mode only.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid       Cluster Identification Address of the BCC device.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_SendNopSpi(bcc_drv_config_t* const drvConfig,
    const bcc_cid_t cid);

/*! @} */

#endif /* __BCC_COMMUNICATION_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/

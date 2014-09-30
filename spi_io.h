/*
 * Copyright 2011-2012 Ayla Networks, Inc.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of Ayla Networks, Inc.
 */
#ifndef __AYLA_SPI_IO_H__
#define __AYLA_SPI_IO_H__

#include "stm32f30x.h"
#include "stm32f3_discovery.h"
/*#include "stm32f30x_it.h"*/
#include "stm32f30x_spi.h"

#define HAVE_UTYPES
#include "utypes.h"
#include "endian.h"
#include "ayla_spi_mcu.h"
#include "props.h"
#include "serial_msg.h"

#define INTR_N_GPIO	GPIOB
#define	INTR_N_PIN	11
#define INTR_N_EXT_LINE	EXTI_Line11
#define INTR_N_IRQ	EXTI15_10_IRQn
#define INTR_N_PORT_SOURCE GPIO_PortSourceGPIOB
#define INTR_N_PIN_SOURCE GPIO_PinSource11

#define GP_SPI_GPIO	GPIOB
#define GP_SPI_NSS	12
#define GP_SPI_SCK	13
#define GP_SPI_MISO	14
#define GP_SPI_MOSI	15

/*
 * definitions needed from stm32f30x.h,
 * which can't be included because of typedef conflicts.
 */
#ifndef SPI_CR1_SPE
#define SPI_CR1_SPE	0x40
#define SPI_CR1_CRCNEXT	0x1000
#define SPI_CR1_CRCEN	0x2000

#define SPI_CR2_SSOE	0x04

#define SPI_SR_RXNE	0x01U
#define SPI_SR_TXE	0x02U
#define SPI_SR_CHSIDE	0x04U
#define SPI_SR_UDR	0x08U
#define SPI_SR_CRCERR	0x10U
#define	SPI_SR_MODF	0x20U
#define SPI_SR_OVR	0x40U
#define SPI_SR_BSY	0x80U
#endif

#ifndef SPI_I2SCFGR_I2SMOD
#define SPI_I2SCFGR_I2SMOD 0x800
#endif

void spi_init(void);
void spi_intr_init(void);

#endif /*  __AYLA_SPI_IO_H__ */

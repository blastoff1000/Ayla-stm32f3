/*
 * Copyright 2011-2013 Ayla Networks, Inc.  All rights reserved.
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
#include <string.h>
#include "mcu_io.h"
#include "spi_io.h"

static void spi_wait_idle(void)
{
	SPI_TypeDef *spi = SPI2;

	/*
	 * Make sure SPI I/O is complete first.
	 * The reference manual says to check for TXE followed by not busy.
	 */
	while (!(spi->SR & SPI_I2S_FLAG_TXE))
		;
	while (spi->SR & SPI_I2S_FLAG_BSY)
		;
}

void spi_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	/* Enable GPIOB clocks */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	GPIO_SetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_14 | GPIO_Pin_15);

	/* Configure PB12,13,14,15 pins for SPI2 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_5);

	/* Enable the SPI clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	/* Configure SPI2 for Master mode */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);

	SPI_RxFIFOThresholdConfig(SPI2, SPI_RxFIFOThreshold_QF);

	/* Clear flags and any RX data */
	SPI_I2S_ReceiveData16(SPI2);
	SPI_I2S_GetFlagStatus(SPI2, 0x00);

	SPI_Cmd(SPI2, ENABLE);

	/* Configure PB11 pin as input for INTR_N */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void spi_intr_init(void)
{
	NVIC_InitTypeDef nvic_init;
	EXTI_InitTypeDef exti_init;

	/*
	 * Set module interrupt line to cause ext interrupt on falling edge.
	 */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource11);
	exti_init.EXTI_Line = INTR_N_EXT_LINE;
	exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
	exti_init.EXTI_Trigger = EXTI_Trigger_Falling;
	exti_init.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti_init);

	nvic_init.NVIC_IRQChannel = INTR_N_IRQ;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 15;
	nvic_init.NVIC_IRQChannelSubPriority = 0;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);
}

void spi_slave_select(void)
{
	GP_SPI_GPIO->BRR = bit(GP_SPI_NSS);
}

void spi_slave_deselect(void)
{
	spi_wait_idle();
	GP_SPI_GPIO->BSRR = bit(GP_SPI_NSS);
}

static void spi_out(u8 byte)
{
	SPI_TypeDef *spi = SPI2;

	while (!(spi->SR & SPI_I2S_FLAG_TXE))
		;
	SPI_SendData8(SPI2, byte);
}

static u8 spi_in(void)
{
	SPI_TypeDef *spi = SPI2;
	u16 sr;

	for (;;) {
		sr = spi->SR;
		if (sr & SPI_I2S_FLAG_OVR) {
			(void)spi->DR;
			(void)spi->SR;
		}
		if (sr & SPI_I2S_FLAG_RXNE) {
			return SPI_ReceiveData8(SPI2);
		}
	}
}

u8 spi_io(u8 byte)
{
	spi_out(byte);
	return spi_in();
}

/*
 * Send and receive the last byte of a SPI message, followed by the CRC byte.
 *
 * NB:  The setting of CRCNEXT must happen immediately after the last
 * byte is sent.  See RM0008.
 */
u8 spi_io_crc(u8 byte)
{
	SPI_TypeDef *spi = SPI2;
	u16 sr;

	while (!(spi->SR & SPI_I2S_FLAG_TXE))
		;
	SPI_SendData8(SPI2, byte);
	spi->CR1 |= SPI_CR1_CRCNEXT;
	byte = spi_in();

	/*
	 * Wait for CRC byte.
	 */
	for (;;) {
		sr = spi->SR;
		if (sr & SPI_I2S_FLAG_OVR) {
			SPI_ReceiveData8(SPI2);
			(void)spi->SR;
		}
		if (sr & SPI_I2S_FLAG_RXNE) {
			SPI_ReceiveData8(SPI2);
			break;
		}
	}
	return byte;
}

void spi_crc_en(void)
{
	spi_wait_idle();
	SPI2->CR1 |= SPI_CR1_CRCEN;
}

int spi_crc_err(void)
{
	SPI_TypeDef *spi = SPI2;
	int err = 0;

	spi_wait_idle();
	if (spi->SR & SPI_SR_CRCERR) {
		spi->SR &= ~SPI_SR_CRCERR;
		err = 1;
	}
	spi->CR1 &= ~(SPI_CR1_CRCEN | SPI_CR1_CRCNEXT);
	return err;
}

/*
 * Interrupt handler for external interrupts 10 thru 15.
 * EXTI 11 is the module interrupt line, when asserted it means there is a
 * SPI message waiting to be received.
 */
void EXTI15_10_IRQHandler(void)
{
	intr_stats.intr_line++;
	if (!EXTI_GetITStatus(INTR_N_EXT_LINE)) {
		return;
	}
	EXTI_ClearITPendingBit(INTR_N_EXT_LINE);
}

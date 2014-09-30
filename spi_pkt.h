/*
 * Copyright 2012 Ayla Networks, Inc.  All rights reserved.
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
#ifndef __SPI_PKT_H__
#define __SPI_PKT_H__

/*
 * In spi_pkt.c.
 */
void *spi_tx_buf_get(size_t len);
void *spi_tx_buf_get_for_props(size_t len);
void spi_tx_buf_trim(size_t len);
int spi_tx(void);
void spi_poll(void);
int spi_is_ads_busy(void);

int spi_tx_buf_set(u8 * buf, size_t len);
void spi_tx_crc_ctrl(int on);
void spi_tx_wait(void);

/*
 * Provided by platform.
 */

u8 spi_io(u8);
u8 spi_io_crc(u8);

void spi_slave_select(void);
void spi_slave_deselect(void);

void spi_crc_en(void);
int spi_crc_err(void);
			 
int spi_rx_pending(void);             

/*
 * In spi_ping.c.
 */
void spi_ping_rx(void *buf, size_t len);
int spi_ping_test(size_t len, u16 pattern);

#endif /* __SPI_PKT_H__ */

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
#include <string.h>
#include <stdarg.h>
#include <limits.h>

#include <stm32f30x.h>

#define HAVE_UTYPES
#include "utypes.h"
#include "ayla_proto_mcu.h"
#include "conf_token.h"
#include "spi_pkt.h"
#include "props.h"
#include "serial_msg.h"
#include "endian.h"
#ifdef DEMO_TIME_LIB
#include "sched.h"
#endif /* DEMO_TIME_LIB */

#include "conf_internal.h"

/*
 * This code demonstrates reading some configuration items from the Ayla module.
 * For now it just reads the DSN (device serial number) and the model.
 */
#define CONF_DEV_SN_MAX		20	/* max string length for DSN */
#define CONF_DEV_MODEL_MAX	20	/* max string length for Ayla model */
#define CONF_PATH_MAX		8	/* maximum depth of config path */

#ifdef DEMO_REG_TOKEN
#define CONF_REG_TOKEN_MAX	8	/* max length of registration token */

char conf_reg_token[CONF_REG_TOKEN_MAX];
static u8 conf_reg_token_req;		/* true if reg token requested */
#endif /* DEMO_REG_TOKEN */

char conf_dsn[CONF_DEV_SN_MAX];
char conf_model[CONF_DEV_MODEL_MAX];
/* handler for OTA notification */
void (*conf_ota_cb)(int, void *buf, size_t len);
static u8 conf_mod_ota_cmd;	/* non-zero if module OTA can go ahead */

static void (*conf_cb)(void *buf, size_t len);	/* handler for response */

#define STATS(x)	do { (conf_stats.x++); } while (0)

/*
 * Optional debug counters for various conditions.
 * These can be deleted to save space but may help debugging.
 */
struct conf_stats {
	u16 tx_get;
	u16 tx_set;
	u16 rx_nak;
	u16 rx_resp;
} conf_stats;

/*
 * Get the first TLV of the specified type from a received packet.
 */
struct ayla_tlv *tlv_get(enum ayla_tlv_type type, void *buf, size_t len)
{
	struct ayla_cmd *cmd;
	struct ayla_tlv *tlv;
	size_t rlen;
	size_t tlen;

	if (len < sizeof(*cmd)) {
		return NULL;
	}
	rlen = len - sizeof(*cmd);
	cmd = buf;
	tlv = (struct ayla_tlv *)(cmd + 1);

	while (rlen >= sizeof(*tlv)) {
		tlen = sizeof(*tlv) + tlv->len;
		if (tlen > rlen) {
			return NULL;
		}
		if (tlv->type == type) {
			return tlv;
		}
		rlen -= tlen;
		tlv = (struct ayla_tlv *)((char *)tlv + tlen);
	}
	return NULL;
}

/*
 * Place TLV of a given type to a buffer.
 */
size_t tlv_put(void *buf, enum ayla_tlv_type type, const void *val, size_t len)
{
	struct ayla_tlv *tlv;

	if ((len > 0xff && type != ATLV_FILE) || len > 0x7fff) {
		return 0;
	}
	tlv = buf;
	tlv->type = type | (len >> 8);
	tlv->len = len;
	memcpy(tlv + 1, val, len);
	return len + sizeof(*tlv);
}

/*
 * Send go-ahead message for module/MCU OTA.
 */
static void conf_mod_ota_send(void)
{
	struct ayla_cmd *cmd;

	if (!conf_mod_ota_cmd) {
		return;
	}

	cmd = serial_tx_buf_get(sizeof(*cmd), 0, NULL);
	if (!cmd) {
		return;
	}
	cmd->protocol = ASPI_PROTO_CMD;
	cmd->opcode = ACMD_OTA_CMD;
	conf_mod_ota_cmd = 0;

	put_ua_be16((void *)&cmd->req_id, tlv_req_id);
	tlv_req_id++;

	serial_tx_buf_send(0, tlv_req_id - 1);
}

/*
 * Set flag to send module OTA go-ahead.
 */
void conf_mod_ota_go(void)
{
	conf_mod_ota_cmd = 1;
}

/*
 * Handle response for request DSN.
 */
static void conf_dsn_rx(void *buf, size_t len)
{
	struct ayla_tlv *tlv;

	tlv = tlv_get(ATLV_UTF8, buf, len);
	if (!tlv) {
		return;
	}
	if (tlv->len >= sizeof(conf_dsn)) {
		return;
	}
	memcpy(conf_dsn, tlv + 1, tlv->len);
	conf_dsn[tlv->len] = '\0';
}

/*
 * Handle response for request model
 */
static void conf_model_rx(void *buf, size_t len)
{
	struct ayla_tlv *tlv;

	tlv = tlv_get(ATLV_UTF8, buf, len);
	if (!tlv) {
		return;
	}
	if (tlv->len >= sizeof(conf_model)) {
		return;
	}
	memcpy(conf_model, tlv + 1, tlv->len);
	conf_model[tlv->len] = '\0';
}

#ifdef DEMO_TIME_LIB

/*
 * Put up to 4 UTF-8 bytes to a buffer from a int value up to 0x10ffff.
 *
 * 4-byte UTF-8 can handle up to 21-bit values, but Unicode further restricts
 * this to values up to 0x10ffff.
 */
int utf8_put_wchar(u8 *bp, u32 val)
{
	if (val > 0x10ffff) {
		return -1;
	}
	if (val >= 0x10000) {
		/* 21 bits */
		bp[0] = 0xf0 + ((val >> 18) & 7);
		bp[1] = 0x80 + ((val >> 12) & 0x3f);
		bp[2] = 0x80 + ((val >> 6) & 0x3f);
		bp[3] = 0x80 + (val & 0x3f);
		return 4;
	}
	if (val >= 0x800) {
		/* 16 bits */
		bp[0] = 0xe0 + ((val >> 12) & 0xf);
		bp[1] = 0x80 + ((val >> 6) & 0x3f);
		bp[2] = 0x80 + (val & 0x3f);
		return 3;
	}
	if (val >= 0x80) {
		/* 11 bits */
		bp[0] = 0xc0 + ((val >> 6) & 0x1f);
		bp[1] = 0x80 + (val & 0x3f);
		return 2;
	}
	/* 7 bits */
	bp[0] = val;
	return 1;
}

/*
 * Get next UTF-8 token from a buffer.
 * Returns length, or zero if invalid UTF-8 code encountered.
 */
static size_t utf8_get(u32 *result, u8 *buf, size_t len)
{
	u32 val = 0;
	size_t rlen = 0;
	u8 test[4];
	int i;
	u8 c;

	if (len == 0) {
		return 0;
	}
	c = buf[0];
	if (c < 0x80) {
		*result = c;
		return 1;
	}
	if ((c & 0xf8) == 0xf0) {
		val = c & 7;
		rlen = 4;
	} else if ((c & 0xf0) == 0xe0) {
		val = c & 0xf;
		rlen = 3;
	} else if ((c & 0xe0) == 0xc0) {
		if (c == 0xc0 || c == 0xc1) {
			return 0;
		}
		val = c & 0x1f;
		rlen = 2;
	} else if ((c & 0xc0) == 0x80) {
		return 0;
	}
	if (len < rlen) {
		return 0;
	}
	for (i = 1; i < rlen; i++) {
		c = buf[i];
		if ((c & 0xc0) != 0x80) {
			return 0;
		}
		val = (val << 6) | (c & 0x3f);
	}

	/*
	 * Check for over-long coding, which is invalid.
	 */
	if (utf8_put_wchar(test, val) != rlen) {
		return 0;
	}
	*result = val;
	return rlen;
}

/*
 * Get multiple UTF-8 wide characters from a buffer.
 * Returns the number of characters put in result, or -1 on error.
 */
static int utf8_gets(u32 *result, int rcount, u8 *buf, size_t len)
{
	int tlen;
	int count;

	for (count = 0; count < rcount && len > 0; count++) {
		tlen = utf8_get(&result[count], buf, len);
		if (tlen <= 0) {
			return -1;
		}
		len -= tlen;
		buf += tlen;
	}
	if (len > 0) {
		return -1;
	}
	return count;
}

/*
 * Handle configuration updates from module
 */
static void conf_update(void *buf, size_t len)
{
	struct ayla_cmd *cmd;
	struct ayla_tlv *tlv;
	size_t rlen = len;
	size_t tlen;
	void *vp;
	int plen = 0;
	int i;
	u32 path[CONF_PATH_MAX];
	enum conf_token conf_path[CONF_PATH_MAX];
	u8 time_info_updated = 0;

	if (len < sizeof(*cmd)) {
		return;
	}
	rlen -= sizeof(*cmd);
	cmd = buf;
	tlv = (struct ayla_tlv *)(cmd + 1);

	while (rlen > 0) {
		if (rlen < sizeof(*tlv)) {
			return;
		}
		rlen -= sizeof(*tlv);
		tlen = tlv->len;
		if (rlen < tlen) {
			return;
		}
		vp = tlv + 1;
		rlen -= tlen;

		if (tlv->type != ATLV_CONF) {
			return; /* config path not given */
		}
		plen = utf8_gets(path, CONF_PATH_MAX, vp, tlen);
		if (plen < 1) {
			return; /* conf path reading error */
		}
		/*
		 * The size of enum conf_token may be smaller than u32.
		 * The sizeof() expressions below are completely
		 * evalutated at compile time.
		 */
		for (i = 0; i < plen && i < CONF_PATH_MAX; i++) {
			if ((sizeof(conf_path[0]) == 1 && path[i] > MAX_U8) ||
			    (sizeof(conf_path[0]) == 2 && path[i] > MAX_U16)) {
				return;	/* bad conf path */
			} else {
				conf_path[i] = (enum conf_token)path[i];
			}
		}
		if (rlen < sizeof(*tlv)) {
			return;
		}
		tlv = (struct ayla_tlv *)((char *)vp + tlen);
		vp = tlv + 1;
		rlen -= sizeof(*tlv);
		tlen = tlv->len;
		rlen -= tlen;
		if (conf_path[0] == CT_sys) {
			switch (conf_path[1]) {
			case CT_time:
			case CT_timezone_valid:
			case CT_timezone:
			case CT_dst_valid:
			case CT_dst_active:
			case CT_dst_change:
				if (feature_mask & MCU_TIME_SUBSCRIPTION) {
					time_info_updated = 1;
					sched_update_time_info(conf_path[1],
					    tlv);
				}
				break;
			default:
				break;
			}
		}
		tlv = (struct ayla_tlv *)((char *)vp + tlen);
	}
	if (time_info_updated) {
		sched_run_all(NULL);
	}
}
#endif /* DEMO_TIME_LIB */

/*
 * Send GET_CONF request.
 */
int conf_read(const enum conf_token * const tokens, unsigned int ntokens,
    void (*cb)(void *buf, size_t len))
{
	struct ayla_cmd *cmd;
	struct ayla_tlv *tk;
	size_t len;

	len = sizeof(*cmd) + sizeof(*tk) + ntokens;

	cmd = serial_tx_buf_get(len, 0, NULL);
	if (!cmd) {
		return -1;
	}
	cmd->protocol = ASPI_PROTO_CMD;
	cmd->opcode = ACMD_GET_CONF;
	put_ua_be16((void *)&cmd->req_id, tlv_req_id);
	tlv_req_id++;

	tk = (struct ayla_tlv *)(cmd + 1);
	tk->type = ATLV_CONF;
	tk->len = ntokens;
	memcpy(tk + 1, tokens, ntokens);

	STATS(tx_get);
	conf_cb = cb;

	return serial_tx_buf_send(0, tlv_req_id - 1);
}

/*
 * enum conf_token conf_tokens[] = {CT_token0, CT_token1};
 * int ntokens = sizeof (conf_tokens) / sizeof (conf_tokens[0];
 * int val_type;
 * int val;
 * int val_sz = sizeof (val);
 *
 * conf_write(conf_tokens, ntokens, val_type, val, val_sz, ...);
 *
 * Arguments to conf_write are repeated in a set of 5,
 * 1. config tokens,
 * 2. number of config tokens,
 * 3. config variable type id
 * 4. pointer to value conf variable should be set to
 * 5. size of the conf variable value
 */
int conf_write(int cnt, ...)
{
	va_list ap;
	enum conf_token *tokens;
	int ntokens;
	enum ayla_tlv_type val_type;
	void *val;
	int val_len;
	int tot_len = 0;
	u8 *ptr;
	struct ayla_cmd *cmd;
	int i;

	/*
	 * First calculate size of the message and validate arguments.
	 */
	va_start(ap, cnt);
	tot_len = 0;
	for (i = 0; i < cnt; i++) {
		tokens = va_arg(ap, enum conf_token *);
		ntokens = va_arg(ap, int);
		val_type = (enum ayla_tlv_type)va_arg(ap, int);
		val = va_arg(ap, void *);
		val_len = va_arg(ap, int);
		tot_len += sizeof(struct ayla_tlv) + ntokens +
		    sizeof(struct ayla_tlv) + val_len;
	}
	tot_len += sizeof(struct ayla_cmd);
	va_end(ap);

	/*
	 * Then construct the message.
	 */
	cmd = serial_tx_buf_get(tot_len, 0, NULL);
	if (!cmd) {
		return -1;
	}
	cmd->protocol = ASPI_PROTO_CMD;
	cmd->opcode = ACMD_SET_CONF;
	put_ua_be16((void *)&cmd->req_id, tlv_req_id);
	tlv_req_id++;

	va_start(ap, cnt);
	ptr = (u8 *)(cmd + 1);

	for (i = 0; i < cnt; i++) {
		tokens = va_arg(ap, enum conf_token *);
		ntokens = va_arg(ap, int);

		ptr += tlv_put(ptr, ATLV_CONF, tokens, ntokens);

		val_type = (enum ayla_tlv_type)va_arg(ap, int);
		val = va_arg(ap, void *);
		val_len = va_arg(ap, int);

		ptr += tlv_put(ptr, val_type, val, val_len);
	}

	STATS(tx_set);
	return serial_tx_buf_send(0, tlv_req_id - 1);
}

/*
 * Tell module to persist it's current running configuration.
 */
void conf_save(void)
{
	struct ayla_cmd *cmd;

	cmd = serial_tx_buf_get(sizeof(struct ayla_cmd), 0, NULL);
	if (!cmd) {
		return;
	}
	cmd->protocol = ASPI_PROTO_CMD;
	cmd->opcode = ACMD_SAVE_CONF;
	put_ua_be16((void *)&cmd->req_id, tlv_req_id);
	tlv_req_id++;
	serial_tx_buf_send(0, tlv_req_id - 1);
}

/*
 * Receive a configuration item.
 */
void conf_rx(void *buf, size_t len)
{
	struct ayla_cmd *cmd;
	void (*func)(void *buf, size_t len);

	if (len < sizeof(*cmd)) {
		return;
	}
	cmd = buf;
	switch (cmd->opcode) {
	case ACMD_RESP:
		STATS(rx_resp);
		if (conf_cb) {
			func = conf_cb;
			conf_cb = NULL;
			func(buf, len);
		}
		break;
	case ACMD_NAK:
		STATS(rx_nak);
		if (conf_cb) {
			func = conf_cb;
			conf_cb = NULL;
			func(buf, len);
		}
		break;
	case ACMD_OTA_STAT:
		if (conf_ota_cb) {
			conf_ota_cb(1, buf, len);
		} else {
			conf_mod_ota_go();
		}
		break;
#ifdef AYLA_HOST_OTA
	case ACMD_MCU_OTA:
		conf_mcu_ota_start(buf, len);
		break;
	case ACMD_MCU_OTA_LOAD:
		conf_mcu_ota_load(buf, len);
		break;
	case ACMD_MCU_OTA_BOOT:
		conf_mcu_ota_boot(buf, len);
		break;
#endif
#ifdef DEMO_TIME_LIB
	case ACMD_CONF_UPDATE:
		conf_update(buf, len);
		break;
#endif
	default:
		break;
	}
}

#ifdef DEMO_REG_TOKEN

/*
 * Handle response from request for registration token.
 */
static void conf_reg_token_rx(void *buf, size_t len)
{
	struct ayla_tlv *tlv;

	tlv = tlv_get(ATLV_UTF8, buf, len);
	if (!tlv) {
		return;
	}
	if (tlv->len >= sizeof(conf_reg_token)) {
		return;
	}
	memcpy(conf_reg_token, tlv + 1, tlv->len);
	conf_reg_token[tlv->len] = '\0';
}

/*
 * Request new registration token if it has not been already requested.
 * This will clear the registration token on the module until it is re-fetched
 * from the device service.
 * Get the registration token if it is not already set.
 */
static void conf_reg_poll(void)
{
	static enum conf_token reg_path[] = {CT_client, CT_reg, CT_start};
	u8 val = 1;

	if (!(valid_dest_mask & ADS_BIT)) {
		return;
	}
	if (!conf_reg_token_req) {
		if (conf_write(1, reg_path, 3, ATLV_BOOL, &val, sizeof(val))) {
			return;
		}
		conf_reg_token_req = 1;
	}
	if (conf_reg_token[0] == '\0') {
		conf_read(reg_path, 2, conf_reg_token_rx);
	}
}
#endif /* DEMO_REG_TOKEN */

void conf_poll(void)
{
	const enum conf_token devid_tokens[] = { CT_sys, CT_dev_id };
	const enum conf_token model_tokens[] = { CT_sys, CT_model };

	if (conf_cb) {
		return;			/* a request is pending */
	}
	conf_mod_ota_send();
#ifdef AYLA_HOST_OTA
	conf_mcu_ota_send();
#endif
	if (conf_dsn[0] == '\0') {
		if (conf_read(devid_tokens, 2, conf_dsn_rx)) {
			return;		/* request could not be sent */
		}
		return;
	}
	if (conf_model[0] == '\0') {
		if (conf_read(model_tokens, 2, conf_model_rx)) {
			return;		/* request could not be sent */
		}
		return;
	}
#ifdef DEMO_REG_TOKEN
	conf_reg_poll();
#endif
}

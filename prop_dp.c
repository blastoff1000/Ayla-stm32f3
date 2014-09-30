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

#include <stm32f30x.h>

#define HAVE_UTYPES
#include "utypes.h"
#include "ayla_proto_mcu.h"
#include "props.h"
#include "serial_msg.h"
#include "spi_pkt.h"
#include "endian.h"

struct prop_dp *prop_dp_active;

#define PROP_DP_STATS

#ifndef PROP_DP_STATS
#define STATS(x)
#else
#define STATS(x)	do { (prop_dp_stats.x++); } while (0)

/*
 * Optional debug counters for various conditions.
 * These can be deleted to save space but may help debugging.
 */
struct prop_dp_stats {
	u16 rx_resp;
	u16 rx_not_active;
	u16 rx_bad_req_id;
	u16 rx_len_err;

	u16 rx_tlv_len_err;
	u16 rx_no_loc;
	u16 rx_bad_loc;
	u16 rx_wrong_type;

	u16 rx_read_only;
	u16 rx_unk_tlv;
} prop_dp_stats;
#endif /* PROP_DP_STATS */

static void prop_dp_done(struct prop_dp *dp)
{
	dp->state = DS_IDLE;
	dp->prop->send_mask = 0;
	if (prop_dp_active == dp) {
		prop_dp_active = NULL;
	}
}

static void prop_dp_fatal_err(struct prop_dp *dp)
{
	prop_dp_done(dp);
}

static void prop_dp_create_tx(struct prop_dp *dp)
{
	struct prop *prop;
	struct ayla_cmd *cmd;
	struct ayla_tlv *name;
	u16 name_len;
	int tot_len;

	prop = dp->prop;
	name_len = strlen(prop->name);
	tot_len = sizeof(*cmd) + sizeof(*name) + name_len;
	if (tot_len > ASPI_LEN_MAX) {
		dp->prop->send_mask = 0;
		return;
	}

	cmd = serial_tx_buf_get(tot_len, 1, NULL);
	if (!cmd) {
		return;
	}

	dp->state = DS_CREATE_RESP;
	dp->prop->send_mask = 0;

	cmd->protocol = ASPI_PROTO_DATA;
	cmd->opcode = AD_DP_CREATE;
	dp->req_id = tlv_req_id;
	put_ua_be16((void *)&cmd->req_id, tlv_req_id);
	tlv_req_id++;

	name = (struct ayla_tlv *)(cmd + 1);
	name->type = ATLV_NAME;
	name->len = name_len;
	memcpy(name + 1, prop->name, name_len);

	serial_tx_buf_send(1, dp->req_id);
}

/*
 * Send status request for datapoint put.
 * This is for recovery from an interrupted data point operation where a
 * datapoint operation was in progress.
 */
static void prop_dp_status_tx(struct prop_dp *dp)
{
	struct prop *prop;
	struct ayla_cmd *cmd;
	struct ayla_tlv *tlv;
	int tot_len;

	prop = dp->prop;
	tot_len = sizeof(*cmd) + sizeof(*tlv) + dp->loc_len;
	if (tot_len > ASPI_LEN_MAX) {
		prop_dp_fatal_err(dp);
		return;
	}

	cmd = serial_tx_buf_get(tot_len, 1, NULL);
	if (!cmd) {
		prop_dp_fatal_err(dp);
		return;
	}

	dp->state = DS_STATUS_WAIT;
	prop->send_mask = 0;

	cmd->protocol = ASPI_PROTO_DATA;
	cmd->opcode = AD_DP_STATUS;
	dp->req_id = tlv_req_id;
	put_ua_be16((void *)&cmd->req_id, tlv_req_id);
	tlv_req_id++;

	tlv = (struct ayla_tlv *)(cmd + 1);
	tlv->type = ATLV_LOC;
	tlv->len = dp->loc_len;;
	memcpy(tlv + 1, dp->loc, dp->loc_len);

	serial_tx_buf_send(1, dp->req_id);
}

static void prop_dp_data_tx(struct prop_dp *dp)
{
	struct ayla_cmd *cmd;
	struct ayla_tlv *tlv;
	s32 val_len;
	s32 old_len;
	u32 max_len;
	size_t tot_len;
	size_t next_off;
	int eof;

	tot_len = sizeof(*cmd) + sizeof(*tlv) + dp->loc_len;
	val_len = dp->tot_len;
	next_off = dp->next_off;
	if (next_off || val_len) {
		tot_len += sizeof(*tlv) + sizeof(u32);
	}

	/*
	 * determine how much of the value we can send.
	 */
	eof = 0;
	if (!val_len) {
		val_len = MAX_U8;
	} else {
		max_len = val_len - next_off;
		if (max_len > MAX_U8) {
			max_len = MAX_U8;
		}
		if (val_len > max_len) {
			val_len = max_len;
		}
		if (next_off + val_len == dp->tot_len &&
		    max_len + tot_len + sizeof(*tlv) < ASPI_LEN_MAX) {
			tot_len += sizeof(*tlv);
			eof = 1;
		}
	}
	tot_len += sizeof(*tlv) + val_len;

	cmd = serial_tx_buf_get(tot_len, 1, NULL);
	if (!cmd) {
		return;
	}

	cmd->protocol = ASPI_PROTO_DATA;
	cmd->opcode = AD_DP_SEND;
	put_ua_be16((void *)&cmd->req_id, dp->req_id);

	tlv = (struct ayla_tlv *)(cmd + 1);
	tlv->type = ATLV_LOC;
	tlv->len = dp->loc_len;
	memcpy(tlv + 1, dp->loc, dp->loc_len);

	if (next_off) {
		tlv = (struct ayla_tlv *)((u8 *)(tlv + 1) + tlv->len);
		tlv->type = ATLV_OFF;
		tlv->len = sizeof(u32);
		put_ua_be32(tlv + 1, next_off);
	} else if (dp->tot_len) {
		tlv = (struct ayla_tlv *)((u8 *)(tlv + 1) + tlv->len);
		tlv->type = ATLV_LEN;
		tlv->len = sizeof(u32);
		put_ua_be32(tlv + 1, dp->tot_len);
	}

	tlv = (struct ayla_tlv *)((u8 *)(tlv + 1) + tlv->len);
	tlv->type = ATLV_BIN;
	old_len = val_len;
	val_len = dp->prop_get(dp->prop, tlv + 1, val_len);
	if (val_len < 0) {		/* XXX can't happen? */
		prop_dp_fatal_err(dp);
		serial_tx_cancel();
		return;
	}
	tlv->len = (u8)val_len;
	if (val_len < old_len) {
		if (!eof) {
			tot_len += sizeof(*tlv);
			eof = 1;
		}
		spi_tx_buf_trim(tot_len + val_len - old_len);
	}
	dp->next_off = next_off + val_len;

	if (eof) {
		tlv = (struct ayla_tlv *)((u8 *)(tlv + 1) + tlv->len);
		tlv->type = ATLV_EOF;
		tlv->len = 0;
	}

	/*
	 * Send the packet.
	 */
	serial_tx_buf_send(1, dp->req_id);

	if (eof) {
		prop_dp_done(dp);
	}
}

/*
 * Send a request for a long data point.
 * The request contains a location TLV, and possibly an offset TLV.
 */
static void prop_dp_request(struct prop_dp *dp)
{
	struct prop *prop;
	struct ayla_cmd *cmd;
	struct ayla_tlv *tlv;
	int tot_len;

	prop = dp->prop;
	tot_len = sizeof(*cmd) + sizeof(*tlv) + dp->loc_len;
	if (dp->next_off) {
		tot_len += sizeof(*tlv) + sizeof(u32);
	}
	if (tot_len > ASPI_LEN_MAX) {
		prop_dp_fatal_err(dp);
		return;
	}

	cmd = serial_tx_buf_get(tot_len, 1, NULL);
	if (!cmd) {
		return;
	}

	dp->state = DS_RECV;
	prop->send_mask = 0;

	cmd->protocol = ASPI_PROTO_DATA;
	cmd->opcode = AD_DP_REQ;
	dp->req_id = tlv_req_id;
	put_ua_be16((void *)&cmd->req_id, tlv_req_id);
	tlv_req_id++;

	tlv = (struct ayla_tlv *)(cmd + 1);
	tlv->type = ATLV_LOC;
	tlv->len = dp->loc_len;;
	memcpy(tlv + 1, dp->loc, dp->loc_len);

	if (dp->next_off) {
		tlv = (struct ayla_tlv *)((u8 *)(tlv + 1) + tlv->len);
		tlv->type = ATLV_OFF;
		put_ua_be32((void *)(tlv + 1), dp->next_off);
	}

	serial_tx_buf_send(1, dp->req_id);
}

/*
 * Send Datapoint Fetched operation to the module.
 */
static void prop_dp_fetched(struct prop_dp *dp)
{
	struct ayla_cmd *cmd;
	struct ayla_tlv *tlv;
	int tot_len;

	tot_len = sizeof(*cmd) + sizeof(*tlv) + dp->loc_len;
	if (tot_len > ASPI_LEN_MAX) {
		prop_dp_fatal_err(dp);
		return;
	}

	cmd = serial_tx_buf_get(tot_len, 1, NULL);
	if (!cmd) {
		return;
	}

	cmd->protocol = ASPI_PROTO_DATA;
	cmd->opcode = AD_DP_FETCHED;
	dp->req_id = tlv_req_id;
	put_ua_be16((void *)&cmd->req_id, tlv_req_id);
	tlv_req_id++;

	tlv = (struct ayla_tlv *)(cmd + 1);
	tlv->type = ATLV_LOC;
	tlv->len = dp->loc_len;;
	memcpy(tlv + 1, dp->loc, dp->loc_len);

	serial_tx_buf_send(1, dp->req_id);
	prop_dp_done(dp);
}

static void prop_dp_step(struct prop_dp *dp)
{
	switch (dp->state) {

	case DS_IDLE:
		prop_dp_done(dp);
		break;

	/*
	 * CREATE: send "datapoint create" command.
	 * This will callback to set the location for the datapoint.
 	 * Turn off send_mask until the location response is received.
	 */
	case DS_CREATE:
		prop_dp_create_tx(dp);
		break;

	case DS_CREATE_RESP:
	case DS_STATUS_WAIT:
	case DS_RECV:
		break;

	case DS_STATUS:
		prop_dp_status_tx(dp);
		break;

	case DS_SEND:
		prop_dp_data_tx(dp);
		break;

	case DS_REQUEST:
		prop_dp_request(dp);
		break;

	case DS_FETCHED:
		prop_dp_fetched(dp);
		break;
	}
}

u8 prop_dp_last_err;		/* debug */

/*
 * Receive NAK (negative acknowlegement) for a datapoint operation.
 */
int prop_dp_nak(struct ayla_cmd *cmd, void *buf, size_t len)
{
	struct prop_dp *dp = prop_dp_active;
	struct ayla_tlv *tlv;
	u8 error;

	if (!dp) {
		return -2;
	}
	if (ntohs(cmd->req_id) != dp->req_id) {
		return -1;
	}
	tlv = buf;
	if (len < sizeof(*tlv) || len < sizeof(*tlv) + tlv->len) {
		return -1;		/* XXX invalid NAK */
	}
	if (tlv->type != ATLV_ERR) {
		return 0; 
	}
	error = *(u8 *)(tlv + 1);
	prop_dp_last_err = error;
	switch (dp->state) {
	case DS_SEND:
		dp->state = DS_STATUS;
		dp->prop->send_mask = 1;
		break;
	default:
		prop_dp_fatal_err(dp);
		break;
	}
	return 0;
}


/*
 * Receive response from datapoint create request.
 * Caller has verified our state and the request ID.
 */
static void prop_dp_create_resp(struct prop_dp *dp, void *buf, size_t len)
{
	struct ayla_tlv *loc;

	loc = buf;
	if (len < sizeof(*loc) || len < sizeof(*loc) + loc->len) {
		return; 			/* XXX bad response */
	}
	if (loc->type != ATLV_LOC) {
		prop_dp_active = NULL;		/* XXX failed */
		return;	/* XXX */
	}
	if (loc->len > sizeof(dp->loc) - 1) {
		prop_dp_fatal_err(dp);
		return;
	}
	memcpy(dp->loc, loc + 1, loc->len);
	dp->loc[loc->len] = '\0';
	dp->loc_len = loc->len;
	dp->state = DS_SEND;
	dp->next_off = 0;
	dp->req_id = tlv_req_id++;		/* req ID for all sends */
	dp->prop->send_mask = 1;
}

/*
 * Receive response from status request.
 * Caller has verified our state and the request ID.
 */
static void prop_dp_status_resp(struct prop_dp *dp, void *buf, size_t len)
{
	struct ayla_tlv *tlv, *off, *loc;

	off = loc = NULL;
	tlv = buf;

	while (len >= sizeof(*tlv) && len >= sizeof(*tlv) + tlv->len) {
		if (tlv->type == ATLV_LOC) {
			loc = tlv;
		} else if (tlv->type == ATLV_OFF) {
			off = tlv;
		}
		tlv = (struct ayla_tlv *)((char *)tlv + sizeof(*tlv) +
		    tlv->len);
	}
	if (!off || !loc) {
		prop_dp_fatal_err(dp);
		return;
	}

	/* Match location */
	if (loc->len != dp->loc_len || memcmp(loc + 1, dp->loc, dp->loc_len)) {
		prop_dp_fatal_err(dp);
		return;
	}
	if (off->len != sizeof(u32)) {
		prop_dp_fatal_err(dp);
		return;
	}
	dp->next_off = get_ua_be32(off + 1);
	dp->state = DS_SEND;
	dp->req_id = tlv_req_id++;		/* new req ID for all sends */
	dp->prop->send_mask = 1;
}

/*
 * Receive response for data point request.
 */
static void prop_dp_rx(struct prop_dp *dp, void *buf, size_t len)
{
	struct prop *prop;
	struct ayla_tlv *tlv = (struct ayla_tlv *)buf;
	struct ayla_tlv *loc = NULL;
	size_t offset = 0;
	u8 eof = 0;
	void *valp = NULL;
	size_t val_len = 0;
	size_t rlen = len;
	size_t tlen;

	prop = dp->prop;
	while (rlen > 0) {
		if (rlen < sizeof(*tlv)) {
			STATS(rx_len_err);
			return;
		}

		tlen = tlv->len;
		if (tlen + sizeof(*tlv) > rlen) {
			STATS(rx_tlv_len_err);
			return;
		}

		switch (tlv->type) {
		case ATLV_LOC:
			loc = tlv;
			break;
		case ATLV_BIN:
			valp = (void *)(tlv + 1);
			val_len = tlv->len;
			break;
		case ATLV_OFF:
			if (tlv->len != sizeof(u32)) {
				STATS(rx_tlv_len_err);
				return;
			}
			offset = get_ua_be32((be32 *)(tlv + 1));
			break;
		case ATLV_EOF:
			eof = 1;
			break;
		default:
			STATS(rx_unk_tlv);
			break;
		}
		tlv = (void *)((char *)(tlv + 1) + tlen);
		rlen -= sizeof(*tlv) + tlen;
	}

	if (!loc) {
		STATS(rx_no_loc);
		return;
	}
	if (loc->len != dp->loc_len || memcmp(loc + 1, dp->loc, dp->loc_len)) {
		STATS(rx_bad_loc);
		return;
	}
	if (!dp->prop_set) {
		STATS(rx_read_only);
		return;
	}
	if (eof) {
		dp->state = DS_FETCHED;
		prop->send_mask = 1;
	}
	dp->prop_set(prop, offset, valp, val_len, eof);
}

/*
 * Receive response from datapoint create or status request.
 */
void prop_dp_resp(struct ayla_cmd *cmd, void *buf, size_t len)
{
	struct prop_dp *dp = prop_dp_active;

	STATS(rx_resp);
	if (!dp) {
		STATS(rx_not_active);
		return;
	}
	if (ntohs(cmd->req_id) != dp->req_id) {
		STATS(rx_bad_req_id);
		return;
	}
	switch (dp->state) {
	case DS_CREATE_RESP:
		prop_dp_create_resp(dp, buf, len);
		break;
	case DS_STATUS_WAIT:
		prop_dp_status_resp(dp, buf, len);
		break;
	case DS_RECV:
		prop_dp_rx(dp, buf, len);
		break;
	default:
		break;
	}
}

/*
 * Initiate or continue send or receive of large data point.
 * Never return 0, so prop->send_mask doesn't get cleared by caller.
 * We clear it ourselves after finishing the send.
 */
int prop_dp_send(struct prop *prop, void *arg)
{
	struct prop_dp *dp = prop->arg;

	dp->prop = prop;
	if (!prop_dp_active) {
		prop_dp_active = dp;
	}
	prop_dp_step(prop_dp_active);
	return -1;
}

/*
 * The set handler for large data points.
 * This receives the location and starts the get state machine.
 * Ignore setting with same location, since that is presumably already
 * fetched or in progress.
 */
void prop_dp_set(struct prop *prop, void *arg, void *val, size_t len)
{
	struct prop_dp *dp = arg;

	dp->prop = prop;
	if (len > sizeof(dp->loc) - 1) {
		return;
	}
	if (len == dp->loc_len && !memcmp(dp->loc, val, len)) {
		return;
	}
	memcpy(dp->loc, val, len);
	dp->loc[len] = '\0';
	dp->loc_len = len;
	dp->state = DS_REQUEST;
	dp->next_off = 0;
	dp->tot_len = 0;
	dp->prop->send_mask = 1;
}

void prop_dp_start_send(struct prop *prop, struct prop_dp *dp, u32 len)
{
	dp->prop = prop;
	if (dp->state == DS_IDLE) {
		dp->state = DS_CREATE;
		dp->tot_len = len;
		dp->next_off = 0;
		prop->send_mask = 1;
	}
}

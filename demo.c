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
#include "conf_token.h"
#ifdef DEMO_STREAM
#include "demo_stream.h"
#endif /* DEMO_STREAM */
#ifdef DEMO_IR
#include "ir_io.h"
#endif /* DEMO_IR */

#ifdef DEMO_STREAM
#define VERSION "demo_dp 1.2"
#else
#define VERSION "demo 1.2"
#endif

#ifdef DEMO_SCHED_LIB
#include "schedeval.h"
#include "sched.h"
#endif

#ifdef DEMO_IMG_MGMT
#include "flash_layout.h"
#define BUILD_DATE ""
/*
 * Image header location is fixed.
 */
#define IMG_HDR_LOC		(MCU_IMG_ACTIVE + IMAGE_HDR_OFF)
#define IMG_HDR_VER_LOC		(IMG_HDR_LOC + sizeof(struct image_hdr))

const struct image_hdr __img_hdr
			__attribute__((used))
			__attribute((at(IMG_HDR_LOC)));
const char version[72] __attribute((at(IMG_HDR_VER_LOC))) =
	 VERSION " " BUILD_DATE;
#else
const char version[] = VERSION;
#endif /* DEMO_IMG_MGMT || AYLA_BUILD_VERSION */

static u8 factory_reset;

static void set_input(struct prop *, void *arg, void *valp, size_t len);
static void set_cmd(struct prop *, void *arg, void *valp, size_t len);
static void set_dec_in(struct prop *prop, void *arg, void *valp, size_t len);
static void set_schedule_in(struct prop *prop, void *arg, void *valp,
    size_t len);
#ifdef DEMO_IMG_MGMT
extern u8 boot2inactive;
extern u8 template_req;
void mcu_img_mgmt_init(void);
int send_inactive_version(struct prop *, void *arg);
void set_boot2inactive(struct prop *, void *arg, void *valp, size_t len);
int send_template_version(struct prop *, void *arg);
void template_version_sent(void);
#endif

struct debounce {
	u8	val;		/* exported value */
	u8	raw;		/* last read value */
	u32	bounce_tick;	/* bouncing - ignore changes while this set */
	u32	off_tick;	/* on time extended while this is set */
};
static struct debounce button;

struct intr_stats intr_stats;
volatile u32 tick;

static s32 input;
static s32 output;
static s32 decimal_in;
static s32 decimal_out;

static const char *log_msg = "start";

static u8 sched_buf[256];
static u8 sched_out_length;
static const u8 *sched_out;
#ifdef DEMO_SCHED_LIB
static struct sched_prop schedule_in;
#endif

static void intr_init(void)
{
	NVIC_InitTypeDef nvic_init;
	EXTI_InitTypeDef exti_init;
	extern void *__Vectors;

	/*
	 * Make sure vector table offset is set correctly before enabling
	 * interrupts.
	 */
	SCB->VTOR = (u32)&__Vectors;

	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);	/* 4 bits priority */

	spi_intr_init();
	/*
	 * Set button to cause external interrupt on either edge.
	 */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
	exti_init.EXTI_Line = BUTTON_EXT_LINE;
	exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
	exti_init.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	exti_init.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti_init);

	nvic_init.NVIC_IRQChannel = BUTTON_IRQ;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 15;
	nvic_init.NVIC_IRQChannelSubPriority = 0;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);
}

static void set_led0(int on)
{
	if (on) {
		LED_GPIO->BSRR = bit(LED0_PIN);
	} else {
		LED_GPIO->BRR = bit(LED0_PIN);
	}
}

static void set_led(struct prop *prop, void *arg, void *valp, size_t len)
{
	u8 val = *(u8 *)valp;
	u32 pin = (u32)arg;

	if (val) {
		LED_GPIO->BSRR = pin;
	} else {
		LED_GPIO->BRR = pin;
	}
}

static int send_led(struct prop *prop, void *arg)
{
	u8 val;

	val = ((u32)prop->arg & LED_GPIO->ODR) != 0;
	return prop_send(prop, &val, sizeof(val), arg);
}

static char cmd_buf[TLV_MAX_STR_LEN + 1];

static int send_cmd(struct prop *prop, void *arg)
{
	return prop_send(prop, &cmd_buf, strlen(cmd_buf), arg);
}

static int send_log(struct prop *prop, void *arg)
{
	return prop_send(prop, log_msg, strlen(log_msg), arg);
}

static int send_int(struct prop *prop, void *arg)
{
	return prop_send(prop, prop->arg, prop->val_len, arg);
}

static int send_utf8(struct prop *prop, void *arg)
{
	return prop_send(prop, prop->arg, strlen(prop->arg), arg);
}

static int send_version(struct prop *prop, void *arg)
{
	return prop_send(prop, version, strlen(version), arg);
}

static int send_schedule(struct prop *prop, void *arg)
{
	return prop_send(prop, sched_out, sched_out_length, arg);
}

static void set_module_down(struct prop *prop, void *arg, void *valp,
    size_t len)
{
	u8 val = *(u8 *)valp;

	if (val) {
		WKUP_GPIO->BRR = bit(WKUP_PIN);
	} else {
		WKUP_GPIO->BSRR = bit(WKUP_PIN);
	}
}

static int send_module_down(struct prop *prop, void *arg)
{
	u8 val;

	val = (ready() == 0);
	return prop_send(prop, &val, sizeof(val), arg);
}

char power_mode_set_val[32];

static void power_mode_set(struct prop *prop, void *arg, void *valp, size_t len)
{
	enum conf_token pwr_mode[] = {CT_power, CT_current};
	u8 val;

	if (len >= sizeof(power_mode_set_val)) {
		return;
	}

	memcpy(power_mode_set_val, valp, len);
	power_mode_set_val[len] = '\0';
	if (!strcmp(power_mode_set_val, "max_perf")) {
		val = CT_max_perf;
	} else if (!strcmp(power_mode_set_val, "default")) {
		val = CT_default;
	} else if (!strcmp(power_mode_set_val, "min")) {
		val = CT_min;
	} else if (!strcmp(power_mode_set_val, "standby")) {
		val = CT_standby;
	} else {
		return;
	}

	conf_write(1, pwr_mode, 2, ATLV_INT, &val, sizeof(val));
}
#ifdef DEMO_IR
char ir_encode_proto[20];
u32 ir_encode;

static void set_ir_encode_proto(struct prop *prop, void *arg,
	void *valp, size_t len)
{
	if (len >= sizeof(ir_encode_proto) - 1) {
		return;
	}
	memcpy(ir_encode_proto, valp, len);
	ir_encode_proto[len] = '\0';
}

static void set_ir_encode(struct prop *prop, void *arg, void *valp, size_t len)
{
	if (len != sizeof(u32)) {
		return;
	}
	ir_encode = *(u32 *)valp;
	if (ir_encode_proto[0]) {
		ir_send(ir_encode_proto, ir_encode);
	}
}

#endif /* DEMO_IR */

struct prop prop_table[] = {
	{ "Blue_button", ATLV_BOOL, NULL, send_int,
	    &button.val, sizeof(button.val), AFMT_READ_ONLY},
#define PROP_BUTTON 0
	{ "output", ATLV_INT, NULL, send_int, &output,
	    sizeof(output), AFMT_READ_ONLY},
#define PROP_OUTPUT 1
	{ "log", ATLV_UTF8, NULL, send_log, NULL, 0, AFMT_READ_ONLY},
#define PROP_LOG 2
	{ "decimal_out", ATLV_CENTS, NULL, send_int, &decimal_out,
	    sizeof(decimal_out), AFMT_READ_ONLY},
#define PROP_DEC_OUT 3
	{ "schedule_out", ATLV_SCHED, NULL, send_schedule, NULL, 0,
	  AFMT_READ_ONLY},
#define PROP_SCHED_OUT 4
	{ "decimal_in", ATLV_CENTS, set_dec_in, send_int,
	    &decimal_in, sizeof(decimal_in)},
#ifdef DEMO_SCHED_LIB
	{ "schedule_in", ATLV_SCHED, set_schedule_in, NULL, &schedule_in},
#else
	{ "schedule_in", ATLV_SCHED, set_schedule_in},
#endif
	{ "Blue_LED", ATLV_BOOL, set_led, send_led,
	    (void *)(1 << LED0_PIN), 1},
	{ "Green_LED", ATLV_BOOL, set_led, send_led,
	    (void *)(1 << LED1_PIN), 1},
	{ "module_down", ATLV_BOOL, set_module_down, send_module_down},
	{ "cmd", ATLV_UTF8, set_cmd, send_cmd},
	{ "input", ATLV_INT, set_input, send_int, &input, sizeof(input)},
	{ "version", ATLV_UTF8, NULL, send_version, NULL, 0, AFMT_READ_ONLY},
#ifdef DEMO_IMG_MGMT
	{ "inactive_version", ATLV_UTF8, NULL, send_inactive_version, NULL,
	  0, AFMT_READ_ONLY },
	{ "boot_to_inactive", ATLV_BOOL, set_boot2inactive, send_int,
	  &boot2inactive, sizeof(boot2inactive) },
	{ "oem_host_version", ATLV_UTF8, NULL, send_template_version },
#endif
#ifdef DEMO_STREAM
	/*
	 * Long data points must use property type ATLV_LOC in this table,
	 * even though they have type ATLV_BIN in the protocol.
	 */
	{ "stream_up_len", ATLV_INT, set_length_up, send_int,
	    &stream_up_state.tot_len, sizeof(stream_up_state.tot_len)},
	{ "stream_up", ATLV_LOC, NULL, prop_dp_send, &stream_up_state, 0},
	{ "stream_down", ATLV_LOC, prop_dp_set, prop_dp_send,
	    &stream_down_state, 0},
	{ "stream_down_len", ATLV_UINT, NULL, send_int,
	   &stream_down_state.next_off, sizeof(stream_down_state.next_off)},
	{ "stream_down_match_len", ATLV_UINT, NULL, send_int,
	   &stream_down_patt_match_len, sizeof(stream_down_patt_match_len)},
#endif /* DEMO_STREAM */
#ifdef DEMO_IR
	{ "ir_encode_proto", ATLV_UTF8, set_ir_encode_proto, send_utf8,
	   &ir_encode_proto, sizeof(ir_encode_proto) - 1},
	{ "ir_encode", ATLV_UINT, set_ir_encode, send_int,
	   &ir_encode, sizeof(ir_encode)},
#endif /* DEMO_IR */
	{ "current_power_mode", ATLV_UTF8, power_mode_set, send_utf8,
	  &power_mode_set_val, sizeof(power_mode_set_val) },
	{ NULL }
};
u8 prop_count = (sizeof(prop_table) / sizeof(prop_table[0])) - 1;

static void set_input(struct prop *prop, void *arg, void *valp, size_t len)
{
	s32 i = *(s32 *)valp;

	if (len != sizeof(s32)) {
		return;
	}
	input = i;
	if (i > 0x7fff || i < -0x8000) {
		output = -1;		/* square would overflow */
	} else {
		output = i * i;
	}
	prop_table[PROP_OUTPUT].send_mask = valid_dest_mask;
}

static void set_dec_in(struct prop *prop, void *arg, void *valp, size_t len)
{
	s32 i = *(s32 *)valp;

	if (len != sizeof(s32)) {
		return;
	}
	decimal_in = i;
	decimal_out = i;
	prop_table[PROP_DEC_OUT].send_mask = valid_dest_mask;
}

static void set_cmd(struct prop *prop, void *arg, void *valp, size_t len)
{
	if (len >= sizeof(cmd_buf)) {
		len = sizeof(cmd_buf) - 1;
	}
	memcpy(cmd_buf, valp, len);
	cmd_buf[len] = '\0';
	log_msg = cmd_buf;
	prop_table[PROP_LOG].send_mask = valid_dest_mask;
}

static void set_schedule_in(struct prop *prop, void *arg, void *valp,
    size_t len)
{
	if (len > sizeof(sched_buf)) {
		len = sizeof(sched_buf);
	}
	memcpy(sched_buf, valp, len);
	sched_out = sched_buf;
	sched_out_length = len;
	prop_table[PROP_SCHED_OUT].send_mask = valid_dest_mask;
#ifdef DEMO_SCHED_LIB
	memcpy(schedule_in.tlvs, valp, len);
	schedule_in.len = sched_out_length;
	sched_run_all(NULL);
#endif
}

/*
 * Read pushbutton and debounce it.
 * Extend the on time to make it at least BUTTON_ON_MIN_TICKS long.
 * Don't report changes within BUTTON_DEBOUNCE_TICKS of a previous change.
 * Return the debounced, not the extended value.
 */
static u8 button_read(void)
{
	u8 new;

	new = (BUTTON_GPIO->IDR & bit(BUTTON_PIN)) != 0;
	if (button.raw == new) {
		return new;
	}
	button.raw = new;
	if (button.bounce_tick) {
		button.bounce_tick++;
		new = button.val;
	} else {
		button.bounce_tick = tick + BUTTON_DEBOUNCE_TICKS;
		if (!button.off_tick) {
			if (new) {
				button.off_tick =
				     tick + BUTTON_ON_MIN_TICKS;
			}
			button.val = new;
			prop_table[PROP_BUTTON].send_mask = valid_dest_mask;
		}
	}
	return new;
}

int ready(void)
{
	return (READY_N_GPIO->IDR & bit(READY_N_PIN)) == 0;
}

/*
 * Send a rising edge on wakeup line.
 */
int wakeup(void)
{
	int i;

	WKUP_GPIO->BRR = bit(WKUP_PIN);
	for (i = 1000; i--; )
		;
	WKUP_GPIO->BSRR = bit(WKUP_PIN);
	return 0;
}

/*
 * Only works if node is in standby mode.
 */
int module_powerdown(void)
{
	WKUP_GPIO->BRR = bit(WKUP_PIN);
	return 0;
}

/*
 * Button Interrupt Handler.
 */
void EXTI0_IRQHandler(void)
{
	intr_stats.button++;
	if (!EXTI_GetITStatus(BUTTON_EXT_LINE)) {
		return;
	}
	EXTI_ClearITPendingBit(BUTTON_EXT_LINE);
	button_read();
}

/*
 * Return true 1, if a > b with time wrapping.
 */
static int time_gt(u32 a, u32 b)
{
	return ((int)(a - b) > 0);
}

void SysTick_Handler(void)
{
	tick++;
	if (button.bounce_tick && time_gt(tick, button.bounce_tick)) {
		button.bounce_tick = 0;
		button_read();
	}
	if (button.off_tick && time_gt(tick, button.off_tick)) {
		button.off_tick = 0;
		button_read();
		if (button.val > button.raw) {
			button.val = button.raw;
			prop_table[PROP_BUTTON].send_mask = valid_dest_mask;
		}
	}
}


static void systick_init(void)
{
	SysTick_Config(CPU_CLK_HZ / SYSTICK_HZ);
}

/*
 * See if user is holding down the blue button during boot.
 * The button must be down for 5 seconds total.  After 1 second we
 * start blinking the blue LED.  After 5 seconds we do the reset.
 */
static void factory_reset_detect(void)
{
	int next_blink = 0;
	int reset_time;
	int led = 0;

	reset_time = tick + SYSTICK_HZ * 5;
	while (time_gt(reset_time, tick)) {
		if (!button_read()) {
			set_led0(0);
			return;
		}
		if (time_gt(tick, next_blink)) {
			next_blink = tick + SYSTICK_HZ / 8; /* 8 Hz blink */
			led ^= 1;
			set_led0(led);
		}
	}
	set_led0(1);
	factory_reset = 1;
}

void delay_time(u32 ms)
{
	u32 end_tick = tick + (ms * SYSTICK_HZ) / 1000;

	while (time_gt(end_tick, tick)) {
		;
	}
}

int main(int argc, char **argv)
{
	struct prop *prop;

	feature_mask |= MCU_LAN_SUPPORT;
#ifdef DEMO_IMG_MGMT
	mcu_img_mgmt_init();
	feature_mask |= MCU_OTA_SUPPORT;
#endif
#ifdef DEMO_SCHED_LIB
	feature_mask |= MCU_TIME_SUBSCRIPTION;
#endif
	io_init();
	spi_init();
	reset_module();
	systick_init();
	intr_init();
#ifdef DEMO_IR
	ir_init();
#endif /* DEMO_IR */
	factory_reset_detect();
#ifdef DEMO_STREAM
	demo_stream_init();
#endif /* DEMO_STREAM */
	for (;;) {
		if (ready()) {
			if (factory_reset &&
			    !serial_tx_cmd(ACMD_LOAD_FACTORY, NULL, 0)) {
				factory_reset = 0;
				set_led0(0);
				while (ready()) {
					serial_poll();
				}
			}
			conf_poll();
			prop_poll();
			serial_poll();
#ifdef DEMO_SCHED_LIB
			if (next_event_tick && tick >= next_event_tick) {
				sched_run_all(&next_event_tick);
			}
#endif
		} else {
			if (prop_pending()) {
				/*
				 * Module sleeping? Toggle wakeup.
				 */
				wakeup();
			}
		}
#ifdef DEMO_IMG_MGMT
		if (template_req &&
		    prop_send_done(prop_lookup("oem_host_version")) == 0) {
			/*
			 * Template version number has been sent.
			 */
			template_version_sent();
		}
#endif
		prop = prop_lookup_error();
		if (prop != NULL) {
			/*
			 * Property send has failed with error code.
			 * Error code is available in prop->send_err
			 *
			 * Insert logic here to handle the failure.
			 */
			prop->send_err = 0;
		}
	}
}

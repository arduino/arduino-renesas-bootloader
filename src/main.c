/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 * Copyright (c) 2023 Arduino SA
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bsp/board_api.h"
#include "tusb.h"
#include "bsp_api.h"
#include "r_gpt.h"

#include "flash.h"

uint8_t aucbuffer[PROGRAM_BLOCK_SIZE];
uint16_t auclen = 0;
uint16_t auc_block_num = 0;

void bgo_callback(flash_callback_args_t *p_args)
{
  if (p_args->event == FLASH_EVENT_WRITE_COMPLETE && auclen > 0) {
    tud_dfu_finish_flashing(DFU_STATUS_OK);
    auclen = 0;
  }
}

void flash_ready_interrupt_handler();
void flash_error_interrupt_handler();

flash_instance_ctrl_t g_flash_ctrl;
const flash_cfg_t g_flash_cfg =
{ .data_flash_bgo = false, .p_callback = bgo_callback, .p_context = NULL,
  .irq = 7,
  .err_irq = 8,
  .err_ipl = (2),
  .ipl = (2),
};

/* Instance structure to use this module. */
const flash_instance_t g_flash ={ .p_ctrl = &g_flash_ctrl, .p_cfg = &g_flash_cfg, .p_api = &flash_apis };

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void led_blinking_task(void);
void set_double_tap_data(uint32_t data);

void boot5(uint32_t address) {

  R_SYSTEM->PRCR = (uint16_t) BSP_PRV_PRCR_PRC1_UNLOCK;
  R_BSP_MODULE_STOP(FSP_IP_USBFS, 0);
  R_BSP_MODULE_STOP(FSP_IP_USBHS, 0);
  R_SYSTEM->PRCR = (uint16_t) BSP_PRV_PRCR_LOCK;

    /* Disable MSP monitoring. */
#if BSP_FEATURE_TZ_HAS_TRUSTZONE
    __set_MSPLIM(0);
#else
    R_MPU_SPMON->SP[0].CTL = 0;
#endif

  __disable_irq(); // Note: remember to enable IRQ in application
  __DSB();
  __ISB();

  // Disable SysTick
  SysTick->CTRL = 0;

  SCB->VTOR  = address;

  uint32_t mainStackPointer = *(volatile uint32_t *)(address);
  __set_MSP(mainStackPointer);
  uint32_t programResetHandlerAddress = *(volatile uint32_t *) (address + 4);
  void (* programResetHandler)(void) = (void (*)(void)) programResetHandlerAddress;
  programResetHandler();
}

#if defined(RENESAS_CORTEX_M23)
#define BOOT_DOUBLE_TAP_DATA              (*((volatile uint32_t *)0x20007FF0))
#else
#define BOOT_DOUBLE_TAP_DATA              (*((volatile uint32_t *) &R_SYSTEM->VBTBKR[0]))
#endif
#define DOUBLE_TAP_MAGIC                  0x07738135

#define USEC_PER_SEC	(1000000U)
#define PERIOD 			(USEC_PER_SEC / 50U)
#define FADESTEP		2000

const gpt_extended_cfg_t g_timer_extend = {
  .gtioca = { .output_enabled = LED_FADE_PWM_OUT_A ? true : false, .stop_level = GPT_PIN_LEVEL_LOW },
  .gtiocb = { .output_enabled = LED_FADE_PWM_OUT_B ? true : false, .stop_level = GPT_PIN_LEVEL_LOW },
};


const timer_cfg_t pwm_cfg = {
    .mode = TIMER_MODE_PWM,
    .period_counts = (uint32_t) 0x9896, .duty_cycle_counts = 0x4c4b,
    .source_div =	(timer_source_div_t) LED_TIMER_SOURCE_DIV, .channel = LED_FADE_PWM_CHANNEL, .p_callback = NULL,
    .p_context = NULL, .p_extend = &g_timer_extend, .cycle_end_ipl = (BSP_IRQ_DISABLED),
};


static gpt_instance_ctrl_t pwm_ctrl;
#include "r_ioport.h"

static const ioport_pin_cfg_t extra_pin_cfg[] = {
#ifdef BOSSA_LOADER
  { .pin = BOSSA_TX_PIN, .pin_cfg = (IOPORT_CFG_PERIPHERAL_PIN  | IOPORT_CFG_PULLUP_ENABLE | BOSSA_PERIPHERAL_FLAGS) },
  { .pin = BOSSA_RX_PIN, .pin_cfg = (IOPORT_CFG_PERIPHERAL_PIN  | IOPORT_CFG_PULLUP_ENABLE | BOSSA_PERIPHERAL_FLAGS) },
#endif
  { .pin = LED_FADE_GPIO, .pin_cfg = IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_GPT1 }
};

static const ioport_cfg_t pin_cfg = {
    .number_of_pins = sizeof(extra_pin_cfg) / sizeof(ioport_pin_cfg_t),
    .p_pin_cfg_data = extra_pin_cfg,
};
ioport_instance_ctrl_t port_ctrl;

int pwm_channel = LED_FADE_PWM_OUT_A ? GPT_IO_PIN_GTIOCA : GPT_IO_PIN_GTIOCB;

/* ***
 * UNO R4 MINIMA AND C33 BOOTLOADER ** ONLY DFU **
 * *** */
__WEAK void run_bootloader() {
  // init device stack on configured roothub port
  tud_init(BOARD_TUD_RHPORT);
  if (board_init_after_tusb) {
    board_init_after_tusb();
  }

  while (1)
  {
    tud_task(); // tinyusb device task
    led_blinking_task();
  }
} 

/*------------- MAIN -------------*/
int main(void)
{
  board_init();

  // set magic for double tap
  if (BOOT_DOUBLE_TAP_DATA == DOUBLE_TAP_MAGIC) {
    set_double_tap_data(0);
    goto bootloader;
  }

  if (!R_SYSTEM->RSTSR0_b.PORF) {
	set_double_tap_data(DOUBLE_TAP_MAGIC);
  }
#if !defined(RENESAS_CORTEX_M23) /* no double tap support in Muxto */
#ifdef TURN_OFF_CHARGER_LED
  i2c_begin();
  i2c_write(0x8, 0x9C, (1 << 7));
  i2c_write(0x8, 0x9E, (1 << 5));
#endif
  while (board_millis() < 500) {
  }
#endif
	set_double_tap_data(0);

  int app_valid = (((*(uint32_t *) SKETCH_FLASH_OFFSET) & 0xFF000000) == 0x20000000);

  if (app_valid) {
      boot5(SKETCH_FLASH_OFFSET);
  }

bootloader:

  R_IOPORT_Open(&port_ctrl, &pin_cfg);

  R_GPT_Open(&pwm_ctrl, &pwm_cfg);
  R_GPT_PeriodSet(&pwm_ctrl, PERIOD);
  R_GPT_DutyCycleSet(&pwm_ctrl, 0, pwm_channel);
  R_GPT_Start(&pwm_ctrl);

#if BSP_FEATURE_FLASH_HP_VERSION
  R_FLASH_HP_Open(g_flash.p_ctrl, g_flash.p_cfg);
  R_FLASH_HP_Reset(g_flash.p_ctrl);
  R_FLASH_HP_StartUpAreaSelect(g_flash.p_ctrl, FLASH_STARTUP_AREA_BLOCK0, true);
#endif

#if BSP_FEATURE_FLASH_LP_VERSION
  R_FLASH_LP_Open(g_flash.p_ctrl, g_flash.p_cfg);
  R_FLASH_LP_Reset(g_flash.p_ctrl);
#endif

  run_bootloader();

  return 0;
}


#ifdef DFU_LOADER

/* !!!
*
* UNO R4 MINIMA / C33 / OPTA ANALOG / OPTA DIGITAL ->  define this so that they have DFU callbacks
*
* UNO WIFI  does ** NOT ** define this so it avoids DFU callback
*
* !!! */
  

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

//--------------------------------------------------------------------+
// DFU callbacks
// Note: alt is used as the partition number, in order to support multiple partitions like FLASH, EEPROM, etc.
//--------------------------------------------------------------------+

// Invoked right before tud_dfu_download_cb() (state=DFU_DNBUSY) or tud_dfu_manifest_cb() (state=DFU_MANIFEST)
// Application return timeout in milliseconds (bwPollTimeout) for the next download/manifest operation.
// During this period, USB host won't try to communicate with us.
uint32_t tud_dfu_get_timeout_cb(uint8_t alt, uint8_t state)
{
  if ( state == DFU_DNBUSY )
  {
    // For this example
    // - Atl0 Flash is fast : 1   ms
    // - Alt1 EEPROM is slow: 100 ms
    return (alt == 0) ? 0 : 100;
  }
  else if (state == DFU_MANIFEST)
  {
    // since we don't buffer entire image and do any flashing in manifest stage
    return 0;
  }

  return 0;
}

#if BSP_FEATURE_FLASH_HP_VERSION
int flash_write_block() {
  return R_FLASH_HP_Write(g_flash.p_ctrl, (uint32_t)aucbuffer, SKETCH_FLASH_OFFSET + ((auc_block_num * CFG_TUD_DFU_XFER_BUFSIZE)/sizeof(aucbuffer))*sizeof(aucbuffer) , auclen);
}
#endif

#if BSP_FEATURE_FLASH_LP_VERSION
int flash_write_block() {
  return R_FLASH_LP_Write(g_flash.p_ctrl, (uint32_t)aucbuffer, SKETCH_FLASH_OFFSET + ((auc_block_num * CFG_TUD_DFU_XFER_BUFSIZE)/sizeof(aucbuffer))*sizeof(aucbuffer) , PROGRAM_BLOCK_SIZE);
}
#endif

// Invoked when received DFU_DNLOAD (wLength>0) following by DFU_GETSTATUS (state=DFU_DNBUSY) requests
// This callback could be returned before flashing op is complete (async).
// Once finished flashing, application must call tud_dfu_finish_flashing()
void tud_dfu_download_cb(uint8_t alt, uint16_t block_num, uint8_t const* data, uint16_t length)
{
  (void) alt;
  auc_block_num = block_num;

  //printf("\r\nReceived Alt %u BlockNum %u of length %u\r\n", alt, wBlockNum, length);

  if ((SKETCH_FLASH_OFFSET + (block_num * CFG_TUD_DFU_XFER_BUFSIZE)) % PROGRAM_BLOCK_SIZE == 0) {
    // erase block
    __disable_irq();
#if BSP_FEATURE_FLASH_HP_VERSION
    int err = R_FLASH_HP_Erase(g_flash.p_ctrl, SKETCH_FLASH_OFFSET + (block_num * CFG_TUD_DFU_XFER_BUFSIZE), 1);
#endif
#if BSP_FEATURE_FLASH_LP_VERSION
    int err = R_FLASH_LP_Erase(g_flash.p_ctrl, SKETCH_FLASH_OFFSET + (block_num * CFG_TUD_DFU_XFER_BUFSIZE), 1);
#endif
    __enable_irq();
  }
  memcpy(&aucbuffer[auclen], data, length);
  auclen += length;

  if (auclen == sizeof(aucbuffer)) {
    __disable_irq();
    flash_write_block();
    __enable_irq();
    auclen = 0;
  }

  tud_dfu_finish_flashing(DFU_STATUS_OK);

  // flashing op for download complete without error
}

// Invoked when download process is complete, received DFU_DNLOAD (wLength=0) following by DFU_GETSTATUS (state=Manifest)
// Application can do checksum, or actual flashing if buffered entire image previously.
// Once finished flashing, application must call tud_dfu_finish_flashing()
void tud_dfu_manifest_cb(uint8_t alt)
{
  (void) alt;
  //printf("Download completed, enter manifestation\r\n");

  __disable_irq();
  flash_write_block();
  __enable_irq();

  // flashing op for manifest is complete without error
  // Application can perform checksum, should it fail, use appropriate status such as errVERIFY.
  tud_dfu_finish_flashing(DFU_STATUS_OK);

  // Try booting the application
  // boot5(SKETCH_FLASH_OFFSET);
#if BSP_FEATURE_FLASH_HP_VERSION
    R_FLASH_HP_Close(g_flash.p_ctrl);
#endif
#if BSP_FEATURE_FLASH_LP_VERSION
    R_FLASH_LP_Close(g_flash.p_ctrl);
#endif
}

// Invoked when received DFU_UPLOAD request
// Application must populate data with up to length bytes and
// Return the number of written bytes
uint16_t tud_dfu_upload_cb(uint8_t alt, uint16_t block_num, uint8_t* data, uint16_t length)
{
  (void) block_num;
  (void) alt;

  memcpy(data, (void*)(block_num * length), length);

  return length;
}

// Invoked when the Host has terminated a download or upload transfer
void tud_dfu_abort_cb(uint8_t alt)
{
  (void) alt;
  //printf("Host aborted transfer\r\n");
}

// Invoked when a DFU_DETACH request is received
void tud_dfu_detach_cb(void)
{
  //printf("Host detach, we should probably reboot\r\n");
  uint32_t start_ms = board_millis();
  while (board_millis() - start_ms < 100) {
    tud_task();
  }
  NVIC_SystemReset();
}

#endif

//--------------------------------------------------------------------+
// BLINKING TASK + Indicator pulse
//--------------------------------------------------------------------+

void pulse_led()
{
	static uint32_t pulse_width;
	static uint8_t dir;

  R_GPT_DutyCycleSet(&pwm_ctrl, pulse_width, pwm_channel);

	if (dir) {
		if (pulse_width < FADESTEP) {
			dir = 0U;
			pulse_width = FADESTEP;
		} else {
			pulse_width -= FADESTEP;
		}
	} else {
		pulse_width += FADESTEP;

		if (pulse_width >= PERIOD) {
			dir = 1U;
			pulse_width = PERIOD;
		}
	}
}

void led_blinking_task(void)
{
  static uint32_t start_ms = 0;

  // Blink every interval ms
  if ( board_millis() - start_ms < 100) return;
  start_ms += 100;

  pulse_led();
}

//--------------------------------------------------------------------+
// Double tap magic
//--------------------------------------------------------------------+
void set_double_tap_data(uint32_t data)
{
  R_SYSTEM->PRCR = (uint16_t) BSP_PRV_PRCR_PRC1_UNLOCK;
  BOOT_DOUBLE_TAP_DATA = data;
  R_SYSTEM->PRCR = (uint16_t) BSP_PRV_PRCR_LOCK;
}

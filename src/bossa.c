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

#if BOSSA_LOADER

#include "bsp_api.h"
#include "r_ioport.h"
#include "flash.h"

/* Instance structure to use this module. */
extern const flash_instance_t g_flash;

#include "r_sci_uart.h"

void sci_uart_txi_isr(void);
void sci_uart_tei_isr(void);
void sci_uart_rxi_isr(void);
void sci_uart_eri_isr(void);

BSP_DONT_REMOVE const
  fsp_vector_t g_vector_table[BSP_ICU_VECTOR_MAX_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) = {
    [0] = sci_uart_txi_isr,
    [1] = sci_uart_tei_isr,
    [2] = sci_uart_rxi_isr,
    [3] = sci_uart_eri_isr
};

const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_MAX_ENTRIES] = {
  [0] = BSP_PRV_IELS_ENUM(EVENT_SCI9_TXI),
  [1] = BSP_PRV_IELS_ENUM(EVENT_SCI9_TEI),
  [2] = BSP_PRV_IELS_ENUM(EVENT_SCI9_RXI),
  [3] = BSP_PRV_IELS_ENUM(EVENT_SCI9_ERI)
};

sci_uart_instance_ctrl_t g_uart_ctrl;

baud_setting_t uart_baud = {
  .semr_baudrate_bits_b.abcse          = 0,
  .semr_baudrate_bits_b.abcs           = 0,
  .semr_baudrate_bits_b.bgdm           = 1,
  .cks                                 = 0,
  .brr                                 = 25,
  .mddr                                = (uint8_t) 256,
  .semr_baudrate_bits_b.brme           = false,
};

const sci_uart_extended_cfg_t   uart_cfg_extend = {
  .clock                         = SCI_UART_CLOCK_INT,
  .rx_edge_start                 = SCI_UART_START_BIT_FALLING_EDGE,
  .noise_cancel                  = SCI_UART_NOISE_CANCELLATION_DISABLE,
  .rx_fifo_trigger               = SCI_UART_RX_FIFO_TRIGGER_1,
  .p_baud_setting                = &uart_baud,
  .flow_control                  = SCI_UART_FLOW_CONTROL_RTS,
  .flow_control_pin              = (bsp_io_port_pin_t) UINT16_MAX,
  .rs485_setting.enable          = SCI_UART_RS485_DISABLE,
  .rs485_setting.polarity        = SCI_UART_RS485_DE_POLARITY_HIGH,
  .rs485_setting.de_control_pin  = (bsp_io_port_pin_t) UINT16_MAX,
};

volatile uint8_t command_buffer[64];
volatile uint32_t command_buffer_index = 0;
volatile bool rx_complete = false;

void user_uart_callback(uart_callback_args_t *p_args) {
  if(UART_EVENT_RX_CHAR == p_args->event) {
    if (command_buffer_index < sizeof(command_buffer)) {
      command_buffer[command_buffer_index++] = p_args->data;
    }
  }
  if (UART_EVENT_RX_COMPLETE == p_args->event) {
    rx_complete = true;
  }
}

#include "r_dtc.h"

dtc_instance_ctrl_t g_transfer21_ctrl;

transfer_info_t g_transfer21_info =
{ .transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED,
  .transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_DESTINATION,
  .transfer_settings_word_b.irq = TRANSFER_IRQ_END,
  .transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_DISABLED,
  .transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_FIXED,
  .transfer_settings_word_b.size = TRANSFER_SIZE_1_BYTE,
  .transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL,
  .p_dest = (void*) NULL,
  .p_src = (void const*) NULL,
  .num_blocks = 0,
  .length = 0, };

const dtc_extended_cfg_t g_transfer21_cfg_extend =
{ .activation_source = 2, };
const transfer_cfg_t g_transfer21_cfg =
{ .p_info = &g_transfer21_info, .p_extend = &g_transfer21_cfg_extend, };

/* Instance structure to use this module. */
const transfer_instance_t g_transfer21 =
{ .p_ctrl = &g_transfer21_ctrl, .p_cfg = &g_transfer21_cfg, .p_api = &g_transfer_on_dtc };

const uart_cfg_t g_uart_cfg = {
  .channel                              = 9,
  .p_context                            = NULL,
  .p_extend                             = &uart_cfg_extend,
  .p_transfer_tx                        = NULL,
  .p_transfer_rx                        = &g_transfer21,

  .data_bits = UART_DATA_BITS_8,
  .parity = UART_PARITY_OFF,
  .stop_bits = UART_STOP_BITS_1,
  .p_callback = user_uart_callback,

  .rxi_ipl = (2),
  .txi_ipl = (2),
  .tei_ipl = (2),
  .eri_ipl = (2),
  .txi_irq = 0,
  .tei_irq = 1,
  .rxi_irq = 2,
  .eri_irq = 3,
};

static const uint16_t crc16Table[256]=
{
	0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
	0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
	0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
	0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
	0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
	0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
	0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
	0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
	0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
	0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
	0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
	0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
	0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
	0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
	0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
	0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
	0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
	0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
	0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
	0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
	0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
	0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
	0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
	0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
	0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
	0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
	0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
	0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
	0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
	0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
	0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
	0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};

unsigned short serial_add_crc(char ptr, unsigned short crc)
{
	return (crc << 8) ^ crc16Table[((crc >> 8) ^ ptr) & 0xff];
}

uint8_t data_buffer[8192];
uint8_t flash_buffer[4096];
uint32_t copyOffset = 0;

void pulse_led();

void bossa_task() {

  if (command_buffer_index > 0) {

    // printf("%c", b);

    if (command_buffer[command_buffer_index-1] == '#') {
      command_buffer[command_buffer_index] = '\0';
      char* str;

      switch (command_buffer[0]) {
        case 'N':
          R_SCI_UART_Write(&g_uart_ctrl, "\n\r", 2);
          break;

        case 'V':
          str = "Arduino Bootloader (SAM-BA extended) 2.0 [Arduino:IKXYZ]\n\r";
          R_SCI_UART_Write(&g_uart_ctrl, str, strlen(str));
          break;

        case 'I':
          str = "nRF52840-QIAA\n\r";
          R_SCI_UART_Write(&g_uart_ctrl, str, strlen(str));
          break;

        case 'X':
          R_SCI_UART_Write(&g_uart_ctrl, "X\n\r", 3);
          break;

        case 'S': {
          uint32_t address = 0;
          uint32_t length = 0;

          address = strtoul(&command_buffer[1], NULL, 16);
          length = strtoul(&command_buffer[10], NULL, 16);

          rx_complete = false;
          R_SCI_UART_Read(&g_uart_ctrl, &data_buffer[address], length);
          while (rx_complete == false) {
          }

          break;
        }

        case 'Y': {
          uint32_t arg1 = 0;
          uint32_t arg2 = 0;

          arg1 = strtoul(&command_buffer[1], NULL, 16);
          arg2 = strtoul(&command_buffer[10], NULL, 16);

          if (arg2 != 0) {
            __disable_irq();
            if ((SKETCH_FLASH_OFFSET + arg1) % PROGRAM_BLOCK_SIZE == 0) {
              R_FLASH_LP_Erase(g_flash.p_ctrl, SKETCH_FLASH_OFFSET + arg1, 2);
            }
            R_FLASH_LP_Write(g_flash.p_ctrl, (uint32_t)&data_buffer[copyOffset], SKETCH_FLASH_OFFSET + arg1, arg2);
            __enable_irq();
          } else {
            copyOffset = arg1;
          }

          R_SCI_UART_Write(&g_uart_ctrl, "Y\n\r", 3);

          break;
        }

        case 'Q': {
          for (size_t i = 0; i < 8192; i++) {
            uint8_t* ptr = (uint8_t*)(SKETCH_FLASH_OFFSET + i);
            char result[4];
            result[2] = '\r';
            result[3] = '\n';
            itoa(*ptr, result, 16);
            R_SCI_UART_Write(&g_uart_ctrl, result, 4);
            R_BSP_SoftwareDelay(5, BSP_DELAY_UNITS_MILLISECONDS);
          }
          break;
        }

        case 'Z': {
          uint32_t address = 0;
          uint32_t size = 0;
          uint16_t crc = 0;
          uint8_t result[1 + 8 + 3 + 1];

          address = strtoul(&command_buffer[1], NULL, 16);
          size = strtoul(&command_buffer[10], NULL, 16);

          memcpy(flash_buffer, (void*)(SKETCH_FLASH_OFFSET + address), size);

          for (uint32_t i = 0; i < size; i++) {
            crc = serial_add_crc(flash_buffer[i], crc);
          }

          sprintf(result, "Z%08X#\n\r", crc);

          R_SCI_UART_Write(&g_uart_ctrl, result, sizeof(result) - 1);

          break;
        }

        case 'K': {
          NVIC_SystemReset();
          break;
        }
      }

      command_buffer_index = 0;
    }
  }
}

extern ioport_instance_ctrl_t port_ctrl;

void restore_usb_switch() {
  if (R_SYSTEM->VBTBKR[1] == 40) {
    R_IOPORT_PinCfg(&port_ctrl, BSP_IO_PORT_04_PIN_08, IOPORT_CFG_PORT_DIRECTION_OUTPUT);
    R_IOPORT_PinWrite(&port_ctrl, BSP_IO_PORT_04_PIN_08, BSP_IO_LEVEL_HIGH);
    R_BSP_SoftwareDelay((uint32_t) 2, BSP_DELAY_UNITS_MILLISECONDS);
    R_IOPORT_PinWrite(&port_ctrl, BSP_IO_PORT_04_PIN_08, BSP_IO_LEVEL_LOW);
    R_IOPORT_PinCfg(&port_ctrl, BSP_IO_PORT_04_PIN_08, IOPORT_CFG_PORT_DIRECTION_INPUT);
    R_SYSTEM->PRCR = (uint16_t) BSP_PRV_PRCR_PRC1_UNLOCK;
    R_SYSTEM->VBTBKR[1] = 0;
    R_SYSTEM->PRCR = (uint16_t) BSP_PRV_PRCR_LOCK;
  }
}

void led_blinking_task();

void run_bootloader() {
  restore_usb_switch();
  R_SCI_UART_BaudCalculate(230400, true, 5000, &uart_baud);
  R_SCI_UART_Open(&g_uart_ctrl, &g_uart_cfg);
  R_SCI_UART_BaudSet(&g_uart_ctrl, (void *) &uart_baud);

  while (1) {
    bossa_task();
    led_blinking_task();
  }
}

#endif
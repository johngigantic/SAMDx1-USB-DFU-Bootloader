/*
 * 1kByte USB DFU bootloader for Atmel SAMD11 microcontrollers
 *
 * Copyright (c) 2018-2020, Peter Lawrence
 * derived from https://github.com/ataradov/vcp Copyright (c) 2016, Alex Taradov <alex@taradov.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* 
NOTES:
- anything pointed to by udc_mem[*].*.ADDR.reg *MUST* BE IN RAM and be 32-bit aligned... no exceptions
*/


/*- Includes ----------------------------------------------------------------*/
#include <stdbool.h>
#include <string.h>
#include <sam.h>
#include "usb.h"
#include "usb_descriptors.h"

/*- Definitions -------------------------------------------------------------*/
#define USE_DBL_TAP /* comment out to use GPIO input for bootloader entry */
#define REBOOT_AFTER_DOWNLOAD /* comment out to prevent boot into app after it has been downloaded */
#define USB_CMD(dir, rcpt, type) ((USB_##dir##_TRANSFER << 7) | (USB_##type##_REQUEST << 5) | (USB_##rcpt##_RECIPIENT << 0))
#define SIMPLE_USB_CMD(rcpt, type) ((USB_##type##_REQUEST << 5) | (USB_##rcpt##_RECIPIENT << 0))

/*- Types -------------------------------------------------------------------*/
typedef struct
{
    usb_device_desc_bank_registers_t  out;
    usb_device_desc_bank_registers_t  in;
} udc_mem_t;

/*- Variables ---------------------------------------------------------------*/
static uint32_t usb_config = 0;
static uint32_t dfu_status_choices[4] =
{ 
  0x00000000, 0x00000002, /* normal */
  0x00000000, 0x00000005, /* dl */
};

static udc_mem_t udc_mem[USB_EPT_NUM];
static uint32_t udc_ctrl_in_buf[16];
static uint32_t udc_ctrl_out_buf[16];

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void __attribute__((noinline)) udc_control_send(const uint32_t *data, uint32_t size)
{
  /* USB peripheral *only* reads valid data from 32-bit aligned RAM locations */
  udc_mem[0].in.USB_ADDR = (uint32_t)data;

  udc_mem[0].in.USB_PCKSIZE = USB_DEVICE_PCKSIZE_BYTE_COUNT(size) | USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(0) | USB_DEVICE_PCKSIZE_SIZE(3 /*64 Byte*/);

  USB_REGS->DEVICE.DEVICE_ENDPOINT[0].USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_TRCPT1(1);
  USB_REGS->DEVICE.DEVICE_ENDPOINT[0].USB_EPSTATUSSET = USB_DEVICE_EPSTATUSSET_BK1RDY(1);

  while (0 == (USB_REGS->DEVICE.DEVICE_ENDPOINT[0].USB_EPINTFLAG & USB_DEVICE_EPINTFLAG_TRCPT1(1)));
}

//-----------------------------------------------------------------------------
static void __attribute__((noinline)) udc_control_send_zlp(void)
{
  udc_control_send(NULL, 0); /* peripheral can't read from NULL address, but size is zero and this value takes less space to compile */
}

//-----------------------------------------------------------------------------
static void __attribute__((noinline)) USB_Service(void)
{
  static uint32_t dfu_addr;

  if (USB_REGS->DEVICE.USB_INTFLAG & USB_DEVICE_INTFLAG_EORST(1)) /* End Of Reset */
  {
    USB_REGS->DEVICE.USB_INTFLAG = USB_DEVICE_INTFLAG_EORST(1);
    USB_REGS->DEVICE.USB_DADD = USB_DEVICE_DADD_ADDEN(1);

    USB_REGS->DEVICE.DEVICE_ENDPOINT[0].USB_EPCFG = USB_DEVICE_EPCFG_EPTYPE0(1 /*CONTROL*/) | USB_DEVICE_EPCFG_EPTYPE1(1 /*CONTROL*/);
    USB_REGS->DEVICE.DEVICE_ENDPOINT[0].USB_EPSTATUSSET = USB_DEVICE_EPSTATUSSET_BK0RDY(1);
    USB_REGS->DEVICE.DEVICE_ENDPOINT[0].USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_BK1RDY(1);

    udc_mem[0].in.USB_ADDR = (uint32_t)udc_ctrl_in_buf;
    udc_mem[0].in.USB_PCKSIZE = USB_DEVICE_PCKSIZE_BYTE_COUNT(0) | USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(0) | USB_DEVICE_PCKSIZE_SIZE(3 /*64 Byte*/);

    udc_mem[0].out.USB_ADDR = (uint32_t)udc_ctrl_out_buf;
    udc_mem[0].out.USB_PCKSIZE = USB_DEVICE_PCKSIZE_BYTE_COUNT(64) | USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(0) | USB_DEVICE_PCKSIZE_SIZE(3 /*64 Byte*/);

    USB_REGS->DEVICE.DEVICE_ENDPOINT[0].USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_BK0RDY(1);
  }

  if (USB_REGS->DEVICE.DEVICE_ENDPOINT[0].USB_EPINTFLAG & USB_DEVICE_EPINTFLAG_TRCPT0(1)) /* Transmit Complete 0 */
  {
    if (dfu_addr)
    {
      if (0 == ((dfu_addr >> 6) & 0x3))
      {
        NVMCTRL_REGS->NVMCTRL_ADDR = dfu_addr >> 1;
        NVMCTRL_REGS->NVMCTRL_CTRLB = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD(NVMCTRL_CTRLB_CMD_EB);  // TODO: This erases a block, not a row.
        while (!(NVMCTRL_REGS->NVMCTRL_STATUS & NVMCTRL_STATUS_READY(1)));
      }

      uint16_t *nvm_addr = (uint16_t *)(dfu_addr);
      uint16_t *ram_addr = (uint16_t *)udc_ctrl_out_buf;
      for (unsigned i = 0; i < 32; i++)
        *nvm_addr++ = *ram_addr++;
      while (!(NVMCTRL_REGS->NVMCTRL_STATUS & NVMCTRL_STATUS_READY(1)));

      udc_control_send_zlp();
      dfu_addr = 0;
    }

    USB_REGS->DEVICE.DEVICE_ENDPOINT[0].USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_TRCPT0(1);
  }

  if (USB_REGS->DEVICE.DEVICE_ENDPOINT[0].USB_EPINTFLAG & USB_DEVICE_EPINTFLAG_RXSTP(1)) /* Received Setup */
  {
    USB_REGS->DEVICE.DEVICE_ENDPOINT[0].USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_RXSTP(1);
    USB_REGS->DEVICE.DEVICE_ENDPOINT[0].USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_BK0RDY(1);

    usb_request_t *request = (usb_request_t *)udc_ctrl_out_buf;
    uint8_t type = request->wValue >> 8;
    uint16_t length = request->wLength;
    static uint32_t *dfu_status = dfu_status_choices + 0;

    /* for these other USB requests, we must examine all fields in bmRequestType */
    if (USB_CMD(OUT, INTERFACE, STANDARD) == request->bmRequestType)
    {
      udc_control_send_zlp();
      return;
    }

    /* for these "simple" USB requests, we can ignore the direction and use only bRequest */
    switch (request->bmRequestType & 0x7F)
    {
    case SIMPLE_USB_CMD(DEVICE, STANDARD):
    case SIMPLE_USB_CMD(INTERFACE, STANDARD):
      switch (request->bRequest)
      {
        case USB_GET_DESCRIPTOR:
          if (USB_DEVICE_DESCRIPTOR == type)
          {
            udc_control_send((uint32_t *)&usb_device_descriptor, length);
          }
          else if (USB_CONFIGURATION_DESCRIPTOR == type)
          {
            udc_control_send((uint32_t *)&usb_configuration_hierarchy, length);
          }
          else
          {
            USB_REGS->DEVICE.DEVICE_ENDPOINT[0].USB_EPSTATUSSET = USB_DEVICE_EPSTATUSSET_STALLRQ1(1);
          }
          break;
        case USB_GET_CONFIGURATION:
          udc_control_send(&usb_config, 1);
          break;
        case USB_GET_STATUS:
          udc_control_send(dfu_status_choices + 0, 2); /* a 32-bit aligned zero in RAM is all we need */
          break;
        case USB_SET_FEATURE:
        case USB_CLEAR_FEATURE:
          USB_REGS->DEVICE.DEVICE_ENDPOINT[0].USB_EPSTATUSSET = USB_DEVICE_EPSTATUSSET_STALLRQ1(1);
          break;
        case USB_SET_ADDRESS:
          udc_control_send_zlp();
          USB_REGS->DEVICE.USB_DADD = USB_DEVICE_DADD_ADDEN(1) | USB_DEVICE_DADD_DADD(request->wValue);
          break;
        case USB_SET_CONFIGURATION:
          usb_config = request->wValue;
          udc_control_send_zlp();
          break;
      }
      break;
    case SIMPLE_USB_CMD(INTERFACE, CLASS):
      switch (request->bRequest)
      {
        case 0x03: // DFU_GETSTATUS
          udc_control_send(&dfu_status[0], 6);
          break;
        case 0x05: // DFU_GETSTATE
          udc_control_send(&dfu_status[1], 1);
          break;
        case 0x01: // DFU_DNLOAD
          dfu_status = dfu_status_choices + 0;
          if (request->wLength)
          {
            dfu_status = dfu_status_choices + 2;
            dfu_addr = 0x400 + request->wValue * 64;
          }
#ifdef REBOOT_AFTER_DOWNLOAD
          else
          {
            /* the download has now finished, so now reboot */
            WDT_REGS->WDT_CONFIG = WDT_CONFIG_PER_CYC8 | WDT_CONFIG_WINDOW_CYC8;
            WDT_REGS->WDT_CTRLA = WDT_CTRLA_ENABLE(1);
          }
#endif
          /* fall through */
        default: // DFU_UPLOAD & others
          /* 0x00 == DFU_DETACH, 0x04 == DFU_CLRSTATUS, 0x06 == DFU_ABORT, and 0x01 == DFU_DNLOAD and 0x02 == DFU_UPLOAD */
          if (!dfu_addr)
            udc_control_send_zlp();
          break;
      }
      break;
    }
  }
}

#ifdef USE_DBL_TAP
  #define DBL_TAP_MAGIC 0xf02669ef
  static volatile uint32_t __attribute__((section(".vectors_ram"))) double_tap;
#endif

void bootloader(void)
{
#ifndef USE_DBL_TAP
  /* configure PA15 (bootloader entry pin used by SAM-BA) as input pull-up */
  PORT_REGS->GROUP[0].PORT_PINCFG[15] = PORT_PINCFG_PULLEN(1) | PORT_PINCFG_INEN(1);
  PORT_REGS->GROUP[0].PORT_OUTSET = (1UL << 15);
#endif

  PAC_REGS->PAC_WRCTRL = PAC_WRCTRL_PERID(ID_DSU) | PAC_WRCTRL_KEY_CLR;

  DSU_REGS->DSU_ADDR = 0x400; /* start CRC check at beginning of user app */
  DSU_REGS->DSU_LENGTH = *(volatile uint32_t *)0x410; /* use length encoded into unused vector address in user app */

  /* ask DSU to compute CRC */
  DSU_REGS->DSU_DATA = 0xFFFFFFFF;
  DSU_REGS->DSU_CTRL |= DSU_CTRL_CRC(1);
  while (!(DSU_REGS->DSU_STATUSA & DSU_STATUSA_DONE(1)));

  if (DSU_REGS->DSU_DATA)
    goto run_bootloader; /* CRC failed, so run bootloader */

#ifndef USE_DBL_TAP
  if (!(PORT_REGS->GROUP[0].PORT_IN & (1UL << 15)))
    goto run_bootloader; /* pin grounded, so run bootloader */

  return; /* we've checked everything and there is no reason to run the bootloader */
#else
  if (RSTC_REGS->RSTC_RCAUSE & RSTC_RCAUSE_POR(1))
    double_tap = 0; /* a power up event should never be considered a 'double tap' */
  
  if (double_tap == DBL_TAP_MAGIC)
  {
    /* a 'double tap' has happened, so run bootloader */
    double_tap = 0;
    goto run_bootloader;
  }

  /* postpone boot for a short period of time; if a second reset happens during this window, the "magic" value will remain */
  double_tap = DBL_TAP_MAGIC;
  volatile int wait = 65536; while (wait--);
  /* however, if execution reaches this point, the window of opportunity has closed and the "magic" disappears  */
  double_tap = 0;
  return;
#endif

run_bootloader:
  /*
  configure oscillator for crystal-free USB operation (USBCRM / USB Clock Recovery Mode)
  */
  
  // SYSCTRL->OSC8M.bit.PRESC = 0;

  // SYSCTRL->INTFLAG.reg = SYSCTRL_INTFLAG_BOD33RDY | SYSCTRL_INTFLAG_BOD33DET | SYSCTRL_INTFLAG_DFLLRDY;
  SUPC_REGS->SUPC_INTFLAG = SUPC_INTFLAG_BOD33RDY(1) | SUPC_INTFLAG_BOD33DET(1);

  // NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_RWS_DUAL;
  NVMCTRL_REGS->NVMCTRL_CTRLA = NVMCTRL_CTRLA_RWS(0X2);

  OSCCTRL_REGS->OSCCTRL_DFLLCTRLA = 0; // See Errata 9905

  while (!(OSCCTRL_REGS->OSCCTRL_STATUS & OSCCTRL_STATUS_DFLLRDY(1)));

  OSCCTRL_REGS->OSCCTRL_DFLLMUL = OSCCTRL_DFLLMUL_MUL(48000);

  // OSCCTRL_REGS->OSCCTRL_DFLLVAL = OSCCTRL_DFLLVAL_COARSE( NVM_READ_CAL(NVM_DFLL48M_COARSE_CAL) ) | OSCCTRL_DFLLVAL_FINE( NVM_READ_CAL(NVM_DFLL48M_FINE_CAL) );
  OSCCTRL_REGS->OSCCTRL_DFLLCTRLB = OSCCTRL_DFLLCTRLB_BPLCKC(1) | OSCCTRL_DFLLCTRLB_CCDIS(1) | OSCCTRL_DFLLCTRLB_USBCRM(1) | OSCCTRL_DFLLCTRLB_STABLE(1) | OSCCTRL_DFLLCTRLB_MODE(1);
  OSCCTRL_REGS->OSCCTRL_DFLLCTRLA = OSCCTRL_DFLLCTRLA_ENABLE(1);

  while (!(OSCCTRL_REGS->OSCCTRL_STATUS & OSCCTRL_STATUS_DFLLRDY(1)));

  GCLK_REGS->GCLK_GENCTRL[0] = GCLK_GENCTRL_SRC_DFLL | GCLK_GENCTRL_RUNSTDBY(1) | GCLK_GENCTRL_GENEN(1);
  while (GCLK_REGS->GCLK_SYNCBUSY);

  /*
  initialize USB
  */

  PORT_REGS->GROUP[0].PORT_WRCONFIG = PORT_WRCONFIG_HWSEL(1) | PORT_WRCONFIG_WRPINCFG(1) | PORT_WRCONFIG_WRPMUX(1) | PORT_WRCONFIG_PMUXEN(1) | PORT_WRCONFIG_PMUX(PORT_PMUX_PMUXE_G_Val) | PORT_WRCONFIG_PINMASK(0x0300);

  MCLK_REGS->MCLK_APBBMASK |= MCLK_APBBMASK_USB(1);

  GCLK_REGS->GCLK_PCHCTRL[USB_GCLK_ID] = GCLK_PCHCTRL_GEN(0) | GCLK_PCHCTRL_CHEN(1);

  USB_REGS->DEVICE.USB_CTRLA = USB_CTRLA_SWRST(1);
  while (USB_REGS->DEVICE.USB_SYNCBUSY & USB_SYNCBUSY_SWRST(1));

  USB_REGS->DEVICE.USB_PADCAL = USB_PADCAL_TRANSN( (SW0_FUSES_REGS->FUSES_SW0_WORD_1 & FUSES_SW0_WORD_1_USB_TRANSN_Msk) >> FUSES_SW0_WORD_1_USB_TRANSN_Pos )
      | USB_PADCAL_TRANSP( (SW0_FUSES_REGS->FUSES_SW0_WORD_1 & FUSES_SW0_WORD_1_USB_TRANSP_Msk) >> FUSES_SW0_WORD_1_USB_TRANSP_Pos )
      | USB_PADCAL_TRIM( (SW0_FUSES_REGS->FUSES_SW0_WORD_1 & FUSES_SW0_WORD_1_USB_TRIM_Msk) >> FUSES_SW0_WORD_1_USB_TRIM_Pos );

  USB_REGS->DEVICE.USB_DESCADD = (uint32_t)udc_mem;

  USB_REGS->DEVICE.USB_CTRLA = USB_CTRLA_MODE_DEVICE | USB_CTRLA_RUNSTDBY(1);
  USB_REGS->DEVICE.USB_CTRLB = USB_DEVICE_CTRLB_SPDCONF_FS;
  USB_REGS->DEVICE.USB_CTRLA |= USB_CTRLA_ENABLE(1);

  /*
  service USB
  */

  while (1)
    USB_Service();
}

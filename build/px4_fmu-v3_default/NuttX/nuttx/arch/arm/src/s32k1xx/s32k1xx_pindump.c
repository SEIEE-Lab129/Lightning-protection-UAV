/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_pindump.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <debug.h>

#include <nuttx/irq.h>
#include "arm_arch.h"

#include "hardware/s32k1xx_gpio.h"
#include "hardware/s32k1xx_port.h"
#include "s32k1xx_pin.h"

#ifdef CONFIG_DEBUG_GPIO_INFO

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Port letters for prettier debug output */

static const char g_portchar[S32K1XX_NPORTS] =
{
#if S32K1XX_NPORTS > 9
#  error "Additional support required for this number of GPIOs"
#elif S32K1XX_NPORTS > 8
  'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I'
#elif S32K1XX_NPORTS > 7
  'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'
#elif S32K1XX_NPORTS > 6
  'A', 'B', 'C', 'D', 'E', 'F', 'G'
#elif S32K1XX_NPORTS > 5
  'A', 'B', 'C', 'D', 'E', 'F'
#elif S32K1XX_NPORTS > 4
  'A', 'B', 'C', 'D', 'E'
#elif S32K1XX_NPORTS > 3
  'A', 'B', 'C', 'D'
#elif S32K1XX_NPORTS > 2
  'A', 'B', 'C'
#elif S32K1XX_NPORTS > 1
  'A', 'B'
#elif S32K1XX_NPORTS > 0
  'A'
#else
#  error "Bad number of GPIOs"
#endif
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  s32k1xx_pindump
 *
 * Description:
 *   Dump all GPIO registers associated with the provided pin description
 *   along with a descriptive messasge.
 *
 ****************************************************************************/

void s32k1xx_pindump(uint32_t pinset, const char *msg)
{
  irqstate_t flags;
  uintptr_t base;
  int port;

  /* Decode the port and pin.  Use the port number to get the GPIO base
   * address.
   */

  port = (pinset & _PIN_PORT_MASK) >> _PIN_PORT_SHIFT;
  DEBUGASSERT((unsigned)port < S32K1XX_NPORTS);
  base = S32K1XX_GPIO_BASE(port);

  /* The following requires exclusive access to the GPIO registers */

  flags = enter_critical_section();

  gpioinfo("GPIO%c pinset: %08x base: %08x -- %s\n",
           g_portchar[port], pinset, base, msg);
  gpioinfo("  PDOR: %08x  PDIR: %08x  PDDR: %08x\n",
           getreg32(base + S32K1XX_GPIO_PDOR_OFFSET),
           getreg32(base + S32K1XX_GPIO_PDIR_OFFSET),
           getreg32(base + S32K1XX_GPIO_PDDR_OFFSET));

  leave_critical_section(flags);
}

#endif /* CONFIG_DEBUG_GPIO_INFO */

/* MIT License
 *
 * Copyright (c) 2023 - 2026 Andreas Merkle <web@blue-andi.de>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*******************************************************************************
    DESCRIPTION
*******************************************************************************/
/**
 *  @brief Mock WiFi class for ArduinoNative
 *  @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "WiFi.h"
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Prototypes
 *****************************************************************************/

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/* Extern WiFi Class. */
WiFi_ WiFi;

/******************************************************************************
 * Public Methods
 *****************************************************************************/

WiFi_::WiFi_()
{
}

WiFi_::~WiFi_()
{
}

String WiFi_::macAddress()
{
    char           simMAC[18U];
    uint32_t       processId                = static_cast<uint32_t>(getpid());
    const uint32_t OUI_UNICAST              = 0U << 16U;
    const uint32_t OUI_LOCALLY_ADMINISTERED = 1U << 17U;
    uint32_t       oui = OUI_UNICAST | OUI_LOCALLY_ADMINISTERED; /* Organisationally unique identifier */
    uint32_t       nic = 0U;                                     /* Network interface controller specific */

    /* The wifi MAC address is derived from the process id to ensure that
     * DCS can be started several times with different MAC addresses.
     */

    oui |= (processId >> 24U) & 0xFFU;
    nic |= processId & 0x00FFFFFFU;

    snprintf(simMAC, sizeof(simMAC), "%02X:%02X:%02X:%02X:%02X:%02X", (oui >> 16U) & 0xFFU, (oui >> 8U) & 0xFFU,
             (oui >> 0U) & 0xFFU, (nic >> 16U) & 0xFFU, (nic >> 8U) & 0xFFU, (nic >> 0U) & 0xFFU);

    return String(simMAC);
}

/******************************************************************************
 * Private Methods
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/

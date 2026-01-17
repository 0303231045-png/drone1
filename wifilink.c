/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2012 BitCraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * wifilink.c: ESP32 Wi-Fi implementation of the CRTP link
 */

#include <stdbool.h>
#include <string.h>

#include "config.h"
#include "wifilink.h"
#include "wifi_esp32.h"
#include "crtp.h"
#include "configblock.h"
#include "ledseq.h"
#include "pm_esplane.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "queuemonitor.h"
#include "semphr.h"
#include "stm32_legacy.h"

#define DEBUG_MODULE "WIFILINK"
#include "debug_cf.h"
#include "static_mem.h"

#define WIFI_ACTIVITY_TIMEOUT_MS (1000)

static bool isInit = false;
static xQueueHandle crtpPacketDelivery;
STATIC_MEM_QUEUE_ALLOC(crtpPacketDelivery, 16, sizeof(CRTPPacket));
static UDPPacket wifiIn;
static uint32_t lastPacketTick;

static int wifilinkSendPacket(CRTPPacket *p);
static int wifilinkSetEnable(bool enable);
static int wifilinkReceiveCRTPPacket(CRTPPacket *p);

STATIC_MEM_TASK_ALLOC(wifilinkTask, WIFILINK_TASK_STACKSIZE);

static bool wifilinkIsConnected(void)
{
    return (xTaskGetTickCount() - lastPacketTick) < M2T(WIFI_ACTIVITY_TIMEOUT_MS);
}

static struct crtpLinkOperations wifilinkOp = {
    .setEnable         = wifilinkSetEnable,
    .sendPacket        = wifilinkSendPacket,
    .receivePacket     = wifilinkReceiveCRTPPacket,
    .isConnected       = wifilinkIsConnected,
};

static void wifilinkTask(void *param)
{
    CRTPPacket p;

    while (1) {
        wifiGetDataBlocking(&wifiIn);
        lastPacketTick = xTaskGetTickCount();

        if (wifiIn.size < 1) {
            continue;
        }

        // parse CRTP
        p.header = wifiIn.data[0];
        p.size = wifiIn.size - 1;

        if (p.size > CRTP_MAX_DATA_SIZE) {
            p.size = CRTP_MAX_DATA_SIZE;
        }

        memcpy(p.data, &wifiIn.data[1], p.size);

        // DEBUG phai xoa
       // printf("[CRTP RX] header=0x%02X size=%d\n", p.header, p.size);
      //  for (int i = 0; i < p.size; i++) {
           // printf("D[%02d]=%02X ", i, p.data[i]);
     //   }
      //  printf("\n");

        // DÒNG QUYẾT ĐỊNH
        xQueueSend(crtpPacketDelivery, &p, M2T(5));
    }
}

static int wifilinkReceiveCRTPPacket(CRTPPacket *p)
{
    if (xQueueReceive(crtpPacketDelivery, p, M2T(100)) == pdTRUE) {
        ledseqRun(&seq_linkUp);
        return 0;
    }

    return -1;
}

static int wifilinkSendPacket(CRTPPacket *p)
{
    ASSERT(p->size <= CRTP_MAX_DATA_SIZE);
    ledseqRun(&seq_linkDown);
    return wifiSendData(p->size + 1, p->raw);
}

static int wifilinkSetEnable(bool enable)
{
    return 0;
}

/*
 * Public functions
 */

void wifilinkInit()
{
    if (isInit) {
        return;
    }

    crtpPacketDelivery = STATIC_MEM_QUEUE_CREATE(crtpPacketDelivery);
    //DEBUG_QUEUE_MONITOR_REGISTER(crtpPacketDelivery);

    STATIC_MEM_TASK_CREATE(wifilinkTask, wifilinkTask, WIFILINK_TASK_NAME,NULL, WIFILINK_TASK_PRI);

    isInit = true;
}

bool wifilinkTest()
{
    return isInit;
}

struct crtpLinkOperations *wifilinkGetLink()
{
    return &wifilinkOp;
}

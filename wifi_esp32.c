#include <string.h>

#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "queuemonitor.h"
#include "wifi_esp32.h"
#include "stm32_legacy.h"

#define DEBUG_MODULE  "WIFI_UDP"
#include "debug_cf.h"

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
#include "esp_mac.h"
#endif

#include "espnow.h"
#include "espnow_ctrl.h"
#include "espnow_utils.h"

/* ================= CONFIG ================= */
#define UDP_SERVER_PORT         2390
#define UDP_SERVER_BUFSIZE      64
/* ========================================== */

static struct sockaddr_storage source_addr;

static char WIFI_SSID[32] = "";
static char WIFI_PWD[64]  = CONFIG_WIFI_PASSWORD;
static uint8_t WIFI_CH   = CONFIG_WIFI_CHANNEL;
#define WIFI_MAX_STA_CONN CONFIG_WIFI_MAX_STA_CONN

#ifndef MAC2STR
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#endif

static int sock;
static xQueueHandle udpDataRx;
static xQueueHandle udpDataTx;

static bool isInit         = false;
static bool isUDPInit      = false;
static bool isUDPConnected = false;

/* ================= CHECKSUM ================= */
static uint8_t calculate_cksum(void *data, size_t len)
{
    uint8_t *c = data;
    uint8_t cksum = 0;
    for (size_t i = 0; i < len; i++) {
        cksum += c[i];
    }
    return cksum;
}
/* ============================================ */

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = event_data;
        DEBUG_PRINT_LOCAL("STA " MACSTR " joined", MAC2STR(event->mac));
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = event_data;
        DEBUG_PRINT_LOCAL("STA " MACSTR " left", MAC2STR(event->mac));
        //isUDPConnected = false;
    }
}

/* ================= API ================= */

bool wifiTest(void)
{
    return isInit;
}

bool wifiGetDataBlocking(UDPPacket *in)
{
    while (xQueueReceive(udpDataRx, in, portMAX_DELAY) != pdTRUE) {
        vTaskDelay(M2T(10));
    }
    return true;
}

bool wifiSendData(uint32_t size, uint8_t *data)
{
    if (size > WIFI_RX_TX_PACKET_SIZE - 1) {
        return false;
    }

    UDPPacket out = {0};
    out.size = size;
    memcpy(out.data, data, size);

    return (xQueueSend(udpDataTx, &out, M2T(100)) == pdTRUE);
}

/* ================= UDP ================= */

static esp_err_t udp_server_create(void *arg)
{
    if (isUDPInit) return ESP_OK;

    struct sockaddr_in addr = {0};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(UDP_SERVER_PORT);

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        DEBUG_PRINT_LOCAL("socket create failed");
        return ESP_FAIL;
    }

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) 
    {
        DEBUG_PRINT_LOCAL("socket bind failed");
        return ESP_FAIL;
    }
        struct sockaddr_in *dst = (struct sockaddr_in *)&source_addr;
dst->sin_family = AF_INET;
dst->sin_port   = htons(UDP_SERVER_PORT);
dst->sin_addr.s_addr = inet_addr("192.168.43.43"); // IP app

    isUDPInit = true;

    /* ?? FIX QUAN TR?NG:
       App ESP-Drone / LiteWing KH NG g?i g i d?u
       ? cho ph p TX ngay */
    isUDPConnected = true;

    DEBUG_PRINT_LOCAL("UDP socket ready");
    return ESP_OK;
}

static void udp_server_rx_task(void *pv)
{
    socklen_t len_addr = sizeof(source_addr);
    uint8_t rxbuf[UDP_SERVER_BUFSIZE];
    UDPPacket pkt;
    

    while (1) {
        if (!isUDPInit) {
            vTaskDelay(20);
            continue;
        }

        int len = recvfrom(sock, rxbuf, sizeof(rxbuf), 0,
                           (struct sockaddr *)&source_addr,
                           &len_addr);
        if (len <= 1) continue;

        uint8_t cksum = rxbuf[len - 1];
        if (cksum != calculate_cksum(rxbuf, len - 1)) {
            DEBUG_PRINT_LOCAL("checksum error");
            continue;
        }

        pkt.size = len - 1;
        memcpy(pkt.data, rxbuf, pkt.size);
        xQueueSend(udpDataRx, &pkt, 0);
        //DEBUG_PRINT_LOCAL("RX %d bytes", pkt.size);

        isUDPConnected = true;
    }
}

static void udp_server_tx_task(void *pv)
{
    UDPPacket pkt;

    while (1) {
        if (!isUDPInit) {
            vTaskDelay(20);
            continue;
        }

        if (xQueueReceive(udpDataTx, &pkt, portMAX_DELAY) == pdTRUE &&
            isUDPConnected) {

            pkt.data[pkt.size] = calculate_cksum(pkt.data, pkt.size);
            pkt.size++;

            sendto(sock, pkt.data, pkt.size, 0,
                   (struct sockaddr *)&source_addr,
                   sizeof(source_addr));
                   //DEBUG_PRINT_LOCAL("TX %d bytes", pkt.size);
        }
    }
}

/* ================= INIT ================= */

void wifiInit(void)
{
    if (isInit) return;

    udpDataRx = xQueueCreate(16, sizeof(UDPPacket));
    udpDataTx = xQueueCreate(16, sizeof(UDPPacket));

  
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));

    uint8_t mac[6];
    esp_wifi_get_mac(ESP_IF_WIFI_AP, mac);

    sprintf(WIFI_SSID, "%s_%02X%02X%02X%02X%02X%02X",
            CONFIG_WIFI_BASE_SSID,
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    wifi_config_t wifi_cfg = {
        .ap = {
            .channel = WIFI_CH,
            .max_connection = WIFI_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };

    strcpy((char *)wifi_cfg.ap.ssid, WIFI_SSID);
    strcpy((char *)wifi_cfg.ap.password, WIFI_PWD);
    wifi_cfg.ap.ssid_len = strlen(WIFI_SSID);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* IP TINH   B?T BU?C CHO ESP-DRONE / LITEWING */
    esp_netif_ip_info_t ip = {
        .ip.addr = ipaddr_addr("192.168.43.42"),
        .netmask.addr = ipaddr_addr("255.255.255.0"),
        .gw.addr = ipaddr_addr("192.168.43.42"),
    };

    esp_netif_dhcps_stop(ap_netif);
    esp_netif_set_ip_info(ap_netif, &ip);
    esp_netif_dhcps_start(ap_netif);

    udp_server_create(NULL);

    xTaskCreate(udp_server_rx_task, "udp_rx", 4096, NULL, 5, NULL);
    xTaskCreate(udp_server_tx_task, "udp_tx", 4096, NULL, 5, NULL);

    isInit = true;
    DEBUG_PRINT_LOCAL("WiFi + UDP READY");
}


/*
 * OpenLaserDAC - Captive Portal DNS Server
 * Minimal DNS responder: answers every A-record query with the AP's own IP.
 * This makes captive-portal probes (Android/iOS/Windows) resolve to us,
 * so the HTTP captive-portal handlers are actually reachable.
 */

#include "dns_server.h"
#include "config.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include <string.h>

static const char* TAG = "DNS";

#define DNS_PORT       53
#define DNS_BUF_SIZE   512

static int       s_sock = -1;
static TaskHandle_t s_task = NULL;
static uint32_t  s_reply_ip;          /* network byte order */

/*
 * DNS header (RFC 1035 §4.1.1)
 */
typedef struct __attribute__((packed)) {
    uint16_t id;
    uint16_t flags;
    uint16_t qdcount;
    uint16_t ancount;
    uint16_t nscount;
    uint16_t arcount;
} dns_header_t;

/*
 * Build a minimal response:
 *   - copy the query header + question section
 *   - flip QR bit, set RA, ancount=1
 *   - append one A-record answer (pointer to name in question, TTL=60, A=ap_ip)
 *
 * Returns total response length, or 0 on error.
 */
static int build_response(const uint8_t* query, int qlen, uint8_t* resp, int resp_cap)
{
    if (qlen < (int)sizeof(dns_header_t) + 5) return 0;   /* too short */

    const dns_header_t* qh = (const dns_header_t*)query;
    if (ntohs(qh->qdcount) == 0) return 0;

    /* Walk past the first QNAME to find QTYPE+QCLASS (skip labels) */
    int pos = sizeof(dns_header_t);
    while (pos < qlen) {
        uint8_t len = query[pos];
        if (len == 0) { pos++; break; }          /* root label terminator */
        if ((len & 0xC0) == 0xC0) { pos += 2; break; } /* compressed */
        pos += 1 + len;
    }
    if (pos + 4 > qlen) return 0;                /* need QTYPE+QCLASS */
    int question_end = pos + 4;                   /* end of first question */

    /* Only answer A (type 1) / IN (class 1) queries */
    uint16_t qtype  = (query[pos] << 8) | query[pos + 1];
    uint16_t qclass = (query[pos + 2] << 8) | query[pos + 3];
    if (qtype != 1 || qclass != 1) return 0;

    int resp_len = question_end + 16;             /* answer: ptr(2)+type(2)+class(2)+ttl(4)+rdlen(2)+rdata(4) = 16 */
    if (resp_len > resp_cap) return 0;

    /* Copy header + question */
    memcpy(resp, query, question_end);

    /* Patch header flags */
    dns_header_t* rh = (dns_header_t*)resp;
    rh->flags   = htons(0x8180);                  /* QR=1, AA=1, RA=1, RCODE=0 */
    rh->ancount = htons(1);
    rh->nscount = 0;
    rh->arcount = 0;

    /* Append answer RR */
    uint8_t* a = resp + question_end;
    a[0] = 0xC0; a[1] = 0x0C;                    /* name pointer → offset 12 (question name) */
    a[2] = 0x00; a[3] = 0x01;                    /* TYPE  A */
    a[4] = 0x00; a[5] = 0x01;                    /* CLASS IN */
    a[6] = 0x00; a[7] = 0x00; a[8] = 0x00; a[9] = 0x3C; /* TTL 60s */
    a[10] = 0x00; a[11] = 0x04;                  /* RDLENGTH 4 */
    memcpy(&a[12], &s_reply_ip, 4);              /* RDATA = AP IP (already network order) */

    return resp_len;
}

static void dns_task(void* arg)
{
    uint8_t buf[DNS_BUF_SIZE];
    uint8_t resp[DNS_BUF_SIZE];

    ESP_LOGI(TAG, "Captive portal DNS running (port %d)", DNS_PORT);

    while (s_sock >= 0) {
        struct sockaddr_in src;
        socklen_t slen = sizeof(src);
        int n = recvfrom(s_sock, buf, sizeof(buf), 0,
                         (struct sockaddr*)&src, &slen);
        if (n <= 0) continue;

        int rlen = build_response(buf, n, resp, sizeof(resp));
        if (rlen > 0) {
            sendto(s_sock, resp, rlen, 0,
                   (struct sockaddr*)&src, slen);
        }
    }

    vTaskDelete(NULL);
}

esp_err_t dns_server_start(uint32_t ap_ip)
{
    if (s_sock >= 0) return ESP_OK;               /* already running */

    s_reply_ip = ap_ip;                           /* already network order */

    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_sock < 0) {
        ESP_LOGE(TAG, "socket: errno %d", errno);
        return ESP_FAIL;
    }

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port   = htons(DNS_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(s_sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind :53 failed: errno %d", errno);
        close(s_sock);
        s_sock = -1;
        return ESP_FAIL;
    }

    if (xTaskCreatePinnedToCore(dns_task, "dns", 3072, NULL, 5,
                                &s_task, CORE_SERVICES) != pdPASS) {
        ESP_LOGE(TAG, "task create failed");
        close(s_sock);
        s_sock = -1;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Captive DNS → " IPSTR, IP2STR((esp_ip4_addr_t*)&s_reply_ip));
    return ESP_OK;
}

esp_err_t dns_server_stop(void)
{
    if (s_sock >= 0) {
        int sock = s_sock;
        s_sock = -1;          /* signal task to exit */
        close(sock);          /* unblocks recvfrom() */
    }
    s_task = NULL;
    return ESP_OK;
}

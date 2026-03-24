/*
 * OpenLaserDAC - Captive Portal DNS Server
 * Responds to ALL DNS queries with the AP gateway IP so that
 * captive-portal detection (Android/iOS/Windows) reaches our HTTP server.
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Start the DNS server (binds UDP :53, resolves everything to ap_ip). */
esp_err_t dns_server_start(uint32_t ap_ip);

/** Stop the DNS server and free resources. */
esp_err_t dns_server_stop(void);

#ifdef __cplusplus
}
#endif

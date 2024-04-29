/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "esp_crt_bundle.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif_sntp.h"
#include "esp_sleep.h"
#include "esp_sntp.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_tls.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "lwip/ip_addr.h"
#include "nvs_flash.h"
#include <esp_timer.h>
#include <string.h>

#include "lwip/dns.h"
#include "lwip/err.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"

#include "cJSON.h"

#include <hal/Timing.hxx>

#include <ESP32BinaryValueReader.hxx>
#include <ESP32BinaryValueWriter.hxx>
#include <ESP32Persistence.hxx>
#include <ESP32Task.hxx>

#include <staircase/BasicLight.hxx>
#include <staircase/BasicMovingFactory.hxx>
#include <staircase/ProximitySensor.hxx>
#include <staircase/StaircaseLooper.hxx>
#include <staircase/StaircaseRunnable.hxx>

#include <algorithm>
#include <array>
#include <sstream>

/* The examples use WiFi configuration that you can set via project
   configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define STAIRCASE_WIFI_SSID CONFIG_ESP_WIFI_SSID
#define STAIRCASE_WIFI_PASS CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY CONFIG_ESP_MAXIMUM_RETRY

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define STAIRCASE_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define STAIRCASE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define STAIRCASE_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define STAIRCASE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define STAIRCASE_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define STAIRCASE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define STAIRCASE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define STAIRCASE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define STAIRCASE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define STAIRCASE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define STAIRCASE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define STAIRCASE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define STAIRCASE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define STAIRCASE_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

static const char *TAG = "staircase-automate";

#define STORAGE_NAME "storage"
#define STORAGE_TIME_ID "s_time"
#define STORAGE_DOWN_FILTER_ID "s_dfilter"
#define STORAGE_UP_FILTER_ID "s_ufilter"

static esp_err_t set_timezone(const char *timezone) {
    if (setenv("TZ", timezone, 1) == -1) {
        return ESP_FAIL;
    }

    tzset();

    return ESP_OK;
}

static esp_err_t load_time(nvs_handle_t handle) {
    int64_t storedTime;
    esp_err_t ret = nvs_get_i64(handle, STORAGE_TIME_ID, &storedTime);
    if (ret != ESP_OK) {
        return ret;
    }

    timeval tv{storedTime, 0};

    if (settimeofday(&tv, NULL) != 0) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t store_time(nvs_handle_t handle, int64_t offset_seconds) {
    timeval tv{0, 0};

    if (gettimeofday(&tv, NULL) != 0) {
        return ESP_FAIL;
    }

    int64_t newTime = tv.tv_sec + offset_seconds;
    return nvs_set_i64(handle, STORAGE_TIME_ID, newTime);
}

static void print_current_time() {
    time_t now = time(NULL);
    char *time_str = ctime(&now);
    ESP_LOGI(TAG, "The current date/time is : %s", time_str);
}

static esp_err_t init_flash(nvs_handle_t *handle) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ret = nvs_flash_erase();
        if (ret != ESP_OK) {
            return ret;
        }
        ret = nvs_flash_init();
        if (ret != ESP_OK) {
            return ret;
        }
    }

    handle = 0;
    return nvs_open(STORAGE_NAME, NVS_READWRITE, handle);
}

static esp_err_t deinit_flash(nvs_handle_t handle) {
    esp_err_t ret = nvs_commit(handle);
    if (ret != ESP_OK) {
        return ret;
    }

    nvs_close(handle);
    return nvs_flash_deinit();
}

static esp_err_t configure_sntp() {
    esp_sntp_config_t config;
    config.smooth_sync = false;
    config.server_from_dhcp = true;
    config.wait_for_sync = true;
    config.start = true;
    config.sync_cb = NULL;
    config.index_of_first_server = 0;
    config.num_of_servers = 1;
    config.servers[0] = "132.163.97.2";
    config.start = true;
    config.server_from_dhcp = true;
    config.renew_servers_after_new_IP = true;
    config.index_of_first_server = 1;
    config.ip_event_to_renew = static_cast<ip_event_t>(IP_EVENT_STA_GOT_IP);
    return esp_netif_sntp_init(&config);
}

static void wait_for_time_sync() {
    while (esp_netif_sntp_sync_wait(2 * 1000 / portTICK_PERIOD_MS) ==
           ESP_ERR_TIMEOUT) {
        vTaskDelay(2 * 1000 / portTICK_PERIOD_MS);
    }
}

#define WIFI_STA_CONNECTED_BIT BIT0

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
    EventGroupHandle_t wifi_event_group = (EventGroupHandle_t)arg;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT &&
               event_id == WIFI_EVENT_STA_DISCONNECTED) {
        vTaskDelay(10 * 60 * 1000 / portTICK_PERIOD_MS);
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(wifi_event_group, WIFI_STA_CONNECTED_BIT);
    }
}

static void wifi_init() {
    EventGroupHandle_t wifi_event_group = xEventGroupCreate();
    if (wifi_event_group == NULL) {
        ESP_ERROR_CHECK(ESP_FAIL);
    }

    esp_netif_create_default_wifi_sta();

    {
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    }

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, wifi_event_group,
        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, wifi_event_group,
        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta =
            {
                .ssid = STAIRCASE_WIFI_SSID,
                .password = STAIRCASE_WIFI_PASS,
                .scan_method = WIFI_FAST_SCAN,
                .bssid_set = false,
                .bssid = {0},
                .channel = 0,
                .listen_interval = 0,
                .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
                .threshold = {.rssi = 0,
                              .authmode =
                                  STAIRCASE_WIFI_SCAN_AUTH_MODE_THRESHOLD},
                .sae_pwe_h2e = STAIRCASE_WIFI_SAE_MODE,
                .sae_h2e_identifier = STAIRCASE_H2E_IDENTIFIER,
            },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits;
    do {
        bits = xEventGroupWaitBits(wifi_event_group, WIFI_STA_CONNECTED_BIT,
                                   pdFALSE, pdFALSE, portMAX_DELAY);
    } while (!(bits & WIFI_STA_CONNECTED_BIT));
}

#define STAIRCASE_LATITUDE CONFIG_LATITUDE
#define STAIRCASE_LONGITUDE CONFIG_LONGITUDE

#define SUN_TIME_HTTP_SERVER "api.sunrise-sunset.org"
#define SUN_TIME_HTTP_URL_FORMAT                                               \
    "https://api.sunrise-sunset.org/"                                          \
    "json?lat=" STAIRCASE_LATITUDE "&lng=" STAIRCASE_LONGITUDE                 \
    "&date=%s&formatted=0&tzid=Europe/"                                        \
    "Belgrade"

#define SUN_TIME_HTTP_GET_REQUEST_FORMAT                                       \
    "GET %s HTTP/1.1\r\n"                                                      \
    "Host: %s\r\n"                                                             \
    "User-Agent: esp-idf/1.0 esp32\r\n"                                        \
    "\r\n"

esp_err_t get_sunrise_sunset_via_http(const char *date, cJSON **json) {
    if (json == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    char *recv_data = (char *)malloc(1024);
    if (recv_data == NULL) {
        return ESP_ERR_NO_MEM;
    }

    recv_data[0] = '\0';
    ssize_t recv_len = 0;

    esp_tls_cfg_t cfg = {
        .crt_bundle_attach = esp_crt_bundle_attach,
    };

    esp_tls_t *tls = esp_tls_init();
    if (tls == NULL) {
        free(recv_data);
        return ESP_ERR_NO_MEM;
    }

    if (esp_tls_conn_http_new_sync(SUN_TIME_HTTP_SERVER, &cfg, tls) != 1) {
        free(recv_data);
        esp_tls_conn_destroy(tls);
        return ESP_ERR_NO_MEM;
    }

    {
        char url[128];
        snprintf(url, 128, SUN_TIME_HTTP_URL_FORMAT, date);
        char get_req[256];
        const ssize_t len =
            snprintf(get_req, 256, SUN_TIME_HTTP_GET_REQUEST_FORMAT, url,
                     SUN_TIME_HTTP_SERVER);

        ssize_t written_bytes = 0;
        do {
            ssize_t ret = esp_tls_conn_write(tls, get_req + written_bytes,
                                             len - written_bytes);
            if (ret >= 0) {
                written_bytes += ret;
            } else if (ret != ESP_TLS_ERR_SSL_WANT_READ &&
                       ret != ESP_TLS_ERR_SSL_WANT_WRITE) {
                free(recv_data);
                esp_tls_conn_destroy(tls);
                return ESP_FAIL;
            }
        } while (written_bytes < len);
    }

    {
        bool json_found = false;
        do {
            char buf[128];
            ssize_t ret = esp_tls_conn_read(tls, buf, sizeof(buf));

            if (ret == ESP_TLS_ERR_SSL_WANT_WRITE ||
                ret == ESP_TLS_ERR_SSL_WANT_READ) {
                break;
            } else if (ret < 0) {
                free(recv_data);
                esp_tls_conn_destroy(tls);
                return ESP_FAIL;
            } else if (ret == 0) {
                break;
            }

            if (json_found) {
                recv_len += strncat(recv_data, buf, 1024 - recv_len);
            } else {
                char *bracket_pos = strchr(buf, '{');
                if (bracket_pos != NULL) {
                    recv_len +=
                        strncat(recv_data, bracket_pos, 1024 - recv_len);
                    json_found = true;
                }
            }
        } while (true);

        if (!json_found) {
            free(recv_data);
            esp_tls_conn_destroy(tls);
            return ESP_FAIL;
        }
    }

    if (esp_tls_conn_destroy(tls) != 0) {
        free(recv_data);
        return ESP_FAIL;
    }

    char *bracket_pos = strrchr(recv_data, '}');

    if (bracket_pos == NULL) {
        free(recv_data);
        return ESP_FAIL;
    }

    bracket_pos[1] = '\0';

    *json = cJSON_Parse(recv_data);
    free(recv_data);
    return ESP_OK;
}

#define JSON_STATUS_FIELD "status"
#define JSON_STATUS_OK "OK"
#define JSON_RESULTS_FIELD "status"
#define JSON_SUNRISE_FIELD "sunrise"
#define JSON_SUNSET_FIELD "sunset"

esp_err_t check_sunrise_sunset_response(const cJSON *json) {
    const cJSON *status = cJSON_GetObjectItem(json, JSON_STATUS_FIELD);
    if (!cJSON_IsString(status) || (status->valuestring == NULL)) {
        return ESP_FAIL;
    }

    if (strcmp(status->valuestring, JSON_STATUS_OK) != 0) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

struct sun_day_data {
    int sunrise_hr;
    int sunrise_min;
    int sunset_hr;
    int sunset_min;
};

esp_err_t parse_sunrise_sunset_response(const cJSON *json, sun_day_data *data) {
    if ((json == NULL) || (data == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    const cJSON *results = cJSON_GetObjectItem(json, JSON_RESULTS_FIELD);
    if (!cJSON_IsObject(results)) {
        return ESP_FAIL;
    }

    const cJSON *sunrise = cJSON_GetObjectItem(results, JSON_SUNRISE_FIELD);
    if (!cJSON_IsString(sunrise) || (sunrise->valuestring == NULL)) {
        return ESP_FAIL;
    }

    const cJSON *sunset = cJSON_GetObjectItem(results, JSON_SUNSET_FIELD);
    if (!cJSON_IsString(sunset) || (sunset->valuestring == NULL)) {
        return ESP_FAIL;
    }

    sscanf(sunrise->valuestring + 11, "%d:%d", &data->sunrise_hr,
           &data->sunrise_min);
    sscanf(sunset->valuestring + 11, "%d:%d", &data->sunset_hr,
           &data->sunset_min);

    return ESP_OK;
}

#define TIME_HISTEREZIS_MIN (15)

esp_err_t add_sunrise_sunset_histerezis(sun_day_data *data) {
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    data->sunrise_min += 15;
    if (data->sunrise_min > 59) {
        data->sunrise_min -= 60;
        data->sunrise_hr++;
    }

    data->sunset_min -= 15;
    if (data->sunset_min < 0) {
        data->sunset_min += 60;
        data->sunset_hr--;
    }
}

enum staircase_day_phase { BEFORE_SUNRISE, DAY, AFTER_SUNSET };

esp_err_t get_staircase_day_phase(const sun_day_data *data,
                                  staircase_day_phase *day_phase) {

    if ((data == NULL) || (day_phase == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    time_t now = time(NULL);
    tm local_time;
    localtime_r(&now, &local_time);

    if ((local_time.tm_hour < data->sunrise_hr) ||
        ((local_time.tm_hour == data->sunrise_hr) &&
         (local_time.tm_min < data->sunrise_min))) {
        return BEFORE_SUNRISE;
    }

    if ((local_time.tm_hour < data->sunset_hr) ||
        ((local_time.tm_hour == data->sunset_hr) &&
         (local_time.tm_min < data->sunset_min))) {
        return DAY;
    }

    return AFTER_SUNSET;
}

extern "C" void app_main(void) {

    // 1. initialize flash
    // 2. load and setup time
    // 3. start task for staircase
    // 4. setup wifi (and pre sntp, what is needed)
    // 5. start wifi
    // 6. wait some time with retries to establish wifi connection
    // success:
    //  -> start sntp
    //  -> wait some time to sync with retries
    //  -> gether intel on sunrise (either by calculating it or by
    //  http(preferably))
    // 7. task sleep until sunrise + delta
    // 8. calculate when to wake up (sunset - delta)
    // 9. store new time
    // stop sntp if needed stop wifi (if needed)
    // stop handling of staircase
    // 10. deinitialize flash
    // 11. enter deepsleep

    // change interface of looper to accept only references to filters

    ESP_ERROR_CHECK(set_timezone("CET-1CEST,M3.5.0,M10.5.0/3"));

    nvs_handle_t handle;
    ESP_ERROR_CHECK(init_flash(&handle));
    if (load_time(handle) != ESP_OK) {
        ESP_LOGI(TAG, "Failed to load time from persistence, continue...");
    }

    print_current_time();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(configure_sntp());

    wifi_init();

    wait_for_time_sync();

    print_current_time();

    sun_day_data today_data;
    {
        cJSON *today_json = NULL;
        esp_err_t ret;
        do {
            vTaskDelay(2 * 1000 / portTICK_PERIOD_MS);
            ret = get_sunrise_sunset_via_http("today", &today_json);
            if (ret != ESP_OK) {
                continue;
            }
            ret = check_sunrise_sunset_response(today_json);
            if (ret != ESP_OK) {
                continue;
            }
            ret = parse_sunrise_sunset_response(today_json, &today_data);
            if (ret != ESP_OK) {
                continue;
            }
            ret = add_sunrise_sunset_histerezis(&today_data);
        } while (ret != ESP_OK);

        cJSON_Delete(today_json);
    }

    sun_day_data tomorrow_data;
    {
        cJSON *tomorrow_json = NULL;
        esp_err_t ret;
        do {
            vTaskDelay(2 * 1000 / portTICK_PERIOD_MS);
            ret = get_sunrise_sunset_via_http("tomorrow", &tomorrow_json);
            if (ret != ESP_OK) {
                continue;
            }
            ret = check_sunrise_sunset_response(tomorrow_json);
            if (ret != ESP_OK) {
                continue;
            }
            ret = parse_sunrise_sunset_response(tomorrow_json, &tomorrow_data);
            if (ret != ESP_OK) {
                continue;
            }
            ret = add_sunrise_sunset_histerezis(&tomorrow_data);
        } while (ret != ESP_OK);

        cJSON_Delete(tomorrow_json);
    }

    

    delay_until_tomorrow_sunrise();

    int64_t seconds_to_sleep = 60;

    ESP_ERROR_CHECK(store_time(handle, seconds_to_sleep));

    ESP_ERROR_CHECK(deinit_flash(handle));

    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(
        static_cast<uint64_t>(seconds_to_sleep) * 1000000));
    esp_deep_sleep_start();

    // TaskHandle_t xHandle = NULL;
    // xTaskCreate(power_management, "power_management", 1024, NULL,
    // tskIDLE_PRIORITY, &xHandle);

    // for (;;)
    // {
    // xTaskCreate(&https_request_task, "https_get_task", 8192, NULL, 5, NULL);

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT)
     * or connection failed for the maximum number of re-tries (WIFI_FAIL_BIT).
     * The bits are set by event_handler() (see above) */
    // EventBits_t bits = 0;

    // while (bits == 0)
    // {
    //     bits = xEventGroupWaitBits(s_https_connection_event_group,
    //                                         TIME_ACQUIRED_BIT,
    //                                         pdFALSE, pdFALSE, portMAX_DELAY);
    // }

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we
     * can test which event actually happened. */

    // }

    // HAL objects
    // std::array<esp32::ESP32BinaryValueWriter,
    //            staircase::IBasicLight::kLightsNum>
    //     kOutputs{esp32::ESP32BinaryValueWriter{GPIO_NUM_4},
    //              esp32::ESP32BinaryValueWriter{GPIO_NUM_16},
    //              esp32::ESP32BinaryValueWriter{GPIO_NUM_17},
    //              esp32::ESP32BinaryValueWriter{GPIO_NUM_18},
    //              esp32::ESP32BinaryValueWriter{GPIO_NUM_19},
    //              esp32::ESP32BinaryValueWriter{GPIO_NUM_21},
    //              esp32::ESP32BinaryValueWriter{GPIO_NUM_22},
    //              esp32::ESP32BinaryValueWriter{GPIO_NUM_23}};

    // esp32::ESP32BinaryValueReader kInputDown{GPIO_NUM_25};

    // esp32::ESP32BinaryValueReader kInputUp{GPIO_NUM_26};

    // if (std::any_of(std::begin(kOutputs), std::end(kOutputs),
    //                 [](auto &out) { return out.init() != ESP_OK; })) {
    //     ESP_LOGE(TAG, "Failed to initialize all the ports, abort");
    //     std::abort();
    // }

    // if (kInputDown.init() != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to initialize up port, abort");
    //     std::abort();
    // }

    // if (kInputUp.init() != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to initialize down port, abort");
    //     std::abort();
    // }

    // // Staircase objects
    // std::array<staircase::BasicLight, staircase::IBasicLight::kLightsNum>
    //     kLights{staircase::BasicLight{kOutputs[0]},
    //             staircase::BasicLight{kOutputs[1]},
    //             staircase::BasicLight{kOutputs[2]},
    //             staircase::BasicLight{kOutputs[3]},
    //             staircase::BasicLight{kOutputs[4]},
    //             staircase::BasicLight{kOutputs[5]},
    //             staircase::BasicLight{kOutputs[6]},
    //             staircase::BasicLight{kOutputs[7]}};

    // staircase::BasicLights kBasicLights{kLights[0], kLights[1], kLights[2],
    //                                     kLights[3], kLights[4], kLights[5],
    //                                     kLights[6], kLights[7]};

    // staircase::ProximitySensor kDownSensor{kInputDown};
    // staircase::ProximitySensor kUpSensor{kInputUp};

    // staircase::BasicMovingFactory kMovingFactory;

    // staircase::StaircaseLooper kLooper{kBasicLights, kDownSensor, kUpSensor,
    //                                    kMovingFactory};

    // // Tasks
    // staircase::StaircaseRunnable kStaircaseRunnable{kLooper};
    // esp32::ESP32Task kStaircaseTask{
    //     "staircase", 1024, 0, kStaircaseRunnable,
    //     staircase::StaircaseRunnable::kUpdateInterval};

    // kStaircaseTask.init();

    // std::int64_t currentTime = esp_timer_get_time();

    // while (1) {
    //     std::int64_t newCurrentTime = esp_timer_get_time();
    //     std::int64_t delta = newCurrentTime - currentTime;

    //     if (delta >= 1000000) {
    //         std::lock_guard<std::mutex> lock = kLooper.block();
    //         currentTime = newCurrentTime;

    //         std::ostringstream stream;

    //         stream << "Sensors: down "
    //                       << static_cast<int>(kDownSensor.isClose())
    //                       << " ------- "
    //                       << static_cast<int>(kUpSensor.isClose()) << " up";

    //         ESP_LOGI(TAG, "%s", stream.str().c_str());

    //         stream.str() = "";
    //         stream << "Lights: ";

    //         std::for_each(
    //             std::begin(kLights), std::end(kLights), [&](const auto
    //             &light) {
    //                 stream << static_cast<int>(light.isOn()) << " ";
    //             });

    //         ESP_LOGI(TAG, "%s", stream.str().c_str());
    //     }

    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }
}

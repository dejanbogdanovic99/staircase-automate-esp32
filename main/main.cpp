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
#include "esp_timer.h"
#include "esp_tls.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "lwip/ip_addr.h"
#include "nvs_flash.h"
#include "string.h"

#include "lwip/dns.h"
#include "lwip/err.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"

#include "cJSON.h"

#include <hal/Timing.hxx>

#include <ESP32BinaryValueReader.hxx>
#include <ESP32BinaryValueWriter.hxx>

#include <staircase/BasicLight.hxx>
#include <staircase/BasicMovingFactory.hxx>
#include <staircase/ClippedSquaredMovingDurationCalculator.hxx>
#include <staircase/MTAMovingTimeFilter.hxx>
#include <staircase/ProximitySensor.hxx>
#include <staircase/StaircaseLooper.hxx>

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

#define STORAGE_NAME "ts"
#define STORAGE_TIME_ID "s_time"
#define STORAGE_DOWN_FILTER_ID "s_dfilter"
#define STORAGE_UP_FILTER_ID "s_ufilter"

static bool wifi_working = true;

static esp_err_t set_timezone(const char *timezone) {
    if (setenv("TZ", timezone, 1) == -1) {
        ESP_LOGE(TAG, "Failure: %s:%d", __func__, __LINE__);
        return ESP_FAIL;
    }

    tzset();

    return ESP_OK;
}

static esp_err_t load_time(nvs_handle_t handle) {
    int64_t storedTime;
    esp_err_t ret = nvs_get_i64(handle, STORAGE_TIME_ID, &storedTime);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failure: %s:%d", __func__, __LINE__);
        return ret;
    }

    timeval tv{storedTime, 0};

    if (settimeofday(&tv, NULL) != 0) {
        ESP_LOGE(TAG, "Failure: %s:%d", __func__, __LINE__);
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t store_time(nvs_handle_t handle, int64_t offset_seconds) {
    timeval tv{0, 0};

    if (gettimeofday(&tv, NULL) != 0) {
        ESP_LOGE(TAG, "Failure: %s:%d", __func__, __LINE__);
        return ESP_FAIL;
    }

    int64_t newTime = tv.tv_sec + offset_seconds;
    return nvs_set_i64(handle, STORAGE_TIME_ID, newTime);
}

static esp_err_t load_filter_values(nvs_handle_t handle,
                                    staircase::IMovingTimeFilter &down_filter,
                                    staircase::IMovingTimeFilter &up_filter) {
    uint32_t down_value, up_value;
    esp_err_t ret = nvs_get_u32(handle, STORAGE_DOWN_FILTER_ID, &down_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failure: %s:%d", __func__, __LINE__);
        return ret;
    }

    ret = nvs_get_u32(handle, STORAGE_UP_FILTER_ID, &up_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failure: %s:%d", __func__, __LINE__);
        return ret;
    }

    down_filter.reset(static_cast<hal::Milliseconds>(down_value));
    up_filter.reset(static_cast<hal::Milliseconds>(up_value));
    return ESP_OK;
}

static esp_err_t
store_filter_values(nvs_handle_t handle,
                    const staircase::IMovingTimeFilter &down_filter,
                    const staircase::IMovingTimeFilter &up_filter) {
    esp_err_t ret =
        nvs_set_u32(handle, STORAGE_DOWN_FILTER_ID,
                    static_cast<uint32_t>(down_filter.getCurrentMovingTime()));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failure: %s:%d", __func__, __LINE__);
        return ret;
    }

    ret = nvs_set_u32(handle, STORAGE_UP_FILTER_ID,
                      static_cast<uint32_t>(up_filter.getCurrentMovingTime()));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failure: %s:%d", __func__, __LINE__);
        return ret;
    }

    return ESP_OK;
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
            ESP_LOGE(TAG, "Failure: %s:%d", __func__, __LINE__);
            return ret;
        }
        ret = nvs_flash_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failure: %s:%d", __func__, __LINE__);
            return ret;
        }
    }

    return nvs_open(STORAGE_NAME, NVS_READWRITE, handle);
}

static esp_err_t deinit_flash(nvs_handle_t handle) {
    esp_err_t ret = nvs_commit(handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failure: %s:%d", __func__, __LINE__);
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

#define SNTP_SYNC_WAIT_PERIOD_MS (2 * 1000)

static void wait_for_time_sync() {
    while (esp_netif_sntp_sync_wait(SNTP_SYNC_WAIT_PERIOD_MS / portTICK_PERIOD_MS) ==
           ESP_ERR_TIMEOUT) {
        ESP_LOGI(TAG, "Waiting for time sync...");
    }
}

#define WIFI_STA_CONNECTED_BIT BIT0

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
    EventGroupHandle_t wifi_event_group = (EventGroupHandle_t)arg;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());
    } else if (event_base == WIFI_EVENT &&
               event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "WiFi disconnected, reconnecting...");
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "WiFi connected!");
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

    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, wifi_event_group));
    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, wifi_event_group));

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

static void wifi_deinit() {
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler));
    ESP_ERROR_CHECK(esp_wifi_disconnect());
    ESP_ERROR_CHECK(esp_wifi_stop());
    ESP_ERROR_CHECK(esp_wifi_deinit());
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
    "Host: " SUN_TIME_HTTP_SERVER "\r\n"                                       \
    "User-Agent: esp-idf/1.0 esp32\r\n"                                        \
    "\r\n"

#define MAX_HTTP_RESPONSE_SIZE (1024)

static esp_err_t get_sunrise_sunset_via_http(const char *date, cJSON **json) {
    if (json == NULL) {
        ESP_LOGE(TAG, "Failure: %s:%d", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }

    char *recv_data = (char *)malloc(MAX_HTTP_RESPONSE_SIZE);
    if (recv_data == NULL) {
        ESP_LOGE(TAG, "Failure: %s:%d", __func__, __LINE__);
        return ESP_ERR_NO_MEM;
    }

    recv_data[0] = '\0';

    esp_tls_cfg_t cfg = {
        .crt_bundle_attach = esp_crt_bundle_attach,
    };

    esp_tls_t *tls = esp_tls_init();
    if (tls == NULL) {
        ESP_LOGE(TAG, "Failure: %s:%d", __func__, __LINE__);
        free(recv_data);
        return ESP_ERR_NO_MEM;
    }

    {
        char url[128];
        snprintf(url, 128, SUN_TIME_HTTP_URL_FORMAT, date);
        char get_req[256];
        const ssize_t len =
            snprintf(get_req, 256, SUN_TIME_HTTP_GET_REQUEST_FORMAT, url);

        if (esp_tls_conn_http_new_sync(url, &cfg, tls) != 1) {
            ESP_LOGE(TAG, "Failure: %s:%d", __func__, __LINE__);
            free(recv_data);
            esp_tls_conn_destroy(tls);
            return ESP_ERR_NO_MEM;
        }

        ssize_t written_bytes = 0;
        do {
            ssize_t ret = esp_tls_conn_write(tls, get_req + written_bytes,
                                             len - written_bytes);
            if (ret >= 0) {
                written_bytes += ret;
            } else if (ret != ESP_TLS_ERR_SSL_WANT_READ &&
                       ret != ESP_TLS_ERR_SSL_WANT_WRITE) {
                ESP_LOGE(TAG, "Failure: %s:%d", __func__, __LINE__);
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
            ssize_t ret = esp_tls_conn_read(tls, buf, sizeof(buf) - 1);

            if (ret == ESP_TLS_ERR_SSL_WANT_WRITE ||
                ret == ESP_TLS_ERR_SSL_WANT_READ) {
                break;
            } else if (ret < 0) {
                ESP_LOGE(TAG, "Failure: %s:%d", __func__, __LINE__);
                free(recv_data);
                esp_tls_conn_destroy(tls);
                return ESP_FAIL;
            } else if (ret == 0) {
                break;
            }

            buf[ret] = '\0';

            if (json_found) {
                strcat(recv_data, buf);
            } else {
                char *bracket_pos = strchr(buf, '{');
                if (bracket_pos != NULL) {
                    strcat(recv_data, bracket_pos);
                    json_found = true;
                }
            }
        } while (true);

        if (!json_found) {
            ESP_LOGW(TAG, "No JSON found in HTTP response");
            free(recv_data);
            esp_tls_conn_destroy(tls);
            return ESP_FAIL;
        }
    }

    esp_tls_conn_destroy(tls);

    char *bracket_pos = strrchr(recv_data, '}');

    if (bracket_pos == NULL) {
        ESP_LOGW(TAG, "No closing bracket found");
        free(recv_data);
        return ESP_FAIL;
    }

    bracket_pos[1] = '\0';

    *json = cJSON_Parse(recv_data);
    free(recv_data);

    if (*json == NULL) {
        ESP_LOGE(TAG, "Failure: %s:%d", __func__, __LINE__);
        return ESP_FAIL;
    }

    return ESP_OK;
}

#define JSON_STATUS_FIELD "status"
#define JSON_STATUS_OK "OK"
#define JSON_RESULTS_FIELD "results"
#define JSON_SUNRISE_FIELD "sunrise"
#define JSON_SUNSET_FIELD "sunset"

static esp_err_t check_sunrise_sunset_response(const cJSON *json) {
    const cJSON *status = cJSON_GetObjectItem(json, JSON_STATUS_FIELD);
    if (!cJSON_IsString(status) || (status->valuestring == NULL)) {
        ESP_LOGW(TAG, "Invalid field: %s", JSON_STATUS_FIELD);
        return ESP_FAIL;
    }

    if (strcmp(status->valuestring, JSON_STATUS_OK) != 0) {
        ESP_LOGE(TAG, "Response NOK");
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

#define DATE_FORMAT_TIME_OFFSET (11)

static esp_err_t parse_sunrise_sunset_response(const cJSON *json,
                                               sun_day_data *data) {
    if ((json == NULL) || (data == NULL)) {
        ESP_LOGE(TAG, "Failure: %s:%d", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }

    const cJSON *results = cJSON_GetObjectItem(json, JSON_RESULTS_FIELD);
    if (!cJSON_IsObject(results)) {
        ESP_LOGW(TAG, "Invalid field: %s", JSON_RESULTS_FIELD);
        return ESP_FAIL;
    }

    const cJSON *sunrise = cJSON_GetObjectItem(results, JSON_SUNRISE_FIELD);
    if (!cJSON_IsString(sunrise) || (sunrise->valuestring == NULL)) {
        ESP_LOGW(TAG, "Invalid field: %s", JSON_SUNRISE_FIELD);
        return ESP_FAIL;
    }

    const cJSON *sunset = cJSON_GetObjectItem(results, JSON_SUNSET_FIELD);
    if (!cJSON_IsString(sunset) || (sunset->valuestring == NULL)) {
        ESP_LOGW(TAG, "Invalid field: %s", JSON_SUNSET_FIELD);
        return ESP_FAIL;
    }

    sscanf(sunrise->valuestring + DATE_FORMAT_TIME_OFFSET, "%d:%d", &data->sunrise_hr,
           &data->sunrise_min);
    sscanf(sunset->valuestring + DATE_FORMAT_TIME_OFFSET, "%d:%d", &data->sunset_hr,
           &data->sunset_min);

    return ESP_OK;
}

#define TIME_HYSTERESIS_MIN (15)

static esp_err_t add_sunrise_sunset_hysteresis(sun_day_data *data) {
    if (data == NULL) {
        ESP_LOGE(TAG, "Failure: %s:%d", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }

    data->sunrise_min += TIME_HYSTERESIS_MIN;
    if (data->sunrise_min > 59) {
        data->sunrise_min -= 60;
        data->sunrise_hr++;
    }

    data->sunset_min -= TIME_HYSTERESIS_MIN;
    if (data->sunset_min < 0) {
        data->sunset_min += 60;
        data->sunset_hr--;
    }

    return ESP_OK;
}

#define HTTP_RESPONSE_FAILURE_WAIT_PERIOD_MS (2 * 1000)

static void obtain_sunrise_sunset_data(const char *date, sun_day_data *data) {
    cJSON *json = NULL;
    esp_err_t ret;
    do {
        vTaskDelay(HTTP_RESPONSE_FAILURE_WAIT_PERIOD_MS / portTICK_PERIOD_MS);
        ret = get_sunrise_sunset_via_http(date, &json);
        if (ret != ESP_OK) {
            continue;
        }
        ret = check_sunrise_sunset_response(json);
        if (ret != ESP_OK) {
            continue;
        }
        ret = parse_sunrise_sunset_response(json, data);
        if (ret != ESP_OK) {
            continue;
        }
        ret = add_sunrise_sunset_hysteresis(data);
    } while (ret != ESP_OK);

    cJSON_Delete(json);
}

enum staircase_day_phase { BEFORE_SUNRISE, DAY, AFTER_SUNSET };

static esp_err_t get_staircase_day_phase(const sun_day_data *data,
                                         staircase_day_phase *day_phase) {

    if ((data == NULL) || (day_phase == NULL)) {
        ESP_LOGE(TAG, "Failure: %s:%d", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }

    time_t now = time(NULL);
    tm local_time;
    localtime_r(&now, &local_time);

    if ((local_time.tm_hour < data->sunrise_hr) ||
        ((local_time.tm_hour == data->sunrise_hr) &&
         (local_time.tm_min < data->sunrise_min))) {
        *day_phase = BEFORE_SUNRISE;
    } else if ((local_time.tm_hour < data->sunset_hr) ||
               ((local_time.tm_hour == data->sunset_hr) &&
                (local_time.tm_min < data->sunset_min))) {
        *day_phase = DAY;
    } else {
        *day_phase = AFTER_SUNSET;
    }

    return ESP_OK;
}

#define IDLING_PERIOD_MS (60 * 1000)

static esp_err_t
delay_until_today_sunrise(const sun_day_data *today_data) {
    if (today_data == NULL) {
        ESP_LOGE(TAG, "Failure: %s:%d", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }

    time_t now;
    tm local_time;

    do {
        vTaskDelay(IDLING_PERIOD_MS / portTICK_PERIOD_MS);
        now = time(NULL);
        localtime_r(&now, &local_time);
    } while ((local_time.tm_hour < today_data->sunrise_hr) ||
             (local_time.tm_hour == today_data->sunrise_hr &&
              local_time.tm_min <= today_data->sunrise_min));

    return ESP_OK;
}

static esp_err_t
delay_until_tomorrow_sunrise(const sun_day_data *today_data,
                             const sun_day_data *tomorrow_data) {
    if (today_data == NULL) {
        ESP_LOGE(TAG, "Failure: %s:%d", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }

    time_t now;
    tm local_time;

    do {
        vTaskDelay(IDLING_PERIOD_MS / portTICK_PERIOD_MS);
        now = time(NULL);
        localtime_r(&now, &local_time);
    } while ((local_time.tm_hour > today_data->sunset_hr) ||
             (local_time.tm_hour == today_data->sunset_hr &&
              local_time.tm_min >= today_data->sunset_min));

    return delay_until_today_sunrise(tomorrow_data);
}

static esp_err_t calculate_sleep_interval(const sun_day_data *data,
                                          int64_t *seconds) {
    if ((data == NULL) || (seconds == NULL)) {
        ESP_LOGE(TAG, "Failure: %s:%d", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }

    time_t now;
    tm local_time;

    now = time(NULL);
    localtime_r(&now, &local_time);

    if (local_time.tm_hour > data->sunset_hr ||
        (local_time.tm_hour == data->sunset_hr &&
         local_time.tm_min > data->sunset_min)) {
        ESP_LOGE(TAG, "Sleep interval invalid, sleep for 1 minute");
        *seconds = 60;
        return ESP_ERR_INVALID_ARG;
    }

    *seconds = (int64_t)(data->sunset_hr - local_time.tm_hour) * 60 * 60 +
               (int64_t)(data->sunset_min - local_time.tm_min) * 60;
    return ESP_OK;
}

#define MAIN_TASK_PERIOD_MS (10)

static void staircase_handler(void *data) {
    if (data == NULL) {
        ESP_LOGE(TAG, "Invalid handler!");
        return;
    }

    staircase::IStaircaseLooper &looper = *(staircase::IStaircaseLooper *)data;

    int64_t currentTime = esp_timer_get_time();

    while (true) {
        int64_t newTime = esp_timer_get_time();
        int64_t delta = newTime - currentTime;
        currentTime = newTime;

        if (delta < 0) {
            delta += INT64_MAX;
        }

        looper.update(static_cast<hal::Milliseconds>(delta / 1000));

        vTaskDelay(MAIN_TASK_PERIOD_MS / portTICK_PERIOD_MS);
    }
}

extern "C" void app_main(void) {

    ESP_ERROR_CHECK(set_timezone("CET-1CEST,M3.5.0,M10.5.0/3"));

    nvs_handle_t handle;
    ESP_ERROR_CHECK(init_flash(&handle));

    if (load_time(handle) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load time from persistence, continue...");
    }

    print_current_time();

    int64_t seconds_to_sleep;
    int32_t number_of_boots = 0;
    nvs_get_i32(handle, "boots", &number_of_boots);
    number_of_boots++;
    ESP_LOGI(TAG, "booted: %ld", number_of_boots);
    nvs_set_i32(handle, "boots", number_of_boots);
    nvs_commit(handle);

    {
        std::array<esp32::ESP32BinaryValueWriter,
                   staircase::IBasicLight::kLightsNum>
            outputs{esp32::ESP32BinaryValueWriter{GPIO_NUM_4},
                    esp32::ESP32BinaryValueWriter{GPIO_NUM_16},
                    esp32::ESP32BinaryValueWriter{GPIO_NUM_17},
                    esp32::ESP32BinaryValueWriter{GPIO_NUM_18},
                    esp32::ESP32BinaryValueWriter{GPIO_NUM_19},
                    esp32::ESP32BinaryValueWriter{GPIO_NUM_21},
                    esp32::ESP32BinaryValueWriter{GPIO_NUM_22},
                    esp32::ESP32BinaryValueWriter{GPIO_NUM_23}};

        esp32::ESP32BinaryValueReader downInput{GPIO_NUM_25};

        esp32::ESP32BinaryValueReader upInput{GPIO_NUM_26};

        std::array<staircase::BasicLight, staircase::IBasicLight::kLightsNum>
            lights{staircase::BasicLight{outputs[0]},
                   staircase::BasicLight{outputs[1]},
                   staircase::BasicLight{outputs[2]},
                   staircase::BasicLight{outputs[3]},
                   staircase::BasicLight{outputs[4]},
                   staircase::BasicLight{outputs[5]},
                   staircase::BasicLight{outputs[6]},
                   staircase::BasicLight{outputs[7]}};

        staircase::BasicLights basicLights{lights[0], lights[1], lights[2],
                                           lights[3], lights[4], lights[5],
                                           lights[6], lights[7]};

        staircase::ProximitySensor downSensor{downInput};
        staircase::ProximitySensor upSensor{upInput};

        staircase::BasicMovingFactory movingFactory;

        staircase::ClippedSquaredMovingDurationCalculator calculator;

        staircase::MTAMovingTimeFilter downFilter{INITIAL_MOVING_DURATION};
        staircase::MTAMovingTimeFilter upFilter{INITIAL_MOVING_DURATION};

        load_filter_values(handle, downFilter, upFilter);

        staircase::StaircaseLooper looper{basicLights,   downSensor, upSensor,
                                          movingFactory, calculator, downFilter,
                                          upFilter};

        TaskHandle_t main_loop_handler = NULL;
        xTaskCreatePinnedToCore(&staircase_handler, "staircase_task", 1024,
                                (void *)&looper, tskIDLE_PRIORITY,
                                &main_loop_handler, 1);

        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());

        ESP_ERROR_CHECK(configure_sntp());

        wifi_working = true;
        wifi_init();

        wait_for_time_sync();

        print_current_time();

        sun_day_data today_data;
        sun_day_data tomorrow_data;
        obtain_sunrise_sunset_data("today", &today_data);
        obtain_sunrise_sunset_data("tomorrow", &tomorrow_data);

        wifi_working = false;

        wifi_deinit();

        staircase_day_phase day_phase;
        get_staircase_day_phase(&today_data, &day_phase);

        const sun_day_data *data_for_calculation = &today_data;
        if (day_phase == BEFORE_SUNRISE) {
            delay_until_today_sunrise(&today_data);
        } else if (day_phase == AFTER_SUNSET) {
            delay_until_tomorrow_sunrise(&today_data, &tomorrow_data);
            data_for_calculation = &tomorrow_data;
        }

        vTaskSuspend(main_loop_handler);
        vTaskDelay(20 / portTICK_PERIOD_MS);
        vTaskDelete(main_loop_handler);

        store_filter_values(handle, downFilter, upFilter);

        calculate_sleep_interval(data_for_calculation, &seconds_to_sleep);
        ESP_LOGI(TAG, "sleeping for: %lld", seconds_to_sleep);
    }

    ESP_ERROR_CHECK(store_time(handle, seconds_to_sleep));

    ESP_ERROR_CHECK(deinit_flash(handle));

    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(
        static_cast<uint64_t>(seconds_to_sleep) * 1000000));
    esp_deep_sleep_start();
}

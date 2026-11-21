/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_bt_main.h"
#include "esp_hf_client_api.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_gap_bt_api.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#include "a2dpSinkHfpHf.h"
#include "a2dpSink.h"
#include "bt_app_avrc.h"
#include "bt_gap.h"
#include "bt_app_hf.h"
#include "bt_i2s.h"
#include "codec.h"
#include "bt_app_pbac.h"
#include "phonebook.h"

#define A2DP_SINK_HFP_HF_TAG "A2DP_SINK_HFP_HF"

#define BT_RECONNECT_FIRST_DELAY_MS  1000
#define BT_RECONNECT_RETRY_DELAY_MS  5000
#define BT_RECONNECT_MAX_ATTEMPTS    10

// Component state
static bool s_component_initialized = false;
static a2dpSinkHfpHf_config_t s_current_config = {0};
static char s_country_code[4] = DEFAULT_COUNTRY_CODE;
static bt_connection_cb_t s_connection_callback = NULL;
static a2dp_audio_state_cb_t s_audio_state_callback = NULL;
static hfp_call_state_cb_t s_call_state_callback = NULL;

// voice recognition
static bool s_hfp_audio_connected = false;
static bool s_voice_recognition_active = false;
static bool s_a2dp_was_playing = false;  // Track if we suspended A2DP
typedef enum {
    VR_STATE_IDLE,
    VR_STATE_STARTING,      // VR command sent, waiting for audio
    VR_STATE_ACTIVE,        // Audio connected, VR running
    VR_STATE_STOPPING       // Stopping VR
} vr_state_t;
static vr_state_t s_vr_state = VR_STATE_IDLE;


extern void BTM_SetPairableMode(bool allow_pairing, bool connect_only_paired);
esp_err_t bt_auto_reconnect(void);

static volatile bool s_reconnecting = false;
static volatile bool s_pairing_window_open = false;
static esp_bd_addr_t s_reconnect_peer;
static int s_reconnect_attempts = 0;
static esp_timer_handle_t s_reconnect_timer = NULL;
static esp_timer_handle_t s_pairing_timer = NULL;
static int8_t _battery_level = -1;

static void bt_pairing_allow(bool allow)
{
    BTM_SetPairableMode(allow, false);
}

static void bt_pairing_window_close(void)
{
    if (s_pairing_timer) esp_timer_stop(s_pairing_timer);
    s_pairing_window_open = false;
    bt_pairing_allow(false);
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "Pairing window closed");
}

static void bt_pairing_timer_cb(void *arg)
{
    bt_pairing_window_close();
}

/**
 * @brief Open a pairing window for new devices.
 *
 * Stops auto-reconnect, allows pairing, and makes the device discoverable.
 * The window closes automatically after @p timeout_ms, or stays open if 0.
 * Call after a2dpSinkHfpHf_init(). Does not delete existing bonds.
 *
 * @param timeout_ms  Window duration in ms (0 = no timeout; 120000 recommended).
 */
esp_err_t a2dpSinkHfpHf_enter_pairing_mode(uint32_t timeout_ms)
{
    s_reconnecting = false;
    if (s_reconnect_timer) esp_timer_stop(s_reconnect_timer);

    s_pairing_window_open = true;
    bt_pairing_allow(true);
    esp_err_t ret = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    if (s_pairing_timer) {
        esp_timer_stop(s_pairing_timer);
        if (timeout_ms) {
            esp_timer_start_once(s_pairing_timer, (uint64_t)timeout_ms * 1000);
        }
    }
    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "Pairing window opened (%lu ms)", (unsigned long)timeout_ms);
    return ret;
}

static void bt_reconnect_finish(void)
{
    s_reconnecting = false;
    if (s_reconnect_timer) esp_timer_stop(s_reconnect_timer);
}

static void bt_reconnect_schedule(uint32_t delay_ms)
{
    if (!s_reconnecting) return;
    if (s_reconnect_attempts >= BT_RECONNECT_MAX_ATTEMPTS) {
        ESP_LOGW(A2DP_SINK_HFP_HF_TAG, "Reconnect: giving up (bond kept)");
        bt_reconnect_finish();
        return;
    }
    esp_timer_stop(s_reconnect_timer);
    esp_timer_start_once(s_reconnect_timer, (uint64_t)delay_ms * 1000);
}

static void bt_reconnect_timer_cb(void *arg)
{
    if (!s_reconnecting) return;
    s_reconnect_attempts++;
    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "Reconnect attempt %d/%d",
             s_reconnect_attempts, BT_RECONNECT_MAX_ATTEMPTS);
    if (esp_a2d_sink_connect(s_reconnect_peer) != ESP_OK) {
        bt_reconnect_schedule(BT_RECONNECT_RETRY_DELAY_MS);
    }
}

static void bt_reconnect_start(const uint8_t *bda, uint32_t delay_ms)
{
    memcpy(s_reconnect_peer, bda, ESP_BD_ADDR_LEN);
    s_reconnect_attempts = 0;
    s_reconnecting = true;
    bt_pairing_allow(false);
    esp_timer_stop(s_reconnect_timer);
    esp_timer_start_once(s_reconnect_timer, (uint64_t)delay_ms * 1000);
}

static void bt_reconnect_on_a2dp_state(bool connected, const uint8_t *bda)
{
    if (!s_reconnecting) return;
    if (bda && memcmp(bda, s_reconnect_peer, ESP_BD_ADDR_LEN) != 0) return;

    if (connected) {
        bt_reconnect_finish();
        esp_hf_client_connect(s_reconnect_peer);
    } else {
        bt_reconnect_schedule(BT_RECONNECT_RETRY_DELAY_MS);
    }
}


static void bt_reconnect_gap_cb(bt_gap_event_type_t event, bt_gap_event_data_t *data)
{
    if (event != BT_GAP_EVT_AUTH_COMPLETE || data == NULL) return;

    if (s_reconnecting &&
        memcmp(data->auth.bda, s_reconnect_peer, ESP_BD_ADDR_LEN) == 0) {
        if (!data->auth.success) {
            ESP_LOGW(A2DP_SINK_HFP_HF_TAG, "Peer forgot us (auth failed), stopping reconnect");
            bt_reconnect_finish();
        }
    } else if (data->auth.success && s_pairing_window_open) {
        bt_pairing_window_close();
    }
}

static esp_err_t bt_reconnect_init(void)
{
    const esp_timer_create_args_t rc = { .callback = bt_reconnect_timer_cb, .name = "bt_reconn" };
    const esp_timer_create_args_t pw = { .callback = bt_pairing_timer_cb,  .name = "bt_pair_win" };
    esp_err_t ret = esp_timer_create(&rc, &s_reconnect_timer);
    if (ret != ESP_OK) return ret;
    return esp_timer_create(&pw, &s_pairing_timer);
}

static void bt_reconnect_deinit(void)
{
    s_reconnecting = false;
    if (s_reconnect_timer) { esp_timer_stop(s_reconnect_timer); esp_timer_delete(s_reconnect_timer); s_reconnect_timer = NULL; }
    if (s_pairing_timer)   { esp_timer_stop(s_pairing_timer);   esp_timer_delete(s_pairing_timer);   s_pairing_timer = NULL; }
}

static void bt_boot_connect_policy(void)
{
    if (bt_auto_reconnect() == ESP_OK) {
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
    } else {
        a2dpSinkHfpHf_enter_pairing_mode(0);
    }
}

// ============================================================================
// COMPONENT INITIALIZATION
// ============================================================================

/**
 * @brief Set custom configuration (optional)
 * This function allows runtime configuration override.
 * Must be called BEFORE a2dpSinkHfpHf_init()
 *
 * @param config Pointer to configuration structure
 * @return ESP_OK on success
 */
esp_err_t a2dpSinkHfpHf_config(const a2dpSinkHfpHf_config_t *config)
{
    if (s_component_initialized) {
        ESP_LOGE(A2DP_SINK_HFP_HF_TAG, "Cannot configure after initialization");
        return ESP_ERR_INVALID_STATE;
    }

    if (!config) {
        ESP_LOGE(A2DP_SINK_HFP_HF_TAG, "Config is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(&s_current_config, config, sizeof(a2dpSinkHfpHf_config_t));

    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "Custom configuration stored");
    return ESP_OK;
}

/**
 * @brief Initialize all BT subsystems in sequence
 */
esp_err_t a2dpSinkHfpHf_init(const a2dpSinkHfpHf_config_t *config)
{
    if (s_component_initialized) {
        ESP_LOGW(A2DP_SINK_HFP_HF_TAG, "Component already initialized");
        return ESP_OK;
    }

    a2dpSinkHfpHf_config_t default_config;
    if (!config) {
        default_config = (a2dpSinkHfpHf_config_t){
            .device_name = CONFIG_A2DPSINK_HFPHF_DEVICE_NAME,
            .i2s_tx_bck = CONFIG_A2DPSINK_HFPHF_I2S_TX_BCK,
            .i2s_tx_ws = CONFIG_A2DPSINK_HFPHF_I2S_TX_WS,
            .i2s_tx_dout = CONFIG_A2DPSINK_HFPHF_I2S_TX_DOUT,
            .i2s_rx_bck = CONFIG_A2DPSINK_HFPHF_I2S_RX_BCK,
            .i2s_rx_ws = CONFIG_A2DPSINK_HFPHF_I2S_RX_WS,
            .i2s_rx_din = CONFIG_A2DPSINK_HFPHF_I2S_RX_DIN
        };
        config = &default_config;

        bt_gap_set_pin(CONFIG_A2DPSINK_HFPHF_PIN_CODE, strlen(CONFIG_A2DPSINK_HFPHF_PIN_CODE));
        strncpy(s_country_code, CONFIG_A2DPSINK_HFPHF_COUNTRY_CODE, sizeof(s_country_code) - 1);
        s_country_code[sizeof(s_country_code) - 1] = '\0';
    }

    memcpy(&s_current_config, config, sizeof(a2dpSinkHfpHf_config_t));

    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "========================================");
    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "Initializing A2DP Sink + HFP Hands-Free");
    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "Device: %s", config->device_name);
    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "========================================");

    esp_err_t ret;

    // ===== STEP 0: Initialize Bluetooth Controller & Bluedroid =====
    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "[0/5] Initializing Bluetooth Controller");

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    bt_cfg.mode = ESP_BT_MODE_CLASSIC_BT;
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(A2DP_SINK_HFP_HF_TAG, "Failed to init BT controller: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK) {
        ESP_LOGE(A2DP_SINK_HFP_HF_TAG, "Failed to enable BT controller: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "  ✓ BT Controller initialized");

    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "[0/5] Initializing Bluedroid Stack");
    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    bluedroid_cfg.ssp_en = true;
    ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(A2DP_SINK_HFP_HF_TAG, "Failed to init Bluedroid: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(A2DP_SINK_HFP_HF_TAG, "Failed to enable Bluedroid: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "  ✓ Bluedroid initialized (SSP disabled)");

    // ===== STEP 1: Initialize I2S interface =====
    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "[1/5] Initializing I2S interface");
    bt_i2s_set_tx_I2S_pins(config->i2s_tx_bck, config->i2s_tx_ws, config->i2s_tx_dout, 0);
    bt_i2s_set_rx_I2S_pins(config->i2s_rx_bck, config->i2s_rx_ws, 0, config->i2s_rx_din);
    bt_i2s_init();
    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "  ✓ I2S interface initialized");

    // ===== STEP 2: Initialize audio codec =====
    /* ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "[2/5] Initializing audio codec (mSBC)");
    msbc_enc_open();
    msbc_dec_open();
    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "  ✓ Audio codec initialized"); */

    // ===== STEP 3: Initialize GAP layer =====
    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "[3/5] Initializing GAP layer");
    ret = bt_gap_init();
    if (ret != ESP_OK) {
        ESP_LOGE(A2DP_SINK_HFP_HF_TAG, "Failed to initialize GAP: %d", ret);
        goto err_cleanup;
    }

    ret = bt_gap_set_device_name(config->device_name);
    if (ret != ESP_OK) {
        ESP_LOGE(A2DP_SINK_HFP_HF_TAG, "Failed to set device name: %d", ret);
        goto err_cleanup;
    }

    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "  ✓ GAP layer initialized");

    ret = bt_reconnect_init();
    if (ret != ESP_OK) {
        ESP_LOGE(A2DP_SINK_HFP_HF_TAG, "Failed to init reconnect timers: %d", ret);
        goto err_cleanup;
    }
    bt_gap_register_event_callback(bt_reconnect_gap_cb);

    // ===== STEP 4: Initialize HFP Hands-Free profile =====
    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "[4/5] Initializing HFP Hands-Free profile");
    #if CONFIG_BT_PBAC_ENABLED
        phonebook_init();
        phonebook_set_country_code(s_country_code);
        bt_app_pbac_task_start();
    #endif
    ret = esp_hf_client_register_callback(bt_app_hf_client_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(A2DP_SINK_HFP_HF_TAG, "Failed to register HFP callback: %d", ret);
        goto err_cleanup;
    }

    ret = esp_hf_client_init();
    if (ret != ESP_OK) {
        ESP_LOGE(A2DP_SINK_HFP_HF_TAG, "Failed to initialize HFP client: %d", ret);
        goto err_cleanup;
    }

    #if CONFIG_BT_PBAC_ENABLED
        esp_pbac_register_callback(bt_app_pbac_cb);
        esp_pbac_init();
    #endif
    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "  ✓ HFP Hands-Free profile initialized");

    // ===== STEP 5: Initialize A2DP Sink profile =====
    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "[5/5] Initializing A2DP Sink profile");

    bt_app_avrc_init();
    a2dp_sink_init();
    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "  ✓ A2DP Sink profile initialized");

     // ===== STEP 6: Setting Class of Device to Headphones =====

    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "[6/6] Setting Class of Device to Headphones");

        esp_bt_cod_t cod = {
            .major   = ESP_BT_COD_MAJOR_DEV_AV,
            .minor   = 0x01,  // Headphones
            .service = ESP_BT_COD_SRVC_AUDIO | ESP_BT_COD_SRVC_RENDERING | ESP_BT_COD_SRVC_TELEPHONY
        };
        ret = esp_bt_gap_set_cod(cod, ESP_BT_SET_COD_ALL);
        if (ret != ESP_OK) {
            ESP_LOGE(A2DP_SINK_HFP_HF_TAG, "Failed to set CoD: %s", esp_err_to_name(ret));
        } else {
            uint32_t cod_raw = ((uint32_t)cod.service << 13) |
                               ((uint32_t)cod.major   << 8)  |
                               ((uint32_t)cod.minor);
            ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "  ✓ CoD set to 0x%06lX (Audio/Video / Headphones)",
                     (unsigned long)cod_raw);
        }

        esp_bt_cod_t check_cod = {0};
        if (esp_bt_gap_get_cod(&check_cod) == ESP_OK) {
            uint32_t actual_raw = ((uint32_t)check_cod.service << 13) |
                                  ((uint32_t)check_cod.major   << 8)  |
                                  ((uint32_t)check_cod.minor);
            ESP_LOGI(A2DP_SINK_HFP_HF_TAG,
                     "  → CoD readback: major=%d minor=0x%02X service=0x%03X (raw=0x%06lX)",
                     check_cod.major, check_cod.minor, check_cod.service,
                     (unsigned long)actual_raw);
        }

    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "========================================");
    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "✓ Component initialized successfully!");
    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "========================================");

    // ===== Boot policy: reconnect to bonded device or open pairing window =====
    bt_boot_connect_policy();

    s_component_initialized = true;
    return ESP_OK;

err_cleanup:
    ESP_LOGE(A2DP_SINK_HFP_HF_TAG, "Component initialization failed, cleaning up");
    a2dpSinkHfpHf_deinit();
    return ret;
}

/**
 * @brief Deinitialize all BT subsystems
 */
esp_err_t a2dpSinkHfpHf_deinit(void)
{
    if (!s_component_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "========================================");
    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "Deinitializing A2DP Sink + HFP Hands-Free");
    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "========================================");

    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "Deinitializing A2DP Sink");
    a2dp_sink_deinit();
    bt_app_avrc_deinit();

    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "Deinitializing HFP");
    esp_hf_client_deinit();

    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "Deinitializing GAP");
    bt_gap_deinit();

    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "Closing codec");
    msbc_enc_close();
    msbc_dec_close();

    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "Deinitializing I2S");
    bt_i2s_driver_uninstall();

    bt_reconnect_deinit();

    s_component_initialized = false;
    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "========================================");
    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "✓ Component deinitialized");
    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "========================================");

    return ESP_OK;
}

// ============================================================================
// GAP API
// ============================================================================

esp_err_t a2dpSinkHfpHf_register_gap_callback(bt_gap_event_cb_t callback)
{
    if (!s_component_initialized) {
        ESP_LOGE(A2DP_SINK_HFP_HF_TAG, "Component not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    return bt_gap_register_event_callback(callback);
}

esp_err_t a2dpSinkHfpHf_unregister_gap_callback(bt_gap_event_cb_t callback)
{
    if (!s_component_initialized) {
        ESP_LOGE(A2DP_SINK_HFP_HF_TAG, "Component not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    return bt_gap_unregister_event_callback(callback);
}

// ============================================================================
// PHONEBOOK API WRAPPERS
// ============================================================================

a2dpSinkHfpHf_phonebook_handle_t a2dpSinkHfpHf_get_phonebook(void)
{
    if (!s_component_initialized) {
        return NULL;
    }
    return (a2dpSinkHfpHf_phonebook_handle_t)bt_app_pbac_get_current_phonebook();
}

uint16_t a2dpSinkHfpHf_phonebook_get_count(a2dpSinkHfpHf_phonebook_handle_t pb)
{
    if (!pb) {
        return 0;
    }
    return phonebook_get_count((phonebook_t*)pb);
}

a2dpSinkHfpHf_contact_t* a2dpSinkHfpHf_phonebook_search_by_letter(
    a2dpSinkHfpHf_phonebook_handle_t pb,
    char letter,
    uint16_t *count)
{
    if (!pb || !count) {
        return NULL;
    }

    contact_t *internal_contacts = phonebook_search_by_letter((phonebook_t*)pb, letter, count);
    return (a2dpSinkHfpHf_contact_t*)internal_contacts;
}

a2dpSinkHfpHf_contact_t* a2dpSinkHfpHf_phonebook_search_by_name(
    a2dpSinkHfpHf_phonebook_handle_t pb,
    const char *name,
    uint16_t *count)
{
    if (!pb || !name || !count) {
        return NULL;
    }

    contact_t *internal_contacts = phonebook_search_by_name((phonebook_t*)pb, name, count);
    return (a2dpSinkHfpHf_contact_t*)internal_contacts;
}

a2dpSinkHfpHf_contact_t* a2dpSinkHfpHf_phonebook_search_by_number(
    a2dpSinkHfpHf_phonebook_handle_t pb,
    const char *number)
{
    if (!pb || !number) {
        return NULL;
    }

    contact_t *internal_contact = phonebook_search_by_number((phonebook_t*)pb, number);
    return (a2dpSinkHfpHf_contact_t*)internal_contact;
}

a2dpSinkHfpHf_phone_number_t* a2dpSinkHfpHf_phonebook_get_numbers(
    a2dpSinkHfpHf_phonebook_handle_t pb,
    const char *full_name,
    uint8_t *count)
{
    if (!pb || !full_name || !count) {
        return NULL;
    }

    phone_number_t *internal_numbers = phonebook_get_numbers((phonebook_t*)pb, full_name, count);
    return (a2dpSinkHfpHf_phone_number_t*)internal_numbers;
}

// ============================================================================
// PUBLIC API FUNCTIONS
// ============================================================================

esp_err_t a2dpSinkHfpHf_start_discovery(void)
{
    if (!s_component_initialized) {
        ESP_LOGE(A2DP_SINK_HFP_HF_TAG, "Component not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    return bt_gap_start_discovery();
}

esp_err_t a2dpSinkHfpHf_cancel_discovery(void)
{
    if (!s_component_initialized) {
        ESP_LOGE(A2DP_SINK_HFP_HF_TAG, "Component not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    return bt_gap_cancel_discovery();
}

const char *a2dpSinkHfpHf_get_device_name(void)
{
    return s_current_config.device_name;
}

bool a2dpSinkHfpHf_is_connected(void)
{
    return a2dp_sink_is_connected();
}

esp_err_t a2dpSinkHfpHf_set_country_code(const char *country_code)
{
    if (s_component_initialized) {
        ESP_LOGW(A2DP_SINK_HFP_HF_TAG, "Cannot change country code after initialization");
        return ESP_ERR_INVALID_STATE;
    }

    if (country_code == NULL || strlen(country_code) > 3) {
        ESP_LOGE(A2DP_SINK_HFP_HF_TAG, "Invalid country code");
        return ESP_ERR_INVALID_ARG;
    }

    strncpy(s_country_code, country_code, sizeof(s_country_code) - 1);
    s_country_code[sizeof(s_country_code) - 1] = '\0';
    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "Country code set to: %s", s_country_code);
    return ESP_OK;
}

// ============================================================================
// PIN CODE API (Wrapper to bt_gap.c)
// ============================================================================

esp_err_t a2dpSinkHfpHf_set_pin(const char *pin_code, uint8_t pin_len)
{
    if (s_component_initialized) {
        ESP_LOGW(A2DP_SINK_HFP_HF_TAG, "Cannot change PIN after initialization");
        return ESP_ERR_INVALID_STATE;
    }

    return bt_gap_set_pin(pin_code, pin_len);
}

esp_err_t a2dpSinkHfpHf_get_pin(char *pin_code, uint8_t *pin_len)
{
    return bt_gap_get_pin(pin_code, pin_len);
}

// ============================================================================
// AVRC PUBLIC API (Wrappers to bt_app_avrc.c)
// ============================================================================

esp_err_t a2dpSinkHfpHf_set_avrc_metadata_mask(uint8_t attr_mask)
{
    if (s_component_initialized) {
        ESP_LOGE(A2DP_SINK_HFP_HF_TAG, "Cannot change AVRC metadata mask after initialization");
        return ESP_ERR_INVALID_STATE;
    }

    if (!bt_app_avrc_set_metadata_mask(attr_mask)) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

void a2dpSinkHfpHf_register_avrc_conn_callback(bt_avrc_conn_state_cb_t callback)
{
    bt_app_avrc_register_conn_callback(callback);
}

void a2dpSinkHfpHf_register_avrc_metadata_callback(bt_avrc_metadata_cb_t callback)
{
    bt_app_avrc_register_metadata_callback(callback);
}

void a2dpSinkHfpHf_register_avrc_playback_callback(bt_avrc_playback_status_cb_t callback)
{
    bt_app_avrc_register_playback_status_callback(callback);
}

void a2dpSinkHfpHf_register_avrc_volume_callback(bt_avrc_volume_cb_t callback)
{
    bt_app_avrc_register_volume_callback(callback);
}

const bt_avrc_metadata_t* a2dpSinkHfpHf_get_avrc_metadata(void)
{
    return bt_app_avrc_get_metadata();
}

bool a2dpSinkHfpHf_is_avrc_connected(void)
{
    return bt_app_avrc_is_connected();
}

bool a2dpSinkHfpHf_avrc_play(void)
{
    if (!s_component_initialized) {
        ESP_LOGE(A2DP_SINK_HFP_HF_TAG, "Component not initialized");
        return false;
    }

    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "AVRC: Play");
    return bt_app_avrc_cmd_play();
}

bool a2dpSinkHfpHf_avrc_pause(void)
{
    if (!s_component_initialized) {
        ESP_LOGE(A2DP_SINK_HFP_HF_TAG, "Component not initialized");
        return false;
    }

    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "AVRC: Pause");
    return bt_app_avrc_cmd_pause();
}

bool a2dpSinkHfpHf_avrc_next(void)
{
    if (!s_component_initialized) {
        ESP_LOGE(A2DP_SINK_HFP_HF_TAG, "Component not initialized");
        return false;
    }

    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "AVRC: Next track");
    return bt_app_avrc_cmd_next();
}

bool a2dpSinkHfpHf_avrc_prev(void)
{
    if (!s_component_initialized) {
        ESP_LOGE(A2DP_SINK_HFP_HF_TAG, "Component not initialized");
        return false;
    }

    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "AVRC: Previous track");
    return bt_app_avrc_cmd_prev();
}

// ============================================================================
// HFP HANDS-FREE CONTROL FUNCTIONS
// ============================================================================

void bt_hfp_audio_connection_state_changed(bool connected)
{
    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "HFP audio connection state: %s",
             connected ? "CONNECTED" : "DISCONNECTED");

    s_hfp_audio_connected = connected;

    if (connected) {
        if (s_vr_state == VR_STATE_STARTING) {
            ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "✓ Voice recognition active with HFP audio");
            s_vr_state = VR_STATE_ACTIVE;
            s_voice_recognition_active = true;
        }
    } else {
        if (s_vr_state == VR_STATE_ACTIVE || s_vr_state == VR_STATE_STARTING) {
            ESP_LOGW(A2DP_SINK_HFP_HF_TAG, "HFP audio disconnected during voice recognition");

            s_vr_state = VR_STATE_IDLE;
            s_voice_recognition_active = false;

            if (s_a2dp_was_playing) {
                ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "Resuming A2DP after audio disconnect");
                bt_i2s_a2dp_start();
                esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START);
                s_a2dp_was_playing = false;
            }
        }
    }
}

esp_err_t a2dpSinkHfpHf_answer_call(void) {
    return esp_hf_client_answer_call();
}

esp_err_t a2dpSinkHfpHf_reject_call(void) {
    return esp_hf_client_reject_call();
}

esp_err_t a2dpSinkHfpHf_hangup_call(void) {
    return esp_hf_client_reject_call();
}

esp_err_t a2dpSinkHfpHf_dial_number(const char *number) {
    if (number == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return esp_hf_client_dial(number);
}

esp_err_t a2dpSinkHfpHf_redial(void) {
    return esp_hf_client_dial(NULL);
}

esp_err_t a2dpSinkHfpHf_dial_memory(int location) {
    return esp_hf_client_dial_memory(location);
}

// ============================================================================
// Voice Recognition
// ============================================================================

esp_err_t a2dpSinkHfpHf_start_voice_recognition(void)
{
    if (!s_component_initialized) {
        ESP_LOGE(A2DP_SINK_HFP_HF_TAG, "Component not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_vr_state != VR_STATE_IDLE) {
        ESP_LOGW(A2DP_SINK_HFP_HF_TAG, "Voice recognition already active or starting");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "Starting voice recognition");

    if (bt_i2s_is_a2dp_mode()) {
        ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "Suspending A2DP for voice recognition");
        esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_SUSPEND);
        s_a2dp_was_playing = true;
        bt_i2s_a2dp_stop();
        vTaskDelay(pdMS_TO_TICKS(300));
    } else {
        s_a2dp_was_playing = false;
    }

    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "Sending voice recognition command");
    esp_err_t ret = esp_hf_client_start_voice_recognition();
    if (ret != ESP_OK) {
        ESP_LOGE(A2DP_SINK_HFP_HF_TAG, "Failed to start voice recognition: %d", ret);

        if (s_a2dp_was_playing) {
            bt_i2s_a2dp_start();
            esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START);
            s_a2dp_was_playing = false;
        }
        return ret;
    }

    s_vr_state = VR_STATE_STARTING;
    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "✓ Voice recognition command sent, waiting for audio connection...");

    return ESP_OK;
}

esp_err_t a2dpSinkHfpHf_stop_voice_recognition(void)
{
    if (!s_component_initialized) {
        ESP_LOGE(A2DP_SINK_HFP_HF_TAG, "Component not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_vr_state == VR_STATE_IDLE) {
        ESP_LOGW(A2DP_SINK_HFP_HF_TAG, "Voice recognition not active");
        return ESP_OK;
    }

    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "Stopping voice recognition");

    s_vr_state = VR_STATE_STOPPING;

    esp_err_t ret = esp_hf_client_stop_voice_recognition();
    if (ret == ESP_OK) {
        s_voice_recognition_active = false;
    }

    bt_app_hf_disconnect_audio();

    s_vr_state = VR_STATE_IDLE;

    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "✓ Voice recognition stopped");
    return ret;
}

// ============================================================================
// Query Functions
// ============================================================================

esp_err_t a2dpSinkHfpHf_query_operator(void) {
    return esp_hf_client_query_current_operator_name();
}

esp_err_t a2dpSinkHfpHf_query_current_calls(void) {
    return esp_hf_client_query_current_calls();
}

esp_err_t a2dpSinkHfpHf_retrieve_subscriber_info(void) {
    return esp_hf_client_retrieve_subscriber_info();
}

// ============================================================================
// Advanced Features
// ============================================================================

esp_err_t a2dpSinkHfpHf_send_btrh(int btrh) {
    if (btrh < 0 || btrh > 2) {
        return ESP_ERR_INVALID_ARG;
    }
    return esp_hf_client_send_btrh_cmd((esp_hf_btrh_cmd_t)btrh);
}

esp_err_t a2dpSinkHfpHf_send_xapl(const char *features) {
    if (features == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    char info_copy[64];
    strncpy(info_copy, features, sizeof(info_copy) - 1);
    info_copy[sizeof(info_copy) - 1] = '\0';

    char *comma = strchr(info_copy, ',');
    if (comma == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *comma = '\0';
    char *info_part = info_copy;
    uint32_t features_part = (uint32_t)atoi(comma + 1);

    return esp_hf_client_send_xapl(info_part, features_part);
}

esp_err_t a2dpSinkHfpHf_send_iphoneaccev(int bat_level, int docked) {
    if (bat_level < -1 || bat_level > 9) {
        return ESP_ERR_INVALID_ARG;
    }
    if (docked < -1 || docked > 1) {
        return ESP_ERR_INVALID_ARG;
    }

    if (bat_level < 0 && docked < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t battery = (bat_level >= 0) ? (uint32_t)bat_level : 0;
    bool is_docked = (docked > 0);

    return esp_hf_client_send_iphoneaccev(battery, is_docked);
}

void a2dp_sink_hfp_hf_register_connection_cb(bt_connection_cb_t callback)
{
    s_connection_callback = callback;
    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "Connection callback %s",
             callback ? "registered" : "unregistered");
}

void a2dp_sink_hfp_hf_register_audio_state_cb(a2dp_audio_state_cb_t callback)
{
    s_audio_state_callback = callback;
    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "Audio state callback %s",
             callback ? "registered" : "unregistered");
}

void a2dp_sink_hfp_hf_register_call_state_cb(hfp_call_state_cb_t callback)
{
    s_call_state_callback = callback;
    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "Call state callback %s",
             callback ? "registered" : "unregistered");
}

void a2dp_sink_notify_connection(bool connected, const uint8_t *bda)
{
    bt_reconnect_on_a2dp_state(connected, bda);   // intercept A2DP events for auto-reconnect

    if (s_connection_callback) {
        ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "→ Connection callback: %s",
                 connected ? "CONNECTED" : "DISCONNECTED");
        s_connection_callback(connected, bda);
    }
}

void a2dp_sink_notify_audio_state(bool streaming)
{
    if (s_audio_state_callback) {
        ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "→ Audio state callback: %s",
                 streaming ? "STREAMING" : "STOPPED");
        s_audio_state_callback(streaming);
    }
}

void hfp_notify_call_state(bool call_active, int call_state)
{
    if (s_call_state_callback) {
        ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "→ Call state callback: %s (state=%d)",
                 call_active ? "ACTIVE" : "IDLE", call_state);
        s_call_state_callback(call_active, call_state);
    }
}

// ============================================================================
// Volume control
// ============================================================================

esp_err_t a2dpSinkHfpHf_set_hfp_speaker_volume(uint8_t volume)
{
    if (!s_component_initialized) {
        ESP_LOGE(A2DP_SINK_HFP_HF_TAG, "Component not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (volume > 15) {
        ESP_LOGW(A2DP_SINK_HFP_HF_TAG, "Volume %d exceeds max (15), clamping", volume);
        volume = 15;
    }

    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "Setting HFP speaker volume to %d", volume);
    bt_i2s_set_hfp_speaker_volume(volume);

    esp_hf_client_volume_update(ESP_HF_VOLUME_CONTROL_TARGET_SPK, volume);

    return ESP_OK;
}

esp_err_t a2dpSinkHfpHf_set_hfp_mic_volume(uint8_t volume)
{
    if (!s_component_initialized) {
        ESP_LOGE(A2DP_SINK_HFP_HF_TAG, "Component not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (volume > 15) {
        ESP_LOGW(A2DP_SINK_HFP_HF_TAG, "Volume %d exceeds max (15), clamping", volume);
        volume = 15;
    }

    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "Setting HFP microphone volume to %d", volume);
    bt_i2s_set_hfp_mic_volume(volume);

    esp_hf_client_volume_update(ESP_HF_VOLUME_CONTROL_TARGET_MIC, volume);

    return ESP_OK;
}

esp_err_t a2dpSinkHfpHf_set_a2dp_volume(uint8_t volume)
{
    if (!s_component_initialized) {
        ESP_LOGE(A2DP_SINK_HFP_HF_TAG, "Component not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (volume > 15) {
        ESP_LOGW(A2DP_SINK_HFP_HF_TAG, "Volume %d exceeds max (15), clamping", volume);
        volume = 15;
    }

    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "Setting A2DP volume to %d", volume);

    bt_i2s_set_a2dp_volume(volume);

    uint8_t avrcp_volume = (volume * 127) / 15;
    esp_err_t ret = bt_app_avrc_set_absolute_volume(avrcp_volume);

    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(A2DP_SINK_HFP_HF_TAG, "Failed to sync volume with phone");
    }

    return ESP_OK;
}

// ============================================================================
// Auto reconnect
// ============================================================================

esp_err_t bt_auto_reconnect(void)
{
    esp_bd_addr_t bda;
    bool found = false;

    if (bt_get_last_connected_bd_addr(bda) == ESP_OK && bt_is_bonded(bda)) {
        found = true;
    } else {
        int n = esp_bt_gap_get_bond_device_num();
        if (n > 0) {
            esp_bd_addr_t *list = malloc(n * sizeof(esp_bd_addr_t));
            if (list) {
                if (esp_bt_gap_get_bond_device_list(&n, list) == ESP_OK && n > 0) {
                    memcpy(bda, list[0], ESP_BD_ADDR_LEN);
                    found = true;
                }
                free(list);
            }
        }
    }

    if (!found) {
        ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "No bonded device to reconnect");
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(A2DP_SINK_HFP_HF_TAG, "Reconnect target: %02X:%02X:%02X:%02X:%02X:%02X",
             bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
    bt_reconnect_start(bda, BT_RECONNECT_FIRST_DELAY_MS);
    return ESP_OK;
}

void set_battery_level(int8_t battery_level){
    _battery_level = battery_level;
}

int8_t get_battery_level(){
    return _battery_level;
}
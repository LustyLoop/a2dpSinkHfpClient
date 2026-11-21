
extern "C" {
    #include <stdio.h>
    #include <string.h>
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "nvs_flash.h"
    #include "esp_log.h"
    #include "a2dpSink.h"
    #include "esp_hf_client_api.h"
    #include "soc/rtc.h"
    #include "esp_pm.h"
    #include "esp_err.h"
    #include "a2dpSinkHfpHf.h"
}

#define TAG "MAIN"


extern "C" void app_main(void)
{
    esp_err_t ret;
    ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {

        
        ESP_LOGW(TAG, "NVS partition needs to be erased");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    set_battery_level(68);
    a2dpSinkHfpHf_config_t my_config = {
        .device_name = "ESP32-Audio",
        .i2s_tx_bck = 19,
        .i2s_tx_ws = 18,
        .i2s_tx_dout = 17,
        .i2s_rx_bck = 25,
        .i2s_rx_ws = 26,
        .i2s_rx_din = 27
    };
    ret = a2dpSinkHfpHf_init(&my_config);
    

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize component: %s", esp_err_to_name(ret));
        return;
    }
    else{
        a2dpSinkHfpHf_set_hfp_mic_volume(15); 
        a2dpSinkHfpHf_set_hfp_speaker_volume(7);
        a2dpSinkHfpHf_set_a2dp_volume(15);
    }

    /*
    * An optional microcontroller frequency adjustment unit. 
    * Stable operation at 80 MHz.
    esp_pm_config_t pm_config = {
        .max_freq_mhz = 80, 
        .min_freq_mhz = 40,
        .light_sleep_enable = true,
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
    */
    while (1) {
        if(a2dpSinkHfpHf_is_connected() == true && a2dp_sink_is_connected() == true)printf("Connected!\n");
        else printf("Disconnected\n");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}


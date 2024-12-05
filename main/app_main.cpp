/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <nvs_flash.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <vector>
#if CONFIG_PM_ENABLE
#include <esp_pm.h>
#endif

#include <esp_matter.h>
#include <protocols/bdx/BdxMessages.h>
// #include <lib/core/StringBuilderAdapters.h>
#include <lib/support/BufferWriter.h>
#include <lib/support/CHIPMem.h>
#include <lib/support/CodeUtils.h>
#include <esp_matter_ota.h>
#include <app/util/basic-types.h>
#include <common_macros.h>
#include <app_priv.h>
#include <app/AttributeAccessInterface.h>
#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif
#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>
// Include Matter (CHIP) headers
#include <app/server/OnboardingCodesUtil.h>
#include <lib/support/logging/CHIPLogging.h>
#include <app/AttributeAccessInterface.h>

#include "sht30.h"
// #include "button.h"
#include <driver/gpio.h>
#include "button_gpio.h"

#define CLUSTER_ID  0x8003
static char nvs_index = 0;
static constexpr int max_storage_size = 120; // Max size of historical data to store (can be adjusted)
static bool nvs_full = false;
static const char *TAG = "app_main";

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

constexpr auto k_timeout_seconds = 300;
endpoint_t * temp_sensor_ep;
endpoint_t * humidity_sensor_ep;
endpoint_t* blevel_ep;
#define BUTTON_ACTIVE_LEVEL  0  
#define BUTTON_POWER_SAVE_ENABLE true  

button_gpio_config_t button_config = {
    .gpio_num = GPIO_NUM_2,
    .active_level = BUTTON_ACTIVE_LEVEL,
    // .enable_power_save = BUTTON_POWER_SAVE_ENABLE
};
// Custom Battery Level Cluster
// namespace chip {
// namespace app {
// namespace Clusters {

// class BatteryLevel : public chip::app::Clusters::ClusterBase
// {
// public:
//     static constexpr chip::ClusterId Id = 0x1000; // Custom Cluster ID

//     // Define Battery Level Attribute
//     static constexpr chip::AttributeId BatteryLevelId = 0x0000;

//     static void SetBatteryLevel(chip::app::EndpointId endpoint_id, float level)
//     {
//         esp_matter_attr_val_t batteryVal;
//         batteryVal.type = ESP_MATTER_VAL_TYPE_FLOAT;
//         batteryVal.val.f = level;
//         SetAttribute(endpoint_id, BatteryLevelId, &batteryVal);
//     }
// };
// Define NVS namespace
// Save sensor data to NVS
typedef struct {
    int temperature;
    int humidity;
    int battery_volt;
} sensor_data_t;

#define NVS_NAMESPACE "data_storage"
sensor_data_t sensor = {};
// Function to store data
void store_data(sensor_data_t *data) {
    // Capture the current timestamp
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error opening NVS handle for writing");
    }
    // size_t total_size = nvs_index*sensor_data_t;
    // Store the structure as a blob
    char key[16];
    snprintf(key, sizeof(key), "sensor_data_%d", nvs_index);
    err = nvs_set_blob(my_handle, key, data, sizeof(sensor_data_t));
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Failed to save sensor data to NVS");
    } else {
        ESP_LOGI("NVS", "Sensor data saved successfully");
    }
    nvs_close(my_handle);
}
// Load sensor data from NVS
namespace esp_matter {
namespace cluster {
namespace sensor {

    // Define the custom cluster's endpoint and Cluster ID
    static constexpr chip::EndpointId endpoint_id = 0x00000001;
    static constexpr chip::ClusterId Id = 0x131BFC01;  // Custom Cluster ID for Sensor Data

    // Define attribute IDs for the sensor values
    namespace attribute {
        static constexpr chip::AttributeId temperature_id = 0x00000000; // Attribute ID for temperature
        static constexpr chip::AttributeId humidity_id = 0x00000001;    // Attribute ID for humidity
        static constexpr chip::AttributeId battery_level_id = 0x00000002; // Attribute ID for battery level
    }

    // Define the data storage structure (in-memory storage for now)
    // std::vector<SensorData> historical_data; // A vector to store historical sensor data
    

    void load_sensor_data_from_nvs(sensor_data_t *data) {
        nvs_handle_t my_handle;
        char key[16];
        snprintf(key, sizeof(key), "sensor_data_%d", nvs_index);
        esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);
        if (err != ESP_OK) {
            ESP_LOGE("NVS", "Error opening NVS handle for reading");
            return;
        }

        // Read the structure from NVS
        size_t length = sizeof(sensor_data_t);
        err = nvs_get_blob(my_handle, key, data, &length);

        if (err == ESP_OK) {
            ESP_LOGI("NVS", "Sensor data loaded successfully");
        } else {
            ESP_LOGE("NVS", "Failed to load sensor data from NVS");
        }

        nvs_close(my_handle);
    }
    // Function to retrieve historical data (for example, the last n records)
    void get_historical_data(std::vector<sensor_data_t>& snsr_data){
        sensor_data_t * buffer = {};
        for (int i = nvs_index; i > 0; i--)
        {
            load_sensor_data_from_nvs(buffer);
            snsr_data.push_back(*buffer);
        }
    }
     void send_data_chunk_via_bdx(chip::EndpointId endpoint, const std::vector<sensor_data_t>& data_chunk) {
        
        chip::ByteSpan data_span(reinterpret_cast<const uint8_t*>(data_chunk.data()), data_chunk.size() * sizeof(sensor_data_t));

        
        chip::bdx::TransferInit transferInitMessage;
        transferInitMessage.StartOffset = 0;
        transferInitMessage.MaxLength = max_storage_size*sizeof(sensor_data_t);
        transferInitMessage.MaxBlockSize = 256;
        char FileDes[16]    = { "BDX_DATA.txt" };
        transferInitMessage.FileDesignator = reinterpret_cast<uint8_t *>(FileDes); 

        // Encoding::LittleEndian::BufferWriter writer;
        // transferInitMessage.WriteToBuffer(writer);
    
        chip::Encoding::LittleEndian::PacketBufferWriter bbuf(chip::System::PacketBufferHandle::New(max_storage_size));
        transferInitMessage.WriteToBuffer(bbuf);
        chip::bdx::DataBlock dataBlock;
        dataBlock.BlockCounter = 1;
        dataBlock.Data = (uint8_t *)(&(data_chunk[1].temperature));
        // dataBlock.Data = static_cast<int*>(data_chunk.humidity);
        // dataBlock.Data = static_cast<int*>(data_chunk.batttery_volt);
        dataBlock.DataLength = sizeof(data_chunk);

        dataBlock.WriteToBuffer(bbuf);

        // CounterMessage counterMsg;
        // counterMsg.BlockCounter = 1;
        // counterMsg.WriteToBuffer(writer);
        
    }

    // Function to initiate a BDX bulk data transfer
    void start_bdx_transfer(chip::EndpointId endpoint) {
        std::vector<sensor_data_t> buffer;

        // Send data in chunks
        get_historical_data(buffer);
        send_data_chunk_via_bdx(endpoint_id, buffer);  // Send the chunk via BDX
    }

} // namespace sensor
} // namespace cluster
} // namespace esp_matter



static void battery_notification(uint16_t endpoint_id, float b_level, void *user_data)
{
    // schedule the attribute update so that we can report it from matter thread
    chip::DeviceLayer::SystemLayer().ScheduleLambda([endpoint_id, b_level]() {
        attribute_t * attribute = attribute::get(endpoint_id,
                                                 ElectricalPowerMeasurement::Id,
                                                 ElectricalPowerMeasurement::Attributes::Voltage::Id);

        esp_matter_attr_val_t val = esp_matter_invalid(NULL);
        attribute::get_val(attribute, &val);
        val.val.i16 = static_cast<int16_t>(b_level);

        attribute::update(endpoint_id, ElectricalPowerMeasurement::Id, ElectricalPowerMeasurement::Attributes::Voltage::Id, &val);
    });
}
// Application cluster specification, 7.18.2.11. Temperature
// represents a temperature on the Celsius scale with a resolution of 0.01°C.
// temp = (temperature in °C) x 100
static void temp_sensor_notification(uint16_t endpoint_id, float temp, void *user_data)
{
    // schedule the attribute update so that we can report it from matter thread
    chip::DeviceLayer::SystemLayer().ScheduleLambda([endpoint_id, temp]() {
        attribute_t * attribute = attribute::get(endpoint_id,
                                                 TemperatureMeasurement::Id,
                                                 TemperatureMeasurement::Attributes::MeasuredValue::Id);

        esp_matter_attr_val_t val = esp_matter_invalid(NULL);
        attribute::get_val(attribute, &val);
        val.val.i16 = static_cast<int16_t>(temp * 100);

        attribute::update(endpoint_id, TemperatureMeasurement::Id, TemperatureMeasurement::Attributes::MeasuredValue::Id, &val);
    });
}



// Application cluster specification, 2.6.4.1. MeasuredValue Attribute
// represents the humidity in percent.
// humidity = (humidity in %) x 100
static void humidity_sensor_notification(uint16_t endpoint_id, float humidity, void *user_data)
{
    // schedule the attribute update so that we can report it from matter thread
    chip::DeviceLayer::SystemLayer().ScheduleLambda([endpoint_id, humidity]() {
        attribute_t * attribute = attribute::get(endpoint_id,
                                                 RelativeHumidityMeasurement::Id,
                                                 RelativeHumidityMeasurement::Attributes::MeasuredValue::Id);

        esp_matter_attr_val_t val = esp_matter_invalid(NULL);
        attribute::get_val(attribute, &val);
        val.val.u16 = static_cast<uint16_t>(humidity * 100);

        attribute::update(endpoint_id, RelativeHumidityMeasurement::Id, RelativeHumidityMeasurement::Attributes::MeasuredValue::Id, &val);
    });
}


static void open_commissioning_window_if_necessary()
{
    VerifyOrReturn(chip::Server::GetInstance().GetFabricTable().FabricCount() == 0);

    chip::CommissioningWindowManager & commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
    VerifyOrReturn(commissionMgr.IsCommissioningWindowOpen() == false);

    // After removing last fabric, this example does not remove the Wi-Fi credentials
    // and still has IP connectivity so, only advertising on DNS-SD.
    CHIP_ERROR err = commissionMgr.OpenBasicCommissioningWindow(chip::System::Clock::Seconds16(300),
                                    chip::CommissioningWindowAdvertisement::kDnssdOnly);
    if (err != CHIP_NO_ERROR)
    {
        ESP_LOGE(TAG, "Failed to open commissioning window, err:%" CHIP_ERROR_FORMAT, err.Format());
    }
}
void IRAM_ATTR button_isr_handler(void* arg)
{
    gpio_num_t pin = GPIO_NUM_2;
    ESP_LOGI("BUTTON", "Button pressed, handling interrupt!");
    int64_t start, end, passed;
    start = esp_timer_get_time();
    while(1){
        end = esp_timer_get_time();
        if (gpio_get_level(pin))
        {
            break;
        }
        
    }
    passed = end - start;
    if((passed/1000)<10){
        esp_restart();
    }
    else{
        // Clear the configuration data and reset the Matter stack
        chip::Server::GetInstance().GetCommissioningWindowManager().CloseCommissioningWindow();
        chip::DeviceLayer::ConfigurationMgr().InitiateFactoryReset();
        open_commissioning_window_if_necessary();
    }
}

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    esp_err_t err;
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(TAG, "Interface IP Address changed");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        // initialize temperature and humidity sensor driver (shtc3)
        break;

    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(TAG, "Commissioning failed, fail safe timer expired");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(TAG, "Commissioning session started");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(TAG, "Commissioning session stopped");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Commissioning window opened");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(TAG, "Commissioning window closed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        {
            ESP_LOGI(TAG, "Fabric removed successfully");
            if (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0)
            {
                chip::CommissioningWindowManager & commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
                constexpr auto kTimeoutSeconds = chip::System::Clock::Seconds16(k_timeout_seconds);
                if (!commissionMgr.IsCommissioningWindowOpen())
                {
                    /* After removing last fabric, this example does not remove the Wi-Fi credentials
                     * and still has IP connectivity so, only advertising on DNS-SD.
                     */
                    CHIP_ERROR err = commissionMgr.OpenBasicCommissioningWindow(kTimeoutSeconds,
                                                    chip::CommissioningWindowAdvertisement::kDnssdOnly);
                    if (err != CHIP_NO_ERROR)
                    {
                        ESP_LOGE(TAG, "Failed to open commissioning window, err:%" CHIP_ERROR_FORMAT, err.Format());
                    }
                }
            }
        break;
        }

    case chip::DeviceLayer::DeviceEventType::kFabricWillBeRemoved:
        ESP_LOGI(TAG, "Fabric will be removed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricUpdated:
        ESP_LOGI(TAG, "Fabric is updated");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricCommitted:
        ESP_LOGI(TAG, "Fabric is committed");
        break;
    default:
        break;
    }
}

static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *priv_data)
{
    ESP_LOGI(TAG, "Identification callback: type: %u, effect: %u, variant: %u", type, effect_id, effect_variant);
    return ESP_OK;
}

static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    esp_err_t err = ESP_OK;

    if (type == PRE_UPDATE) {
        /* Driver update */
    }

    return err;
}
void main_task(void *pvParameter) {

    while (1) {
        read_sensor_data(sensor.temperature, sensor.humidity,sensor.battery_volt);
        store_data(&sensor);
        vTaskDelay(60000 / portTICK_PERIOD_MS); // Delay for 1 minute
        esp_matter::cluster::sensor::start_bdx_transfer(esp_matter::cluster::sensor::endpoint_id);
    }
}
extern "C" void app_main()
{
    esp_err_t err = ESP_OK;

    /* Initialize the ESP NVS layer */
    nvs_flash_init();

#if CONFIG_PM_ENABLE
    esp_pm_config_t pm_config = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
        .light_sleep_enable = true
#endif
    };
    err = esp_pm_configure(&pm_config);
#endif
    /* Create a Matter node and add the mandatory Root Node device type on endpoint 0 */
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));
    // add temperature sensor device
    temperature_sensor::config_t temp_sensor_config;
    temp_sensor_ep = temperature_sensor::create(node, &temp_sensor_config, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(temp_sensor_ep != nullptr, ESP_LOGE(TAG, "Failed to create temperature_sensor endpoint"));

    // add the humidity sensor device
    humidity_sensor::config_t humidity_sensor_config;
    humidity_sensor_ep = humidity_sensor::create(node, &humidity_sensor_config, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(humidity_sensor_ep != nullptr, ESP_LOGE(TAG, "Failed to create humidity_sensor endpoint"));


    electrical_sensor::config_t blevel_config;
    blevel_ep = electrical_sensor::create(node, &blevel_config, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(blevel_ep != nullptr, ESP_LOGE(TAG, "Failed to create blevel endpoint"));
    
    static sensor_config_t sht30_config = {
        .temperature = {
            .cb = temp_sensor_notification,
            .endpoint_id = endpoint::get_id(temp_sensor_ep),
        },
        .humidity = {
            .cb = humidity_sensor_notification,
            .endpoint_id = endpoint::get_id(humidity_sensor_ep),
        },
        .bat_level = {
            .cb = battery_notification,
            .endpoint_id = endpoint::get_id(blevel_ep),
        },
    };
    err = sensor_init(&sht30_config);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to initialize temperature sensor driver"));

    

    button_gpio_init(&button_config);
    gpio_num_t btn = GPIO_NUM_2;
    // button_gpio_set_intr(btn, GPIO_INTR_POSEDGE, button_isr_handler, NULL);

    // endpoint::on_off_light::config_t endpoint_config;
    // endpoint_t *app_endpoint = endpoint::on_off_light::create(node, &endpoint_config, ENDPOINT_FLAG_NONE, NULL);
    // ABORT_APP_ON_FAILURE(app_endpoint != nullptr, ESP_LOGE(TAG, "Failed to create on off light endpoint"));

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* Set OpenThread platform config */
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);
#endif

    /* Matter start */
    err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));
    // Store and retrieve MeasurementData (e.g., on some event or periodically)
    // MeasurementData data = {50.5f, 25.0f, 85.0f}; // Example values for humidity, temperature, and battery SOC
    // err = customDataDelegate.store_data(data);
    // if (err == ESP_OK) {
    //     ESP_LOGI(TAG, "Measurement data stored successfully");
    // }

}

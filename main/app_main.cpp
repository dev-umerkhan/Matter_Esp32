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
#if CONFIG_PM_ENABLE
#include <esp_pm.h>
#endif

#include <esp_matter.h>
#include <esp_matter_ota.h>
#include <app/util/basic-types.h>
#include <common_macros.h>
#include <app_priv.h>
#include <app/AttributeAccessInterface.h>
#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif

#include <app/server/CommissioningWindowManager.h>
// Include Matter (CHIP) headers
#include <app/server/OnboardingCodesUtil.h>
#include <lib/support/logging/CHIPLogging.h>
// #include <app/ClusterId.h>
#include <app/AttributeAccessInterface.h>
#include <protocols/bdx/BDXTransferServer.h>
// #include <chip/DeviceLayer/ConfigurationManager.h>
// #include <chip/DeviceLayer/PlatformManager.h>
// #include <chip/DeviceLayer/ConnectivityManager.h>
#include "sht30.h"
#include "max17048_app.h"
// #include "button.h"
#include <driver/gpio.h>
#include "button_gpio.h"

#define NVS_NAMESPACE "Measured_Data"
#define CLUSTER_ID  0x8003
static const char *TAG = "app_main";

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

constexpr auto k_timeout_seconds = 300;
endpoint_t * temp_sensor_ep;
endpoint_t * humidity_sensor_ep; 
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
// Define a struct to hold the measurement data (humidity, temperature, and battery SOC)
struct MeasurementData {
    float humidity;
    float temperature;
    float battery_soc;
};
namespace chip {
namespace app {
namespace Clusters {
namespace CustomDataStorage {

// Custom Cluster definition
class Delegate {
public:
    virtual ~Delegate() = default;

    virtual esp_err_t store_data(const MeasurementData& data)  = 0;  // Store Data
    virtual esp_err_t retrieve_data(MeasurementData& data) = 0;  // Retrieve Data
};
class custom_Data: public Delegate{
    private:
        int data_index = 0;
    public:
      // Function to store data in NVS
    esp_err_t store_data(const MeasurementData& data) {
        nvs_handle_t nvs_handle;
        esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
        if (err != ESP_OK) {
            ESP_LOGE("store_data", "Failed to open NVS: %s", esp_err_to_name(err));
            return err;
        }

        // Create a composite key using the index
        std::string key = "measurement_" + std::to_string(data_index);

        // Store the data (entire struct)
        err = nvs_set_blob(nvs_handle, key.c_str(), &data, sizeof(MeasurementData));
        if (err != ESP_OK) {
            ESP_LOGE("store_data", "Failed to store data: %s", esp_err_to_name(err));
            nvs_close(nvs_handle);
            return err;
        }

        // Commit the data to NVS
        err = nvs_commit(nvs_handle);
        if (err != ESP_OK) {
            ESP_LOGE("store_data", "Failed to commit data to NVS: %s", esp_err_to_name(err));
            nvs_close(nvs_handle);
            return err;
        }

        nvs_close(nvs_handle);
        ESP_LOGI("store_data", "Data stored successfully at index %d", data_index);
        return ESP_OK;
    }

    // Function to retrieve data from NVS
    esp_err_t retrieve_data(MeasurementData& data) {
        nvs_handle_t nvs_handle;
        esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
        if (err != ESP_OK) {
            ESP_LOGE("retrieve_data", "Failed to open NVS: %s", esp_err_to_name(err));
            return err;
        }

        // Create a composite key using the index
        std::string key = "measurement_" + std::to_string(data_index);

        // Retrieve the data (entire struct)
        size_t required_size = sizeof(MeasurementData);
        err = nvs_get_blob(nvs_handle, key.c_str(), &data, &required_size);
        if (err != ESP_OK) {
            ESP_LOGE("retrieve_data", "Failed to retrieve data: %s", esp_err_to_name(err));
            nvs_close(nvs_handle);
            return err;
        }

        nvs_close(nvs_handle);
        ESP_LOGI("retrieve_data", "Data retrieved successfully for index %d", data_index);
        return ESP_OK;
    }
};



// Cluster implementation
class Instance : public AttributeAccessInterface {

public:
    static constexpr ClusterId Id = CLUSTER_ID;
    Instance(EndpointId aEndpointId, Delegate & aDelegate) :
    AttributeAccessInterface(chip::Optional<EndpointId>(aEndpointId), Id), mDelegate(aDelegate) {}

    // Instance(EndpointId aEndpointId, Delegate & aDelegate) :
    //     AttributeAccessInterface(aEndpointId, Id), mDelegate(aDelegate) {}

    CHIP_ERROR Init() { return CHIP_NO_ERROR; }
    CHIP_ERROR Read(const ConcreteReadAttributePath & aPath, AttributeValueEncoder & aEncoder) override {
        // Implement read logic based on the attribute path
        return CHIP_NO_ERROR;
    }

private:
    Delegate & mDelegate;
};

} // namespace CustomDataStorage
} // namespace Clusters
} // namespace app
} // namespace chip


// } // namespace Clusters
// } // namespace app
// } // namespace chip
// static float read_battery_level()
// {
//     uint8_t battery_level_raw = 0;
//     // Use your max17048 sensor driver to get the battery level
//     // Assuming the battery level is returned as a percentage
//     int err = max17048_get_battery_level(&battery_level_raw);
//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to read battery level from MAX17048");
//         return 0.0f;
//     }
//     return static_cast<float>(battery_level_raw); // Assuming this is a percentage
// }

// static void update_battery_level(uint16_t endpoint_id)
// {
//     // Read the battery level from the MAX17048 sensor
//     float battery_level = read_battery_level();
    
//     // Set the battery level attribute in the Matter BatteryLevel cluster
//     BatteryLevel::SetBatteryLevel(endpoint_id, battery_level);
// }
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
    while(gpio_get_level(pin)){
        end = esp_timer_get_time();
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
        static sht30_sensor_config_t sht30_config = {
            .temperature = {
                .cb = temp_sensor_notification,
                .endpoint_id = endpoint::get_id(temp_sensor_ep),
            },
            .humidity = {
                .cb = humidity_sensor_notification,
                .endpoint_id = endpoint::get_id(humidity_sensor_ep),
            },
        };
        err = sht30_sensor_init(&sht30_config);
        ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to initialize temperature sensor driver"));
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
    // Create the custom data storage delegate and cluster instance
    CustomDataStorage::custom_Data customDataDelegate;
    CustomDataStorage::Instance customDataInstance(1, customDataDelegate); // EndpointId = 1 for example

    // Register the custom cluster in the Matter stack
    
    CHIP_ERROR chipErr = chip::Server::GetInstance().RegisterCluster(CustomDataStorage::Id, customDataInstance);
    if (chipErr != CHIP_NO_ERROR) {
        ESP_LOGE(TAG, "Failed to register custom cluster: %s", ErrorStr(chipErr));
        return;
    }
    ESP_LOGI(TAG, "Custom cluster registered successfully");
    temperature_sensor::config_t temp_sensor_config;
    temp_sensor_ep = temperature_sensor::create(node, &temp_sensor_config, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(temp_sensor_ep != nullptr, ESP_LOGE(TAG, "Failed to create temperature_sensor endpoint"));

    // add the humidity sensor device
    humidity_sensor::config_t humidity_sensor_config;
    humidity_sensor_ep = humidity_sensor::create(node, &humidity_sensor_config, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(humidity_sensor_ep != nullptr, ESP_LOGE(TAG, "Failed to create humidity_sensor endpoint"));

    // power_source_device::config_t power_source_config;
    // endpoint_t * battery_sensor_ep = power_source_device::create(node, &power_source_config, ENDPOINT_FLAG_NONE, NULL);
    // ABORT_APP_ON_FAILURE(battery_sensor_ep != nullptr, ESP_LOGE(TAG, "Failed to create battery_sensor endpoint"));

    static sht30_sensor_config_t sht30_config = {
        .temperature = {
            .cb = temp_sensor_notification,
            .endpoint_id = endpoint::get_id(temp_sensor_ep),
        },
        .humidity = {
            .cb = humidity_sensor_notification,
            .endpoint_id = endpoint::get_id(humidity_sensor_ep),
        },
    };
    err = sht30_sensor_init(&sht30_config);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to initialize temperature sensor driver"));

    max17048_config_t max_config = {
        .i2c_num = I2C_NUM_0,   // Use I2C port 0
        .i2c_addr = MAX17048_DEFAULT_ADDR,  // Default I2C address
    };

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
    MeasurementData data = {50.5f, 25.0f, 85.0f}; // Example values for humidity, temperature, and battery SOC
    err = customDataDelegate.store_data(data);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Measurement data stored successfully");
    }

    // Retrieve stored data and log it
    MeasurementData retrievedData;
    err = customDataDelegate.retrieve_data(retrievedData);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Retrieved Measurement Data: Humidity=%.2f, Temperature=%.2f, Battery SOC=%.2f",
                 retrievedData.humidity, retrievedData.temperature, retrievedData.battery_soc);
    } else {
        ESP_LOGE(TAG, "Failed to retrieve measurement data");
    }

    // Commissioning management (this will be handled by existing logic)
    open_commissioning_window_if_necessary();
}

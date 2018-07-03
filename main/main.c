#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "nvs_flash.h"
#include "esp_log.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"

#include "driver/gpio.h"

static const char TAG[] = "main";
static const char DEVICE_NAME[] = "ble_battery";

static uint8_t adv_config_done = 0;
static const uint8_t adv_config_flag = (1 << 0);
static const uint8_t scan_rsp_config_flag = (1 << 1);

#define BATTERY_APP_INDEX (0)

static esp_ble_adv_params_t adv_params = {
  .adv_int_min        = 0x20,
  .adv_int_max        = 0x40,
  .adv_type           = ADV_TYPE_IND,
  .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
  //.peer_addr            =
  //.peer_addr_type       =
  .channel_map        = ADV_CHNL_ALL,
  .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static uint8_t adv_service_uuid128[32] = {
  /* LSB <--------------------------------------------------------------------------------> MSB */
  //first uuid, 16bit, [12],[13] is the value
  0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
  //second uuid, 32bit, [12], [13], [14], [15] is the value
  0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

static esp_ble_adv_data_t adv_data = {
  .set_scan_rsp = false,
  .include_name = true,
  .include_txpower = true,
  .min_interval = 0x20,
  .max_interval = 0x40,
  .appearance = 0x00,
  .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
  .p_manufacturer_data =  NULL, //&test_manufacturer[0],
  .service_data_len = 0,
  .p_service_data = NULL,
  .service_uuid_len = 32,
  .p_service_uuid = adv_service_uuid128,
  .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_data_t scan_rsp_data = {
  .set_scan_rsp = true,
  .include_name = true,
  .include_txpower = true,
  .min_interval = 0x20,
  .max_interval = 0x40,
  .appearance = 0x00,
  .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
  .p_manufacturer_data =  NULL, //&test_manufacturer[0],
  .service_data_len = 0,
  .p_service_data = NULL,
  .service_uuid_len = 32,
  .p_service_uuid = adv_service_uuid128,
  .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

struct gatts_profile_inst {
  esp_gatts_cb_t gatts_cb;
  uint16_t gatts_if;
  esp_gatt_srvc_id_t service_id;
  uint16_t conn_id;
  bool connected;
};

static void gatts_profile_ota_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static struct gatts_profile_inst profile_table[] = {
  [BATTERY_APP_INDEX] = {
    .gatts_cb = gatts_profile_ota_event_handler,
    .gatts_if = ESP_GATT_IF_NONE,
    .connected = false,
  },
};

#define PROFILE_NUM (sizeof(profile_table) / sizeof(profile_table[0]))


static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t ccc_desc_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint16_t char_format_desc_uuid = ESP_GATT_UUID_CHAR_PRESENT_FORMAT;

static const uint16_t battery_service_uuid = ESP_GATT_UUID_BATTERY_SERVICE_SVC;
static const uint16_t battery_level_uuid = ESP_GATT_UUID_BATTERY_LEVEL;

static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

static uint8_t battery_level = 50;
static uint16_t battery_level_ccc = 0;

enum battery_service_attribute_index {
  BATTERY_SERVICE = 0,
  BATTERY_LEVEL_CHARACTER_DECLARATION,
  BATTERY_LEVEL_VALUE,
  BATTERY_LEVEL_DESC_CCC,
  BATTERY_LEVEL_DESC_FORMAT,
};

static const esp_gatts_attr_db_t gatt_db[] = {
  [BATTERY_SERVICE] = {
    {ESP_GATT_AUTO_RSP},
    {
      ESP_UUID_LEN_16, (uint8_t*)&primary_service_uuid,
      ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(battery_service_uuid), (uint8_t*)&battery_service_uuid
    }
  },
  [BATTERY_LEVEL_CHARACTER_DECLARATION] = {
    {ESP_GATT_AUTO_RSP},
    {
      ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid,
      ESP_GATT_PERM_READ,
      1, 1, (uint8_t*)&char_prop_read_notify
    }
  },
  [BATTERY_LEVEL_VALUE] = {
    {ESP_GATT_AUTO_RSP},
    {
      ESP_UUID_LEN_16, (uint8_t*)&battery_level_uuid,
      ESP_GATT_PERM_READ,
      sizeof(uint8_t), sizeof(uint8_t), (uint8_t*)&battery_level
    }
  },
  [BATTERY_LEVEL_DESC_CCC] = {
    {ESP_GATT_AUTO_RSP},
    {
      ESP_UUID_LEN_16, (uint8_t*)&ccc_desc_uuid,
      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(uint16_t), (uint8_t*)&battery_level_ccc
    }
  },
  [BATTERY_LEVEL_DESC_FORMAT] = {
    {ESP_GATT_AUTO_RSP},
    {
      ESP_UUID_LEN_16, (uint8_t*)&char_format_desc_uuid,
      ESP_GATT_PERM_READ,
      7, 0, NULL
    }
  },
};

#define NUM_HANDLES (sizeof(gatt_db) / sizeof(gatt_db[0]))
uint16_t handle_table[NUM_HANDLES];

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
      adv_config_done &= (~adv_config_flag);
      if (adv_config_done == 0){
        esp_ble_gap_start_advertising(&adv_params);
      }
      break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
      adv_config_done &= (~scan_rsp_config_flag);
      if (adv_config_done == 0){
        esp_ble_gap_start_advertising(&adv_params);
      }
      break;
    default:
      break;
  }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
  if (event == ESP_GATTS_REG_EVT) {
    if (param->reg.status == ESP_GATT_OK) {
      profile_table[param->reg.app_id].gatts_if = gatts_if;
    } else {
      ESP_LOGI(TAG, "Reg app failed, app_id %04x, status %d\n",
          param->reg.app_id,
          param->reg.status);
      return;
    }
  }

  for (int i = 0; i < PROFILE_NUM; ++i) {
    if (gatts_if == ESP_GATT_IF_NONE || gatts_if == profile_table[i].gatts_if) {
      if (profile_table[i].gatts_cb) {
        profile_table[i].gatts_cb(event, gatts_if, param);
      }
    }
  }
}

static void gatts_profile_ota_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
  esp_err_t ret;
  switch (event) {
    case ESP_GATTS_REG_EVT:
      if (param->reg.status != ESP_GATT_OK) {
        ESP_LOGE(TAG, "app registration failed, status = %x", param->reg.status);
        break;
      }
      ret = esp_ble_gap_set_device_name(DEVICE_NAME);
      if (ret) {
        ESP_LOGE(TAG, "set device name failed, error code = %x", ret);
      }
      ret = esp_ble_gap_config_adv_data(&adv_data);
      if (ret) {
        ESP_LOGE(TAG, "config adv data failed, error code = %x", ret);
      }
      adv_config_done |= adv_config_flag;
      ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
      if (ret) {
        ESP_LOGE(TAG, "config scan response data failed, error code = %x", ret);
      }
      adv_config_done |= scan_rsp_config_flag;
      ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, NUM_HANDLES, BATTERY_SERVICE);
      if (ret) {
        ESP_LOGE(TAG, "create attr table failed, error code = %x", ret);
      }
      break;
    case ESP_GATTS_READ_EVT:
      ESP_LOGI(TAG, "ESP_GATTS_READ_EVT");
      break;
    case ESP_GATTS_WRITE_EVT:
      ESP_LOGI(TAG, "ESP_GATTS_WRITE_EVT");
      break;
    case ESP_GATTS_CONNECT_EVT:
      {
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        conn_params.latency = 0;
        conn_params.max_int = 0x20;
        conn_params.max_int = 0x10;
        conn_params.timeout = 400;
        profile_table[BATTERY_APP_INDEX].conn_id = param->connect.conn_id;
        profile_table[BATTERY_APP_INDEX].connected = true;
        esp_ble_gap_update_conn_params(&conn_params);
      }
      break;
    case ESP_GATTS_DISCONNECT_EVT:
        profile_table[BATTERY_APP_INDEX].connected = false;
      esp_ble_gap_start_advertising(&adv_params);
      break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
      if (param->add_attr_tab.status != ESP_GATT_OK) {
        ESP_LOGE(TAG, "create attr tab failed, error code = %x", param->add_attr_tab.status);
        break;
      }
      if (param->add_attr_tab.num_handle != NUM_HANDLES) {
        ESP_LOGE(TAG, "create attr tab abnormally, num_handle(%d) doesn't match", param->add_attr_tab.num_handle);
        break;
      }
      ESP_LOGI(TAG, "create attr tab successfully");
      memcpy(handle_table, param->add_attr_tab.handles, sizeof(handle_table));
      esp_ble_gatts_start_service(handle_table[BATTERY_SERVICE]);
    default:
      break;
  }
}

static void setup_gpio();

void app_main() {
  esp_err_t ret;
  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ret = esp_bt_controller_init(&bt_cfg);
  if (ret) {
    ESP_LOGE(TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
    return;
  }

  ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (ret) {
    ESP_LOGE(TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
    return;
  }
  ret = esp_bluedroid_init();
  if (ret) {
    ESP_LOGE(TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
    return;
  }
  ret = esp_bluedroid_enable();
  if (ret) {
    ESP_LOGE(TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
    return;
  }

  ret = esp_ble_gatts_register_callback(gatts_event_handler);
  if (ret){
    ESP_LOGE(TAG, "gatts register error, error code = %x", ret);
    return;
  }
  ret = esp_ble_gap_register_callback(gap_event_handler);
  if (ret){
    ESP_LOGE(TAG, "gap register error, error code = %x", ret);
    return;
  }
  ret = esp_ble_gatts_app_register(BATTERY_APP_INDEX);
  if (ret){
    ESP_LOGE(TAG, "gatts app register error, error code = %x", ret);
    return;
  }
  esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
  if (local_mtu_ret){
    ESP_LOGE(TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    return;
  }

  setup_gpio();
}

static xQueueHandle gpio_event_queue = NULL;
static void gpio_task(void *arg) {
  uint32_t gpio_num;
  while (1) {
    if (xQueueReceive(gpio_event_queue, &gpio_num, portMAX_DELAY)) {
      ESP_LOGI(TAG, "button pressed");
      battery_level %= 100;
      battery_level += 1;
      esp_err_t ret = esp_ble_gatts_set_attr_value(handle_table[BATTERY_LEVEL_VALUE], 1, (const uint8_t*)&battery_level);
      if (ret) {
        ESP_LOGE(TAG, "set attr error, error code = %x", ret);
      }

      if (!profile_table[BATTERY_APP_INDEX].connected) {
        continue;
      }

      uint16_t len;
      const uint8_t *val;
      if (esp_ble_gatts_get_attr_value(handle_table[BATTERY_LEVEL_DESC_CCC], &len, &val) != ESP_GATT_OK) {
        ESP_LOGE(TAG, "get attr failed, error code = %x", ret);
        continue;
      }
      if (val[0] != 0x01) {
        ESP_LOGI(TAG, "skip notification");
        continue;
      }
      ret = esp_ble_gatts_send_indicate(
        profile_table[BATTERY_APP_INDEX].gatts_if,
        profile_table[BATTERY_APP_INDEX].conn_id,
        handle_table[BATTERY_LEVEL_VALUE],
        1,
        (uint8_t*)&battery_level,
        false
      );
      if (ret) {
        ESP_LOGE(TAG, "notify error, error code = %x", ret);
      }
      ESP_LOGI(TAG, "notification done");
    }
  }
}

static void IRAM_ATTR gpio_isr_handler(void *arg) {
  uint32_t gpio_num = (uint32_t) arg;
  xQueueSendFromISR(gpio_event_queue, &gpio_num, NULL);
}

#define GPIO_BUTTON_MIDDLE 38
static void setup_gpio() {
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = (1ULL << GPIO_BUTTON_MIDDLE);
  io_conf.pull_down_en = 0;
  io_conf.pull_up_en = 1;
  gpio_config(&io_conf);

  gpio_event_queue = xQueueCreate(16, sizeof(uint32_t));
  xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);

  gpio_set_intr_type(GPIO_BUTTON_MIDDLE, GPIO_INTR_NEGEDGE);
  gpio_install_isr_service(0);
  gpio_isr_handler_add(GPIO_BUTTON_MIDDLE, gpio_isr_handler, (void*) GPIO_BUTTON_MIDDLE);

  ESP_LOGI(TAG, "gpio setup complete");
}

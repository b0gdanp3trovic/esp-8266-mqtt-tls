#include "nrf24.h"

#include <espressif/esp_sta.h>

#include <espressif/esp_wifi.h>

#include "bmp280/bmp280.h"


extern "C" {
  #include "paho_mqtt_c/MQTTESP8266.h"

  #include "paho_mqtt_c/MQTTClient.h"

}

#include <semphr.h>

#define PCF_ADDRESS 0x38
#define MPU_ADDRESS 0x68
#define BUS_I2C 0
#define SCL 14
#define SDA 12

#define rx_size 100
//static char rx_data[rx_size];

#define MQTT_HOST "2.tcp.eu.ngrok.io"
#define MQTT_PORT 13275
#define MQTT_TOPIC "/mqttsender"

#define MQTT_USER "esp"
#define MQTT_PASS "esppass"

static int wifi_alive = 0;
#define configCHECK_FOR_STACK_OVERFLOW 1;

QueueHandle_t publish_queue;
#define PUB_MSG_LEN 16

typedef enum {
  BMP280_TEMPERATURE,
  BMP280_PRESSURE
}
bmp280_quantity;

bmp280_t bmp280_dev;

// read BMP280 sensor values
float read_bmp280(bmp280_quantity quantity) {

  float temperature, pressure;

  bmp280_force_measurement( & bmp280_dev);
  // wait for measurement to complete
  while (bmp280_is_measuring( & bmp280_dev)) {};
  bmp280_read_float( & bmp280_dev, & temperature, & pressure, NULL);

  return temperature;
}

static void mqtt_task(void * pvParameters) {
  int ret = 0;
  struct mqtt_network network;
  mqtt_client_t client = mqtt_client_default;
  char mqtt_client_id[20];
  uint8_t mqtt_buf[100];
  uint8_t mqtt_readbuf[100];
  mqtt_packet_connect_data_t data = mqtt_packet_connect_data_initializer;

  memset(mqtt_client_id, 0, sizeof(mqtt_client_id));
  strcpy(mqtt_client_id, "ESP-");
  strcat(mqtt_client_id, "testid");
  UBaseType_t uxHighWaterMark;

  while (1) {
    if (!wifi_alive) {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }
    //vTaskDelay(pdMS_TO_TICKS(10000));
    printf("%s: started\n\r", __func__);
    printf("NO SSL\n");

    mqtt_network_new( & network);
    ret = mqtt_network_connect( & network, MQTT_HOST, MQTT_PORT);
    if (ret) {
      printf("error: %d\n\r", ret);
      taskYIELD();
      continue;
    }

    uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

    printf("done\n\r");
    mqtt_client_new( & client, & network, 5000, mqtt_buf, 100, mqtt_readbuf,
      100);

    data.MQTTVersion = 4;
    data.keepAliveInterval = 500;
    data.cleansession = 0;
    data.clientID.cstring = "testid";
    data.willFlag = 1;
    data.will.message.cstring = "will_message";
    data.will.retained = 1;
    data.will.topicName.cstring = "MQTTV5/test";
    data.username.cstring = MQTT_USER;
    data.password.cstring = MQTT_PASS;
    printf("Send MQTT connect ... ");
    ret = mqtt_connect( & client, & data);
    if (ret) {
      printf("error: %d\n\r", ret);
      mqtt_network_disconnect( & network);
      taskYIELD();
      continue;
    }
    printf("done\r\n");

    //mqtt_subscribe(&client, "/esptopic", MQTT_QOS1, topic_received);
    xQueueReset(publish_queue);

    while (1) {
      if ((read_byte_pcf() & button1) == 0) {
        char msg[PUB_MSG_LEN - 1] = "%.2f C", read_bmp280(BMP280_TEMPERATURE);
        mqtt_message_t message;
        message.payload = msg;
        message.payloadlen = PUB_MSG_LEN;
        message.dup = 0;
        message.qos = MQTT_QOS2;
        message.retained = 0;
        ret = mqtt_publish( & client, MQTT_TOPIC, & message);
        if (ret != MQTT_SUCCESS) {
          printf("error while publishing message: %d\n", ret);
          break;
        }
        // check again after 200 ms
      }
      vTaskDelay(pdMS_TO_TICKS(200));
    }

    ret = mqtt_yield( & client, 1000);
    if (ret == MQTT_DISCONNECTED)
      break;
  }
  printf("Connection dropped, request restart\n\r");
  mqtt_network_disconnect( & network);
  taskYIELD();
}

static void wifi_task(void * pvParameters) {
  uint8_t status = 0;
  uint8_t retries = 30;
  struct sdk_station_config config = {
    WIFI_SSID,
    WIFI_PASS
  };
  /*struct sdk_station_config config = {
   .ssid = WIFI_SSID,
   .password = WIFI_PASS,
   };*/

  printf("WiFi: connecting to WiFi\n\r");
  sdk_wifi_set_opmode(STATION_MODE);
  sdk_wifi_station_set_config( & config);

  while (1) {
    wifi_alive = 0;
    while ((status != STATION_GOT_IP) && (retries)) {
      status = sdk_wifi_station_get_connect_status();
      printf("%s: status = %d\n\r", __func__, status);
      if (status == STATION_WRONG_PASSWORD) {
        printf("WiFi: wrong password\n\r");
        break;
      } else if (status == STATION_NO_AP_FOUND) {
        printf("WiFi: AP not found\n\r");
        break;
      } else if (status == STATION_CONNECT_FAIL) {
        printf("WiFi: connection failed\r\n");
        break;
      }
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      --retries;
    }

    while ((status = sdk_wifi_station_get_connect_status()) == STATION_GOT_IP) {
      if (wifi_alive == 0) {
        printf("WiFi: Connected\n\r");
        wifi_alive = 1;
      }
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    wifi_alive = 0;
    printf("WiFi: disconnected\n\r");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

extern "C"
void user_init(void) {

  setup_nrf();
  uart_set_baud(0, 115200);
  // fix i2c driver to work with MPU-9250
  gpio_enable(SCL, GPIO_OUTPUT);

  printf("SDK version: %s, free heap %u\n", sdk_system_get_sdk_version(),
    xPortGetFreeHeapSize());
  //radio.openReadingPipe(1, address);
  //radio.startListening();
  //xTaskCreate(listen_task, "Radio listen task", 1000, NULL, 3, NULL);

  // BMP280 configuration
  bmp280_params_t params;
  bmp280_init_default_params( & params);
  params.mode = BMP280_MODE_FORCED;
  bmp280_dev.i2c_dev.bus = BUS_I2C;
  bmp280_dev.i2c_dev.addr = BMP280_I2C_ADDRESS_0;
  bmp280_init( & bmp280_dev, & params);

  i2c_init(BUS_I2C, SCL, SDA, I2C_FREQ_100K);

  publish_queue = xQueueCreate(2, 16);
  xTaskCreate( & wifi_task, "wifi_task", 256, NULL, 2, NULL);
  xTaskCreate( & mqtt_task, "mqtt_task", 2048, NULL, 4, NULL);
}

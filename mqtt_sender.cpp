// this must be ahead of any mbedtls header files so the local mbedtls/config.h can be properly referenced
extern "C" {
  #include "ssl_connection.h"

}

#include "nrf24.h"

#include <espressif/esp_sta.h>

#include <espressif/esp_wifi.h>

extern "C" {
  #include "paho_mqtt_c/MQTTESP8266.h"

  #include "paho_mqtt_c/MQTTClient.h"

}

#include <semphr.h>

#include "bmp280/bmp280.h"


#define rx_size 100
//static char rx_data[rx_size];

#define MQTT_HOST "mqttest.hopto.org"
#define MQTT_PORT 8883
#define MQTT_TOPIC "/mqttsender"

#define MQTT_USER "esp"
#define MQTT_PASS "esppass"

extern char * ca_cert, * client_endpoint, * client_cert, * client_key;
static int ssl_reset;
static SSLConnection * ssl_conn;
static int wifi_alive = 0;
#define configCHECK_FOR_STACK_OVERFLOW 1;

QueueHandle_t publish_queue;
#define PUB_MSG_LEN 16

#define PCF_ADDRESS 0x38
#define MPU_ADDRESS 0x68
#define BUS_I2C 0
#define SCL 14
#define SDA 12

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

static int mqtt_ssl_read(mqtt_network_t * n, unsigned char * buffer, int len,
  int timeout_ms) {
  int r = ssl_read(ssl_conn, buffer, len, timeout_ms);
  if (r <= 0 &&
    (r != MBEDTLS_ERR_SSL_WANT_READ &&
      r != MBEDTLS_ERR_SSL_WANT_WRITE &&
      r != MBEDTLS_ERR_SSL_TIMEOUT)) {
    printf("%s: TLS read error (%d), resetting\n\r", __func__, r);
    ssl_reset = 1;
  };
  return r;
}

static int mqtt_ssl_write(mqtt_network_t * n, unsigned char * buffer, int len,
  int timeout_ms) {
  int r = ssl_write(ssl_conn, buffer, len, timeout_ms);
  if (r <= 0 &&
    (r != MBEDTLS_ERR_SSL_WANT_READ &&
      r != MBEDTLS_ERR_SSL_WANT_WRITE)) {
    printf("%s: TLS write error (%d), resetting\n\r", __func__, r);
    ssl_reset = 1;
  }
  return r;
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char * pcTaskName) {
  /* Stack overflow */
  printf("Task %s stack overflow!n", pcTaskName);
}

static void mqtt_task(void * pvParameters) {
  printf("Initializing ssl connection\n");
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

  ssl_conn = (SSLConnection * ) malloc(sizeof(SSLConnection));
  printf("%s: connecting to MQTT server %s ... ", __func__,
    client_endpoint);
  while (1) {
    if (!wifi_alive) {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }
    //vTaskDelay(pdMS_TO_TICKS(10000));
    printf("%s: started\n\r", __func__);
    printf("Initializing ssl connection\n");

    ssl_reset = 0;
    ssl_init(ssl_conn);
    ssl_conn -> ca_cert_str = ca_cert;
    ssl_conn -> client_cert_str = client_cert;
    ssl_conn -> client_key_str = client_key;

    mqtt_network_new( & network);
    network.mqttread = mqtt_ssl_read;
    network.mqttwrite = mqtt_ssl_write;

    ret = ssl_connect(ssl_conn, client_endpoint, MQTT_PORT);
    uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

    if (ret) {
      printf("error ssl...!!!\n\r");
      printf("error: %d\n\r", ret);
      ssl_destroy(ssl_conn);
      continue;
    }
    printf("done\n\r");
    mqtt_client_new( & client, & network, 5000, mqtt_buf, 100, mqtt_readbuf,
      100);

    printf("what the\n");
    data.willFlag = 0;
    data.MQTTVersion = 4;
    data.clientID.cstring = "test-id";
    data.username.cstring = MQTT_USER;
    data.password.cstring = MQTT_PASS;
    data.keepAliveInterval = 1200;
    data.cleansession = 1;
    //data.ssl.trustStore = "/home/bso/certs/ca.crt";
    //data.ssl.keyStore = "/home/bso/certs/client.pem";
    printf("Send MQTT connect ... ");
    mqtt_connect( & client, & data);
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
        printf("got message to publish\r\n");
        char msg[PUB_MSG_LEN - 1] = "%.2f C", read_bmp280(BMP280_TEMPERATURE);
        mqtt_message_t message;
        message.payload = msg;
        message.payloadlen = PUB_MSG_LEN;
        message.dup = 0;
        message.qos = MQTT_QOS1;
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
  ssl_destroy(ssl_conn);
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
  printf("SDK version: %s, free heap %u\n", sdk_system_get_sdk_version(),
    xPortGetFreeHeapSize());

  // BMP280 configuration
  bmp280_params_t params;
  bmp280_init_default_params( & params);
  params.mode = BMP280_MODE_FORCED;
  bmp280_dev.i2c_dev.bus = BUS_I2C;
  bmp280_dev.i2c_dev.addr = BMP280_I2C_ADDRESS_0;
  bmp280_init( & bmp280_dev, & params);

  i2c_init(BUS_I2C, SCL, SDA, I2C_FREQ_100K);
  //radio.openReadingPipe(1, address);
  //radio.startListening();
  //xTaskCreate(listen_task, "Radio listen task", 1000, NULL, 3, NULL);

  publish_queue = xQueueCreate(2, 16);
  xTaskCreate( & wifi_task, "wifi_task", 256, NULL, 2, NULL);
  xTaskCreate( & mqtt_task, "mqtt_task", 2048, NULL, 4, NULL);
}

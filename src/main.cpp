#include "OV2640.h"
#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiClient.h>

#include <PubSubClient.h>

#include "CRtspSession.h"
#include "OV2640Streamer.h"
#include "SimStreamer.h"
#include "wifikeys.h"

#include "quirc/quirc.h"

#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"

#define WAIT_TIME_BEFORE_CONNECTION_RETRY 5000

#define PICKUP_POINT_N "1"
#define PICKUP_POINT_PUBLISH_BASE "sm_iot_lab/pickup_point"
#define CUBE_SCANNED_PUBLISH "cube/scanned"
#define SCANNED_PUBLISH_TOPIC PICKUP_POINT_PUBLISH_BASE "/" PICKUP_POINT_N "/" CUBE_SCANNED_PUBLISH

static OV2640 cam;

WebServer server(80);

WiFiClient client;
PubSubClient mqttClient(client);

TaskHandle_t QRCodeReader_Task;
struct QRCodeData {
  bool valid;
  int dataType;
  uint8_t payload[1024];
  int payloadLen;
};
struct quirc *q = quirc_new();
uint8_t *image = NULL;
struct quirc_code code;
struct quirc_data data;
char last_qrcode_data[8896];

static void dumpData(const struct quirc_data *data) {
  Serial.printf("Payload: %s\n", data->payload);
  if (strcmp(last_qrcode_data, (char *)data->payload) == 0) {
    Serial.println("new payload is the same as the old, skipping");
    return;
  }

  memcpy(&last_qrcode_data, data->payload, data->payload_len);
  if (mqttClient.connected()) {
    bool res = mqttClient.publish(SCANNED_PUBLISH_TOPIC, (char *)data->payload);
    if (res) {
      Serial.println("qr code payload published");
    } else {
      Serial.println("could not publish qr code payload");
    }
  } else {
    Serial.println("mqtt not connected");
  }
}

void QRCodeReader(void *pvParameters) {
  Serial.println("QRCodeReader is ready\n");

  struct quirc *_q = NULL;
  uint8_t *_image = NULL;
  struct quirc_code _code;
  struct quirc_data _data;

  while (true) {
    _q = quirc_new();
    if (_q == NULL) {
      Serial.print("can't create quirc object\r\n");
      continue;
    }

    uint8_t *buffer = cam.getfb();
    int width = cam.getWidth();
    int height = cam.getHeight();
    size_t size = cam.getSize();

    quirc_resize(_q, width, height);
    _image = quirc_begin(_q, NULL, NULL);
    memcpy(_image, buffer, size);
    quirc_end(_q);

    int count = quirc_count(_q);
    if (count > 0) {
      quirc_extract(_q, 0, &_code);
      quirc_decode_error_t err = quirc_decode(&_code, &_data);

      if (err) {
        Serial.println("Decoding FAILED\n");
      } else {
        dumpData(&_data);
      }
    }

    quirc_destroy(_q);
    _image = NULL;
  }
}

void createTaskQRCodeReader() {
  xTaskCreatePinnedToCore(QRCodeReader, "QRCodeReader_Task", 20000, NULL, tskIDLE_PRIORITY, &QRCodeReader_Task, 0);
}

void try_qrcode_decode(uint8_t *buffer, int width, int height, int size) {
  q = quirc_new();
  if (q == NULL) {
    Serial.print("can't create quirc object\r\n");
    return;
  }

  quirc_resize(q, width, height);
  image = quirc_begin(q, NULL, NULL);
  memcpy(image, buffer, size);
  quirc_end(q);

  int count = quirc_count(q);
  if (count > 0) {
    quirc_extract(q, 0, &code);
    quirc_decode_error_t err = quirc_decode(&code, &data);

    if (err) {
      Serial.println("Decoding FAILED\n");
    } else {
      dumpData(&data);
    }
  }

  quirc_destroy(q);
  image = NULL;
}

void handle_jpg_stream(void) {
  Serial.println("Stream start");

  if (QRCodeReader_Task != NULL) {
    Serial.println("Deleting qrcode reader task");
    vTaskDelete(QRCodeReader_Task);
  }

  WiFiClient client = server.client();
  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  server.sendContent(response);

  response = "--frame\r\n";
  response += "Content-Type: image/jpeg\r\n\r\n";

  uint8_t frames = 0;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;

  uint8_t *buffer = NULL;
  int width = 0;
  int height = 0;
  size_t size = 0;

  bool jpeg_converted = false;

  while (1) {
    frames++;

    cam.run();
    if (!client.connected()) {
      break;
    }
    server.sendContent(response);

    buffer = cam.getfb();
    width = cam.getWidth();
    height = cam.getHeight();
    size = cam.getSize();

    // perform qr code decode every 5 frames
    if (frames == 5) {
      try_qrcode_decode(buffer, width, height, size);
      frames = 0;
    }
    jpeg_converted = frame2jpg(cam.getCameraFb(), 80, &_jpg_buf, &_jpg_buf_len);

    client.write((char *)_jpg_buf, _jpg_buf_len);
    server.sendContent("\r\n");
    free(_jpg_buf);

    if (!client.connected()) {
      break;
    }
  }

  Serial.println("Stream end");

  createTaskQRCodeReader();
}

void handleNotFound() {
  String message = "Server is running!\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  server.send(200, "text/plain", message);
}

void on_mqtt_message_received(char *topic, byte *payload, unsigned int length) {
  ESP_LOGD(TAG, "Message arrived in topic: %s\nMessage:", topic);
  for (int i = 0; i < length; i++) {
    ESP_LOGD(TAG, "%c", (char)payload[i]);
  }
  ESP_LOGD(TAG, "\n");
}

void mqtt_reconnect() {
  while (!mqttClient.connected()) {
    ESP_LOGD(TAG, "Attempting MQTT connection");
    if (mqttClient.connect("esp32-color-sensor")) {
      ESP_LOGD(TAG, "MQTT connection established");
      mqttClient.subscribe("sm_iot_lab");
    } else {
      ESP_LOGE("MAIN", "MQTT connection failed, status=%d\nTry again in %d seconds", mqttClient.state(),
               WAIT_TIME_BEFORE_CONNECTION_RETRY);
      delay(WAIT_TIME_BEFORE_CONNECTION_RETRY);
    }
  }
}

void setup() {
  // Disable brownout detector.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.println("booting");

  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  cam.init(esp32cam_aithinker_config);

  IPAddress ip;

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  ip = WiFi.localIP();
  Serial.println(F("WiFi connected"));
  Serial.println("");
  Serial.println(ip);

  createTaskQRCodeReader();

  server.on("/", HTTP_GET, handle_jpg_stream);
  server.onNotFound(handleNotFound);
  server.begin();

  mqttClient.setServer(BROKER_IP, BROKER_PORT);
  mqttClient.setCallback(on_mqtt_message_received);
}

void loop() {
  server.handleClient();

  if (!mqttClient.connected()) {
    mqtt_reconnect();
  }

  mqttClient.loop();
}

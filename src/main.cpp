#include "OV2640.h"
#include "esp_log.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiClient.h>

#include "quirc/quirc.h"
#include "soc/rtc_cntl_reg.h"

#include "wifikeys.h"

#define TAG                               "MAIN"

#define WAIT_TIME_BEFORE_CONNECTION_RETRY 5000

#define SCANNER_N                         "0"
#define PICKUP_POINT_N                    "0"
#define PICKUP_POINT_N_INT                0
#define PICKUP_POINT_PUBLISH_BASE         "sm_iot_lab/cube_scanner"
#define CUBE_SCANNED_PUBLISH              "cube/scanned"
#define IP_PUBLISH                        "ip/post"
#define SCANNED_PUBLISH_TOPIC             PICKUP_POINT_PUBLISH_BASE "/" PICKUP_POINT_N "/" CUBE_SCANNED_PUBLISH
#define POST_IP_PUBLISH_TOPIC             PICKUP_POINT_PUBLISH_BASE "/" PICKUP_POINT_N "/" IP_PUBLISH

static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<html><head><title></title><meta name="viewport" content="width=device-width, initial-scale=1"><style>body{margin:auto;}img{position:relative;width:384px;height:288px;}#overlay{position:absolute;top:24.31%;left:29.48%;width:40%;height:50%;border:dashed red 2px;}#container{position:absolute;margin-left:calc(50% - 192px);margin-top:10px;}</style></head><body><div id="container"><img src="" id="vdstream"><div id="overlay"></div></div><script>window.onload=document.getElementById("vdstream").src=window.location.href.slice(0, -1) + ":80/stream";</script></body></html>
)rawliteral";

static OV2640 cam;

WebServer server(80);

WiFiClient client;
PubSubClient mqttClient(client);

struct QRCodeData {
  bool valid;
  int dataType;
  uint8_t payload[1024];
  int payloadLen;
};
struct quirc* q = quirc_new();
uint8_t* image = NULL;
struct quirc_code code;
struct quirc_data data;
char last_qrcode_data[8896];

StaticJsonDocument<200> doc;

static void dumpData(const struct quirc_data* data) {
  ESP_LOGD(TAG, "Payload: %s\n", data->payload);
  if (strcmp(last_qrcode_data, (char*)data->payload) == 0) {
    ESP_LOGD(TAG, "new payload is the same as the old, skipping");
    return;
  }

  // if (mqttClient.connected()) {
  StaticJsonDocument<200> qrCodeDoc;
  qrCodeDoc["pickupPointN"] = PICKUP_POINT_N_INT;
  qrCodeDoc["payload"] = (char*)data->payload;
  size_t doc_size = measureJson(qrCodeDoc);
  uint8_t* output = (uint8_t*)malloc(doc_size);

  serializeJson(qrCodeDoc, (void*)output, doc_size);
  bool res = mqttClient.publish(SCANNED_PUBLISH_TOPIC, output, doc_size);

  free(output);
  if (res) {
    ESP_LOGD(TAG, "qr code payload published");
    memcpy(&last_qrcode_data, data->payload, data->payload_len);
    // } else {
    //   ESP_LOGD(TAG, "could not publish qr code payload");
    // }
  } else {
    ESP_LOGD(TAG, "qrcode payload NOT published");
  }
}

void try_qrcode_decode(uint8_t* buffer, int width, int height, int size) {
  q = quirc_new();
  if (q == NULL) {
    ESP_LOGD(TAG, "can't create quirc object\r\n");
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
      ESP_LOGD(TAG, "Decoding FAILED\n");
    } else {
      dumpData(&data);
    }
  }

  quirc_destroy(q);
  image = NULL;
}

void handle_index(void) { server.send(200, "text/html", INDEX_HTML); }

void handle_jpg_stream(void) {
  ESP_LOGD(TAG, "Stream start");

  WiFiClient client = server.client();
  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  server.sendContent(response);

  response = "--frame\r\n";
  response += "Content-Type: image/jpeg\r\n\r\n";

  uint8_t frames = 0;
  size_t _jpg_buf_len = 0;
  uint8_t* _jpg_buf = NULL;

  uint8_t* buffer = NULL;
  int width = 0;
  int height = 0;
  size_t size = 0;

  bool jpeg_converted = false;

  while (1) {
    mqttClient.loop();
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

    client.write((char*)_jpg_buf, _jpg_buf_len);
    server.sendContent("\r\n");
    free(_jpg_buf);

    if (!client.connected()) {
      break;
    }
  }

  ESP_LOGD(TAG, "Stream end");
}

void on_mqtt_message_received(char* topic, byte* payload, unsigned int length) {
  ESP_LOGD(TAG, "Message arrived in topic: Message:");
  for (int i = 0; i < length; i++) {
    // ESP_LOGD(TAG, "%c", (char)payload[i]);
  }
  // ESP_LOGD(TAG, "\n");
}

void mqtt_connect() {
  while (!mqttClient.connected()) {
    ESP_LOGD(TAG, "Attempting MQTT connection");
    if (mqttClient.connect("cube-scanner-" SCANNER_N, "sm_iot_lab/scanner/" SCANNER_N "/status", 2, true, "down")) {
      ESP_LOGD(TAG, "MQTT connection established");

      doc["pickupPointN"] = PICKUP_POINT_N_INT;
      doc["ipAddress"] = WiFi.localIP().toString();
      size_t doc_size = measureJson(doc);
      uint8_t* output = (uint8_t*)malloc(doc_size);

      serializeJson(doc, (void*)output, doc_size);
      mqttClient.publish(POST_IP_PUBLISH_TOPIC, output, doc_size);
      mqttClient.publish("sm_iot_lab/scanner/" SCANNER_N "/status", "up", true);
      // mqttClient.publish("sm_iot_lab/scanner/0/status", "up", true);

      free(output);
    } else {
      ESP_LOGD(TAG, "MQTT connection failed, status=Try again in seconds");
      delay(WAIT_TIME_BEFORE_CONNECTION_RETRY);
    }
  }
}

void setup() {
  // Disable brownout detector.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  ESP_LOGD(TAG, "booting");

  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  cam.init(esp32cam_aithinker_config);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  server.on("/stream", HTTP_GET, handle_jpg_stream);
  server.on("/", HTTP_GET, handle_index);
  server.begin();

  mqttClient.setServer(BROKER_IP, BROKER_PORT);
  mqttClient.setCallback(on_mqtt_message_received);
}

void loop() {
  server.handleClient();

  if (!mqttClient.connected()) {
    mqtt_connect();
  }

  mqttClient.loop();
}

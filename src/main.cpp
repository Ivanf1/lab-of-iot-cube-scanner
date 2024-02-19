#include "OV2640.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiClient.h>

#include "quirc/quirc.h"
#include "soc/rtc_cntl_reg.h"

#include "wifikeys.h"

#define WAIT_TIME_BEFORE_CONNECTION_RETRY 5000

#define PICKUP_POINT_N "1"
#define PICKUP_POINT_N_INT 1
#define PICKUP_POINT_PUBLISH_BASE "sm_iot_lab/cube_scanner"
#define CUBE_SCANNED_PUBLISH "cube/scanned"
#define IP_PUBLISH "ip/post"
#define SCANNED_PUBLISH_TOPIC PICKUP_POINT_PUBLISH_BASE "/" PICKUP_POINT_N "/" CUBE_SCANNED_PUBLISH
#define POST_IP_PUBLISH_TOPIC PICKUP_POINT_PUBLISH_BASE "/" PICKUP_POINT_N "/" IP_PUBLISH

static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<html><head><title></title><meta name="viewport" content="width=device-width, initial-scale=1">
<style>body{margin:auto;}
img{position:relative;width:384px;height:288px;}
#overlay{position:absolute;top:24.31%;left:29.48%;width:40%;height:50%;border:dashed red 2px;}
#container{position:absolute;margin-left:calc(50% - 192px);margin-top:10px;}
</style></head><body><div id="container"><img src="" id="vdstream"><div id="overlay"></div></div><script>window.onload=document.getElementById("vdstream").src=window.location.href.slice(0, -1) + ":80/stream";</script></body></html>
)rawliteral";

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

StaticJsonDocument<200> doc;

static void dumpData(const struct quirc_data *data) {
  Serial.printf("Payload: %s\n", data->payload);
  if (strcmp(last_qrcode_data, (char *)data->payload) == 0) {
    Serial.println("new payload is the same as the old, skipping");
    return;
  }

  if (mqttClient.connected()) {
    bool res = mqttClient.publish(SCANNED_PUBLISH_TOPIC, (char *)data->payload);
    if (res) {
      Serial.println("qr code payload published");
      memcpy(&last_qrcode_data, data->payload, data->payload_len);
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

    // quirc_resize(_q, 280, 200);
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

void handle_index(void) { server.send(200, "text/html", INDEX_HTML); }

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
      // try_qrcode_decode(buffer, 280, 200, size);
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
  Serial.println("Message arrived in topic: Message:");
  for (int i = 0; i < length; i++) {
    // ESP_LOGD(TAG, "%c", (char)payload[i]);
  }
  // ESP_LOGD(TAG, "\n");
}

void mqtt_reconnect() {
  while (!mqttClient.connected()) {
    Serial.println("Attempting MQTT connection");
    if (mqttClient.connect("cube-scanner-" PICKUP_POINT_N)) {
      Serial.println("MQTT connection established");
      // mqttClient.subscribe("sm_iot_lab");

      doc["pickupPointN"] = PICKUP_POINT_N_INT;
      doc["ipAddress"] = WiFi.localIP().toString();
      size_t doc_size = measureJson(doc);
      uint8_t *output = (uint8_t *)malloc(doc_size);

      serializeJson(doc, (void *)output, doc_size);
      mqttClient.publish(POST_IP_PUBLISH_TOPIC, output, doc_size);

      free(output);
    } else {
      Serial.println("MQTT connection failed, status=Try again in seconds");
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

  server.on("/stream", HTTP_GET, handle_jpg_stream);
  server.on("/", HTTP_GET, handle_index);
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

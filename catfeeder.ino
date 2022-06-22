//Setup for an ESP32-CAM to run both a video streaming server and take MQTT messages to move a motor.
//This moves a 28byj 5V stepper the amount sent in the payload that many steps
//This is designed to work with NodeRed/HA or similar,
//where the actual distances to travel and interpretting the camera feed can be done

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h" //disable brownout problems
#include "soc/rtc_cntl_reg.h"  //disable brownout problems
#include "esp_http_server.h"
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <AccelStepper.h>

const char* ssid = "***";
const char* password =  "***";
const char* mqttServer = "192.168.1.**";
const int mqttPort = ***;
int holding_var = 0;
int motorspeedset = 0;
int motortravelset = 0;
int motor_state = 0;
String messageTemp;
//heartbeat vars - every 5mins
unsigned long prev_time = 0;
unsigned long HBinterval = 300000;

WiFiClient espClient;
PubSubClient client(espClient);
const int stepsPerRevolution = 2048;
#define PART_BOUNDARY "123456789000000000000987654321"

// ULN2003 Motor Driver Pins
//IN1 14
//IN2 15
//IN3 13
//IN4 12
AccelStepper stepper (AccelStepper::FULL4WIRE, 14, 13, 15, 12);


#define CAMERA_MODEL_AI_THINKER
  #define PWDN_GPIO_NUM     32
  #define RESET_GPIO_NUM    -1
  #define XCLK_GPIO_NUM      0
  #define SIOD_GPIO_NUM     26
  #define SIOC_GPIO_NUM     27

  #define Y9_GPIO_NUM       35
  #define Y8_GPIO_NUM       34
  #define Y7_GPIO_NUM       39
  #define Y6_GPIO_NUM       36
  #define Y5_GPIO_NUM       21
  #define Y4_GPIO_NUM       19
  #define Y3_GPIO_NUM       18
  #define Y2_GPIO_NUM        5
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     23
  #define PCLK_GPIO_NUM     22


static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;

static esp_err_t stream_handler(httpd_req_t *req){
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK){
    return res;
  }

  while(true){
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      if(fb->width > 400){
        if(fb->format != PIXFORMAT_JPEG){
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if(!jpeg_converted){
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }
    if(res == ESP_OK){
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if(fb){
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if(_jpg_buf){
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if(res != ESP_OK){
      break;
    }
  }
  return res;
}

void startCameraServer(){
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &index_uri);
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  Serial.begin(115200);
  stepper.setMaxSpeed(100);
stepper.setAcceleration(20);

  Serial.setDebugOutput(false);
  pinMode (4, OUTPUT);//led
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // Camera init
  esp_err_t err = esp_camera_init(&config);
   // Wi-Fi connection
 WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
   WiFi.hostname("ESP_catfeed");
client.setServer(mqttServer, mqttPort);
client.setCallback(callback);
 while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
 // Create a random client ID
    String clientId = "ESP_catfeed_";
    clientId += String(random(0xffff), HEX);
if (client.connect(clientId.c_str())) {
Serial.println("connected");
 } else {
 Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
      }
  }

  //OTA stuff --
ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
  Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
   //end of OTA stuff ---
 client.subscribe("esp/ESP_catfeed");
  // Start streaming web server
  startCameraServer();
}

void loop() {
ArduinoOTA.handle();
  if (!client.connected()) {
    reconnect();
  }
  //heartbeat and wifi reconnect
   if(millis() > prev_time + HBinterval){
      client.publish("esp/hbeat","catfeed");
      if (WiFi.status() != WL_CONNECTED){
            Serial.println("Connecting to WiFi..");
        WiFi.begin(ssid, password);
      }
    prev_time = millis();
   }
   client.loop();
  delay(1);
}

//end of loop

//Functions ==================================

//reconnect function for MQTT Dropout
void reconnect() {
  Serial.println("Reconnect activated");
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");


// Create a random client ID
    String clientId = "ESP_catfeed_";
    clientId += String(random(0xffff), HEX);
if (client.connect(clientId.c_str())) {
      Serial.println("connected");
       delay(1000);
        client.subscribe("esp/ESP_catfeed");
        client.publish("esp/test", "Hello from ESP_catfeed(recon)");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

//Callback function for recieving messages ------>
void callback(char* topic, byte* payload, unsigned int length) {
 //clear messageTemp
  messageTemp = "";
  //get the message, write to messageTemp char array
  for (int i = 0; i < length; i++) {
    messageTemp += (char)payload[i];
  }
//if number starts with a 0, then it's a speed
if (messageTemp.startsWith("0")){
  motorspeedset = messageTemp.toInt();
  stepper.setMaxSpeed(motorspeedset);
  stepper.setSpeed(motorspeedset);


}
//else if (messageTemp.startsWith("9")){
//  motorspeedset = messageTemp.toInt();
//  stepper.setAcceleration(messageTemp.toInt());

//}
else {

stepper.runToNewPosition(messageTemp.toInt());
}
 }



//end of callback ----------------------------------------->

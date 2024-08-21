#include "esp_camera.h"
#include <WiFi.h>
#include <WebSocketsClient.h>

#define CAMERA_MODEL_AI_THINKER 
#include "camera_pins.h"

#define IN1 4
#define IN2 14
#define IN3 15
#define IN4 13
#define ENA 2
#define ENB 12

#define FORWARD 1
#define BACKWARD 2
#define LEFT 6
#define RIGHT 7
#define STOP 5

#define MOTOR_SPEED 110
byte currentCommand = 0b00000000;
unsigned long previousMillis = 0;
const long interval = 110; 

const char *ssid = "";
const char *password = "";
const char *websocket_server = "";
const uint16_t websocket_port = ;

WebSocketsClient webSocket;
int speed = 0;

esp_err_t cameraPinSetup(){
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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_QVGA;
  config.pixel_format = PIXFORMAT_JPEG;  
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 20;
  config.fb_count = 1;

  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.frame_size = FRAMESIZE_VGA;
      config.jpeg_quality = 18;
      config.fb_count = 1;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
  }
  return err;
}

void cameraSensorSetup(){
  sensor_t *s = esp_camera_sensor_get();
  if (s->id.PID == OV2640_PID) {
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
    s->set_brightness(s, 1);   
    s->set_saturation(s, -2);
  }
}

void motorSetup(int in1, int in2, int in3, int in4, int ena, int enb){
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT); 
  pinMode(in4, OUTPUT);
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);  
  analogWrite(ena, 0);
  analogWrite(enb, 0);  
}

void setMotorDirection(bool in1, bool in2, bool in3, bool in4){
  digitalWrite(IN1, in1);
  digitalWrite(IN2, in2);
  digitalWrite(IN3, in3);
  digitalWrite(IN4, in4);
}

void setMotorSpeed(int speedA, int speedB){
  analogWrite(ENA, speedA);
  analogWrite(ENB, speedB);
}

void webSocketEventCommand(WStype_t type, uint8_t* payload, size_t length){
  switch(type){
    case WStype_DISCONNECTED:
      Serial.println("WebSocket Command Disconnected");
      stopMoving();
      break;
    case WStype_CONNECTED:
      Serial.println("WebSocket Command Connected");
      stopMoving();
      break;
    case WStype_TEXT:
      Serial.printf("Received Text Command: %s\n", (char*)payload);
      break;
    case WStype_BIN:
      if (length > 0) {
        byte command = payload[0];
        Serial.printf("Received Binary Command: %08b\n", command); 
        handleServerCommand(command);
      }
      break;
  }
}

void handleServerCommand(byte command){
  if(currentCommand == command){
    Serial.println("Excluding The Same Command");
    return;
  }
  currentCommand = command;
  int cmd = int(command >> 5);
  speed = (command & 0b00011111) * 4;
  
  if(cmd == FORWARD){
    moveForward();
  }
  else if(cmd == BACKWARD){
    moveBackward();
  } 
  else if(cmd == LEFT){
    moveLeft(speed);
  }     
  else if(cmd == RIGHT){
    moveRight(speed);
  }    
  else{
    stopMoving();
  }
}

void moveForward(){
  setMotorDirection(HIGH, LOW, LOW, HIGH);
  setMotorSpeed(MOTOR_SPEED, MOTOR_SPEED);
}
void moveBackward() {
  setMotorDirection(LOW, HIGH, HIGH, LOW);
  setMotorSpeed(MOTOR_SPEED, MOTOR_SPEED);
}
void moveLeft(int speed) {
  setMotorDirection(HIGH, LOW, LOW, HIGH);
  setMotorSpeed(MOTOR_SPEED + speed, 80);
}
void moveRight(int speed) {
  setMotorDirection(HIGH, LOW, LOW, HIGH);
  setMotorSpeed(80, MOTOR_SPEED + speed);
}
void stopMoving() {
  setMotorSpeed(0, 0);
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("WiFi Connected");
}

void connectToWebSocket() {
  Serial.print("Connecting to WebSocket...");
  webSocket.begin(websocket_server, websocket_port, "/ws/esp32");
  webSocket.onEvent(webSocketEventCommand);
}

void setup() {
  Serial.begin(115200);

  if(cameraPinSetup() != ESP_OK) {
    Serial.println("CameraPinsetup failed");
    return;
  }
  cameraSensorSetup();
  motorSetup(IN1, IN2, IN3, IN4, ENA, ENB);

  connectToWiFi();
  connectToWebSocket();
  
  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

void loop() {
  webSocket.loop();

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (webSocket.isConnected()) {
      camera_fb_t * fb = esp_camera_fb_get();
      if (fb) {
        if (fb->format != PIXFORMAT_JPEG) {
          size_t jpeg_len = 0;
          uint8_t *jpeg_buf = NULL;
          bool jpeg_converted = frame2jpg(fb, 80, &jpeg_buf, &jpeg_len);
          esp_camera_fb_return(fb);
          if (jpeg_converted) {
            webSocket.sendBIN(jpeg_buf, jpeg_len);
            free(jpeg_buf);
          } else {
            Serial.println("JPEG compression failed");
          }
        } else {
          webSocket.sendBIN(fb->buf, fb->len);
          esp_camera_fb_return(fb);
        }
      } else {
        Serial.println("Camera capture failed");
      }
    } else {
      Serial.println("WebSocket not connected");
    }
  }
}


#include "esp_camera.h"
#include <WiFi.h>

#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#include <MPU9250_asukiaaa.h>
#include <Wire.h>
//#include <Adafruit_Sensor.h>
#include <MadgwickAHRS.h>

// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//
//            You must select partition scheme from the board menu that has at least 3MB APP space.
//            Face Recognition is DISABLED for ESP32 and ESP32-S2, because it takes up from 15
//            seconds to process single frame. Face Detection is ENABLED if PSRAM is enabled as well

// ===================
// Select camera model
// ===================
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
//#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_UNITCAM // No PSRAM
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM
// ** Espressif Internal Boards **
//#define CAMERA_MODEL_ESP32_CAM_BOARD
//#define CAMERA_MODEL_ESP32S2_CAM_BOARD
//#define CAMERA_MODEL_ESP32S3_CAM_LCD

#include "camera_pins.h"

// ===========================
// Enter your WiFi credentials
// ===========================
const char* ssid = "TP-Link_87A4";
const char* password = "59110323";

#define MQTT_SERVER      "192.168.0.222" // static ip address
#define MQTT_PORT        1885
#define MQTT_USERNAME    ""
#define MQTT_PASSWORD    ""

#define I2C_SDA 14
#define I2C_SCL 15

MPU9250_asukiaaa Sensor;
int status;
Madgwick filter;

WiFiClient client;
// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_PORT, MQTT_USERNAME, MQTT_PASSWORD);

Adafruit_MQTT_Publish esp32pub = Adafruit_MQTT_Publish(&mqtt, MQTT_USERNAME "esp32/out");
Adafruit_MQTT_Subscribe esp32sub = Adafruit_MQTT_Subscribe(&mqtt, MQTT_USERNAME "roach0");

void MQTT_connect();

void startCameraServer();

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

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
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }
#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }
#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif
#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  startCameraServer();
  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  mqtt.subscribe(&esp32sub);

  status = Wire.begin(I2C_SDA, I2C_SCL);
  Sensor.setWire(&Wire);
  Sensor.beginGyro(GYRO_FULL_SCALE_250_DPS);
  //  Sensor.beginGyro();
  Sensor.beginAccel(ACC_FULL_SCALE_2_G);
  //  Sensor.beginAccel();
  Sensor.beginMag();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
  filter.begin(100);
}

float gX, gY, gZ;
float accx, accy, accz;
float magx, magy, magz;
float gxbias, gybias, gzbias;
float axbias, aybias, azbias;
float roll, pitch, yaw;
float cfac = 0.1;//calibration factor
float prevTime;//previous time
int i = 0, zero_calib_flag = 0;

void loop() {
  // Do nothing. Everything is done in another task by the web server
  MQTT_connect();
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription())) {
    if (subscription == &esp32sub) {
      char *message = (char *)esp32sub.lastread;
      Serial.print(F("Got: "));
      Serial.println(message);
      delay(10);
      if (strncmp(message, "F", 1) == 0) {
        Serial.println("Forward");
      }
      else if (strncmp(message, "L", 1) == 0) {
        Serial.println("Left");
      }
      else if (strncmp(message, "R", 1) == 0) {
        Serial.println("Right");
      }
      else if (strncmp(message, "S", 1) == 0) {
        Serial.println("Stop");
      }
    }
  }
  Sensor.accelUpdate();
  accx = Sensor.accelX();
  accy = Sensor.accelY();
  accz = Sensor.accelZ();
  Sensor.gyroUpdate();
  gX = Sensor.gyroX();
  gY = Sensor.gyroY();
  gZ = Sensor.gyroZ();
  Sensor.magUpdate();
  magx = Sensor.magX();
  magy = Sensor.magY();
  magz = Sensor.magZ();
  float currentTime = millis();
  float dt = (prevTime - currentTime) / 1000;
  prevTime = currentTime;
  //-----------Median filter: 0bias calib----------//
  if (zero_calib_flag)
  {
    if (i < 200)
    {
      gxbias += gX;
      gybias += gY;
      gzbias += gZ;
      axbias += accx;
      aybias += accy;
      azbias += accz;
      i++;
    }
    else
    {
      gxbias /= 200;
      gybias /= 200;
      gzbias /= 200;
      axbias /= 200;
      aybias /= 200;
      azbias /= 200;
      Serial.print("gxbias: "); Serial.print(gxbias);
      Serial.print("  gybias: "); Serial.print(gybias);
      Serial.print("  gzbias: "); Serial.println(gzbias);
      Serial.print("axbias: "); Serial.print(axbias);
      Serial.print("  aybias: "); Serial.print(aybias);
      Serial.print("  azbias: "); Serial.println(azbias);
      zero_calib_flag = 0;
    }
  }
  else
  {
    //-------------------------Madgwick filter------------------------------
    //    filter.updateIMU(gX - gxbias, gY - gybias, gZ - gzbias, accx, accy, accz);
    filter.updateIMU(gX, gY, gZ, accx, accy, accz);
    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    yaw = filter.getYaw();
    //    Serial.print("gX: "); Serial.print(gX);
    //    Serial.print("  gY: "); Serial.print(gY);
    //    Serial.print("  gZ: "); Serial.println(gZ);
    //    Serial.print("accx: "); Serial.print(accx);
    //    Serial.print("  accy: "); Serial.print(accy);
    //    Serial.print("  accz: "); Serial.println(accz);

    Serial.print(roll, 6);
    Serial.print("\t");
    Serial.print(pitch, 6);
    Serial.print("\t");
    Serial.println(yaw, 6);


   String sensorMessage = String(roll) + "R" + String(pitch) + "P" + String(yaw) + "Y";
   esp32pub.publish(sensorMessage.c_str());
  }
  delay(10);
}

void calib(float *base, float *target) { //calibration
  if (*target - *base > 1)*base += TWO_PI;
  if (*base - *target > 1)*base -= TWO_PI;
  *base = (1 - cfac)**base + cfac**target;
}

void MQTT_connect() {
  int8_t ret;
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
  Serial.print("Connecting to MQTT... ");
  uint8_t retries = 5;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      Serial.println("MQTT Connection Failed!!!");
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
}

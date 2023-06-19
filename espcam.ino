#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include <Wire.h>               
//#include "SSD1306Wire.h" 
//SSD1306Wire display(0x3c, 13, 15);

camera_fb_t *fb = NULL;
uint8_t *rgb_buffer;
framesize_t frameSize = FRAMESIZE_QQVGA;
pixformat_t pixFormat = PIXFORMAT_JPEG;

unsigned long lastTime, dt;
double integral_sum = 0.0;
int error_ant = 0;

const int ENA = 14;
const int IN1 = 2;
const int IN2 = 4;

const int IN3 = 15;
const int IN4 = 13;
const int ENB = 12;

const int freq = 1000; //1kHz
const int pwm1 = 3; //Channel: 3, Timer: 1
const int pwm2 = 4; //Channel: 4, Timer: 2
const int res = 8; //0-255

bool runner;

void setup() {
  //HABILITA LA CAMARA CON BROWN_OUT
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  //HABILTA MONITOR PARA DEBUG
  Serial.begin(115200);

  runner = false;

  //CONFIRACION DE LA CAMARA
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = 5;
  config.pin_d1 = 18;
  config.pin_d2 = 19;
  config.pin_d3 = 21;
  config.pin_d4 = 36;
  config.pin_d5 = 39;
  config.pin_d6 = 34;
  config.pin_d7 = 35;
  config.pin_xclk = 0;
  config.pin_pclk = 22;
  config.pin_vsync = 25;
  config.pin_href = 23;
  config.pin_sscb_sda = 26;
  config.pin_sscb_scl = 27;
  config.pin_pwdn = 32;
  config.pin_reset = -1;
  config.xclk_freq_hz = 10000000; //5-20
  config.pixel_format = pixFormat;   
  config.frame_size = frameSize;     
  config.jpeg_quality = 10; //de 0 a 63          
  config.fb_count = 2; //solo para jpeg 
  bool ok = esp_camera_init(&config) == ESP_OK;
  sensor_t *sensor = esp_camera_sensor_get();
  sensor->set_framesize(sensor, frameSize);
  sensor->set_pixformat(sensor, pixFormat);
  
  //INICIA PANTALLA OLED
  //display.init();

  // RUEDA IZQUIERDA
  pinMode(IN2, OUTPUT);
  digitalWrite(IN2, LOW);
  pinMode(IN1, OUTPUT);
  digitalWrite(IN1, HIGH);
  pinMode(ENA, OUTPUT);
  ledcSetup(pwm1, freq, res);
  ledcAttachPin(ENA, pwm1);

  //RUEDA DERECHA
  pinMode(IN3, OUTPUT);
  digitalWrite(IN3, HIGH);
  pinMode(IN4, OUTPUT);
  digitalWrite(IN4, LOW);
  pinMode(ENB, OUTPUT);
  ledcSetup(pwm2, freq, res);
  ledcAttachPin(ENB, pwm2);

  //OBTIENE EL TIEMPO EN MS
  lastTime = millis();
}

void loop() {
  fb = esp_camera_fb_get();
  //PROCESAMIENTO
  size_t rgb_len = fb->width * fb->height * 3; //160*120*3=57600
  rgb_buffer = (uint8_t *) malloc(rgb_len);
  fmt2rgb888(fb->buf, fb->len, fb->format, rgb_buffer);
  //....
  uint8_t posLine = getPosLine(rgb_buffer);
  //Serial.println(posLine);
  
  //TIEMPO DE MUESTREO O PROCESAMIENTO
  dt = millis()-lastTime;

  if(runner == true){
    int u = computePID(posLine, dt);
    ledcWrite(pwm1, 80-u);
    ledcWrite(pwm2, 80+u);
  } else {
    ledcWrite(pwm1, 0);
    ledcWrite(pwm2, 0);
  }
  

  esp_camera_fb_return(fb);
  free(rgb_buffer);
  
  //Serial.println(dt);
  lastTime = millis();
}

int computePID(uint8_t xline, unsigned long delta_time){
  const int setpoint = 80;  
  const float KP = 0.55;
  const float KI = 0.1;
  const float KD = 0.2;

  int error = setpoint - xline;

  int P = KP * error;

  integral_sum += error * (delta_time/1000.0);
  int I = KI * integral_sum;

  double derivada_err = (error-error_ant)/(delta_time/1000.0);
  error_ant = error;
  int D = KD * derivada_err;

  return P+I+D;
}
uint8_t getPosLine(uint8_t *imageIn){
  size_t start = 28800; // fila 60
  size_t fin = 29280; //fila 60
  uint8_t gray_arr[160];// imagen a procesar
  uint8_t column_gray=0;//aux
  
  for(size_t p=start; p<fin; p+=3){
    uint8_t r = imageIn[p+2];
    uint8_t g = imageIn[p+1];
    uint8_t b = imageIn[p];
    gray_arr[column_gray] = 0.3*r + 0.59*g + 0.11*b;
    column_gray++;
  }
  int diff_arr[159];
  
  int max_d = 0;
  int min_d = 0;
  uint8_t X1 = 0;
  uint8_t X2 = 0;
  uint8_t XL = 0;

  for(size_t p=0; p<159; p+=1){
    int diff = gray_arr[p+1]-gray_arr[p];
    diff_arr[p] = diff;
    //Serial.println(diff_arr[p]);
    if(max_d < diff){
      max_d = diff;
      X2 = p;
    }
    if(diff < min_d){
      min_d = diff;
      X1 = p;
    }
  }
  
  XL = (X1+X2)/2;
  
  int mag = max_d - min_d;

  if(mag>35){
    runner = true;
  } else {
    runner = false;
  }
  
  return XL;
}
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

/*
FORMATO DE IMAGEN:
    PIXFORMAT_YUV422
    PIXFORMAT_GRAYSCALE
    PIXFORMAT_RGB565
    PIXFORMAT_JPEG

RESOLUCION DE LA IMAGEN:
    FRAMESIZE_QQVGA,    // 160x120
    FRAMESIZE_QCIF,     // 176x144
    FRAMESIZE_HQVGA,    // 240x176
    FRAMESIZE_QVGA,     // 320x240
    FRAMESIZE_CIF,      // 400x296
    FRAMESIZE_VGA,      // 640x480
    FRAMESIZE_SVGA,     // 800x600
    FRAMESIZE_XGA,      // 1024x768
    FRAMESIZE_SXGA,     // 1280x1024
    FRAMESIZE_UXGA,     // 1600x1200
*/

camera_fb_t *fb = NULL;
uint8_t *rgb_buffer;
framesize_t frameSize = FRAMESIZE_QQVGA;
pixformat_t pixFormat = PIXFORMAT_JPEG;

unsigned long lastTime, dt;

void setup() {
  //HABILITA LA CAMARA CON BROWN_OUT
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  //HABILTA MONITOR PARA DEBUG
  Serial.begin(115200);

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
  config.xclk_freq_hz = 5000000; //5-20
  config.pixel_format = pixFormat;   
  config.frame_size = frameSize;     
  config.jpeg_quality = 7; //de 0 a 63          
  config.fb_count = 2; //solo para jpeg 
  bool ok = esp_camera_init(&config) == ESP_OK;
  sensor_t *sensor = esp_camera_sensor_get();
  sensor->set_framesize(sensor, frameSize);
  sensor->set_pixformat(sensor, pixFormat);
  
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
  Serial.println("Posicion X del centro: " + posLine);

  esp_camera_fb_return(fb);
  free(rgb_buffer);
  
  //TIEMPO DE MUESTREO O PROCESAMIENTO
  dt = millis()-lastTime;
  Serial.println(dt);
  lastTime = millis();
}

uint8_t getPosLine(uint8_t *imageIn){
  size_t start = 28800;
  size_t end = 29280;
  uint8_t greyArray[160];
  uint8_t greyColumn = 0;

  for(size_t p=start; p<end; p+=3){ //p means "pixel"
    uint8_t r = imageIn[p+2];
    uint8_t g = imageIn[p+1];
    uint8_t b = imageIn[p];

    greyArray[greyColumn] = 0.3*r + 0.59*g + 0.11*b;
    greyColumn ++;
  }

  int diffArray[159];
  int max_d = 0;
  int min_d = 0;
  uint8_t X1 = 0;
  uint8_t X2 = 0;
  uint8_t Xcentro = 0;

  for(size_t p=0; p<159; p+=1){
    int diff = greyArray[p+1] - greyArray[p];
    diffArray[p] = diff;

    if(max_d < diff){
      max_d = diff;
      X2 = p;
    }
    
    if(diff < min_d){
      min_d = diff;
      X1  = p;
    }
  }

  Xcentro = (X1+X2)/2;

  return Xcentro;
}
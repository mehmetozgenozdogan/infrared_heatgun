
#include "U8glib.h"

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NO_ACK); 

#include <Arduino.h>
#define SDA_PORT PORTD
#define SDA_PIN 3
#define SCL_PORT PORTD
#define SCL_PIN 5
#include <SoftI2CMaster.h>

#define temperature_width 18
#define temperature_height 47
static unsigned char temperature_bits[] U8G_PROGMEM = {
   0xc0, 0x0f, 0x00, 0xe0, 0x1f, 0x00, 0x70, 0x38, 0x00, 0x30, 0x30, 0x00,
   0x30, 0x30, 0x00, 0x30, 0x30, 0x00, 0x30, 0x30, 0x00, 0x30, 0x30, 0x00,
   0x30, 0x30, 0x00, 0xb0, 0x37, 0x00, 0xb0, 0x37, 0x00, 0x30, 0x30, 0x00,
   0xb0, 0x37, 0x00, 0xb0, 0x37, 0x00, 0x30, 0x30, 0x00, 0x30, 0x30, 0x00,
   0xb0, 0x37, 0x00, 0xb0, 0x37, 0x00, 0x30, 0x30, 0x00, 0xb0, 0x37, 0x00,
   0xb0, 0x37, 0x00, 0xb0, 0x37, 0x00, 0x30, 0x30, 0x00, 0xb0, 0x37, 0x00,
   0xb0, 0x37, 0x00, 0xb0, 0x37, 0x00, 0xb0, 0x37, 0x00, 0xb0, 0x37, 0x00,
   0xb0, 0x37, 0x00, 0xb0, 0x37, 0x00, 0x98, 0x67, 0x00, 0x8c, 0xc7, 0x00,
   0xc6, 0x8f, 0x01, 0xe2, 0x1f, 0x01, 0xf3, 0x3f, 0x03, 0xf3, 0x3f, 0x03,
   0xf3, 0x3f, 0x03, 0xf3, 0x3f, 0x03, 0xf3, 0x3f, 0x03, 0xf3, 0x3f, 0x03,
   0xf3, 0x3f, 0x03, 0xe6, 0x9f, 0x01, 0xc6, 0x8f, 0x01, 0x0c, 0xc0, 0x00,
   0x38, 0x70, 0x00, 0xf0, 0x3f, 0x00, 0xc0, 0x0f, 0x00 };




uint8_t draw_state = 0;


void setup(void) {
  pinMode(4, OUTPUT);
  pinMode(2, OUTPUT);
  // flip screen, if required
  // u8g.setRot180();

#if (__AVR_ARCH__  == 5) // means ATMEGA 
  Serial.begin(19200);
  Serial.println("Setup...");
#endif
  i2c_init();

}

void loop(void) {
    digitalWrite(2, LOW);
    digitalWrite(4, LOW);
    int dev = 0x5A<<1;
    int data_low = 0;
    int data_high = 0;
    int pec = 0;
    
    i2c_start(dev+I2C_WRITE);
    i2c_write(0x07);
    // read
    i2c_rep_start(dev+I2C_READ);
    data_low = i2c_read(false); //Read 1 byte and then send ack
    data_high = i2c_read(false); //Read 1 byte and then send ack
    pec = i2c_read(true);
    i2c_stop();
    
    //This converts high and low bytes together and processes temperature, MSB is a error bit and is ignored for temps
    double tempFactor = 0.02; // 0.02 degrees per LSB (measurement resolution of the MLX90614)
    double tempData = 0x0000; // zero out the data
    int frac; // data past the decimal point
    
    // This masks off the error bit of the high byte, then moves it left 8 bits and adds the low byte.
    tempData = (double)(((data_high & 0x007F) << 8) + data_low);
    tempData = (tempData * tempFactor)-0.01;
    
    float celcius = tempData - 273.15;
    float fahrenheit = (celcius*1.8) + 32;

#if (__AVR_ARCH__  == 5) // means ATMEGA 
    Serial.print("Celcius: ");
    Serial.println(celcius);
    u8g.firstPage();
    do {
      //draw();
      /*u8g.setFont(u8g_font_unifont);
      u8g.setPrintPos(0, 10);
      u8g.print("CORONA CHECK  ");*/

      if (celcius >= 30.5){
          digitalWrite(2, HIGH);
          digitalWrite(4, LOW);
          u8g.setFont(u8g_font_unifont);
          u8g.setPrintPos(0, 10);
          u8g.print("MEMONET");
          u8g.setPrintPos(17, 64);
          u8g.print(" COVID-19 POS+");
        }
      if (celcius < 30.5){
          digitalWrite(2, LOW);
          digitalWrite(4, HIGH);
          u8g.setFont(u8g_font_unifont);
          u8g.setPrintPos(0, 10);
          u8g.print("MEMONET");
          u8g.setPrintPos(17, 64);
          u8g.print(" COVID-19 NEG-");
        }
      //u8g.print("CORONA CHECK  ");
    
      u8g.setFont(u8g_font_fub25);
    
      u8g.setPrintPos(26, 45);
    
    
      Serial.println(celcius);
      u8g.print(String(celcius,1) + " C");
    
      // Celcius symbol
      u8g.drawCircle(100, 23, 3);
    
      // Thermometer icon
      u8g.drawXBMP(0, 16, temperature_width, temperature_height, temperature_bits);
  
    } while( u8g.nextPage() );
  
     // increase the state
    draw_state++;
  
    // Used if instead of mod operator to avoid running out of int size
    if ( draw_state > 2  )
      draw_state = 0;

#endif
    delay(1000); // wait a second before printing again

}

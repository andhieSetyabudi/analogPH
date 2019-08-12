#include "ADS1112.h"


void ADS1112::init(TwoWire &lineCom, byte A0, byte A1)
{
  if (A0 == L && A1 == L) I2C_ADDRESS = B1001000;
  else if (A0 == L && A1 == F) I2C_ADDRESS = B1001001;
  else if (A0 == L && A1 == H) I2C_ADDRESS = B1001010;
  else if (A0 == H && A1 == L) I2C_ADDRESS = B1001100;
  else if (A0 == H && A1 == F) I2C_ADDRESS = B1001101;
  else if (A0 == H && A1 == H) I2C_ADDRESS = B1001110;
  else if (A0 == F && A1 == L) I2C_ADDRESS = B1001011;
  else if (A0 == F && A1 == H) I2C_ADDRESS = B1001111;
  this->lineCom = &lineCom;
}

void ADS1112::selectChannel(byte channel, byte gain, byte mode)
{
    byte INP1 = 0, INP0 = 0;

    if (mode == MODE_UNIPOLAR) {
    if (channel == CHANNEL_0) {
      INP1 = 1;
      INP0 = 0;
    } else if (channel == CHANNEL_1) {
      INP1 = 1;
      INP0 = 1;
    } else if (channel == CHANNEL_2) {
      INP1 = 0;
      INP0 = 1;
    }       
    } else { //bipolar
    if (channel == CHANNEL_0) {
      INP1 = 0;
      INP0 = 0;
    } else if (channel == CHANNEL_1) {
      INP1 = 0;
      INP0 = 1;
    }   
  }

    //configuration register, assuming 16 bit resolution
  byte reg = 1 << BIT_ST_DRDY |
      INP1 << BIT_INP1 |
      INP0 << BIT_INP0 |
      1 << BIT_DR1 |
      1 << BIT_DR0 |
      1 << BIT_SC  |
      gain;
//Serial.print("\tdata reg = "+String(reg));
  lineCom->beginTransmission(I2C_ADDRESS);
  lineCom->write(reg);
  lineCom->endTransmission();        
}

double ADS1112::readADC()
{
   long t = 0; byte count_ = 0;
   long tt=0;
   unsigned long timeL = millis();
   while( millis() - timeL <= 10UL  || count_ <= 100) 
   {
     //
     lineCom->requestFrom(I2C_ADDRESS, (byte) 3);
    if (lineCom->available() != 3)
        continue;
    t = lineCom->read() << 8 | lineCom->read();
    byte r = lineCom->read();
    //t = h << 8 |  l;
    if (bitRead(r, 7))
      delayMicroseconds(100);
    else 
    {
      tt+=t;
      count_++;
    }  
      //break;
   };
    
  tt = roundl(tt/count_);  
  if (tt >= 32768) tt -= 65536l;
  double v = (double) tt * 2.048/32768.0;
  return v;///count_;
}

#include "core_pins.h"
#ifndef PSLoad_H
#define PSLoad_H

#include <Arduino.h>
#include <movingAvgFloat.h>



class PSLoad
{
private:
  HardwareSerial* serialPort;
  uint8_t DataOut_1;
  uint8_t DataOut_2;


  movingAvgFloat** _Avgs;

  movingAvgFloat* avgVolt;
  movingAvgFloat* avgAmp;
  movingAvgFloat* avgVoltCal;
  movingAvgFloat* avgAmpCal;


public:

  float Vset = 5.0;
  float Ipset = 1.0;
  float Inset = 1.0;
  bool PwSet = 0;

  bool DataReady = 0;
  bool DispData = 0;


  float Volt, Amp, Watt, temp1, temp2;

  PSLoad();

  PSLoad(movingAvgFloat* Avgs[], HardwareSerial* serialPort, uint8_t DataOut_1, uint8_t DataOut_2)
  {
    this->serialPort = serialPort;
    this->DataOut_1 = DataOut_1;
    this->DataOut_2 = DataOut_2;

    _Avgs = Avgs;
    avgVolt     = _Avgs[0];
    avgAmp      = _Avgs[1];
    avgVoltCal  = _Avgs[2];
    avgAmpCal   = _Avgs[3];
  }


  void init()
  {
    serialPort->begin(2000000);

    avgVolt->begin();
    avgAmp->begin();
    avgVoltCal->begin();
    avgAmpCal->begin();
  }

  void SendData(char prefix, float data)
  {
    digitalWrite(DataOut_1, HIGH);
    serialPort->print(prefix);
    serialPort->print(data, 6);
    serialPort->print('t');
    digitalWrite(DataOut_1, LOW);
  }

  void SendData(char data)
  {
    digitalWrite(DataOut_1, HIGH);
    serialPort->print(data);
    digitalWrite(DataOut_1, LOW);
  }

  void SendArray(char prefix, double array[], uint16_t size){   //Serial Parameter
  
    //Serial1.print(prefix);
    for (uint8_t n = 0; n < size; n++){
      digitalWrite(DataOut1, HIGH);
      Serial1.print(prefix);
      Serial1.print(n);
      Serial1.print(' ');
      Serial1.print(array[n], 7);
      digitalWrite(DataOut1, LOW);

      Serial.print(array[n], 7);
      Serial.print("__");

      delay(40);
    }
    Serial.println("__");
  }

  void GetData(void){     // ~6 us
    //myTime = micros();
    while (serialPort->available() > 0){
      //Serial.println("Get Data");
      char first = serialPort->read();
      if (first == 'V'){
        Volt = serialPort->parseFloat();
      }
      if (first == 'A'){
        Amp = serialPort->parseFloat();
      }
      if (first == 'n'){
        temp1 = serialPort->parseFloat();
      }
      if (first == 'm'){
        temp2 = serialPort->parseFloat();
      }
      if (first == 'y'){
        //for (uint8_t n = 0; n <= 9; n++){
        Serial.println(serialPort->parseFloat(), 6);
        //}
        //Serial.println('#');
      }
      if (first == 'v'){
        Serial.println(serialPort->parseInt());
        //Serial.println('#');
      }else{

      }
    }
    DataReady = 0;
    DispData = 1;

    //myTime = micros() - myTime;
    //Serial.println(myTime);
  }

  void dipsData(){        //~4403 us

    //myTime = micros();
    //digitalWrite(Debug, HIGH);
    Volt = avgVolt->reading(Volt);          //mooving Average
    Amp = avgAmp->reading(Amp);             //mooving Average

    tft.setTextDatum(TR_DATUM);
    tft.setTextPadding(135);

    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.drawFloat(Volt, 5, 125, 30, 4);

    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawFloat(Amp, 5, 125, 60, 4);

    Watt = Volt * Amp;
    tft.setTextColor(TFT_SKYBLUE, TFT_BLACK);
    tft.drawFloat(Watt, 5, 125, 90, 4);

    //digitalWrite(Debug, LOW);
    DispData = 0;

    //myTime = micros() - myTime;
    //Serial.println(myTime);
  }

  void SpriteDipsData(){        //~6500 us !!!

    //myTime = micros();
    //digitalWrite(Debug, HIGH);

    Volt = avgVolt->reading(Volt);          //mooving Average
    Amp = avgAmp->reading(Amp);             //mooving Average

    SprCh1.setTextDatum(TR_DATUM);
    SprCh1.setTextColor(TFT_ORANGE, TFT_BLACK);

    SprCh1.setTextPadding(135);
    SprCh1.drawFloat(Volt, 5, 125, 0, 4);

    SprCh1.setTextColor(TFT_YELLOW, TFT_BLACK);
    SprCh1.drawFloat(Amp, 5, 125, 30, 4);

    Watt = Volt * Amp;
    SprCh1.setTextColor(TFT_SKYBLUE, TFT_BLACK);
    SprCh1.drawFloat(Watt, 5, 125, 60, 4);

    
    SprCh1.pushSprite(0, 30);

    //digitalWrite(Debug, LOW);
    DispData = 0;

    //myTime = micros() - myTime;
    //Serial.println(myTime);
  }
  


  void initPSLoad(){
    
    tft.fillRect(0, 0, 320, 240, TFT_BLACK);

    tft.fillRect(159, 16, 2, 225, TFT_RED);
    tft.drawFastHLine(60, 16, 200, TFT_RED);
    tft.drawFastVLine(60, 0, 16, TFT_RED);
    tft.drawFastVLine(260, 0, 16, TFT_RED);

    tft.setTextPadding(0);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    tft.drawChar('C', 97, 0, 2);
    tft.drawChar('%', 175, 0, 2);
    tft.drawString("Fan:", 115, 0, 2);

    tft.drawString("Ch.1", 0, 0, 4);

    //tft.setTextPadding(100);
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.drawChar('V', 135, 30, 4);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawChar('A', 135, 60, 4);
    tft.setTextColor(TFT_SKYBLUE, TFT_BLACK);
    tft.drawChar('W', 135, 90, 4);

    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.drawChar('V', 135, 150, 4);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawChar('A', 135, 180, 4);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawChar('A', 135, 210, 4);

    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    tft.drawString("Set", 0, 120, 4);
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.drawString("V", 0, 150, 4);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawString("I+", 0, 180, 4);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawString("I-", 0, 210, 4);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    if(PwSet) SendData('o');
    if(!PwSet) SendData('f');

    tft.setTextPadding(100);

    delay(100);
    
    SendData('v', Vset);
    SendData('i', Ipset);
    SendData('s', Inset);

    dispSetData();
    underLine();

    Serial.println("Start");
    digitalWrite(DataOut_2, HIGH);
  }

  void dispSetData(){
    tft.setTextDatum(TR_DATUM);
    tft.setTextPadding(100);
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.drawFloat(Vset, 3, 125, 150, 4);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawFloat(Ipset, 4, 125, 180, 4);
    tft.drawFloat(Inset, 4, 125, 210, 4);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    //upDateUndLine = 1;
    //underLine();
  }

  void underLine(){
    myTime = micros();
    tft.fillRect(0, 176, 124, 4, TFT_BLACK);    //172
    tft.fillRect(0, 206, 124, 4, TFT_BLACK);
    tft.fillRect(0, 236, 124, 4, TFT_BLACK);
    if (SetType == 0){
      //tft.fillRect(0, 172, 124, 4, TFT_BLACK);
      if (factor == 0)      tft.fillRect(112, 176, 12, 4, TFT_MAGENTA);
      else if (factor == 1) tft.fillRect(98, 176, 12, 4, TFT_MAGENTA);
      else if (factor == 2) tft.fillRect(84, 176, 12, 4, TFT_MAGENTA);
      else if (factor == 3) tft.fillRect(63, 176, 12, 4, TFT_MAGENTA);
      else if (factor == 4) tft.fillRect(49, 176, 12, 4, TFT_MAGENTA);

    }else if (SetType == 1){
      //tft.fillRect(0, 202, 124, 4, TFT_BLACK);
      if (factor == 0)      tft.fillRect(112, 206, 12, 4, TFT_MAGENTA);
      else if (factor == 1) tft.fillRect(98, 206, 12, 4, TFT_MAGENTA);
      else if (factor == 2) tft.fillRect(84, 206, 12, 4, TFT_MAGENTA);
      else if (factor == 3) tft.fillRect(70, 206, 12, 4, TFT_MAGENTA);
      else if (factor == 4) tft.fillRect(49, 206, 12, 4, TFT_MAGENTA);

    }else if (SetType == 2){
      //tft.fillRect(0, 232, 124, 4, TFT_BLACK);
      if (factor == 0)      tft.fillRect(112, 236, 12, 4, TFT_MAGENTA);
      else if (factor == 1) tft.fillRect(98, 236, 12, 4, TFT_MAGENTA);
      else if (factor == 2) tft.fillRect(84, 236, 12, 4, TFT_MAGENTA);
      else if (factor == 3) tft.fillRect(70, 236, 12, 4, TFT_MAGENTA);
      else if (factor == 4) tft.fillRect(49, 236, 12, 4, TFT_MAGENTA);
    }
    myTime = micros() - myTime;
    Serial.println(myTime);
  }

  void CalMode(){   //Serial Parameter

    Buzzer(700);

    tft.fillRect(0, 0, 320, 240, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    tft.setTextPadding(200);
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.drawString("A: Calibrate Voltage", 0, 0, 4);
    tft.drawString("B: Calibrate Current", 0, 30, 4);
    tft.drawString("D: Exit", 0, 60, 4);

    //uint8_t n = 0;
    while(mode == 1){
      char key = keypad.getKey();
      if (key == 'D'){
        mode = 0;
      }
      else if (key == 'A'){            //Enter Volt Cal Mode
        CalVoltage();
        mode = 0;
      }
      else if (key == 'B'){             //Enter Curr Cal Mode
        CalCurrent();
        mode = 0;
      }
    }
    //if (FanMetro.check())  FanContr();
    Serial.println("finish");
    //mode = 0;
  }

  void CalCurrent(){   //Serial Parameter
    float CalAmp;
    //double AmpAdcSetPoints[] =     {-5.0,     -4.9,     -4.5,    -4.0,    -1.0,    -0.5,    -0.1,    -0.05,    -0.005,     -0.0000001,     0.0,     0.005,     0.05,    0.1,    0.5,    1.0,    4.0,    4.5,    4.9,     5.0};
    double AmpAdcOffsets[20];
    double AmpSetPoints[] =     {0.0,     0.005,     0.05,    0.1,    0.5,    1.0,    4.0,    4.5,    4.9,     5.0};
    double AmpDacPOffsets[10];
    double AmpDacNOffsets[10];
    Serial.println("enter Cal Func");
    digitalWrite(DataOut_2, HIGH);
    SendData('o');
    
    tft.fillRect(0, 0, 320, 240, TFT_BLACK);
    SendData('x');
    analogWrite(Fan, 255);
    uint8_t n = 0;
    while (n <= 9){                   //Cal Amp Source
      tft.fillRect(0, 0, 320, 240, TFT_BLACK);
      tft.setTextDatum(TL_DATUM);
      tft.setTextPadding(200);
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
      tft.drawString("Meas A Surc. Ch1:", 0, 10, 4);
      tft.drawString("WAIT!", 0, 40, 4);
      tft.drawFloat(AmpSetPoints[n], 4, 240, 10, 4);
      SendData('i', AmpSetPoints[n]);
      //Serial.println("Cal V1");
      uint8_t AvgCounter = 0;
      uint8_t Check_n = n;
      while(Check_n == n){
        //Serial.println("Ch1.DataReady: ");
        //Serial.println(Ch1.DataReady);
        if (DataReady)   GetData();
        if (DispData){
          CalAmp = avgAmpCal->reading(Amp);
          DispData = 0;
          AvgCounter++;
        }
        if(AvgCounter >= 50){
          if (AvgCounter == 50){
            tft.setTextDatum(TL_DATUM);
            tft.setTextPadding(200);
            tft.setTextColor(TFT_ORANGE, TFT_BLACK);
            tft.drawString("Enter Value", 0, 40, 4);
          }
          AvgCounter = 200;
          
          double keyreturn = checkKeys();
          if (keyreturn != -1.0){
            AmpDacPOffsets[n] = AmpSetPoints[n] - keyreturn;
            AmpAdcOffsets[10+n] = keyreturn - CalAmp;
            Serial.print("ADC Meas: ");
            Serial.print(CalAmp, 4);
            Serial.print("   Offset DAC: ");
            Serial.print(AmpDacPOffsets[n],6);
            Serial.print("  Offset ADC: ");
            Serial.println(AmpAdcOffsets[10+n],6);
            n++;
          }
        }
      }
    }
    SendData('i', 0.0);
    SendData('v', 0.1);
    n = 0;
    while (n <= 9){                   //Cal Amp Sink
      tft.fillRect(0, 0, 320, 240, TFT_BLACK);
      tft.setTextDatum(TL_DATUM);
      tft.setTextPadding(200);
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
      tft.drawString("Meas A Sink Ch1:", 0, 10, 4);
      tft.drawString("WAIT!", 0, 40, 4);
      tft.drawFloat(AmpSetPoints[n], 4, 240, 10, 4);
      SendData('s', AmpSetPoints[n]);
      //Serial.println("Cal V1");
      uint8_t AvgCounter = 0;
      uint8_t Check_n = n;
      while(Check_n == n){
        //Serial.println("Ch1.DataReady: ");
        //Serial.println(Ch1.DataReady);
        if (DataReady)   GetData();
        if (DispData){
          CalAmp = avgAmpCal->reading(Amp);
          DispData = 0;
          AvgCounter++;
        }
        if(AvgCounter >= 50){
          if (AvgCounter == 50){
            tft.setTextDatum(TL_DATUM);
            tft.setTextPadding(200);
            tft.setTextColor(TFT_ORANGE, TFT_BLACK);
            tft.drawString("Enter Value", 0, 40, 4);
          }
          AvgCounter = 200;
          
          double keyreturn = checkKeys();
          if (keyreturn != -1.0){
            AmpDacNOffsets[n] = AmpSetPoints[n] - keyreturn;
            AmpAdcOffsets[9-n] = keyreturn - CalAmp;
            Serial.print("ADC Meas: ");
            Serial.print(CalAmp, 4);
            Serial.print("   Offset DAC: ");
            Serial.print(AmpDacNOffsets[n],6);
            Serial.print("  Offset ADC: ");
            Serial.println(AmpAdcOffsets[9-n],6);
            n++;
          }
        }
      }
    }
    //I   Cal Coeff. CurrDacPos
    //O   Cal Coeff. CurrDacNeg
    //P   Cal Coeff. CurrAdc
    SendArray('I', AmpDacPOffsets, 10);
    SendArray('O', AmpDacNOffsets, 10);
    SendArray('P', AmpAdcOffsets, 20);

    SendData('S');

    Serial.println("Send save cmd");

    SendData('i', 0.1);
    SendData('s', 0.1);
    
  }
  
  void CalVoltage(){   //Serial Parameter
    float CalVolt;
    double VoltSetPoints[] =     {0.03,   0.05,     0.1,    1.0,    5.0,    10.0,   15.0,   20.0,   24.0,   25.0};
    double VoltDacOffsets[10];
    double VoltAdcOffsets[10];
    Serial.println("enter Cal Func");
    digitalWrite(DataOut_2, HIGH);

    SendData('o');
    
    tft.fillRect(0, 0, 320, 240, TFT_BLACK);

    SendData('X');

    uint8_t n = 0;
    while (n <= 9){
      tft.fillRect(0, 0, 320, 240, TFT_BLACK);
      tft.setTextDatum(TL_DATUM);
      tft.setTextPadding(200);
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
      tft.drawString("Meas Volt Ch1:", 0, 10, 4);
      tft.drawString("WAIT!", 0, 40, 4);
      tft.drawFloat(VoltSetPoints[n], 3, 180, 10, 4);
      SendData('v', VoltSetPoints[n]);
      //Serial.println("Cal V1");
      uint8_t AvgCounter = 0;
      uint8_t Check_n = n;
      while(Check_n == n){
        //Serial.println("Ch1.DataReady: ");
        //Serial.println(Ch1.DataReady);
        if (DataReady)   GetData();
        if (DispData){
          CalVolt = avgVoltCal->reading(Volt);
          DispData = 0;
          AvgCounter++;
        }
        if(AvgCounter >= 60){
          if (AvgCounter == 60){
            tft.setTextDatum(TL_DATUM);
            tft.setTextPadding(200);
            tft.setTextColor(TFT_ORANGE, TFT_BLACK);
            tft.drawString("Enter Value", 0, 40, 4);
          }
          AvgCounter = 200;
          
          double keyreturn = checkKeys();
          if (keyreturn != -1.0){
            VoltDacOffsets[n] = VoltSetPoints[n] - keyreturn;
            VoltAdcOffsets[n] = keyreturn - CalVolt;
            Serial.print("Offset DAC: ");
            Serial.print(VoltDacOffsets[n],6);
            Serial.print("  Offset ADC: ");
            Serial.println(VoltAdcOffsets[n],6);
            n++;
          }
        }
      }
    }
    SendArray('L', VoltDacOffsets, 10);
    SendArray('K', VoltAdcOffsets, 10);

    SendData('S');

    Serial.println("finish Volt Cal");
  }

};


#endif
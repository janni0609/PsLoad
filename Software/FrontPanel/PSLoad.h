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

  void GetData(void){
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
        Serial.println(serialPort->parseFloat(),9);
        Serial.println('#');
      }else{

      }
    }
    DataReady = 0;
    DispData = 1;

  }

  void dipsData(){
    //digitalWrite(Debug, HIGH);
    Volt = avgVolt->reading(Volt);          //mooving Average
    Amp = avgAmp->reading(Amp);             //mooving Average

    tft.setTextDatum(TR_DATUM);
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);

    tft.setTextPadding(135);
    tft.drawFloat(Volt, 5, 125, 30, 4);

    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawFloat(Amp, 5, 125, 60, 4);

    Watt = Volt * Amp;
    tft.setTextColor(TFT_SKYBLUE, TFT_BLACK);
    tft.drawFloat(Watt, 5, 125, 90, 4);

    //digitalWrite(Debug, LOW);
    DispData = 0;
  }


  void initPSLoad(){
    
    tft.fillRect(0, 0, 320, 240, TFT_BLACK);

    tft.fillRect(159, 0, 2, 240, TFT_RED);

    tft.setTextPadding(0);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("C", 35, 0, 2);
    tft.drawString("C", 95, 0, 2);

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

    /*
    digitalWrite(DataOut1, HIGH);
    if(PwSetCH1) Serial1.print("o");
    if(!PwSetCH1) Serial1.print("f");
    digitalWrite(DataOut1, LOW);
    */

    if(PwSet) SendData('o');
    if(!PwSet) SendData('f');

    tft.setTextPadding(100);
    
    SendData('v', Vset);
    SendData('i', Ipset);
    SendData('s', Inset);

    /*
    SendDataToCh1('v', VsetCH1);
    SendDataToCh1('i', IpsetCH1);
    SendDataToCh1('s', InsetCH1);
    */

    dispSetData();

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
    underLine();
  }

  void underLine(){
    tft.fillRect(0, 172, 124, 4, TFT_BLACK);
    tft.fillRect(0, 202, 124, 4, TFT_BLACK);
    tft.fillRect(0, 232, 124, 4, TFT_BLACK);
    if (SetType == 0){
      if (factor == 0)      tft.fillRect(112, 172, 12, 4, TFT_MAGENTA);
      else if (factor == 1) tft.fillRect(98, 172, 12, 4, TFT_MAGENTA);
      else if (factor == 2) tft.fillRect(84, 172, 12, 4, TFT_MAGENTA);
      else if (factor == 3) tft.fillRect(63, 172, 12, 4, TFT_MAGENTA);
      else if (factor == 4) tft.fillRect(49, 172, 12, 4, TFT_MAGENTA);

    }else if (SetType == 1){
      if (factor == 0)      tft.fillRect(112, 202, 12, 4, TFT_MAGENTA);
      else if (factor == 1) tft.fillRect(98, 202, 12, 4, TFT_MAGENTA);
      else if (factor == 2) tft.fillRect(84, 202, 12, 4, TFT_MAGENTA);
      else if (factor == 3) tft.fillRect(70, 202, 12, 4, TFT_MAGENTA);
      else if (factor == 4) tft.fillRect(49, 202, 12, 4, TFT_MAGENTA);

    }else if (SetType == 2){
      if (factor == 0)      tft.fillRect(112, 232, 12, 4, TFT_MAGENTA);
      else if (factor == 1) tft.fillRect(98, 232, 12, 4, TFT_MAGENTA);
      else if (factor == 2) tft.fillRect(84, 232, 12, 4, TFT_MAGENTA);
      else if (factor == 3) tft.fillRect(70, 232, 12, 4, TFT_MAGENTA);
      else if (factor == 4) tft.fillRect(49, 232, 12, 4, TFT_MAGENTA);
    }
  }
};


#endif
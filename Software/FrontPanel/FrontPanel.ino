#include <movingAvgFloat.h>
#include "SPI.h"
#include "TFT_eSPI.h"

TFT_eSPI tft = TFT_eSPI();

//float VoltCH1, AmpCH1, WattCH1, temp1, temp2;//

const uint8_t DataIn1 = 5;
const uint8_t DataIn2 = 3;
const uint8_t DataOut1 = 4;
const uint8_t DataOut2 = 2;

const uint8_t InterrSW = 33;

const uint8_t Debug = 39;

const uint8_t Fan = 19;

bool Error = 0;

//bool DataReadyCh1 = 0;//
//bool DispData = 0;//

movingAvgFloat avgVoltCH1(10);
movingAvgFloat avgAmpCH1(10);

movingAvgFloat avgVoltCalCH1(50);
movingAvgFloat avgAmpCH1Cal(50);

//------------------------------------------------------------------------------------------------
//Encoder
const uint8_t RotClk = 21;
const uint8_t RotDt = 22;

const uint8_t RotBtn = 13;

bool last_run = LOW;
bool aState;
bool lastaState;

bool sendData = 0;

bool sendVolt = 0;
bool sendCurrP = 0;
bool sendCurrN = 0;

//------------------------------------------------------------------------------------------------
//SetPoints CH1
//float VsetCH1 = 5.0;//
//float IpsetCH1 = 1.0;//
//float InsetCH1 = 1.0;//

//bool PwSetCH1 = 0;//
bool PwSetCH2 = 0;

int8_t factor = 0;
uint8_t SetType = 0;            // 0: Vset, 1: Ipset, 2:Inset
bool ChannelSet = 0;            // 0: CH1, 1: CH2

//bool upDateUndLine = 1;
volatile uint8_t mode = 0;               // 0: normla PSLoad Mode,

bool CalVal = 0;

//------------------------------------------------------------------------------------------------
//Metro
#include <Metro.h>                      //Include Metro library

Metro FanMetro = Metro(250);            // Instanciate a metro object and set the interval to 250 milliseconds (0.25 seconds).

//Metro IoT = Metro(1000);            // Instanciate a metro object and set the interval to 250 milliseconds (0.25 seconds).

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//Cal

const double VoltSetPoints[] =     {0.03,   0.05,     0.1,    1.0,    5.0,    10.0,   15.0,   20.0,   24.0,   25.0};
double VoltDacOffsets[] =    {0.0,    0.0,      0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0};
double VoltAdcOffsets[] =    {0.0,    0.0,      0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0};


const double AmpAdcSetPoints[] =     {-5.0,     -4.9,     -4.5,    -4.0,    -1.0,    -0.5,    -0.1,    -0.05,    -0.005,     0.0,     0.0,     0.005,     0.05,    0.1,    0.5,    1.0,    4.0,    4.5,    4.9,     5.0};
double AmpAdcOffsets[20];

const double AmpSetPoints[] =     {0.0,     0.005,     0.05,    0.1,    0.5,    1.0,    4.0,    4.5,    4.9,     5.0};
double AmpDacPOffsets[] =   {0.0,     0.0,      0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0};
double AmpDacNOffsets[] =   {0.0,     0.0,      0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0};


//------------------------------------------------------------------------------------------------
//SetPoints CH1
#include <Keypad.h>

const byte ROWS = 5; //four rows
const byte COLS = 5; //three columns
char keys[ROWS][COLS] = {
  {'1','2','3','A','I'},
  {'4','5','6','B','J'},
  {'7','8','9','C','K'},
  {'.','0','#','D','L'},
  {'E','F','G','H','M'}
};
byte rowPins[ROWS] = {32, 31, 30, 29, 28}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {35, 36, 37, 38, 34}; //connect to the column pinouts of the keypad

Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );



class PSLoad
{
private:
  HardwareSerial* serialPort;
  uint8_t DataOut_1;
public:

  float Vset = 5.0;
  float Ipset = 1.0;
  float Inset = 1.0;
  bool PwSet = 0;

  bool DataReady = 0;
  bool DispData = 0;


  float Volt, Amp, Watt, temp1, temp2;

  PSLoad();

  PSLoad(HardwareSerial* serialPort, uint8_t DataOut_1)
  {
    this->serialPort = serialPort;
    this->DataOut_1 = DataOut_1;
  }


  void init()
  {
    serialPort->begin(1000000);
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
    Volt = avgVoltCH1.reading(Volt);          //mooving Average
    Amp = avgAmpCH1.reading(Amp);             //mooving Average

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


};

//PSLoad Test(&Serial2);

PSLoad Ch1(&Serial1, DataOut1);


void setup() 
{
  //Test.init();
  //delay(100);

  //Test.SendData('A', 1.23456);
  //Test.SendData('l');

  Ch1.init();



  //Encoder
  pinMode(RotClk, INPUT);
  pinMode(RotDt, INPUT);
  pinMode(RotBtn, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RotBtn),  RotBtnISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(RotClk),  shaft_moved, CHANGE);
  //------------------------------------------------------------------------------------------------

  //Serial1.begin(1000000);

  Serial.begin(1000000);

  analogWriteFrequency(Fan, 25000);

  pinMode(Debug, OUTPUT);
  pinMode(DataOut1, OUTPUT);
  pinMode(DataOut2, OUTPUT);

  pinMode(InterrSW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(InterrSW),  ModeSwISR, FALLING);

  digitalWrite(DataOut1, LOW);
  digitalWrite(DataOut2, LOW);

  pinMode(DataIn2, INPUT);

  pinMode(DataIn1, INPUT);
  attachInterrupt(DataIn1, DataReadyCH1, RISING);

  avgVoltCH1.begin();
  avgAmpCH1.begin();

  avgVoltCalCH1.begin();
  avgAmpCH1Cal.begin();

  tft.begin();
  tft.setRotation(1);

  tft.fillScreen(TFT_BLACK);


  /*

  for (uint8_t n  = 0; n <= 9; n++){
    digitalWrite(DataOut1, HIGH);
    Serial1.print('L');
    Serial1.print(n);
    Serial1.print(' ');
    Serial1.print(0.0);
    digitalWrite(DataOut1, LOW);
  }

  */
  

  


  //Serial.println(sizeof(VoltDacOffsets));
 
  //initPSLoad();
}

/*
  Commands Send to module:

  v   set Voltage         (float)
  i   set Current Source  (float)
  s   set Current Sink    (float)
  o   Output on
  f   Output off
  h   Sense internal
  u   Sense External
  r   set Votage Raw Data (uint16_t)
  X   set Volt Cal Coeff. to Zero
  x   set Curr Cal Coeff. to Zero
  L   Cal Coeff. VoltDac
  K   Cal Coeff. VoltAdc
  I   Cal Coeff. CurrDacPos
  O   Cal Coeff. CurrDacNeg
  P   Cal Coeff. CurrAdc


  Commands Send to Front:

  V   meas. Voltage       (float)
  A   meas. Current       (float)
  n   meas. temp 1        (float)
  m   meas. temp 2        (float)
  E   Error               (uint8_t)
      OverTemp        Set     1
      OverTemp        Reset   0

*/

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

  if(Ch1.PwSet) Ch1.SendData('o');
  if(!Ch1.PwSet) Ch1.SendData('f');

  tft.setTextPadding(100);

  
  Ch1.SendData('v', Ch1.Vset);
  Ch1.SendData('i', Ch1.Ipset);
  Ch1.SendData('s', Ch1.Inset);

  /*
  SendDataToCh1('v', VsetCH1);
  SendDataToCh1('i', IpsetCH1);
  SendDataToCh1('s', InsetCH1);
  */

  dispSetData();

  Serial.println("Start");
  digitalWrite(DataOut2, HIGH);
}

void LoopPSload(){

  Serial.println("enter PSLoad Func");

  initPSLoad();

  while (mode == 0){                  //PSLoad normal mode
    if (Ch1.DataReady)  Ch1.GetData();
    if (Ch1.DispData)   Ch1.dipsData();


    if (sendVolt) {
      dispSetData();
      Ch1.SendData('v', Ch1.Vset);
      sendVolt = 0;
    }
    if (sendCurrP) {
      dispSetData();
      Ch1.SendData('i', Ch1.Ipset);
      sendCurrP = 0;
    }
    if (sendCurrN) {
      Ch1.SendData('s', Ch1.Inset);
      dispSetData();
      sendCurrN = 0;
    }

    float keyreturn = checkKeys();
    if (keyreturn != -1.0) Serial.println(keyreturn, 4);
    processNumPad(keyreturn);


    if (FanMetro.check())  FanContr();

  /*
    if (IoT.check()){
      Serial.print("Volt: ");
      Serial.print(VoltCH1,5);
      Serial.print("  Amp: ");
      Serial.print(AmpCH1,5);
      Serial.print("  Temp: ");
      Serial.println(temp1,1);
    }
  */

    
    while (Serial.available() > 0){
      //Serial.println("Start");
      char firstChar = Serial.read();
      if (firstChar == 'H'){
        for (uint8_t n = 0; n <= 3; n++){
          Serial.println(Serial.parseFloat());
        }
        Serial.println("Finish");
      }
      if (firstChar == 'L'){
        uint8_t n = Serial.parseInt();
        float flt = Serial.parseFloat();
        Serial.print(n);
        Serial.print(' ');
        Serial.println(flt,6);
      }
    }
  }       //while loop
}         //function

void CalMode(){   //Serial Parameter
  tft.fillRect(0, 0, 320, 240, TFT_BLACK);

  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(200);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.drawString("A: Calibrate Voltage", 0, 0, 4);
  tft.drawString("B: Calibrate Current", 0, 30, 4);
  tft.drawString("D: Exit", 0, 60, 4);

  uint8_t n = 0;

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

  double AmpAdcSetPoints[] =     {-5.0,     -4.9,     -4.5,    -4.0,    -1.0,    -0.5,    -0.1,    -0.05,    -0.005,     -0.0000001,     0.0,     0.005,     0.05,    0.1,    0.5,    1.0,    4.0,    4.5,    4.9,     5.0};
  double AmpAdcOffsets[20];

  double AmpSetPoints[] =     {0.0,     0.005,     0.05,    0.1,    0.5,    1.0,    4.0,    4.5,    4.9,     5.0};
  double AmpDacPOffsets[10];
  double AmpDacNOffsets[10];


  Serial.println("enter Cal Func");

  digitalWrite(DataOut2, HIGH);

  digitalWrite(DataOut1, HIGH);
  Serial1.print("o");
  digitalWrite(DataOut1, LOW);
  

  tft.fillRect(0, 0, 320, 240, TFT_BLACK);

  digitalWrite(DataOut1, HIGH);
  Serial1.print('x');
  digitalWrite(DataOut1, LOW);


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
    Ch1.SendData('i', AmpSetPoints[n]);
    //Serial.println("Cal V1");

    uint8_t AvgCounter = 0;

    uint8_t Check_n = n;

    while(Check_n == n){
      //Serial.println("Ch1.DataReady: ");
      //Serial.println(Ch1.DataReady);
      if (Ch1.DataReady)   Ch1.GetData();
      if (Ch1.DispData){
        CalAmp = avgAmpCH1Cal.reading(Ch1.Amp);
        Ch1.DispData = 0;
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

  Ch1.SendData('i', 0.0);
  Ch1.SendData('v', 0.1);


  n = 0;
  while (n <= 9){                   //Cal Amp Sink
    tft.fillRect(0, 0, 320, 240, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    tft.setTextPadding(200);
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.drawString("Meas A Sink Ch1:", 0, 10, 4);
    tft.drawString("WAIT!", 0, 40, 4);
    tft.drawFloat(AmpSetPoints[n], 4, 240, 10, 4);
    Ch1.SendData('s', AmpSetPoints[n]);
    //Serial.println("Cal V1");

    uint8_t AvgCounter = 0;

    uint8_t Check_n = n;

    while(Check_n == n){
      //Serial.println("Ch1.DataReady: ");
      //Serial.println(Ch1.DataReady);
      if (Ch1.DataReady)   Ch1.GetData();
      if (Ch1.DispData){
        CalAmp = avgAmpCH1Cal.reading(Ch1.Amp);
        Ch1.DispData = 0;
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


  digitalWrite(DataOut1, HIGH);           //Store Cal Cost. in Module EEprom
  Serial1.print('S');
  digitalWrite(DataOut1, LOW);

  Serial.println("Send save cmd");

  Ch1.SendData('i', 0.1);
  Ch1.SendData('s', 0.1);
  
}

void CalVoltage(){   //Serial Parameter
  float CalVolt;

  double VoltSetPoints[] =     {0.03,   0.05,     0.1,    1.0,    5.0,    10.0,   15.0,   20.0,   24.0,   25.0};
  double VoltDacOffsets[10];
  double VoltAdcOffsets[10];

  Serial.println("enter Cal Func");

  digitalWrite(DataOut2, HIGH);

  digitalWrite(DataOut1, HIGH);
  Serial1.print("o");
  digitalWrite(DataOut1, LOW);
  

  tft.fillRect(0, 0, 320, 240, TFT_BLACK);

  digitalWrite(DataOut1, HIGH);
  Serial1.print('X');
  digitalWrite(DataOut1, LOW);
  uint8_t n = 0;

  while (n <= 9){
    tft.fillRect(0, 0, 320, 240, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    tft.setTextPadding(200);
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.drawString("Meas Volt Ch1:", 0, 10, 4);
    tft.drawString("WAIT!", 0, 40, 4);
    tft.drawFloat(VoltSetPoints[n], 3, 180, 10, 4);
    Ch1.SendData('v', VoltSetPoints[n]);
    //Serial.println("Cal V1");

    uint8_t AvgCounter = 0;

    uint8_t Check_n = n;

    while(Check_n == n){
      //Serial.println("Ch1.DataReady: ");
      //Serial.println(Ch1.DataReady);
      if (Ch1.DataReady)   Ch1.GetData();
      if (Ch1.DispData){
        CalVolt = avgVoltCalCH1.reading(Ch1.Volt);
        Ch1.DispData = 0;
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

  digitalWrite(DataOut1, HIGH);           //Store Cal Cost. in Module EEprom
  Serial1.print('S');
  digitalWrite(DataOut1, LOW);

  Serial.println("finish Volt Cal");
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

void ModeSwISR(){
  //mode = !mode;
  mode = 1;
}

/*
void SendDataToCh1(char prefix, float data){   //Serial Parameter


  digitalWrite(DataOut1, HIGH);
  Serial1.print(prefix);
  Serial1.print(data, 6);
  Serial1.print('t');
  digitalWrite(DataOut1, LOW);
}
*/

void RotBtnISR(){   //ISR
  //sendData = 1;
}

float factorSelV(void){
  if (factor == 0) return 0.001;
  if (factor == 1) return 0.01;
  if (factor == 2) return 0.1;
  if (factor == 3) return 1.0;
  if (factor == 4) return 10.0;
}

float factorSelI(void){
  if (factor == 0) return 0.0001;
  if (factor == 1) return 0.001;
  if (factor == 2) return 0.01;
  if (factor == 3) return 0.1;
  if (factor == 4) return 1.0;
}

void shaft_moved(){   //ISR
  aState = digitalRead(RotClk);
  
  last_run = !last_run;
  if(last_run == LOW){
    if (digitalRead(RotDt) != aState)
    {
      if(ChannelSet == 0){                      //Set Ch1
        if (SetType == 0){                      //Set Volts
          Ch1.Vset = Ch1.Vset +  factorSelV();
          sendVolt = 1;
        }else if(SetType == 1){                 //Set I +
          Ch1.Ipset = Ch1.Ipset + factorSelI();
          sendCurrP = 1;
        }else if(SetType == 2){                 //Set I +
          Ch1.Inset = Ch1.Inset + factorSelI();
          sendCurrN = 1;
        }
      }
      if(ChannelSet == 1){                      //Set Ch2

      }


    }else{
      if(ChannelSet == 0){                      //Set Ch1
        if (SetType == 0){                      //Set Volts
          Ch1.Vset = Ch1.Vset -  factorSelV();
          sendVolt = 1;
        }else if(SetType == 1){                 //Set I +
          Ch1.Ipset = Ch1.Ipset - factorSelI();
          sendCurrP = 1;
        }else if(SetType == 2){                 //Set I +
          Ch1.Inset = Ch1.Inset - factorSelI();
          sendCurrN = 1;
        }
      }
      if(ChannelSet == 1){                      //Set Ch2

      }
    }
    Ch1.Vset = constrain(Ch1.Vset, 0.02, 25.0);
    Ch1.Ipset = constrain(Ch1.Ipset, 0.000001, 5.0);
    Ch1.Inset = constrain(Ch1.Inset, 0.000001, 5.0);
  }
}

void DataReadyCH1(void){  //ISR
  Ch1.DataReady = 1;
}

/*
void GetDataCH1(void){
  while (Serial1.available() > 0){
    //Serial.println("Get Data");
    char first = Serial1.read();
    if (first == 'V'){
      Ch1.Volt = Serial1.parseFloat();
    }
    if (first == 'A'){
      Ch1.Amp = Serial1.parseFloat();
    }
    if (first == 'n'){
      Ch1.temp1 = Serial1.parseFloat();
    }
    if (first == 'm'){
      Ch1.temp2 = Serial1.parseFloat();
    }
    if (first == 'y'){
      //for (uint8_t n = 0; n <= 9; n++){
        Serial.println(Serial1.parseFloat(), 6);
      //}
      //Serial.println('#');
    }
    if (first == 'v'){
      Serial.println(Serial1.parseFloat(),9);
      Serial.println('#');
    }else{

    }
  }
  Ch1.DataReady = 0;
  Ch1.DispData = 1;
}
*/


/*
void dipsData(){
  //digitalWrite(Debug, HIGH);
  Ch1.Volt = avgVoltCH1.reading(Ch1.Volt);          //mooving Average
  Ch1.Amp = avgAmpCH1.reading(Ch1.Amp);             //mooving Average

  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);

  tft.setTextPadding(135);
  tft.drawFloat(Ch1.Volt, 5, 125, 30, 4);

  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawFloat(Ch1.Amp, 5, 125, 60, 4);

  Ch1.Watt = Ch1.Volt * Ch1.Amp;
  tft.setTextColor(TFT_SKYBLUE, TFT_BLACK);
  tft.drawFloat(Ch1.Watt, 5, 125, 90, 4);

  //digitalWrite(Debug, LOW);
  Ch1.DispData = 0;
}
*/

void dispSetData(){
  tft.setTextDatum(TR_DATUM);
  tft.setTextPadding(100);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.drawFloat(Ch1.Vset, 3, 125, 150, 4);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawFloat(Ch1.Ipset, 4, 125, 180, 4);
  tft.drawFloat(Ch1.Inset, 4, 125, 210, 4);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  //upDateUndLine = 1;
  underLine();
}

float checkKeys(){

  const byte MAX_CHARS = 9;
  static char inputBuffer[MAX_CHARS];
  static uint8_t inputIndex = 0;
  static float floatTotal = 0;
  static uint8_t intTotal;

  char key = keypad.getKey();

  //if (key >= '0'){
    //Serial.print("key:  ");
    //Serial.println(key);
  //}
  
  //tft.setTextDatum(TL_DATUM);
  //tft.setTextPadding(135);
  //tft.drawChar(key, 10, 150, 4);
  //tft.drawNumber(factor, 10, 170, 4);

  //if (key == 0) return;

  if (key == 'H'){
    factor++;
    factor = constrain(factor, 0, 4);
    //upDateUndLine = 1;
    underLine();
    return -1.0;
  }else if (key =='G'){
    factor--;
    factor = constrain(factor, 0, 4);
    //upDateUndLine = 1;
    underLine();
    return -1.0;
  }else if (key =='E'){
    Ch1.PwSet = !Ch1.PwSet;               //PowerToggle Ch1
    if (Ch1.PwSet) Ch1.SendData('o');
    else Ch1.SendData('f');
    Serial.println("Power Toggle");
    return -1.0;
  }else if (key =='F'){
    PwSetCH2 = !PwSetCH2;               //PowerToggle Ch2
    return -1.0;
  }else if (key =='J'){
    ChannelSet = 0;
    //upDateUndLine = 1;
    underLine();
    return -1.0;
  }else if (key =='I'){
    ChannelSet = 1;
    //upDateUndLine = 1;
    underLine();
    return -1.0;
  }else if (key =='K'){
    return -1.0;
  }else if (key =='L'){
    Ch1.SendData('Y');
    Serial.println("Request Data");
    return -1.0;
  }else if (key =='M'){
    //if (digitalRead(InterrSW) == 0) mode = 1;       //Enter Cal Function
    //Serial.println("why: ");
    return -1.0;
  }else if (key == 'A'){
    SetType = 0;                        //Set V
    //upDateUndLine = 1;
    underLine();
    //Serial.println("A");
    return -1.0;

  }else if (key == 'B'){
    SetType = 1;                        //Set I+
    //upDateUndLine = 1;
    underLine();
    //Serial.println("B");
    return -1.0;

  }else if (key == 'C'){
    SetType = 2;                        //Set I-
    //upDateUndLine = 1;
    underLine();
    //Serial.println("C");
    return -1.0;

  }else if (key =='D'){
    //Serial.println("D");
    //SendDataToCh1('q', DAC1m);
    //SendDataToCh1('w', DAC1b);
    return -1.0;
  }
  
  if (key == '#') //user signal that entry has finished
  {
    //Serial.println();
    //Serial.println("entry is complete");
    floatTotal = atof(inputBuffer);  //convert buffer to a float
    //Serial.println(floatTotal, 3);
    //floatTotal = 0;
    inputIndex = 0;

    tft.setTextDatum(TR_DATUM);
    tft.setTextPadding(112);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("", 150, 120, 4);

    return floatTotal; //exit the function
  }
  if (key >= '0' && key <= '9' || key == '.') //only act on numeric or '.' keys
  {
    inputBuffer[inputIndex] = key;  //put the key value in the buffer
    if (inputIndex != MAX_CHARS - 1)
    {
      inputIndex++; //increment the array
    }
    inputBuffer[inputIndex] = '\0';  //terminate the string

    tft.setTextDatum(TR_DATUM);
    tft.setTextPadding(112);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(inputBuffer, 150, 120, 4);
    //Serial.print("inputBuffer:  ");
    //Serial.println(inputBuffer);
    return -1.0;
  }

}

void underLine(){
  tft.fillRect(0, 172, 124, 4, TFT_BLACK);
  tft.fillRect(0, 202, 124, 4, TFT_BLACK);
  tft.fillRect(0, 232, 124, 4, TFT_BLACK);
  if (ChannelSet == 0)
  {
    if (SetType == 0){
      if (factor == 0)      tft.fillRect(112, 172, 12, 4, TFT_MAGENTA);
      else if (factor == 1) tft.fillRect(98, 172, 12, 4, TFT_MAGENTA);
      else if (factor == 2) tft.fillRect(84, 172, 12, 4, TFT_MAGENTA);
      else if (factor == 3) tft.fillRect(63, 172, 12, 4, TFT_MAGENTA);
      else if (factor == 4) tft.fillRect(49, 172, 12, 4, TFT_MAGENTA);
      else Error = 1;

    }else if (SetType == 1){
      if (factor == 0)      tft.fillRect(112, 202, 12, 4, TFT_MAGENTA);
      else if (factor == 1) tft.fillRect(98, 202, 12, 4, TFT_MAGENTA);
      else if (factor == 2) tft.fillRect(84, 202, 12, 4, TFT_MAGENTA);
      else if (factor == 3) tft.fillRect(70, 202, 12, 4, TFT_MAGENTA);
      else if (factor == 4) tft.fillRect(49, 202, 12, 4, TFT_MAGENTA);
      else Error = 1;

    }else if (SetType == 2){
      if (factor == 0)      tft.fillRect(112, 232, 12, 4, TFT_MAGENTA);
      else if (factor == 1) tft.fillRect(98, 232, 12, 4, TFT_MAGENTA);
      else if (factor == 2) tft.fillRect(84, 232, 12, 4, TFT_MAGENTA);
      else if (factor == 3) tft.fillRect(70, 232, 12, 4, TFT_MAGENTA);
      else if (factor == 4) tft.fillRect(49, 232, 12, 4, TFT_MAGENTA);
      else Error = 1;
    }
    else Error = 1;
  }
  //upDateUndLine = 0;
}

void FanContr(void){
  float maxTemp = Ch1.temp1;
  if (maxTemp < Ch1.temp2) maxTemp = Ch1.temp2;
  uint16_t PWM;
  if (maxTemp <= 35.0) PWM = 0;
  else if (maxTemp > 35.0) PWM = 15.0 * maxTemp - 380.0;
  PWM = constrain(PWM, 0, 255);

  analogWrite(Fan, PWM);

  uint16_t PWMperct = PWM * 100 / 255;

  //Serial.print("PWM: ");
  //Serial.println(mode);
  //Serial.print("  perct: ");
  //Serial.println(PWMperct);

  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(60);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawNumber(PWMperct, 200, 0, 2);

  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(60);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawFloat(Ch1.temp1, 1, 0, 0, 2);
  tft.drawFloat(Ch1.temp2, 1, 60, 0, 2);
}

void processNumPad(float num){
  if (num == -1.0) return;
  
  if(ChannelSet == 0){                      //Set Ch1
    if (SetType == 0){                      //Set Volts
      num = constrain(num, 0.01, 25.0);
      Ch1.Vset = num;
      sendVolt = 1;
    }else if(SetType == 1){                 //Set I +
      num = constrain(num, 0.000001, 5.0);
      Ch1.Ipset = num;
      sendCurrP = 1;
    }else if(SetType == 2){                 //Set I +
      num = constrain(num, 0.000001, 5.0);
      Ch1.Inset = num;
      sendCurrN = 1;
    }
  }
  if(ChannelSet == 1){                      //Set Ch2

  }
}

void loop() {
  if (mode == 0) LoopPSload();
  if (mode == 1) CalMode();

}


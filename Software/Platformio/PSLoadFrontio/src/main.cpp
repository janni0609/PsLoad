  #include <Arduino.h>

  void ErrorCh1_ISR(void);
  void RotBtnISR(void);
  void DataReadyCH1(void);
  void ModeSwISR(void);
  void processNumPad(float num);
  void FanContr(void);
  float checkKeys(void);
  void shaft_moved(void);
  float factorSelI(void);
  float factorSelV(void);
  void Buzzer(uint32_t buzzMS);
  void BuzzerISR(void);


  #include "SPI.h"
  #include "TFT_eSPI.h"

  TFT_eSPI tft = TFT_eSPI();

  TFT_eSprite SprCh1 = TFT_eSprite(&tft);


  #include <InternalTemperature.h>



  float checkKeys();
  void Buzzer(uint32_t buzzMS);





  uint32_t myTime;

  const uint8_t DataIn1 = 5;
  const uint8_t DataIn2 = 3;
  const uint8_t DataOut1 = 4;
  const uint8_t DataOut2 = 2;

  const uint8_t InterrSW = 33;

  const uint8_t BuzzerPin = 39;

  const uint8_t Debug = 40;

  const uint8_t Fan = 19;

  bool Error = 0;


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

  int8_t factor = 0;
  uint8_t SetType = 0;            // 0: Vset, 1: Ipset, 2:Inset
  bool ChannelSet = 0;            // 0: CH1, 1: CH2

  volatile uint8_t mode = 0;               // 0: normla PSLoad Mode,

  bool CalVal = 0;

  //------------------------------------------------------------------------------------------------
  //Metro
  #include <Metro.h>                      //Include Metro library

  Metro FanMetro = Metro(250);            // Instanciate a metro object and set the interval to 250 milliseconds (0.25 seconds).

  //Metro IoT = Metro(1000);            // Instanciate a metro object and set the interval to 250 milliseconds (0.25 seconds).

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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


  #include "PSLoad.h"

  #include <movingAvgFloat.h>


  movingAvgFloat avgVoltCH1(10);
  movingAvgFloat avgAmpCH1(10);

  movingAvgFloat avgVoltCalCH1(50);
  movingAvgFloat avgAmpCH1Cal(50);

  movingAvgFloat* avgTypeCh1[] = { &avgVoltCH1, &avgAmpCH1, &avgVoltCalCH1, &avgAmpCH1Cal};
  //movingAvgFloat* avgTypeCh2[] = { &avgVoltCH2, &avgAmpCH2, &avgVoltCalCH2, &avgAmpCH2Cal};



  PSLoad Ch1(avgTypeCh1, &Serial1, DataOut1, DataOut2);

  //------------------------------------------------------------------------------------------------
  // Buzzer Timer
  #include <TimerThree.h>

  volatile uint32_t BzCount = 0;
  volatile uint32_t BzThr = 1000;


void setup() 
{



  Timer3.initialize(1000);
  Timer3.attachInterrupt(BuzzerISR); // blinkLED to run every 0.15 seconds
  Timer3.stop();
  Timer3.restart();
  pinMode(Debug, BuzzerPin);
  digitalWrite(BuzzerPin, LOW);



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
  attachInterrupt(DataIn2, ErrorCh1_ISR, RISING);

  pinMode(DataIn1, INPUT);
  attachInterrupt(DataIn1, DataReadyCH1, RISING);

  
  //------------------------------------------------------------------------------------------------
  // TFT
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  SprCh1.setColorDepth(16);
  SprCh1.createSprite(125, 85);
  SprCh1.fillSprite(TFT_BLACK);
  //SprCh1.pushSprite(0,0);

  //------------------------------------------------------------------------------------------------
}


void BuzzerISR(void){   //ISR
  if (BzCount == BzThr){
    digitalWrite(BuzzerPin, LOW);
    Timer3.stop();
  }
  BzCount++;
}

void Buzzer(uint32_t buzzMS){
  BzCount = 0;
  BzThr = buzzMS;
  Timer3.restart();
  digitalWrite(BuzzerPin, HIGH);
}


void LoopPSload(){

  Serial.println("enter PSLoad Func");

  Ch1.initPSLoad();

  while (mode == 0){                  //PSLoad normal mode
    if (Ch1.DataReady)  Ch1.GetData();           // ~6 us
    if (Ch1.DispData)   Ch1.dipsData();          //~4403 us


    if (sendVolt) {
      Ch1.dispSetData();
      Ch1.SendData('v', Ch1.Vset);
      sendVolt = 0;
    }
    if (sendCurrP) {
      Ch1.dispSetData();
      Ch1.SendData('i', Ch1.Ipset);
      sendCurrP = 0;
    }
    if (sendCurrN) {
      Ch1.SendData('s', Ch1.Inset);
      Ch1.dispSetData();
      sendCurrN = 0;
    }

    float keyreturn = checkKeys();                  //~ 1770 us  / or ~ 1 us
    if (keyreturn != -1.0) Serial.println(keyreturn, 4);
    processNumPad(keyreturn);


    if (FanMetro.check())  FanContr();              // 450 us

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

  }       //while loop
}         //function


float factorSelV(void){
  if (factor == 0) return 0.001;
  else if (factor == 1) return 0.01;
  else if (factor == 2) return 0.1;
  else if (factor == 3) return 1.0;
  else if (factor == 4) return 10.0;
  return -1.0;
}

float factorSelI(void){
  if (factor == 0) return 0.0001;
  else if (factor == 1) return 0.001;
  else if (factor == 2) return 0.01;
  else if (factor == 3) return 0.1;
  else if (factor == 4) return 1.0;
  return -1.0;
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

float checkKeys(){      //~ 1770 us  / or ~ 1 us
  float Return = -1.0;

  const byte MAX_CHARS = 9;
  static char inputBuffer[MAX_CHARS];
  static uint8_t inputIndex = 0;
  static float floatTotal = 0;
  //static uint8_t intTotal;

  char key = keypad.getKey();

  //Serial.println(key, HEX);

  //if (key != 0x00){
  //  Serial.print("key:  ");
  //  Serial.println(key);
  //}

  if (key == 0x00) return -1.0;
  
  switch (key){
    case 'H':
      factor++;
      factor = constrain(factor, 0, 4);
      //upDateUndLine = 1;
      Ch1.underLine();
      Return = -1.0;
      break;

    case 'G':
      factor--;
      factor = constrain(factor, 0, 4);
      //upDateUndLine = 1;
      Ch1.underLine();
      Return = -1.0;
      break;

    case 'E':
      Ch1.PwSet = !Ch1.PwSet;               //PowerToggle Ch1
      if (Ch1.PwSet) Ch1.SendData('o');
      else Ch1.SendData('f');
      Serial.println("Power Toggle");
      Return = -1.0;
      break;

    case 'F':
      Return = -1.0;               //PowerToggle Ch2
      break;

    case 'J':
      ChannelSet = 0;
      //upDateUndLine = 1;
      Ch1.underLine();
      Return = -1.0;
      break;

    case 'I':
      ChannelSet = 1;
      //upDateUndLine = 1;
      Ch1.underLine();
      Return = -1.0;
      break;

    case 'K':
      Return = -1.0;
      break;

    case 'L':
      Ch1.SendData('Y');
      Serial.println("Request Data");
      Return = -1.0;
      break;

    case 'M':
      // if (digitalRead(InterrSW) == 0) mode = 1;       // Enter Cal Function
      // Serial.println("why: ");
      Return = -1.0;
      break;

    case 'A':
      SetType = 0;                        // Set V
      // upDateUndLine = 1;
      Ch1.underLine();
      // Serial.println("A");
      Return = -1.0;
      break;

    case 'B':
      SetType = 1;                        // Set I+
      // upDateUndLine = 1;
      Ch1.underLine();
      // Serial.println("B");
      Return = -1.0;
      break;

    case 'C':
      SetType = 2;                        // Set I-
      // upDateUndLine = 1;
      Ch1.underLine();
      // Serial.println("C");
      Return = -1.0;
      break;

    case 'D':
      // Serial.println("D");
      // SendDataToCh1('q', DAC1m);
      // SendDataToCh1('w', DAC1b);
      Return = -1.0;
      break;

    case '#': // user signal that entry has finished
      // Serial.println();
      // Serial.println("entry is complete");
      floatTotal = atof(inputBuffer);  // convert buffer to a float
      // Serial.println(floatTotal, 3);
      // floatTotal = 0;
      inputIndex = 0;

      tft.setTextDatum(TR_DATUM);
      tft.setTextPadding(112);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.drawString("", 150, 120, 4);

      Return = floatTotal; // exit the function
      break;

    default:
      if ((key >= '0' && key <= '9') || key == '.') { // only act on numeric or '.' keys
        inputBuffer[inputIndex] = key;  // put the key value in the buffer
        if (inputIndex != MAX_CHARS - 1) {
          inputIndex++; // increment the array
        }
        inputBuffer[inputIndex] = '\0';  // terminate the string

        tft.setTextDatum(TR_DATUM);
        tft.setTextPadding(112);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawString(inputBuffer, 150, 120, 4);
        // Serial.print("inputBuffer:  ");
        // Serial.println(inputBuffer);

        Return = -1.0;
      }
      break;
  }

  

  return Return;
}

void FanContr(void){    // 450 us
  //uint32_t yTime = micros();

  //Serial.println(InternalTemperature.readTemperatureC(),1);


  float maxTemp = Ch1.temp1;
  if (maxTemp < Ch1.temp2) maxTemp = Ch1.temp2;
  uint16_t PWM;
  if (maxTemp <= 35.0) PWM = 0;
  else PWM = 15.0 * maxTemp - 380.0;
  PWM = constrain(PWM, 0, 255);

  analogWrite(Fan, PWM);

  //uint16_t PWMperct = PWM * 100 / 255;


  tft.setTextDatum(TR_DATUM);
  tft.setTextPadding(25);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawNumber(PWM * 100 / 255, 170, 0, 2);

  tft.setTextPadding(34);
  //tft.setTextDatum(TR_DATUM);
  tft.drawFloat(maxTemp, 1, 95, 0, 2);

  //Serial.println(tft.fontHeight(2));



  //yTime = micros() - yTime;
  //Serial.println(yTime);
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
  if (mode == 1) Ch1.CalMode();

}

void ModeSwISR(){       //ISR
  //mode = !mode;
  mode = 1;
}

void DataReadyCH1(void){  //ISR
  Ch1.DataReady = 1;
}

void RotBtnISR(){   //ISR
  //sendData = 1;
}

void ErrorCh1_ISR(void){   //ISR
  Buzzer(2000);
}
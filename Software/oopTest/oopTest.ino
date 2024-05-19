

class PSLoad
{
private:
  HardwareSerial* serialPort;
  //uint8_t DataOut_1;
  uint8_t a = 2;
public:

  
  uint8_t b;

  PSLoad();

  PSLoad(HardwareSerial* serialPort)
  {
    this->serialPort = serialPort;
    //this->DataOut_1 = DataOut_1;
  }


  void init()
  {
    serialPort->begin(115200);
  }


  void Send()
  {
    uint16_t c = a * b;
    serialPort->println(c);
    a++;
  }
 
};



PSLoad Test(&Serial);


void setup() {
  // put your setup code here, to run once:
  Test.b = 2;
}

void loop() {
  // put your main code here, to run repeatedly:
  Test.Send();
  delay(1000);
}

/*
ESP32-1732S019 ARDUINO IDE 2.3.2 ESP32-S3, 1,9 zoll 170*320 
*/
#include <TFT_eSPI.h>     // by Bodmer ver 2.5.43
//#include <NTPClient.h>    // by F.Weinberg ver 3.2.1
//#include <HTTPClient.h>   // by A.McEwen ver 2.2.0
//#include <ArduinoJson.h>  // by B.Blanchon ver 7.0.3
/*
In Linux copy this to ~/Arduino/libraries/TFT_eSPI/User_Setup.h
And in Windows to C:\Users\YOR_USER_NAME\Documents\Arduino\libraries\TFT_eSPI\User_Setup.h
*/
#define ST7789_DRIVER  
#define TFT_WIDTH 170
#define TFT_HEIGHT 320
#define TFT_MISO -1 
#define TFT_MOSI 13   
#define TFT_SCLK 12
#define TFT_CS   10 
#define TFT_DC   11 
#define TFT_RST  1 
#define TFT_BL   14
#define TFT_BACKLIGHT_ON HIGH
#define LOAD_GLCD  
#define LOAD_FONT2 
#define LOAD_FONT4 
#define LOAD_FONT6 
#define LOAD_FONT7
#define LOAD_FONT8 
#define LOAD_GFXFF 
#define SMOOTH_FONT
#define SPI_FREQUENCY  40000000

/*
// prepare WiFi
const char* ssid = "xxxxxxxxxxxxxxx";                                                 // Replace with your
const char* password = "xxxxxxxxxx";                                                  // WiFi credentials
// search your next airport here and get the ICAO code  "https://en.wikipedia.org/wiki/ICAO_airport_code"
const char* metar = "https://aviationweather.gov/api/data/metar?ids=KDEN&format=json";// KDEN = Denver
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
int wifiTimeOutCounter = 0;
#define TIMEOFFSET -3600 * 5  // no daylight saving time

// init parameter for universe simulation
int const n = 5, m = 200;
float const r = 0.1;
float x = 0, v = 0, t = 0;
*/
// definition for the screen
#define MAX_Y 170
#define MAX_X 320
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);  // Declare Sprite object "spr" with pointer to "tft" object
uint16_t palette[16] = { TFT_GREENYELLOW, TFT_NAVY, TFT_ORANGE, TFT_DARKCYAN, TFT_MAROON,
                         TFT_PURPLE, TFT_PINK, TFT_LIGHTGREY, TFT_YELLOW, TFT_BLUE,
                         TFT_GREEN, TFT_CYAN, TFT_RED, TFT_MAGENTA, TFT_BLUE, TFT_WHITE };

// set global variables for the weather information
int temperature = 0;  // °C
int dew_point = 0;    // °C
int wind_speed_knots = 0;
int pressure = 0;           // hPa
int relative_humidity = 0;  // %
int wind_speed_kmh = 0;
int data_age_min = 0;
unsigned long epochTime = 0;
unsigned long obsTime = 0;

// update parameter for the weather data
unsigned long previousMillis = 0;
#define INTERVAL 60000 * 5   // 5 min

/*
// connect to weather server and get data
void weatherData() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(metar);
    Serial.print("Sending HTTP request to: ");
    Serial.println(metar);
    int httpCode = http.GET();            // Send HTTP GET request
    if (httpCode > 0) {                   // Check if the request was successful
      DynamicJsonDocument doc(2048);      // Create JSON document (adjust the size accordingly)
      String payload = http.getString();  // Retrieve response from the website
      Serial.print("Response received: ");
      Serial.println(payload);                                     // Output METAR data to the Serial Console
      DeserializationError error = deserializeJson(doc, payload);  // Parse JSON data
      if (!error) {
        // start extracting weather data and update global variables
        temperature = doc[0]["temp"];
        dew_point = doc[0]["dewp"];
        wind_speed_knots = doc[0]["wspd"];
        pressure = doc[0]["altim"];
        obsTime = doc[0]["obsTime"];
        relative_humidity = 100 * expf(17.625f * dew_point / (243.04f + dew_point)) / expf(17.625f * temperature / (243.04f + temperature));
        wind_speed_kmh = wind_speed_knots * 1.852f;
        epochTime = timeClient.getEpochTime() - TIMEOFFSET;
        data_age_min = (epochTime - obsTime) / 60;
        // show most important data on serial
        Serial.print("Temperature: ");
        Serial.print(temperature, 0);
        Serial.println();
        Serial.print("Relative Humidity: ");
        Serial.print(relative_humidity, 0);
        Serial.println("%");
        Serial.print("Wind Speed: ");
        Serial.print(wind_speed_kmh, 0);
        Serial.println("km/h");
        Serial.print("Pressure: ");
        Serial.print(pressure, 0);
        Serial.println("hPa");
        Serial.print("Data age: ");
        Serial.print((epochTime - obsTime) / 60);
        Serial.println("min");
      } else Serial.println("Error parsing JSON data");
    } else Serial.println("Error retrieving METAR data");
    http.end();
  }
}



// this is missing in the library
String getFormattedDate() {
#define LEAP_YEAR(Y) ((Y > 0) && !(Y % 4) && ((Y % 100) || !(Y % 400)))
  unsigned long rawTime = timeClient.getEpochTime() / 86400L;
  unsigned long days = 0, year = 1970;
  uint8_t month;
  static const uint8_t monthDays[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
  while ((days += (LEAP_YEAR(year) ? 366 : 365)) <= rawTime) year++;
  rawTime -= days - (LEAP_YEAR(year) ? 366 : 365);
  for (month = 0; month < 12; month++) {
    uint8_t monthLength;
    if (month == 1) monthLength = LEAP_YEAR(year) ? 29 : 28;
    else monthLength = monthDays[month];
    if (rawTime < monthLength) break;
    rawTime -= monthLength;
  }
  String monthStr = ++month < 10 ? "0" + String(month) : String(month);
  String dayStr = ++rawTime < 10 ? "0" + String(rawTime) : String(rawTime);
  return String(dayStr) + "-" + monthStr + "-" + year;
}
*/
void setup() {
  // entry point
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("Booting...");
  // print debug infos on serial
  uint32_t chipId = 0;
  for (int i = 0; i < 17; i = i + 8) chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  Serial.printf("ESP32 Chip model = %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
  Serial.printf("This chip has %d cores\n", ESP.getChipCores());
  Serial.print("Chip ID: ");
  Serial.println(chipId);
  Serial.print("Connecting to ");
  //tft.init();
}

void loop() {
  
}
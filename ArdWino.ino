/**********************************************************************
   December 2020 Pierre Hilson

   Small project to activate relays based on temperature measurements

**********************************************************************/

#define VERSION "0.1"
#define M5STACK
//#define DEBUG_MUTEX

/* TODO
 *  Control IOT : https://nl.mathworks.com/help/thingspeak/mqtt-publish-and-subscribe-with-esp8266.html
 */

/* *************
    Includes
 ************* */
#include <Arduino.h>

#define ESP32_RTOS
//#include "OTA.h"
//#include <credentials.h>

//#include "SerialDebug.h" // https://github.com/JoaoLopesF/SerialDebug
//#include <U8g2lib.h>     // Screen library // https://p3dt.net/u8g2sim/
//#include <menu.h> //menu macros and objects
//#include <menuIO/u8g2Out.h>   // Interface to the screen
//#include <menuIO/chainStream.h>
//#include <menuIO/rotaryEventIn.h> // Excellent generic rotary event-based implementation by github@gangkast.nl

//#include <qdec.h> //https://github.com/SimpleHacks/QDEC // machine state handling encoder with sw debouncing
//#include <AceButton.h> // https://github.com/bxparks/AceButton // sw debouncing and more...

#include <DallasTemperature.h>

#include <OneWire.h>

#include <FS.h>
#include <SD.h> // TODO: Use SdFat https://github.com/greiman/SdFat/
#include <IniFile.h>

#include <ESP32Time.h>
#include <RTClib.h>

#include <M5Stack.h>
#include <Free_Fonts.h>
#define TFT_GREY 0x7BEF

#define TINY_GSM_MODEM_SIM800      // Modem is SIM800
#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb
#include <TinyGsmClient.h>
#include <PubSubClient.h>

/* *************
     Typedef
 ************* */

#define SIZE_OF_DEVICEADDRESSSTR 16
typedef char DeviceAddressStr[SIZE_OF_DEVICEADDRESSSTR+1]; // This can hold the ascii version of a DeviceAddress, hence 16 char+\0

/* *************
     Define
 ************* */
// Set serial for debug console (to Serial Monitor, default speed 115200)
#define SerialMon Serial

#define CSV_SEPARATOR ";"
#define DEBUG 1
#define MEMDBG 0

#define MAXLOGSIZE 20000000
#define MAXCSVSIZE 20000000

/* some things to help me with my code */
#define DEFAULT_BUFFER_LEN 100

/* INI file and config related values */
#define BUS_COUNT 2 // If this is changed, the DallasTemperature sensorsBuses declaration must be adapted
#define MAX_SENSORS_PER_BUS 8

/* ESP32 DEVKIT V1 Pin usage */

/* SPI connections
  SPI   MOSI    MISO    CLK     CS
  VSPI  GPIO 23 GPIO 19 GPIO 18 GPIO 5
  HSPI  GPIO 13 GPIO 12 GPIO 14 GPIO 15 */

/* VSPI: 
M5Stack: LCD & SD
Other:   SD and NRF24L01+ modules in HW SPI mode 
*/
#define VSPI_MOSI_PIN 23 // To EXP2-6 SD MOSI
#define VSPI_MISO_PIN 19 // To EXP2-1 SD MISO 
#define VSPI_SCK_PIN  18 // To EXP2-2 SD SCK

/* HSPI: ST7920 LCD driver in HW SPI mode */
/* LCD EXT1-9 : GND */
/* LCD EXT1-10: 5V */
//#ifndef M5STACK // NO HSPI Available on M5stack
//#define HSPI_MOSI_PIN 13 // To EXP1-3 = LCDE  = ST7920 5 RW(SID)
//#define HSPI_MISO_PIN 12 // 
//#define HSPI_SCK_PIN  14 // To EXP1-5 = LCD4  = ST7920 6 E(SCK) 
//#endif

/* SPI CS pins */
//#ifdef M5STACK
#define SD_CS_PIN   4  
#define LCD_CS_PIN  14 
//#else
//#define SD_CS_PIN   5  // To EXP2-4 SD CS
//#define LCD_CS_PIN  15 // To EXP1-4 = LCDRS = ST7920 4 RS(CS)
//#endif

/* I2C: RTC & future humidity sensor */
#define SDA_PIN 21
#define SCL_PIN 22

/* Relays */
//#ifdef M5STACK
#define RELAY_CLOSED HIGH
#define RELAY_OPENED LOW
#define RELAY1_PIN 15
#define RELAY2_PIN 12
////#define RELAY3_PIN 33
////#define RELAY4_PIN 32 //Used for OneWire2
//#else
//#define RELAY1_PIN 25
//#define RELAY2_PIN 26
//#endif

/* LCD Encoder */
//#define ROTARY_PIN_BUT 36 // To EXP1-2 // 
//#define ROTARY_PIN_A 34   // To EXP2-5 // 
//#define ROTARY_PIN_B 39   // To EXP2-3 // 

#define BUTTON_PIN_A 37
#define BUTTON_PIN_B 38
#define BUTTON_PIN_C 39

/* Backlight */
//#ifndef M5STACK
//#define BACKLIGHT_PIN 27
//#endif

/* GPRS SIM800L */
#define SerialAT Serial2 // The SIM800L is hooked to the 2nd hardware serial port
// Define the serial console for debug prints, if needed
//#define DUMP_AT_COMMANDS
#define GPRS_RX 16 // To SIM800L_TX
#define GPRS_TX 17 // To SIM800L_RX
// Define how you're planning to connect to the internet
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

/* OneWire */
//#ifdef M5STACK
#define ONEWIRE1_PIN 13 // I did not use GPIO2 because it must be low for flashing which the pull up made not possible
#define ONEWIRE2_PIN 5
//#else
//#define ONEWIRE1_PIN 4
//#define ONEWIRE2_PIN 32
//#endif

/* Built in LED */
//#ifndef M5STACK
//#define LEDBUILTIN_PIN 2
//#endif

/* Reset: Switch connected to the EN pin */

/* u8g2 - st7920*/
//#define fontName u8g2_font_5x7_tf // u8g2_font_7x13_mf
//#define fontX 5 // 7
//#define fontY 9 // 16
//#define offsetX 0
//#define offsetY 3
//#define U8_Width 128
//#define U8_Height 64
//#define USE_HWI2C
//#define fontMarginX 2
//#define fontMarginY 2

#define CF_OL24 &Orbitron_Light_24
#define CF_OL32 &Orbitron_Light_32

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif
    
// TinyGSM Client for Internet connection
TinyGsmClient client(modem);

//MQTT Client for connection to ThingSpeak
PubSubClient mqttClient(client);


//#define MAX_DEPTH 1 // used by ArduinoMenu https://github.com/neu-rah/ArduinoMenu

/* *************
    Classes
 ************* */

/* TempSensor Class

    Stores temperature in cent of Celcius degree

    Setter:
    - Manages a mutex
    - Keep a moving average on the last X value for a sensor.
    - Keep the max stddev on the last X value for a sensor.
    Getter:
    - Respects a mutex
    - Returns the moving avg
    - Returns the max stddev
    Reset:
    - Manages a mutex
    - Resets the moving average and stddev arrays with null values

    Notes:
    - https://github.com/JChristensen/movingAvg
*/

void convertAddress2Bytes(const DeviceAddressStr deviceAddressChars, DeviceAddress deviceAddressBytes) {
  char tmp[] = "00"; //This is actually a 3 char array with the 3rd char being \0
  for (uint8_t i = 0; i < 8; i++) {
    tmp[0] = deviceAddressChars[i * 2];
    tmp[1] = deviceAddressChars[i * 2 + 1];
    deviceAddressBytes[i] = strtol(tmp, NULL, 16); // store the numeric value
  }
}

void convertAddress2Chars(const DeviceAddress deviceAddressBytes, DeviceAddressStr deviceAddressChars) {
  deviceAddressChars[0] = '\0';
  for (uint8_t i = 0; i < 8; i++) {
    char buf[3];
    sprintf(buf, "%02X", deviceAddressBytes[i]);
    strcat(deviceAddressChars, buf);
  }
  deviceAddressChars[16] = '\0';
}

class ThingSpeakClient {
  private:
    SemaphoreHandle_t modemMutex = NULL;
    SemaphoreHandle_t dataMutex = NULL;
    uint32_t _gsmBaudRate = 115200; // Set to 0 for Auto-Detect
    char _simPIN[10];
    char _apn[60];
    char _apnUser[20];
    char _apnPwd[20];
    char _mqttBroker[40];
    char _mqttUser[40];
    char _mqttPwd[40];
    volatile bool _isNetworkConnected = false;
    volatile bool _isSimUnlocked = false;
    volatile bool _isGprsConnected = false;
    volatile bool _isMqttClientConnected = false;
    volatile uint32_t _signalQuality = 0;
    
    //TinyGsm _modem;
    //TinyGsmClient _client;
    //PubSubClient _mqttClient;
    char alphanum[63] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";  // For random generation of client ID.
    
  public:
    void begin(const char* simPIN, const char* apn, const char* apnUser,
                       const char* apnPwd, const char* mqttBroker, 
                       const char* mqttUser, const char* mqttPwd) {
      this->modemMutex = xSemaphoreCreateMutex();
      this->dataMutex = xSemaphoreCreateMutex();
      strcpy(this->_simPIN, simPIN);
      strcpy(this->_apn, apn);
      strcpy(this->_apnUser, apnUser);
      strcpy(this->_apnPwd, apnPwd);
      strcpy(this->_mqttBroker, mqttBroker);
      strcpy(this->_mqttUser, mqttUser);
      strcpy(this->_mqttPwd, mqttPwd);
      xSemaphoreTake( this->modemMutex, portMAX_DELAY );
      if (!this->_gsmBaudRate) {
        SerialMon.println("Getting gsmBaudRate");
        this->_gsmBaudRate = TinyGsmAutoBaud(SerialAT);
        SerialMon.print("gsmBaudRate: ");
        SerialMon.println(this->_gsmBaudRate); //115200
      }
      xSemaphoreGive( this->modemMutex );
      if (!this->_gsmBaudRate) {
        SerialMon.println(F("***********************************************************"));
        SerialMon.println(F(" Module does not respond!"));
        SerialMon.println(F("   Check your Serial wiring"));
        SerialMon.println(F("   Check the module is correctly powered and turned on"));
        SerialMon.println(F("***********************************************************"));
        delay(30000L);
        this->_gsmBaudRate=115200; //Force the baudrate to 115200
      }
      
      // Set GSM module baud rate and UART pins
      SerialMon.println("Initializing modem serial interface");
      SerialAT.begin(this->_gsmBaudRate);
      //SerialAT.begin(115200, SERIAL_8N1, RXD2, TXD2);
      delay(6000);
    }

    bool isGprsConnected() {
      bool isGprsConnected;
      xSemaphoreTake( this->dataMutex, ( TickType_t ) 100 );
      isGprsConnected=this->_isGprsConnected;
      xSemaphoreGive( this->dataMutex );
      return isGprsConnected;
    }
    void setIsGprsConnected(bool isGprsConnected) {
      xSemaphoreTake( this->dataMutex, ( TickType_t ) 100 );
      this->_isGprsConnected = isGprsConnected;
      xSemaphoreGive( this->dataMutex ); 
    }

    bool isNetworkConnected() {
      bool isNetworkConnected;
      xSemaphoreTake( this->dataMutex, ( TickType_t ) 100 );
      isNetworkConnected=this->_isNetworkConnected;
      xSemaphoreGive( this->dataMutex );
      return isNetworkConnected;
    }
    void setIsNetworkConnected(bool isNetworkConnected) {
      xSemaphoreTake( this->dataMutex, ( TickType_t ) 100 );
      this->_isNetworkConnected = isNetworkConnected;
      xSemaphoreGive( this->dataMutex ); 
    }

    bool isSimUnlocked() {
      bool isSimUnlocked;
      xSemaphoreTake( this->dataMutex, ( TickType_t ) 100 );
      isSimUnlocked=this->_isSimUnlocked;
      xSemaphoreGive( this->dataMutex );
      return isSimUnlocked;
    }
    void setIsSimUnlocked(bool isSimUnlocked) {
      xSemaphoreTake( this->dataMutex, ( TickType_t ) 100 );
      this->_isSimUnlocked = isSimUnlocked;
      xSemaphoreGive( this->dataMutex ); 
    }

    bool isMqttClientConnected() {
      bool isMqttClientConnected;
      xSemaphoreTake( this->dataMutex, ( TickType_t ) 100 );
      isMqttClientConnected=this->_isMqttClientConnected;
      xSemaphoreGive( this->dataMutex );
      return isMqttClientConnected;
    }
    void setIsMqttClientConnected(bool isMqttClientConnected) {
      xSemaphoreTake( this->dataMutex, ( TickType_t ) 100 );
      this->_isMqttClientConnected = isMqttClientConnected;
      xSemaphoreGive( this->dataMutex ); 
    }

    uint32_t getSignalQuality() {
      uint32_t signalQuality;
      xSemaphoreTake( this->dataMutex, ( TickType_t ) 100 );
      signalQuality=this->_signalQuality;
      xSemaphoreGive( this->dataMutex );
      return signalQuality;
    }
    void setSignalQuality(uint32_t signalQuality) {
      xSemaphoreTake( this->dataMutex, ( TickType_t ) 100 );
      this->_signalQuality = signalQuality;
      xSemaphoreGive( this->dataMutex ); 
    }

    void getNetworkTime(int* year, int* month, int* day, int* hour, int* minute,
                      int* second, float* timezone) {
      if (xSemaphoreTake( this->modemMutex, ( TickType_t ) 100 ) == pdTRUE ) {
        modem.getGSMDateTime(DATE_FULL);
        modem.getNetworkTime(year, month, day, hour, minute, second, timezone);
        xSemaphoreGive( this->modemMutex );                           
      }
    }

    void check() {
      xSemaphoreTake( this->modemMutex, portMAX_DELAY );
      this->setIsNetworkConnected(modem.isNetworkConnected());
      this->setIsSimUnlocked((modem.getSimStatus() == 1));
      this->setIsGprsConnected(modem.isGprsConnected());
      this->setSignalQuality(modem.getSignalQuality());
      xSemaphoreGive( this->modemMutex );
    }
    
    bool reconnect(bool includeMqtt) {
      xSemaphoreTake( this->modemMutex, portMAX_DELAY );
      SerialMon.println("Reconnecting MQTT");
      
      char clientID[9];
      
      if (!modem.isNetworkConnected()) {
        SerialMon.println("Network not connected. Initializing modem...");
        this->setIsNetworkConnected(false);
        this->setIsGprsConnected(false);
        this->setIsMqttClientConnected(false);
        //modem.init();
        modem.restart();
      
        // Unlock your SIM card with a PIN if needed
        if (strlen(this->_simPIN) && modem.getSimStatus() != 3 ) {
          SerialMon.println("Unlocking SIM.");
          modem.simUnlock(this->_simPIN);
        }
        if( modem.getSimStatus() != 1 ){
          this->setIsSimUnlocked(false);
        }
        else {
          this->setIsSimUnlocked(true);
        }

        SerialMon.print("Waiting for network...");
        if (!modem.waitForNetwork()) {
          SerialMon.println(" fail");
          this->setIsNetworkConnected(false);
          delay(10000);
          xSemaphoreGive( this->modemMutex );
          return false;
        }
        SerialMon.println(" success");
        this->setIsNetworkConnected(true);
      
        if (modem.isNetworkConnected()) {
          SerialMon.println("Network connected");
          this->setIsNetworkConnected(true);
        }
        else {
          SerialMon.println("Network not connected !?!");
          xSemaphoreGive( this->modemMutex );
          this->setIsNetworkConnected(false);
          return false;
        }
      }
      if (!modem.isGprsConnected()) {
        SerialMon.print("Connecting to APN (GPRS): ");
        SerialMon.println(this->_apn);
        this->setIsGprsConnected(false);
        if (!modem.gprsConnect(this->_apn, this->_apnUser, this->_apnPwd)) {
          SerialMon.println(" fail");
          delay(10000);  
          xSemaphoreGive( this->modemMutex );
          return false;
        }
        else {
          SerialMon.println(" OK");
          this->setIsGprsConnected(true);
        }
        
        if (modem.isGprsConnected()) {
          SerialMon.println("GPRS connected");
        }
        else {
          xSemaphoreGive( this->modemMutex );
          return false;
        }
      }
            
      if (includeMqtt && !mqttClient.connected()) {
        this->setIsMqttClientConnected(false);
        mqttClient.setServer(this->_mqttBroker, 1883);   // Set the MQTT broker details.
        Serial.print("Attempting MQTT connection...");
        // Generate ClientID
        for (int i = 0; i < 8; i++) {
            clientID[i] = this->alphanum[random(51)];
        }
        clientID[8]='\0';

        // Connect to the MQTT broker.

        if (mqttClient.connect(clientID, this->_mqttUser, this->_mqttPwd)) 
        {
          Serial.println("connected");
          this->setIsMqttClientConnected(true);
        } else 
        {
          Serial.print("failed, rc=");
          // Print reason the connection failed.
          // See https://pubsubclient.knolleary.net/api.html#state for the failure code explanation.
          Serial.print(mqttClient.state());
          Serial.println(" try again in 5 seconds");
          this->setIsMqttClientConnected(false);
          xSemaphoreGive( this->modemMutex );
          return false;
        }
      }
      this->setIsMqttClientConnected(true);
      xSemaphoreGive( this->modemMutex );
      return true;
    }

    void mqttPublishFeed(const char * topicStr, const char * msgStr) {
      /// Create data string to send to ThingSpeak.
      ///String data = String("field1=1&field2=2&field3=3");
      //String data = "field1=" + String(random(-5, 25)) + "&field2=" + String(random(-5, 25)) + "&field3=" + String(random(-5, 25));
      //int length = data.length();
      //const char *msgBuffer;
      //msgBuffer=data.c_str();
      //Serial.println(msgBuffer);
      
      /// Create a topic string and publish data to ThingSpeak channel feed. 
      //String topicString = "channels/" + String( this->_thingSpeakChannelId ) + "/publish/"+String(this->_writeApiKey);
      //length = topicString.length();
      //const char *topicBuffer;
      //topicBuffer = topicString.c_str();
      xSemaphoreTake( this->modemMutex, 5000/portTICK_PERIOD_MS );
      mqttClient.publish( topicStr, msgStr );
      xSemaphoreGive( this->modemMutex );
    }
};

class TempSensor {
  private:
    SemaphoreHandle_t mutex = NULL;
    uint8_t _sensorBus;
    uint8_t _sensorIndex;
    int _errorHundredth=0;
    bool _isValid = false;
    bool _hasAlert = false;
    char _alertChar[40] = "";
    char _decimalSeparator='.';
    uint8_t _thingspeakField;
    //char _sensorAddressChar[DEFAULT_BUFFER_LEN];
    DeviceAddressStr _sensorAddressChar;
    DeviceAddress _sensorAddressBytes;
    volatile int _tempHundredthCelcius, _stdDevHundredth;
  public:
    TempSensor(uint8_t sensorBus, uint8_t sensorIndex, char *sensorAddressChar, char decimalSeparator, int errorHundredth, int thingspeakField) {
//    TempSensor(uint8_t sensorBus, uint8_t sensorIndex, char *sensorAddressChar) {
      this->mutex = xSemaphoreCreateMutex();
      this->_sensorBus = sensorBus;
      this->_sensorIndex = sensorIndex;
      this->_errorHundredth = errorHundredth;
      this->_thingspeakField = thingspeakField;
      this->_decimalSeparator = decimalSeparator;
      strcpy(this->_sensorAddressChar, sensorAddressChar);
      if (strcmp(this->_sensorAddressChar, "") == 0) {
        SerialMon.println("Dymmy Sensor Object");
      }
      else {
        convertAddress2Bytes(this->_sensorAddressChar, this->_sensorAddressBytes);
        //SerialMon.printf("*sensorAddressChar: %s\n", sensorAddressChar);
        //SerialMon.printf("this->_sensorAddressChar: %s\n", this->_sensorAddressChar);
      }
      //this->setTemperature(65); //Initialised to 65°C which is an impossible value
      this->_stdDevHundredth=0;
    }
    void getIdentificationStr(char *buffer) { // The buffer should be created in the calling code as 'char foo[20]'
      sprintf(buffer, "[%d-%d]%s", this->_sensorBus, this->_sensorIndex, this->_sensorAddressChar);
    }
    
    void getAddress(DeviceAddress addr) { // The buffer should be created in the calling code as 'DeviceAddress foo'
      //DeviceAddress addr;
      for (uint8_t i=0 ; i<8 ; i++ ){
         addr[i] = this->_sensorAddressBytes[i];
      }
    }
    
    float getTemperature() {  
      float temperature;
      xSemaphoreTake( this->mutex, portMAX_DELAY );
      temperature = ((float)this->_tempHundredthCelcius) / 100;
      xSemaphoreGive( this->mutex );
      return temperature;
    }

    uint8_t getThingspeakField() {  
      return this->_thingspeakField;
    }
    
    void getTemperatureStr(char *buffer) { // The buffer should be created in the calling code as 'char foo[20]'
      int tempHundredthCelcius;
      xSemaphoreTake( this->mutex, portMAX_DELAY );
      tempHundredthCelcius = this->_tempHundredthCelcius;
      xSemaphoreGive( this->mutex );
      if (tempHundredthCelcius != -12700) {
        int hundredth = abs(tempHundredthCelcius) % 100;
        int degreesC = (abs(tempHundredthCelcius) - hundredth)/100;
        if (tempHundredthCelcius < 0) {
          sprintf(buffer, "-%d%c%02d", degreesC, this->_decimalSeparator, hundredth);
        }
        else {
          sprintf(buffer, "%d%c%02d", degreesC, this->_decimalSeparator, hundredth);
        }
      }
      else {
        sprintf(buffer, "");
      }
    }
    void setUncorrectedTemperature(float temperature) {
      xSemaphoreTake( this->mutex, portMAX_DELAY );
      this->_tempHundredthCelcius = (int)(temperature * 100) + this->_errorHundredth;
      this->_isValid = true;
      xSemaphoreGive( this->mutex );
    }
    void setTemperature(float temperature) {
      xSemaphoreTake( this->mutex, portMAX_DELAY );
      this->_tempHundredthCelcius = (int)(temperature * 100);
      this->_isValid = true;
      xSemaphoreGive( this->mutex );
    }
    float getStdDev() {
      float stdDev;
      xSemaphoreTake( this->mutex, portMAX_DELAY );
      stdDev = ((float)this->_stdDevHundredth) / 100;
      xSemaphoreGive( this->mutex );
      return stdDev;
    }
    void getStdDevStr(char *buffer) { // The buffer should be created in the calling code as 'char foo[20]'
      float stdDev;
      xSemaphoreTake( this->mutex, portMAX_DELAY );
      stdDev = ((float)this->_stdDevHundredth) / 100;
      xSemaphoreGive( this->mutex );
      int ival = (int)stdDev;
      int frac = (stdDev - ival) * 100;
      sprintf(buffer, "%d%c%02d", ival, this->_decimalSeparator, frac);
    }
    void setStdDev(float stdDev) {
      xSemaphoreTake( this->mutex, portMAX_DELAY );
      this->_stdDevHundredth = (int)(stdDev * 100);
      this->_isValid = true;
      xSemaphoreGive( this->mutex );
    }
};
class Config {
  private:
    int _tzOffset;
    char* _iniFilename;
    bool _uploadToCloudEnabled=false;
    uint8_t _groundSensorBus, _groundSensorIndex, _skySensorBus, _skySensorIndex, _relayIndex, _uploadHour, _uploadMinute;
    uint16_t _minStateDurationSec, _csvFrequencySec;
    float _groundFreezingTemp, _minTempForBlowing;
    TempSensor * _tempSensors[BUS_COUNT][MAX_SENSORS_PER_BUS]; // Create an array of POINTERS to TempSensor objects
                                            // Values of the pointers will be retreived with 'new TempSensor(...);'
                                            // Calls to the object will need to dereference the pointer !
    char _decimalSeparator;
    char _simPin[20];
    char _apn[40];
    char _apnUser[20];
    char _apnPwd[20];
    char _mqttBroker[40];
    char _mqttUser[40];
    char _mqttPwd[40];
    char _thingSpeakDataChannelId[20];
    char _dataWriteApiKey[40];
    char _thingSpeakDebugChannelId[20];
    char _debugWriteApiKey[40];
  public:
    Config(char* iniFilename) {
      this->_tzOffset = 2;
      this->_iniFilename = iniFilename;
    }

    char simPIN[20] = "";
    char apn[40] = "";
    char apnUser[20] = "";
    char apnPwd[20] = "";
    char mqttBroker[40] = "";
    char mqttUser[40] = "";
    char mqttPwd[40] = "";
    char thingSpeakDataChannelId[20] = "";
    char dataWriteApiKey[40] = "";
    char thingSpeakDebugChannelId[20] = "";
    char debugWriteApiKey[40] = "";
    
    void getSimPin(char *buffer) { // The buffer should be created in the calling code as 'char foo[10]'
      strcpy(buffer, this->_simPin);
    }
    void getApn(char *buffer) { // The buffer should be created in the calling code as 'char foo[10]'
      strcpy(buffer, this->_apn);
    }
    void getApnUser(char *buffer) { // The buffer should be created in the calling code as 'char foo[10]'
      strcpy(buffer, this->_apnUser);
    }
    void getApnPwd(char *buffer) { // The buffer should be created in the calling code as 'char foo[10]'
      strcpy(buffer, this->_apnPwd);
    }
    void getMqttBroker(char *buffer) { // The buffer should be created in the calling code as 'char foo[10]'
      strcpy(buffer, this->_mqttBroker);
    }
    void getMqttUser(char *buffer) { // The buffer should be created in the calling code as 'char foo[10]'
      strcpy(buffer, this->_mqttUser);
    }
    void getMqttPwd(char *buffer) { // The buffer should be created in the calling code as 'char foo[10]'
      strcpy(buffer, this->_mqttPwd);
    }
    void getDataWriteApiKey(char *buffer) { // The buffer should be created in the calling code as 'char foo[10]'
      strcpy(buffer, this->_dataWriteApiKey);
    }
    void getDebugWriteApiKey(char *buffer) { // The buffer should be created in the calling code as 'char foo[10]'
      strcpy(buffer, this->_debugWriteApiKey);
    }
    void getThingSpeakDataChannelId(char *buffer) { // The buffer should be created in the calling code as 'char foo[10]'
      strcpy(buffer, this->_thingSpeakDataChannelId);
    }
    void getThingSpeakDebugChannelId(char *buffer) { // The buffer should be created in the calling code as 'char foo[10]'
      strcpy(buffer, this->_thingSpeakDebugChannelId);
    }

    TempSensor * getTempSensor(uint8_t busIndex, uint8_t sensorIndex){
      return this->_tempSensors[busIndex][sensorIndex];
    }

    char getDecimalSeparator(){
      return this->_decimalSeparator;
    }
    uint8_t getRelayIndex(){
      return this->_relayIndex;
    }

    uint8_t getRelayPin(){
      uint8_t relay_pin ;
      switch (this->_relayIndex) {
        case 1:
          relay_pin = RELAY1_PIN;
          break;
        case 2:
          relay_pin = RELAY2_PIN;
          break;
        /*case 3:
          relay_pin = RELAY3_PIN;
          break;*/
        default:
          relay_pin = RELAY1_PIN;
          break;
      }
      return relay_pin;
    }

    uint16_t getMinStateDurationSec(){
      return this->_minStateDurationSec;
    }

    uint16_t getCsvFrequencySec(){
      return this->_csvFrequencySec;
    }
    
    TempSensor * getGroundSensor(){
      return this->getTempSensor(this->_groundSensorBus - 1, this->_groundSensorIndex - 1);
    }

    TempSensor * getSkySensor(){
      //SerialMon.printf("DEBUUUUG: %d - %d\n", this->_skySensorBus - 1, this->_skySensorIndex - 1);
      return this->getTempSensor(this->_skySensorBus - 1, this->_skySensorIndex - 1);
    }

    float getGroundFreezingTemp(){
      return this->_groundFreezingTemp;
    }

    float getMinTempForBlowing(){
      return this->_minTempForBlowing;
    }

    bool isUploadToCloudEnabled(){
      return this->_uploadToCloudEnabled;
    }

    void getUploadTimeStr(char *buffer) { // The buffer should be created in the calling code as 'char foo[10]'
      sprintf(buffer, "%02d:%02d", this->_uploadHour, this->_uploadMinute);
    }

    uint8_t getUploadHour() {
      return this->_uploadHour;
    }
  
    uint8_t getUploadMinute() {
      return this->_uploadMinute;
    }

    void printConfig() {
      char identificationStr[DEFAULT_BUFFER_LEN];
      SerialMon.println("Configuration:");
      SerialMon.printf("MinStateDurationSec = %d\n", this->_minStateDurationSec);
      SerialMon.printf("GroundFreezingTemp  = %f\n", this->_groundFreezingTemp);
      SerialMon.printf("MinTempForBlowing   = %f\n", this->_minTempForBlowing);
      SerialMon.printf("GroundSensorBus     = %d\n", this->_groundSensorBus);
      SerialMon.printf("GroundSensorIndex   = %d\n", this->_groundSensorIndex);
      SerialMon.printf("SkySensorBus        = %d\n", this->_skySensorBus);
      SerialMon.printf("SkySensorIndex      = %d\n", this->_skySensorIndex);
      SerialMon.printf("RelayIndex          = %d\n", this->_relayIndex);
      SerialMon.printf("CsvFrequencySec     = %d\n", this->_csvFrequencySec);
      SerialMon.printf("DecimalSeparator    = %c\n", this->_decimalSeparator);
      SerialMon.printf("UploadToCloudEnabled= %d\n", this->_uploadToCloudEnabled);
      SerialMon.printf("UploadTime          = %02d:%02d\n", this->_uploadHour, this->_uploadMinute);
      SerialMon.printf("SimPin              = %s\n", this->_simPin);
      SerialMon.printf("Apn                 = %s\n", this->_apn);
      SerialMon.printf("ApnUser             = %s\n", this->_apnUser);
      SerialMon.printf("ApnPwd              = %s\n", this->_apnPwd);
      SerialMon.printf("MqttBroker          = %s\n", this->_mqttBroker);
      SerialMon.printf("MqttUser            = %s\n", this->_mqttUser);
      SerialMon.printf("MqttPwd             = %s\n", this->_mqttPwd);
      SerialMon.printf("DataWriteApiKey     = %s\n", this->_dataWriteApiKey);
      SerialMon.printf("DebugWriteApiKey    = %s\n", this->_debugWriteApiKey);
      SerialMon.printf("ThingSpeakDataChannelId = %s\n", this->_thingSpeakDataChannelId);
      SerialMon.printf("ThingSpeakDebugChannelId = %s\n", this->_thingSpeakDebugChannelId);
      SerialMon.println("Sensors:");
      for (uint8_t bus_index=0;bus_index<BUS_COUNT;bus_index++){
        for (uint8_t sensor_index=0;sensor_index<MAX_SENSORS_PER_BUS;sensor_index++){
          //SerialMon.printf("getIdentification: bus:%d sensor:%d\n", bus_index, sensor_index);
          if (this->_tempSensors[bus_index][sensor_index] != NULL) {
            this->_tempSensors[bus_index][sensor_index]->getIdentificationStr(identificationStr);
            SerialMon.printf("   %s\n", identificationStr);
          }
        }
      }
    }
    boolean readFromIni() {
      const size_t bufferLen = DEFAULT_BUFFER_LEN;
      char buffer[bufferLen];
      char buffer2[bufferLen];
      // read OneWire1 sensor addresses
      // read OneWire2 sensor addresses
      // read groundSensorAddress
      // read skySensorAddress
      // read groundFreezingTempCHundredth
      // read skyGroundMinDeltaTempCHundredth
      // read minRelayStateDuration
      //SerialMon.print("Reading ini file: ");
      //SerialMon.println(this->_iniFilename);
      SerialMon.printf("Reading ini file: %s\n", this->_iniFilename);
      IniFile ini(this->_iniFilename);
      if (!ini.open()) {
        SerialMon.print("Ini file ");
        SerialMon.print(this->_iniFilename);
        SerialMon.println(" does not exist");
        return false;
      }
      SerialMon.println("Ini file exists");

      char sectionName[DEFAULT_BUFFER_LEN];
      char keyName[DEFAULT_BUFFER_LEN];

      char fooChar; //dummy char variable to help the scanf to match real numbers https://www.carnetdumaker.net/articles/pour-lamour-du-c-nutilisez-pas-les-fonctions-atoi-atol-atof-et-derivees/
      int iValue, iValue2, iValue3;
      char cValue;
      float fValue;

      this->_readFromIni(ini, "reporting", "decimal_separator", buffer, bufferLen);
      if (sscanf(buffer, "%c%c", &cValue, &fooChar) != 1) {
          SerialMon.println("FATAL: Value should be , or .");
          return false;
      }
      this->_decimalSeparator=char(cValue);
      
      for (uint8_t bus_index=0;bus_index<BUS_COUNT;bus_index++){
        for (uint8_t sensor_index=0;sensor_index<MAX_SENSORS_PER_BUS;sensor_index++){
          sprintf(sectionName, "sensor_bus_%i", bus_index+1);

          sprintf(keyName, "sensor_address_%i", sensor_index+1);
          if (this->_readFromIni(ini, sectionName, keyName, buffer, bufferLen)) {
            if (strlen(buffer) != SIZE_OF_DEVICEADDRESSSTR) {
              SerialMon.printf("%s %s must have a length of %d characters and has a size of %d characters.\n", sectionName, keyName, SIZE_OF_DEVICEADDRESSSTR, strlen(buffer));
              SerialMon.println("  Sensor is ignored.");
            }
            else {
              sprintf(keyName, "sensor_error_hundredth_%i", sensor_index+1);
              if (this->_readFromIni(ini, sectionName, keyName, buffer2, bufferLen)) {
                if (sscanf(buffer2, "%d%c", &iValue, &fooChar) != 1) {
                  SerialMon.printf("%s %s should be an integer\n", sectionName, keyName);
                  SerialMon.println("  Sensor is ignored.");
                }
                else {              
                  sprintf(keyName, "sensor_thingspeak_field_%i", sensor_index+1);
                  if (this->_readFromIni(ini, sectionName, keyName, buffer2, bufferLen)) {
                    if (sscanf(buffer2, "%d%c", &iValue3, &fooChar) != 1) {
                      SerialMon.printf("%s %s should be an integer\n", sectionName, keyName);
                      SerialMon.println("  Sensor is ignored.");
                    }
                    else {              
                      SerialMon.printf("Instanciating Sensor object for %s [%d] %s [%d] addr:%s err:%d field:%d\n",sectionName, bus_index, keyName, sensor_index, buffer, iValue, iValue3);
                      this->_tempSensors[bus_index][sensor_index] = new TempSensor(bus_index+1, sensor_index+1, buffer, this->_decimalSeparator, iValue, iValue3); // The 'new' gives a reference of a new TempSensor object
                    }
                  }
                }
              }
            }
          }
        }
      }
      
      this->_readFromIni(ini, "ground", "sensor_bus", buffer, bufferLen);
      if (sscanf(buffer, "%d%c", &iValue, &fooChar) != 1) {
          SerialMon.println("FATAL: Value should be an integer");
          return false;
      }
      this->_groundSensorBus=uint8_t(iValue);

      this->_readFromIni(ini, "ground", "sensor_index", buffer, bufferLen);
      if (sscanf(buffer, "%d%c", &iValue, &fooChar) != 1) {
          SerialMon.println("FATAL: Value should be an integer");
          return false;
      }
      this->_groundSensorIndex=uint8_t(iValue);

      this->_readFromIni(ini, "ground", "freezing_temp", buffer, bufferLen);
      if (sscanf(buffer, "%f%c", &fValue, &fooChar) != 1) {
          SerialMon.println("FATAL: Value should be a float");
          return false;
      }
      this->_groundFreezingTemp=float(fValue);

      this->_readFromIni(ini, "sky", "sensor_bus", buffer, bufferLen);
      if (sscanf(buffer, "%d%c", &iValue, &fooChar) != 1) {
          SerialMon.println("FATAL: Value should be an integer");
          return false;
      }
      this->_skySensorBus=uint8_t(iValue);

      this->_readFromIni(ini, "sky", "sensor_index", buffer, bufferLen);
      if (sscanf(buffer, "%d%c", &iValue, &fooChar) != 1) {
          SerialMon.println("FATAL: Value should be an integer");
          return false;
      }
      this->_skySensorIndex=uint8_t(iValue);

      this->_readFromIni(ini, "sky", "minimum_temp_for_blowing", buffer, bufferLen);
      if (sscanf(buffer, "%f%c", &fValue, &fooChar) != 1) {
          SerialMon.println("FATAL: Value should be a float");
          return false;
      }
      this->_minTempForBlowing=float(fValue);

      this->_readFromIni(ini, "relays", "index", buffer, bufferLen);
      if (sscanf(buffer, "%d%c", &iValue, &fooChar) != 1) {
          SerialMon.println("FATAL: Value should be an integer");
          return false;
      }
      this->_relayIndex=uint8_t(iValue);

      this->_readFromIni(ini, "relays", "min_state_duration_seconds", buffer, bufferLen);
      if (sscanf(buffer, "%d%c", &iValue, &fooChar) != 1) {
          SerialMon.println("FATAL: Value should be an integer");
          return false;
      }
      this->_minStateDurationSec=uint16_t(iValue);

      this->_readFromIni(ini, "reporting", "csv_frequency_seconds", buffer, bufferLen);
      if (sscanf(buffer, "%d%c", &iValue, &fooChar) != 1) {
          SerialMon.println("FATAL: Value should be an integer");
          return false;
      }
      this->_csvFrequencySec=uint16_t(iValue);
      
      this->_readFromIni(ini, "reporting", "upload_to_cloud", buffer, bufferLen);
      if (sscanf(buffer, "%d%c", &iValue, &fooChar) != 1) {
          SerialMon.println("FATAL: Value should be an integer");
          return false;
      }
      if (iValue>0) {
        this->_uploadToCloudEnabled=true;
      }
      else {
        this->_uploadToCloudEnabled=false;
      }

      /* this->_readFromIni(ini, "reporting", "upload_time", buffer, bufferLen);
      if (sscanf(buffer, "%d:%d%c", &iValue, &iValue2, &fooChar) != 2) {
          SerialMon.println("FATAL: Value should be something like 10:20");
          return false;
      }
      this->_uploadHour=uint8_t(iValue);
      this->_uploadMinute=uint8_t(iValue2);
      */

      if (this->_readFromIni(ini, "reporting", "sim_pin", buffer, bufferLen)) {
        strcpy(this->_simPin, buffer);
      }    
      if (this->_readFromIni(ini, "reporting", "apn", buffer, bufferLen)) {
        strcpy(this->_apn, buffer);
      }    
      if (this->_readFromIni(ini, "reporting", "apn_user", buffer, bufferLen)) {
        strcpy(this->_apnUser, buffer);
      }    
      if (this->_readFromIni(ini, "reporting", "apn_pwd", buffer, bufferLen)) {
        strcpy(this->_apnPwd, buffer);
      }    
      if (this->_readFromIni(ini, "reporting", "mqtt_broker", buffer, bufferLen)) {
        strcpy(this->_mqttBroker, buffer);
      }    
      if (this->_readFromIni(ini, "reporting", "mqtt_user", buffer, bufferLen)) {
        strcpy(this->_mqttUser, buffer);
      }    
      if (this->_readFromIni(ini, "reporting", "mqtt_pwd", buffer, bufferLen)) {
        strcpy(this->_mqttPwd, buffer);
      }    
      if (this->_readFromIni(ini, "reporting", "data_write_api_key", buffer, bufferLen)) {
        strcpy(this->_dataWriteApiKey, buffer);
      }    
      if (this->_readFromIni(ini, "reporting", "debug_write_api_key", buffer, bufferLen)) {
        strcpy(this->_debugWriteApiKey, buffer);
      }    
      if (this->_readFromIni(ini, "reporting", "thingspeak_data_channel_id", buffer, bufferLen)) {
        strcpy(this->_thingSpeakDataChannelId, buffer);
      }    
      if (this->_readFromIni(ini, "reporting", "thingspeak_debug_channel_id", buffer, bufferLen)) {
        strcpy(this->_thingSpeakDebugChannelId, buffer);
      }    

      ini.close();
      return true;
    }
    
    bool _readFromIni(const IniFile ini, const char* section, const char* key, char* buffer, size_t len)
    {
      if (ini.getValue(section, key, buffer, len)) {
        SerialMon.printf("[%s] %s=%s\n", section, key, buffer);
      }
      else {
        SerialMon.printf("[%s] %s : ", section, key);
        this->_printErrorMessage(ini.getError());
        return false;
      }
      return true;
    }
    
    void _printErrorMessage(uint8_t e, bool eol = true)
    {
      switch (e) {
        case IniFile::errorNoError:
          SerialMon.print("no error");
          break;
        case IniFile::errorFileNotFound:
          SerialMon.print("file not found");
          break;
        case IniFile::errorFileNotOpen:
          SerialMon.print("file not open");
          break;
        case IniFile::errorBufferTooSmall:
          SerialMon.print("buffer too small");
          break;
        case IniFile::errorSeekError:
          SerialMon.print("seek error");
          break;
        case IniFile::errorSectionNotFound:
          SerialMon.print("section not found");
          break;
        case IniFile::errorKeyNotFound:
          SerialMon.print("key not found");
          break;
        case IniFile::errorEndOfFile:
          SerialMon.print("end of file");
          break;
        case IniFile::errorUnknownError:
          SerialMon.print("unknown error");
          break;
        default:
          SerialMon.print("unknown error value");
          break;
      }
      if (eol)
        SerialMon.println();
    }
    //    [sensor_bus1]
    //    sensor_address_1=28F1DC4302000003
    //    sensor_address_2=28F1DC4302000004
    //    sensor_address_3=28F1DC4302000005
    //    sensor_address_4=28F1DC4302000006
    //    sensor_address_5=28F1DC4302000007
    //    sensor_address_6=28F1DC4302000008
    //    sensor_address_7=28F1DC4302000009
    //    [sensor_bus2]
    //    sensor_address_1=28F2DC4302000003
    //    sensor_address_2=28F2DC4302000004
    //    sensor_address_3=28F2DC4302000005
    //    sensor_address_4=28F2DC4302000006
    //    sensor_address_5=28F2DC4302000007
    //    sensor_address_6=28F2DC4302000008
    //    sensor_address_7=28F2DC4302000009
    //    [ground]
    //    sensor_bus=1
    //    sensor_index=3
    //    freezing_temp=0.5
    //    [sky]
    //    sensor_bus=1
    //    sensor_index=5
    //    minimum_temp=0.5
    //    [relays]
    //    index=1
    //    min_state_duration_seconds=300
};



/* *********************
   Global declarations
 ********************* */

char iniFilename[] = "/ardwino.ini"; // ini file Name
char csvFilename[] = "/ardwino.csv"; // csv file Name
char logFilename[] = "/ardwino.log"; // log file Name // TODO: look at https://github.com/embeddedartistry/arduino-logger/blob/master/README.md
char logBuffer[200];
unsigned long dbgcnt = 0;
unsigned long writecnt = 0;
unsigned long sderrorcnt = 0;
unsigned int csvSize = 0;

unsigned long lastRelayCheckTime=0;
unsigned long lastGsmCheckTime=0;
unsigned long lastIdleScreenRefresh=0;
unsigned long lastCsvUpdateTime=0;
unsigned long lastRtcUpdateTime=0;

//using namespace ::ace_button;
//using namespace ::SimpleHacks;
//QDecoder qdec(ROTARY_PIN_A, ROTARY_PIN_B, true); // rotary part
//AceButton button(ROTARY_PIN_BUT, HIGH); // button part

int16_t m5LcdHeight;
int16_t m5LcdWidth;
bool initScreen=false;

RTC_DS1307 rtc;
SemaphoreHandle_t rtcMutex = xSemaphoreCreateMutex();
ESP32Time esp32rtc;

Config configuration(iniFilename);

OneWire oneWire[] = {OneWire(ONEWIRE1_PIN), OneWire(ONEWIRE2_PIN)};

DallasTemperature sensorsBuses[] = {DallasTemperature(&oneWire[0]), DallasTemperature(&oneWire[1])};
//DallasTemperature sensorsBus1(&oneWire1);
//DallasTemperature sensorsBus2(&oneWire2);

//ThingSpeakClient
ThingSpeakClient thingSpeakClient;

/* https://github.com/olikraus/u8g2/wiki/u8g2setupcpp#st7920-128x64 */
//// VSPI SW SPI => OK but slow
////U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, /* clock=*/ VSPI_SCK_PIN, /* data=*/ VSPI_MOSI_PIN, /* CS=*/ SD_CS_PIN, /* reset=*/ U8X8_PIN_NONE);
//// VSPI HW SPI => OK
////U8G2_ST7920_128X64_F_HW_SPI u8g2(U8G2_R0, /* CS=*/ SD_CS_PIN, /* reset=*/ U8X8_PIN_NONE);
//// HSPI SW SPI => OK but slow
////U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, /* clock=*/ HSPI_SCK_PIN, /* data=*/ HSPI_MOSI_PIN, /* CS=*/ LCD_CS_PIN, /* reset=*/ U8X8_PIN_NONE);
//// HSPI HW SPI => OK : https://github.com/olikraus/u8g2/issues/1331
//U8G2_ST7920_128X64_F_2ND_HW_SPI u8g2(U8G2_R0, /* CS=*/ LCD_CS_PIN, /* reset=*/ U8X8_PIN_NONE);

//const colorDef<uint8_t> colors[6] MEMMODE = {
//  {{0, 0}, {0, 1, 1}}, //bgColor
//  {{1, 1}, {1, 0, 0}}, //fgColor
//  {{1, 1}, {1, 0, 0}}, //valColor
//  {{1, 1}, {1, 0, 0}}, //unitColor
//  {{0, 1}, {0, 0, 1}}, //cursorColor
//  {{1, 1}, {1, 0, 0}}, //titleColor
//};

//unsigned int timeOn = 10;
//unsigned int timeOff = 90;
//
//using namespace Menu;
//MENU(mainMenu, "ArdWino - Home", Menu::doNothing, Menu::noEvent, Menu::wrapStyle
//     , FIELD(timeOn, "On", "ms", 0, 1000, 10, 1, Menu::doNothing, Menu::noEvent, Menu::noStyle)
//     , FIELD(timeOff, "Off", "ms", 0, 10000, 10, 1, Menu::doNothing, Menu::noEvent, Menu::noStyle)
//     , EXIT("<Back")
//    );
//
//RotaryEventIn reIn(
//  RotaryEventIn::EventType::BUTTON_CLICKED | // select
//  RotaryEventIn::EventType::BUTTON_DOUBLE_CLICKED | // back
//  RotaryEventIn::EventType::BUTTON_LONG_PRESSED | // also back
//  RotaryEventIn::EventType::ROTARY_CCW | // up
//  RotaryEventIn::EventType::ROTARY_CW // down
//); // register capabilities, see ArduinoMenu MenuIO/RotaryEventIn.h file
//MENU_INPUTS(in, &reIn);
//
//MENU_OUTPUTS(out, MAX_DEPTH
//             , U8G2_OUT(u8g2, colors, fontX, fontY, offsetX, offsetY, {0, 0, U8_Width / fontX, U8_Height / fontY})
//             , NONE
//            );
//NAVROOT(nav, mainMenu, MAX_DEPTH, in, out);

/* *************
    Functions
 ************* */

//// This is the handler/callback for button events
//// We will convert/relay events to the RotaryEventIn object
//// Callback config in setup()
//void handleButtonEvent(AceButton* /* button */, uint8_t eventType, uint8_t buttonState) {
//  switch (eventType) {
//    case AceButton::kEventClicked:
//      //SerialMon.println("Button clicked");
//#ifndef M5STACK
//      digitalWrite(BACKLIGHT_PIN, HIGH); // activate screen backlight
//#endif
//      reIn.registerEvent(RotaryEventIn::EventType::BUTTON_CLICKED);
//      break;
//    case AceButton::kEventDoubleClicked:
//      //SerialMon.println("Button double clicked");
//#ifndef M5STACK
//      digitalWrite(BACKLIGHT_PIN, HIGH); // activate screen backlight
//#endif
//      reIn.registerEvent(RotaryEventIn::EventType::BUTTON_DOUBLE_CLICKED);
//      break;
//    case AceButton::kEventLongPressed:
//      //SerialMon.println("Button long clicked");
//#ifndef M5STACK
//      digitalWrite(BACKLIGHT_PIN, HIGH); // activate screen backlight
//#endif
//      reIn.registerEvent(RotaryEventIn::EventType::BUTTON_LONG_PRESSED);
//      break;
//  }
//}

////when menu is suspended
////this function is called when entering or leaving suspended state
//// with idleStart and idleend cases
////and at least once in between them (idling case)
////it might also be called for every pool (when in suspended state, idling case)
////for output devices that require refresh (repeated draw, idling case)
//result idle(menuOut &o, idleEvent e) {
//  // I manage the idle screen from the main loop
//  // https://github.com/neu-rah/ArduinoMenu/wiki/Idling
//  //  o.clear();
//  //  switch(e) {
//  //    case idleStart:
//  //      o.println("suspending menu!");
//  //      break;
//  //    case idling:
//  //      o.println("suspended...");
//  //      break;
//  //    case idleEnd:
//  //      o.println("resuming menu.");
//  //      nav.reset(); //clear nav state if desired
//  //      break;
//  //    default:break;
//  //  }
//  //  return proceed;
//}

//// This is the ISR (interrupt service routine) for rotary events
//// We will convert/relay events to the RotaryEventIn object
//// Callback config in setup()
//void IsrForQDEC(void) {
//  QDECODER_EVENT event = qdec.update();
//  if (event & QDECODER_EVENT_CW) {
//#ifndef M5STACK
//    digitalWrite(BACKLIGHT_PIN, HIGH); // activate screen backlight
//#endif
//    reIn.registerEvent(RotaryEventIn::EventType::ROTARY_CW);
//  }
//  else if (event & QDECODER_EVENT_CCW) {
//#ifndef M5STACK
//    digitalWrite(BACKLIGHT_PIN, HIGH); // activate screen backlight
//#endif
//    reIn.registerEvent(RotaryEventIn::EventType::ROTARY_CCW);
//  }
//}

void scanOneWireBus(uint8_t busIndex, DallasTemperature sensorsBus){
  bool checkAgain;
  uint8_t sensorIndex;
  DeviceAddress addr;
  SerialMon.printf("Scanning sensor_bus_%d:\n", busIndex+1);
  checkAgain = true;
  sensorIndex = 0;
  //Loop on getAddress until it returns false
  char msg[40];
  while (checkAgain) {
    if(sensorsBus.getAddress(addr, sensorIndex)) { //addr is DeviceAddress
      DeviceAddressStr addrStr;
      convertAddress2Chars(addr, addrStr);
      if ( OneWire::crc8(addr, 7) != addr[7]) {
        sprintf(msg, "   %s : Invalid CRC!", addrStr);
      }
      else {
        sprintf(msg, "   %s resolution:%dbits", addrStr, sensorsBus.getResolution(addr));
        //sensorsBus.setResolution(addr, 12);
      }
      logmsg(msg);
      sensorIndex++;
    }
    else checkAgain = false;
  }
}

void rotateFile(char *filename, int maxBytes) {
  File file = SD.open(logFilename, FILE_READ);
  int fileSize = file.size();
  file.close();
  if ( fileSize > maxBytes) {
    Serial.printf("%s size = %d\n", filename, fileSize);
    char backupFilename[40];
    sprintf(backupFilename, "%s.bak", filename);
    SD.remove(backupFilename);
    if (SD.rename(filename, backupFilename)) {
        Serial.printf("File %s renamed to %s\n", filename, backupFilename);
    } else {
        Serial.printf("%s rename failed\n", filename);
    }
  }
}
  
void logmsg(char *logMessage) {
  //struct tm timeinfo = esp32rtc.getTimeStruct();
  char logBuffer[200];

  rotateFile(logFilename, MAXLOGSIZE);
  File logFile = SD.open(logFilename, FILE_APPEND);
  sprintf(logBuffer, "%02i/%02i/%04i %02i:%02i:%02i : %s", esp32rtc.getDay(), esp32rtc.getMonth()+1, esp32rtc.getYear(), esp32rtc.getHour(true), esp32rtc.getMinute(), esp32rtc.getSecond(), logMessage);
  SerialMon.println(logMessage);
  SerialMon.flush();
  logFile.println(logBuffer);
  logFile.close();
}

void printDirectory(File dir, int numTabs) {
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      SerialMon.print('\t');
    }
    SerialMon.print(entry.name());
    if (entry.isDirectory()) {
      SerialMon.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      SerialMon.print("\t\t");
      SerialMon.println(entry.size(), DEC);
    }
    entry.close();
  }
}

void updateSystemRtcFromRtc(){
  DateTime now;
  xSemaphoreTake( rtcMutex, portMAX_DELAY );
  now = rtc.now();
  xSemaphoreGive( rtcMutex );
  esp32rtc.setTime(now.unixtime());
}
void updateExtRtcFromGsmNetwork() {
  xSemaphoreTake( rtcMutex, portMAX_DELAY );
  int   netYear    = 0;
  int   netMonth   = 0;
  int   netDay     = 0;
  int   netHour    = 0;
  int   netMin     = 0;
  int   netSec     = 0;
  float netTimezone = 0;
  thingSpeakClient.getNetworkTime(&netYear, &netMonth, &netDay, &netHour, &netMin, &netSec, &netTimezone);
  SerialMon.printf("Network Time: %d/%d/%d %d:%d:%d TZ%.1f\n", netYear, netMonth, netDay, netHour, netMin, netSec, netTimezone);
  //NOK: 2004 1 1 0 1 58 1.000000
  //OK:  2021 3 18 18 4 13 1.000000
  if (netYear != 2004) {
    rtc.adjust(DateTime(netYear, netMonth, netDay, netHour, netMin, netSec));
  }
  xSemaphoreGive( rtcMutex );
}

//void refreshIdleScreen() {
//  u8g2.setFont(fontName);
//  if (MEMDBG && (millis() / 5000) % 2 == 0) {
//    u8g2.setCursor(0, 15);
//    u8g2.printf("Millis: %d", millis());
//    u8g2.setCursor(0, 25);
//    u8g2.printf("Free Heap: %d", ESP.getFreeHeap());
//    u8g2.setCursor(0, 35);
//    u8g2.printf("Min Free Heap: %d", ESP.getMinFreeHeap());
//    u8g2.setCursor(0, 45);
//    u8g2.printf("Max Alloc Heap: %d", ESP.getMaxAllocHeap());
//    u8g2.setCursor(0, 55);
//    //u8g2.printf("Millis: %d", millis());
//    u8g2.printf("Loops: %d CSV: %d", dbgcnt, csvSize);
//    u8g2.setCursor(0, 65);
//    u8g2.printf("Writes: %d  Err: %d", writecnt, sderrorcnt);
//  }
//  else {
//    uint8_t x;
//    uint8_t y;
//    uint8_t fontSize=7;
//    uint8_t cr = fontSize + 1;
//    uint8_t p = fontSize + 2;
//    char charBuf[200];
//
//
//    //Draw header rectangle
//    u8g2.drawBox(0,0,U8_Width,fontSize+1);
//    
//    //First line is Timestamp
//    u8g2.setColorIndex(0);
//    y += fontSize+1;
//  xSemaphoreTake( rtcMutex, portMAX_DELAY );
//  DateTime now = rtc.now();
//  xSemaphoreGive( rtcMutex );
//    //sprintf(charBuf, "%02i/%02i/%04i %02i:%02i:%02i", now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second());
//    u8g2.setCursor(31,y); u8g2.printf("%02i/%02i/%04i %02i:%02i:%02i", now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second());
//    u8g2.setColorIndex(1);
//    //u8g2.drawHLine(0, y, U8_Width);
//
//    //Second line is Ground temp
//    y += p-1;
//    TempSensor * groundSensorPtr = configuration.getGroundSensor();
//    TempSensor * skySensorPtr = configuration.getSkySensor();
//    TempSensor * tempSensorPtr ;
//    char comparisonChar[2];
//    char warningChar[3];
//    float groundTempC = groundSensorPtr->getTemperature();
//    float groundFreezingTemp = configuration.getGroundFreezingTemp();
//    if (groundTempC < groundFreezingTemp) {
//      strcpy(comparisonChar, "<");
//      strcpy(warningChar, "!!");
//    }
//    else {
//      strcpy(comparisonChar, ">");
//      strcpy(warningChar, "  ");
//    }
//    u8g2.setCursor(0,y); u8g2.printf("%s Ground: %6.2f %s %-6.2f", warningChar, groundTempC, comparisonChar, groundFreezingTemp);
//
//    //Third line is Sky temp
//    y += cr;
//    float skyTempC = skySensorPtr->getTemperature();
//    float minTempForBlowing = configuration.getMinTempForBlowing();
//    if (skyTempC < minTempForBlowing) {
//      strcpy(comparisonChar, "<");
//      strcpy(warningChar, "!!");
//    }
//    else {
//      strcpy(comparisonChar, ">");
//      strcpy(warningChar, "  ");
//    }
//    u8g2.setCursor(0,y); u8g2.printf("%s    Sky: %6.2f %s %-6.2f", warningChar, skyTempC, comparisonChar, minTempForBlowing);
//
//    //Fourth line is relay state
//    uint8_t relayState = digitalRead(configuration.getRelayPin());    
//    y += cr;
//    unsigned int minStateDurationSec = configuration.getMinStateDurationSec();
//    unsigned long secToRelayCheck = (millis() - lastRelayCheckTime)/1000;
//    u8g2.setCursor(10,y); u8g2.printf("  Relay:  %s (%d sec)", (relayState)?"OFF":"ON", minStateDurationSec-secToRelayCheck);
//
//    u8g2.drawHLine(0, y, U8_Width);
//
//    //Fifth group is displaying the sensors by BUS
//    const uint8_t max_sensor_for_bus[2] = {6,3};
//    y += p;
//    for (uint8_t bus_index=0;bus_index<2;bus_index++){
//    //for (uint8_t bus_index=0;bus_index<BUS_COUNT;bus_index++){
//      for (uint8_t sensor_index=0;sensor_index<max_sensor_for_bus[bus_index];sensor_index++){
//      //for (uint8_t sensor_index=0;sensor_index<MAX_SENSORS_PER_BUS;sensor_index++){
//        if (sensor_index == 0) {
//          u8g2.setCursor(0,y); u8g2.printf("LINE%d", bus_index);
//        }
//        tempSensorPtr = configuration.getTempSensor(bus_index, sensor_index);
//        if (tempSensorPtr != NULL) {
//          u8g2.setColorIndex(1);
//          float sensorTemperature = tempSensorPtr->getTemperature();
//          uint8_t curX, curY;
//          curX = 28+((sensor_index%3)*34);
//          curY = y+cr*(sensor_index/3);
//          if (sensorTemperature == -127.00 || tempSensorPtr == groundSensorPtr || tempSensorPtr == skySensorPtr){
//            const uint8_t boxMargin = 2;
//            u8g2.drawBox(curX-boxMargin,curY-(fontSize+1),30+boxMargin*2,fontSize+1);
//            u8g2.setColorIndex(0);
//          }
//          if (sensorTemperature == -127.00) {
//            u8g2.setCursor(curX+3,curY); 
//            u8g2.print("ERROR");
//            //u8g2.drawGlyph(5, 20, 0x2603);
//          }
//          else {
//            u8g2.setCursor(curX,curY); 
//            u8g2.printf("%6.2f", sensorTemperature);
//          }
//        }
//        u8g2.setColorIndex(1);
//      }
//      //y += p + ((max_sensor_for_bus[bus_index]/3)-1)*cr ;
//      y += ((max_sensor_for_bus[bus_index]/3))*cr ;
//    }
//    //y -= p;
//    y -= cr;
//
//    u8g2.drawHLine(0, y, U8_Width);
//
//    //Last line for debug
//    u8g2.setFont(u8g2_font_micro_tr);
//    u8g2.setCursor(0,64); u8g2.printf("HEAP:%d/%d CSV:%d", ESP.getFreeHeap(), ESP.getMaxAllocHeap(), csvSize);
//    
////    u8g2.drawStr(26,16,"-12.20");
////    u8g2.drawStr(61,16,"-12.21");
////    u8g2.drawStr(96,16,"-12.20");
////    u8g2.drawStr(26,24,"-12.20");
////    u8g2.drawStr(61,24,"-12.21");
////    u8g2.drawStr(96,24,"-12.20");
////    u8g2.drawStr(26,32,"-12.20");
////    u8g2.drawStr(61,32,"-12.21");
////    u8g2.drawStr(96,32,"-12.20");
////    u8g2.drawStr(26,40,"-12.20");
////    u8g2.drawStr(61,40,"-12.21");
////    u8g2.drawStr(96,40,"-12.20");
//  }
//  return;
//}

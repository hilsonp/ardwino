void setup() { 
  //setupOTA("Ardwino", mySSID, myPASSWORD);
  M5.begin();
  m5LcdWidth=M5.Lcd.width();
  m5LcdHeight=M5.Lcd.height();
  
  // setup relay pin as soon as possible
  pinMode(RELAY1_PIN, OUTPUT);
  digitalWrite(RELAY1_PIN, RELAY_OPENED);
  pinMode(RELAY2_PIN, OUTPUT);
  digitalWrite(RELAY2_PIN, RELAY_OPENED);
  //pinMode(RELAY3_PIN, OUTPUT);
  //digitalWrite(RELAY3_PIN, LOW);
  //pinMode(RELAY4_PIN, OUTPUT);
  //digitalWrite(RELAY4_PIN, LOW);

//  digitalWrite(RELAY1_PIN, RELAY_CLOSED);
//  delay(1000);
//  digitalWrite(RELAY1_PIN, RELAY_OPENED);

  // Initialize the SerialMon
  SerialMon.begin(115200); // Can change it to 230400, if you dont use debugIsr* macros
  while (!SerialMon) {} // wait for SerialMon port to connect... needed for boards with native USB SerialMon support
  SerialMon.println(); // To not stay in end of dirty chars in boot
  ///printlnA(F("**** Setup: initializing..."));
  ///printlnA(F("**** Init LCD..."));
  delay(1000); // Sleep to get the SerialMon ready

  // setup builtin LED
//#ifndef M5STACK
//  pinMode(LEDBUILTIN_PIN, OUTPUT);
//#endif

//#ifndef M5STACK
//  // setup backlight pin
//  pinMode(BACKLIGHT_PIN, OUTPUT);
//  digitalWrite(BACKLIGHT_PIN, HIGH);
//#endif

  // Configure all of the SPI select pins as outputs and make SPI
  // devices inactive, otherwise the earlier init routines may fail
  // for devices which have not yet been configured.
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  pinMode(LCD_CS_PIN, OUTPUT);
  digitalWrite(LCD_CS_PIN, HIGH);
        
  // Splatch screen
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setFreeFont(CF_OL32);
  M5.Lcd.drawString("Ardwino", 55, 80, GFXFF);
  //M5.Lcd.setCursor(15, 50);
  //M5.Lcd.setTextFont(8);
  //M5.Lcd.setTextSize(2);
  //M5.Lcd.printf("Ardwino");
//  M5.Lcd.setCursor(250, 110);
//  M5.Lcd.setTextFont(2);
//  M5.Lcd.setTextSize(1);
//  M5.Lcd.printf("%s", VERSION);
  M5.Lcd.setFreeFont(CF_OL24);
  M5.Lcd.drawString(VERSION, 235, 88, GFXFF);

  //M5.Speaker.tone(262);
  //delay(50);
  M5.Speaker.tone(330);
  //delay(50);
  delay(10);
  M5.Speaker.mute();

  delay(2000);
  M5.update();
  bool csvResetAsked = M5.BtnB.isPressed();
  if (csvResetAsked) {
    M5.Speaker.tone(440);
    delay(30);
    M5.Speaker.mute();
  }

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setFreeFont(FSS12);
  //M5.Lcd.setTextFont(2);
  //M5.Lcd.setTextSize(1);
  //M5.Lcd.setTextSize(1);
  uint8_t lineY = 20;
  uint8_t marginX = 20;
  uint8_t lineHeight = 23;

//  // setup rotary encoder
//  qdec.begin();
//  attachInterrupt(digitalPinToInterrupt(ROTARY_PIN_A), IsrForQDEC, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(ROTARY_PIN_B), IsrForQDEC, CHANGE);
//
//  // setup rotary button
//  pinMode(ROTARY_PIN_BUT, INPUT);
//  ButtonConfig* buttonConfig = button.getButtonConfig();
//  buttonConfig->setEventHandler(handleButtonEvent);
//  buttonConfig->setFeature(ButtonConfig::kFeatureClick);
//  buttonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
//  buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);
//  buttonConfig->setFeature(ButtonConfig::kFeatureSuppressClickBeforeDoubleClick);
//  buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterClick);
//  buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterDoubleClick);
//
//  // setup display
//  u8g2.setBusClock(600000); // Needed with the "RepRap 12864 LCD" (ST7920)
//  u8g2.begin();
//  u8g2.clearBuffer();
//  u8g2.setFont(u8g2_font_maniac_tf);
//  u8g2.drawUTF8(5, 50, "ArdWino");
//  //u8g2.setFont(u8g2_font_unifont_t_symbols); // https://unicode-table.com/fr/emoji/
//  u8g2.setFont(u8g2_font_5x7_tf);
//  u8g2.setCursor(105, 50);
//  u8g2.print(VERSION);
//  u8g2.sendBuffer();
//  delay(2000);
//  u8g2.setFont(fontName); //A nice font is also u8g2_font_5x7_tf
//  u8g2.clearBuffer();
//  uint8_t lineY = 7;
//  uint8_t lineHeight = 8;
//
  // Start the RTC
//  u8g2.setCursor(0, lineY);

  M5.Lcd.setCursor(marginX, lineY);

  if (! rtc.begin()) {
    SerialMon.printf("FATAL: Couldn't find RTC");
    SerialMon.flush();
//    u8g2.println("FATAL: RTL init failed");
//    u8g2.sendBuffer();
    M5.Lcd.println("FATAL: RTL init failed");
    while (1);
  }
  else {
    SerialMon.println("RTC initialized\n");
    SerialMon.flush();
//    u8g2.println("RTC initialized");
//    u8g2.sendBuffer();
    M5.Lcd.println("RTC initialized");
    lineY += lineHeight;
  }

  if (! rtc.isrunning()) {
    SerialMon.printf("RTC is NOT running, let's set the time!\n");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  
  // Init the SD card
//  u8g2.setCursor(0, lineY);
  M5.Lcd.setCursor(marginX, lineY);
  if (!SD.begin(SD_CS_PIN)) {
    SerialMon.println("FATAL: SD card could not be initialized!");
    SerialMon.flush();
//    u8g2.println("FATAL: SD card ini file");
//    u8g2.sendBuffer();
    M5.Lcd.println("FATAL: SD card ini file");
    while (1);
  }
  else {
    SerialMon.println("SD card initialized");
    SerialMon.flush();
//    u8g2.println("SD card initialized");
//    u8g2.sendBuffer();
    M5.Lcd.println("SD card initialized");
    lineY += lineHeight;
  }

  File root = SD.open("/", FILE_READ);
  printDirectory(root, 0);
  root.close();
    
  // Try to open the log file on the SD card in write mode
//  u8g2.setCursor(0, lineY);
  M5.Lcd.setCursor(marginX, lineY);
  File logFile = SD.open(logFilename, FILE_APPEND);
  if (!logFile) {
    SerialMon.println("FATAL: log file could not be opened on the SD card.");
    SerialMon.flush();
//    u8g2.println("FATAL: LOG file read error");
//    u8g2.sendBuffer();
    M5.Lcd.println("FATAL: LOG file read error");
    while (1);
  }
  else {
    SerialMon.println("LOG file opened");
    SerialMon.flush();
//    u8g2.println("LOG file opened");
//    u8g2.sendBuffer();
    M5.Lcd.println("LOG file opened");
    lineY += lineHeight;
  }
  logFile.close();

  logmsg("Starting");

  // Try to open the ini file from the SD card in read mode
//  u8g2.setCursor(0, lineY);
  M5.Lcd.setCursor(marginX, lineY);
  File iniFile = SD.open(iniFilename, FILE_READ);
  if (!iniFile) {
    logmsg("FATAL: ini file could not be opened from the SD card");
//    u8g2.println("FATAL: INI file read error");
//    u8g2.sendBuffer();
    M5.Lcd.println("FATAL: INI file read error");
    while (1);
  }
  else {
    logmsg("INI file opened");
//    u8g2.println("INI file opened");
//    u8g2.sendBuffer();
    M5.Lcd.println("INI file opened");
    lineY += lineHeight;
  }
  iniFile.close();

//  u8g2.setCursor(0, lineY);
  M5.Lcd.setCursor(marginX, lineY);
  if (! configuration.readFromIni()) {
    logmsg("FATAL: read config from ini");
//    u8g2.println("FATAL: INI config read");
//    u8g2.sendBuffer();
    M5.Lcd.println("FATAL: INI config read");
    while (1);
  }
  else {
    logmsg("INI config read");
//    u8g2.println("INI config read");
//    u8g2.sendBuffer();
    M5.Lcd.println("INI config read");
    lineY += lineHeight;
  }
  configuration.printConfig();
  
//  u8g2.setCursor(0, lineY);
  M5.Lcd.setCursor(marginX, lineY);
  //Print the sensor addresses to the log file.
//  u8g2.println("Scanning Sensors");
//  u8g2.sendBuffer();
    M5.Lcd.println("Scanning Sensors");
  lineY += lineHeight;
  for (uint8_t i=0; i<BUS_COUNT; i++){
    sensorsBuses[i].begin();
    delay(3000);
    //deviceCount = sensorsBus1.getDeviceCount(); // getDeviceCount() is buggy on ESP32 https://github.com/milesburton/Arduino-Temperature-Control-Library/issues/85
    scanOneWireBus(i, sensorsBuses[i]);
  }
//  
//  //TODO: Put a Warning if this does not match the ini file.
//  
//  //csvFile = SD.open(iniFilename, FILE_APPEND);
//  u8g2.setCursor(0, lineY);
  M5.Lcd.setCursor(marginX, lineY);
  File csvFile;
  
  if (csvResetAsked) {
    logmsg("Reseting csv file.");
    csvFile = SD.open(csvFilename, FILE_WRITE);
  }
  else {
    csvFile = SD.open(csvFilename, FILE_APPEND);
  }
  if (!csvFile) {
    logmsg("FATAL: csv file could not be opened from the SD card");
//    u8g2.println("FATAL: CSV file write error");
//    u8g2.sendBuffer();
    M5.Lcd.println("FATAL: CSV file write error");
    while (1);
  }
  else {
    if (csvResetAsked) {
      logmsg("CSV file resetted");
  //    u8g2.println("CSV file opened");
  //    u8g2.sendBuffer();
      M5.Lcd.println("CSV file resetted");
    }
    else {
      logmsg("CSV file opened");
  //    u8g2.println("CSV file opened");
  //    u8g2.sendBuffer();
      M5.Lcd.println("CSV file opened");
    }
    lineY += lineHeight;
  }
  csvFile.close();

  if (configuration.isUploadToCloudEnabled()){
    //  u8g2.setCursor(0, lineY);
      M5.Lcd.setCursor(marginX, lineY);
      //Print the sensor addresses to the log file.
    //  u8g2.println("Scanning Sensors");
    //  u8g2.sendBuffer();
      M5.Lcd.println("Starting GPRS module");
      lineY += lineHeight;

      char simPIN[10]; 
      configuration.getSimPin(simPIN);
      char apn[60];
      configuration.getApn(apn);
      char apnUser[20];
      configuration.getApnUser(apnUser);
      char apnPwd[20];
      configuration.getApnPwd(apnPwd);
      char mqttBroker[40];
      configuration.getMqttBroker(mqttBroker);
      char mqttUser[40];
      configuration.getMqttUser(mqttUser);
      char mqttPwd[40];
      configuration.getMqttPwd(mqttPwd);
      thingSpeakClient.begin(simPIN, apn, apnUser, apnPwd, mqttBroker, mqttUser, mqttPwd);
  }

  xTaskCreatePinnedToCore(sensorManagementTask, "sensorManagementTask", 10000, NULL, 5, NULL, 1);
  delay(5000);
  xTaskCreatePinnedToCore(relayManagementTask, "relayManagementTask", 10000, NULL, 10, NULL, 1);
//
//  nav.showTitle = true; // Show titles in the menus and submenus
//  nav.idleTask = idle; //point a function to be used when menu is suspended
//  nav.timeOut = 30;  // Timeout after 60 seconds of inactivity
//  nav.idleOn(idle);//enter idle mode, this menu will start on idle state, press select  to enter menu
//  ///printlnA(F("*** Setup end"));
  M5.Lcd.fillScreen(BLACK);
}

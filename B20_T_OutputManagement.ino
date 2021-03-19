void drawBox(uint16_t y, uint16_t x, uint16_t width, uint16_t height, uint16_t color, uint16_t backgroundColor, bool filled, char *text, uint16_t fontSize) {
//  uint16_t colPixels=40;
//  uint16_t rowPixels=30;
//  uint16_t y=colPixels * col;
//  uint16_t x=rowPixels * row;
//  uint16_t cellWidth=colPixels * colSize;
//  uint16_t cellHeight=rowPixels * rowSize;
  if (filled) {
    M5.Lcd.fillRect(y, x, width, height, backgroundColor);
  }
  else {
    M5.Lcd.fillRect(y+1, x+1, width-2, height-2, TFT_BLACK);
    M5.Lcd.drawRect(y, x, width, height, backgroundColor);
  }
  M5.Lcd.setTextColor(color);
  switch (fontSize) {
    case 1:
      M5.Lcd.setTextFont(2);
      M5.Lcd.setTextSize(1);
      //M5.Lcd.setFreeFont(TT1);
      break;
    case 2:
      M5.Lcd.setTextFont(4);
      M5.Lcd.setTextSize(1);
      //M5.Lcd.setFreeFont(GLCD);
      break;
    case 3:
      M5.Lcd.setTextFont(4);
      M5.Lcd.setTextSize(1);
      //M5.Lcd.setFreeFont(GLCD);
      break;
    default:
      M5.Lcd.setTextFont(4);
      M5.Lcd.setTextSize(1);
      //M5.Lcd.setFreeFont(GLCD);
      break;    
  }
  M5.Lcd.drawString(text, y+(width/2), x+2+(height/2));
}
void drawSensorBox(uint16_t y, uint16_t x, uint16_t width, uint16_t height, uint16_t color, uint16_t backgroundColor, uint8_t bus_index, uint8_t sensor_index ) {
  TempSensor * tempSensorPtr ;
  float sensorTemperature;
  char charBuf[30];
  tempSensorPtr = configuration.getTempSensor(bus_index, sensor_index);
  if (tempSensorPtr != NULL) {
    sensorTemperature = tempSensorPtr->getTemperature();
    if (sensorTemperature != -127.0) {
      sprintf(charBuf, "B%dS%d: %-6.2f", bus_index+1, sensor_index+1, sensorTemperature);
    }
    else {
      sprintf(charBuf, "B%dS%d: %s", bus_index+1, sensor_index+1, " ERR ");
    }
  }
  else {
    sprintf(charBuf, "B%dS%d: %s", bus_index+1, sensor_index+1, " ERR ");
  }
  drawBox(y, x, width, height, color, backgroundColor, false, charBuf, 3);
}

void refreshM5Screen(bool init, uint8_t screen_idx){
#ifdef DEBUG_MUTEX
  SerialMon.println(">>> refreshM5Screen IN");
#endif
  char charBuf[30];
  uint16_t mesureBackgroundColor, referenceBackgroundColor, warningBackgroundColor;
  unsigned int csvFrequencySec = configuration.getCsvFrequencySec();
  
  switch (screen_idx % 2) {
    case 0:  
      {
        M5.Lcd.setTextDatum(MC_DATUM);
        uint16_t boxColor, boxBgColor;
        // Timestamp
        //struct tm timeinfo = esp32rtc.getTimeStruct();
        //sprintf(charBuf, "%02i/%02i %02i:%02i:%02i", now.day(), now.month(), now.hour(), now.minute(), now.second());
        sprintf(charBuf, "%02i/%02i %02i:%02i", esp32rtc.getDay(), esp32rtc.getMonth()+1, esp32rtc.getHour(true), esp32rtc.getMinute());
        //SerialMon.printf("%d-%d=%d >= (%d * 1000 - 5000)=%d\n",  millis(), lastCsvUpdateTime, millis()-lastCsvUpdateTime, (unsigned long)csvFrequencySec, (((unsigned long)csvFrequencySec * 1000))-5000);
        if (millis()-lastCsvUpdateTime >= (((unsigned long)csvFrequencySec * 1000))-5000){
          drawBox(0, 0, 160, 30, TFT_WHITE, TFT_ORANGE, true, charBuf, 2);
        }
        else {
          drawBox(0, 0, 160, 30, TFT_WHITE, TFT_BLUE, true, charBuf, 2);
        }
        // Relay
        uint8_t relayState = digitalRead(configuration.getRelayPin());    
        unsigned int minStateDurationSec = configuration.getMinStateDurationSec();
        unsigned long secToRelayCheck = (millis() - lastRelayCheckTime)/1000;
        //    u8g2.setCursor(10,y); u8g2.printf("  Relay:  %s (%d sec)", (relayState)?"OFF":"ON", minStateDurationSec-secToRelayCheck);
        if (relayState==RELAY_OPENED) {
          drawBox(160, 0, 30, 30, TFT_WHITE, TFT_GREY, true, "R", 2);
        }
        else {
          drawBox(160, 0, 30, 30, TFT_BLACK, TFT_YELLOW, true, "R", 2);
        }

        // Network status
        unsigned long debugTimer = millis();
        uint32_t statusColor;
        //SerialMon.printf("### Start : %d\n", millis()-debugTimer);
        statusColor = (thingSpeakClient.isSimUnlocked())?TFT_DARKGREEN:TFT_GREY;
        //SerialMon.printf("### Sim : %d\n", millis()-debugTimer);
        //debugTimer = millis();
        drawBox(190, 0, 25, 30, TFT_WHITE, statusColor, true, "S", 2);

        statusColor = (thingSpeakClient.isNetworkConnected())?TFT_DARKGREEN:TFT_GREY;
        //SerialMon.printf("### Net : %d\n", millis()-debugTimer);
        //debugTimer = millis();
        drawBox(215, 0, 25, 30, TFT_WHITE, statusColor, true, "N", 2);

        sprintf(charBuf, "%02d", thingSpeakClient.getSignalQuality());
        //SerialMon.printf("### Signal : %d\n", millis()-debugTimer);
        //debugTimer = millis();
        drawBox(240, 0, 30, 30, TFT_WHITE, statusColor, true, charBuf, 2);

        statusColor = (thingSpeakClient.isGprsConnected())?TFT_DARKGREEN:TFT_GREY;
        //SerialMon.printf("### Gprs : %d\n", millis()-debugTimer);
        //debugTimer = millis();
        drawBox(270, 0, 25, 30, TFT_WHITE, statusColor, true, "I", 2);

        statusColor = (thingSpeakClient.isMqttClientConnected())?TFT_DARKGREEN:TFT_GREY;
        //SerialMon.printf("### Mqtt : %d\n", millis()-debugTimer);
        //debugTimer = millis();
        drawBox(295, 0, 25, 30, TFT_WHITE, statusColor, true, "C", 2);

        //SerialMon.printf("### Refresh : %d\n", millis()-debugTimer);
        
        // Countdown
        sprintf(charBuf, "%ds", minStateDurationSec-secToRelayCheck);
        drawBox(0, 30, 100, 30, TFT_BLACK, TFT_WHITE, true, charBuf, 2);
      
        // Col Headers
        if (!init) {
          drawBox(100, 30, 110, 30, TFT_BLACK, TFT_WHITE, true, "Meas.", 2);
          drawBox(210, 30, 110, 30, TFT_BLACK, TFT_WHITE, true, "Refs.", 2);
        
          // Sky
          drawBox(0, 60, 100, 40, TFT_BLACK, TFT_WHITE, true, "SKY", 3);
          // Ground
          drawBox(0, 100, 100, 40, TFT_BLACK, TFT_WHITE, true, "GND", 3);
        }
      
        // Warning char
          TempSensor * groundSensorPtr = configuration.getGroundSensor();
          TempSensor * skySensorPtr = configuration.getSkySensor();
          TempSensor * tempSensorPtr ;
          char comparisonChar[2];
          char warningChar[3];
      
          // Sky
          float skyTempC = skySensorPtr->getTemperature();
          float minTempForBlowing = configuration.getMinTempForBlowing();
          //skySensorPtr->getTemperatureStr(charBuf);
          if (skyTempC < minTempForBlowing) {
            strcpy(comparisonChar, "<");
            strcpy(warningChar, "!!");
            warningBackgroundColor=TFT_RED;    }
          else {
            strcpy(comparisonChar, ">");
            strcpy(warningChar, "  ");
            warningBackgroundColor=TFT_DARKGREEN;
          }
          if (skyTempC != -127.0) {
            sprintf(charBuf, "%-6.2f", skyTempC);
          }
          else {
            sprintf(charBuf, "%s", " ERR ");
          }
          drawBox(100, 60, 100, 40, TFT_WHITE, TFT_BLACK, true, charBuf, 3);
          drawBox(200, 60, 20, 40, TFT_BLACK, warningBackgroundColor, true, comparisonChar, 3);
          sprintf(charBuf, "%-6.2f", minTempForBlowing);
          drawBox(220, 60, 100, 40, TFT_WHITE, TFT_BLACK, true, charBuf, 3);
      
          //Ground
          float groundTempC = groundSensorPtr->getTemperature();
          float groundFreezingTemp = configuration.getGroundFreezingTemp();
          //groundSensorPtr->getTemperatureStr(charBuf);
          if (groundTempC < groundFreezingTemp) {
            strcpy(comparisonChar, "<");
            strcpy(warningChar, "!!");
            warningBackgroundColor=TFT_RED;
          }
          else {
            strcpy(comparisonChar, ">");
            strcpy(warningChar, "  ");
            warningBackgroundColor=TFT_DARKGREEN;
          }
          if (groundTempC != -127.0) {
            sprintf(charBuf, "%-6.2f", groundTempC);
          }
          else {
            sprintf(charBuf, "%s", " ERR ");
          }
          drawBox(100, 100, 100, 40, TFT_WHITE, TFT_BLACK, true, charBuf, 3);
          drawBox(200, 100, 20, 40, TFT_BLACK, warningBackgroundColor, true, comparisonChar, 3);
          sprintf(charBuf, "%-6.2f", groundFreezingTemp);
          drawBox(220, 100, 100, 40, TFT_WHITE, TFT_BLACK, true, charBuf, 3);
      
          //Sensors 
      //    switch ((now.second()/4)%3) {
      //      case 0:
              drawSensorBox(0, 140, 160, 30, TFT_WHITE, TFT_DARKGREY, 0, 0);
              drawSensorBox(159, 140, 161, 30, TFT_WHITE, TFT_DARKGREY, 0, 1);
      //        break;
      //      case 1:
              drawSensorBox(0, 169, 160, 30, TFT_WHITE, TFT_DARKGREY, 0, 2);
              drawSensorBox(159, 169, 161, 30, TFT_WHITE, TFT_DARKGREY, 0, 3);
      //        break;
      //      case 2:
              drawSensorBox(0, 198, 160, 30, TFT_WHITE, TFT_DARKGREY, 0, 4);
              drawSensorBox(159, 198, 161, 30, TFT_WHITE, TFT_DARKGREY, 1, 0);
      //        break;
      //    }
      }
      break;
    case 1:
      SerialMon.println("TBD");
      break;
    default:
      break;
  }
#ifdef DEBUG_MUTEX
  SerialMon.println("<<< refreshM5Screen OUT");
#endif
}

void relayManagementTask(void *pvParameters) {
  const char *pcTaskName = "RelayMngt";
  uint8_t core;
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  uint8_t relay_pin = configuration.getRelayPin();
  
  unsigned int minStateDurationSec = configuration.getMinStateDurationSec();
  unsigned int csvFrequencySec = configuration.getCsvFrequencySec();
  float groundFreezingTemp = configuration.getGroundFreezingTemp();
  float minTempForBlowing = configuration.getMinTempForBlowing();
  TempSensor * groundSensorPtr = configuration.getGroundSensor();
  TempSensor * skySensorPtr = configuration.getSkySensor();
  float skyTempC=95.0;
  float groundTempC=95.0;
  char msg[80];
  uint8_t initialRelayState;
  bool needReport = true;
  //unsigned long lastCsvUpdateTime=0;
  //TODO: For now, the first upload happen 24h after the boot. I should store the lastCloudUpload timestamp in EEPROM to survive reboot
  unsigned long lastCloudUploadTime=0;
  bool hadRelayOnInLastCsvPeriod = (digitalRead(relay_pin) == RELAY_CLOSED);
  bool hadRelayOnSinceLastCloudPublish = hadRelayOnInLastCsvPeriod;
  
  char identificationStr[DEFAULT_BUFFER_LEN];
  TempSensor * tempSensorPtr;
  //DateTime now;

  //ThingsSpeak related things
  char measuresTopicStr[100];
  // "channels/" + String( this->_thingSpeakChannelId ) + "/publish/"+String(this->_writeApiKey)
  char thingSpeakDataChannelId[40];
  configuration.getThingSpeakDataChannelId(thingSpeakDataChannelId);
  char thingSpeakDebugChannelId[40];
  configuration.getThingSpeakDebugChannelId(thingSpeakDebugChannelId);
  char dataWriteApiKey[40];
  configuration.getDataWriteApiKey(dataWriteApiKey);
  char debugWriteApiKey[40];
  configuration.getDebugWriteApiKey(debugWriteApiKey);
  
  sprintf(measuresTopicStr, "channels/%s/publish/%s", thingSpeakDataChannelId, dataWriteApiKey);

  char debugTopicStr[100];
  sprintf(debugTopicStr, "channels/%s/publish/%s", thingSpeakDebugChannelId, debugWriteApiKey);
  
  char measuresMsgStr[400];
  char debugMsgStr[400];

  //Logic is made with relay activated when pin is LOW.
  for(;;) {
#ifdef DEBUG_MUTEX
    SerialMon.println(">>> relayManagementTask IN");
#endif
    dbgcnt++;
    core = xPortGetCoreID();
    //SerialMon.printf("%s(%d): Wake up !!! \n", pcTaskName, core);

    if (lastRtcUpdateTime == 0 || millis()-lastRtcUpdateTime > (unsigned long)60 * 1000) {
      SerialMon.println("Updating External RTC from GSM Network time");
      updateExtRtcFromGsmNetwork();
      SerialMon.println("Updating Internal RTC from External RTC");
      updateSystemRtcFromRtc();
      lastRtcUpdateTime=millis();
    }

    if (lastGsmCheckTime == 0 || millis()-lastGsmCheckTime > (unsigned long)60 * 1000) {
      thingSpeakClient.check();
      lastGsmCheckTime=millis();
    }

    
    //Manage relay and update time from RTC
    if (millis()-lastRelayCheckTime > (unsigned long)minStateDurationSec * 1000) {
      //SerialMon.printf("###### DEBUG: hadRelayOnInLastCsvPeriod = %d\n", hadRelayOnInLastCsvPeriod);
      lastRelayCheckTime = millis();
      SerialMon.printf("%s(%d): Manage relay (pin=%d)\n", pcTaskName, core, relay_pin);
      initialRelayState = digitalRead(relay_pin);
      skyTempC = skySensorPtr->getTemperature();
      groundTempC = groundSensorPtr->getTemperature();
      
      if (groundTempC <= groundFreezingTemp && skyTempC >= minTempForBlowing) {
//        SerialMon.println("##### DEBUG : Closing relay.");
        digitalWrite(relay_pin, RELAY_CLOSED);
        hadRelayOnInLastCsvPeriod=true;
        hadRelayOnSinceLastCloudPublish=true;
      }
      else {
//        SerialMon.println("##### DEBUG : Opening relay.");
        digitalWrite(relay_pin, RELAY_OPENED);
      }
      if (DEBUG || needReport || initialRelayState != digitalRead(relay_pin)) {
        sprintf(msg, "Ground Freezing Temp: %.2f", groundFreezingTemp);
        logmsg(msg);
        sprintf(msg, "         Ground Temp: %.2f", groundTempC);
        logmsg(msg);
        sprintf(msg, "Min Temp For Blowing: %.2f", minTempForBlowing);
        logmsg(msg);
        sprintf(msg, "            Sky Temp: %.2f", skyTempC);
        logmsg(msg);
        sprintf(msg, "Relay State: %d", (digitalRead(relay_pin) == RELAY_CLOSED));
        logmsg(msg);
      }
      needReport = false;
    }

    //Manage csv
    if (millis()-lastCsvUpdateTime >= (unsigned long)csvFrequencySec * 1000) {
      if (!SD.begin(SD_CS_PIN)) {
        sderrorcnt++;
        SerialMon.println("FATAL: SD card could not be re-initialized!");
      }
      else {
        //SerialMon.println("DEBUG: SD card re-initialized!");
      }
      writecnt++;
      lastCsvUpdateTime = millis();
      SerialMon.printf("%s(%d): Manage csv\n", pcTaskName, core);
      
      rotateFile(csvFilename, MAXCSVSIZE);
      File csvFile = SD.open(csvFilename, FILE_APPEND);
      
      if (!csvFile) {
        SerialMon.println("ERROR: csv file could not be opened from the SD card");
      }
      else {
        //SerialMon.println("DEBUG: SD.open returned true");
        //Write headers if new file
        if (csvFile.size() == 0) {
          //SerialMon.println("DEBUG: SD.size returned 0");
          csvFile.printf("%s%s%s%s%s%s%s%s%s%s%s%s%s%s", "Date", CSV_SEPARATOR, "Time", CSV_SEPARATOR, "Relay", CSV_SEPARATOR, "Ground_C", CSV_SEPARATOR, "Ground_SD", CSV_SEPARATOR, "Sky_C", CSV_SEPARATOR, "Sky_SD", CSV_SEPARATOR);
          for (uint8_t bus_index=0;bus_index<BUS_COUNT;bus_index++){
            for (uint8_t sensor_index=0;sensor_index<MAX_SENSORS_PER_BUS;sensor_index++){
              tempSensorPtr = configuration.getTempSensor(bus_index, sensor_index);
              if (tempSensorPtr != NULL) {
                csvFile.printf("B%dS%d_C%s", bus_index+1, sensor_index+1, CSV_SEPARATOR);
                csvFile.printf("B%dS%d_SD%s", bus_index+1, sensor_index+1, CSV_SEPARATOR);
              }
            }
          }
          csvFile.println();
        }
        //Print the csv line
        //SerialMon.println("DEBUG: SD print values");
        //TODO: Raise warning if RTC confidence lost https://starthardware.org/en/arduino-time-with-real-time-clock-rtc/
        //struct tm timeinfo = esp32rtc.getTimeStruct();
        float sensorTemperature;
        char sensorTemperatureStr[10];
        char sensorStdDevStr[10];
        csvFile.printf("%04d/%02d/%02d%s",esp32rtc.getYear(), esp32rtc.getMonth()+1, esp32rtc.getDay(), CSV_SEPARATOR);
        csvFile.printf("%02d:%02d:%02d%s", esp32rtc.getHour(true), esp32rtc.getMinute(), esp32rtc.getSecond(), CSV_SEPARATOR);
        SerialMon.printf("%04d/%02d/%02d %02d:%02d:%02d\n", esp32rtc.getYear(), esp32rtc.getMonth()+1, esp32rtc.getDay(), esp32rtc.getHour(true), esp32rtc.getMinute(), esp32rtc.getSecond());
        csvFile.print(hadRelayOnInLastCsvPeriod);
        hadRelayOnInLastCsvPeriod=false;
        csvFile.print(CSV_SEPARATOR);
//        sensorTemperature = configuration.getGroundSensor()->getTemperature();
//        if (sensorTemperature == -127.00) {
//          csvFile.printf("%s%s", "",CSV_SEPARATOR);
//        }
//        else {
//          csvFile.printf("%.2f%s", sensorTemperature,CSV_SEPARATOR);
//        }
        configuration.getGroundSensor()->getTemperatureStr(sensorTemperatureStr);
        csvFile.printf("%s%s", sensorTemperatureStr,CSV_SEPARATOR);
        configuration.getGroundSensor()->getStdDevStr(sensorStdDevStr);
        csvFile.printf("%s%s", sensorStdDevStr,CSV_SEPARATOR);
        SerialMon.printf("Ground: %s %s\n", sensorTemperatureStr,sensorStdDevStr);
        //csvFile.printf("%.2f%s", configuration.getGroundSensor()->getStdDev(), CSV_SEPARATOR);
//        sensorTemperature = configuration.getSkySensor()->getTemperature();
//        if (sensorTemperature == -127.00) {
//          csvFile.printf("%s%s", "",CSV_SEPARATOR);
//        }
//        else {
//          csvFile.printf("%.2f%s", sensorTemperature,CSV_SEPARATOR);
//        }
        configuration.getSkySensor()->getTemperatureStr(sensorTemperatureStr);
        csvFile.printf("%s%s", sensorTemperatureStr,CSV_SEPARATOR);
        configuration.getSkySensor()->getStdDevStr(sensorStdDevStr);
        csvFile.printf("%s%s", sensorStdDevStr,CSV_SEPARATOR);
        SerialMon.printf("Sky: %s %s\n", sensorTemperatureStr,sensorStdDevStr);

        uint8_t field_index=1;
        sprintf(measuresMsgStr, "field%d=%d", field_index, hadRelayOnSinceLastCloudPublish);
        //field_index++;
        
        //csvFile.printf("%.2f%s", configuration.getSkySensor()->getStdDev(), CSV_SEPARATOR);
        for (uint8_t bus_index=0;bus_index<BUS_COUNT;bus_index++){
          for (uint8_t sensor_index=0;sensor_index<MAX_SENSORS_PER_BUS;sensor_index++){
            tempSensorPtr = configuration.getTempSensor(bus_index, sensor_index);
            if (tempSensorPtr != NULL) {
              tempSensorPtr->getIdentificationStr(identificationStr);
              sensorTemperature = tempSensorPtr->getTemperature();
//              if (sensorTemperature == -127.00) {
//                csvFile.printf("%s%s", "",CSV_SEPARATOR);
//              }
//              else {
//                csvFile.printf("%.2f%s", sensorTemperature,CSV_SEPARATOR);
//              }
//              csvFile.printf("%.2f%s", tempSensorPtr->getStdDev(),CSV_SEPARATOR);
                tempSensorPtr->getTemperatureStr(sensorTemperatureStr);
                csvFile.printf("%s%s", sensorTemperatureStr,CSV_SEPARATOR);
                tempSensorPtr->getStdDevStr(sensorStdDevStr);
                csvFile.printf("%s%s", sensorStdDevStr,CSV_SEPARATOR);
                SerialMon.printf("%s: %s %s\n", identificationStr, sensorTemperatureStr,sensorStdDevStr);
                field_index=tempSensorPtr->getThingspeakField();
                if (field_index > 0 && sensorTemperature != -127.00) { // !! We store the relay state in field1
                  sprintf(measuresMsgStr, "%s&field%d=%.2f", measuresMsgStr, field_index, sensorTemperature);
                }
            }
          }
        }
        csvFile.println();
        //SerialMon.println("DEBUG: SD all values printed");
        csvSize=csvFile.size();
      }
      csvFile.close();
      
      //Upload to cloud
      //if (millis()-lastCloudUploadTime >= (unsigned long)(23 * 60 * 60 * 1000) && hour() >= configuration.getUploadHour() && minute() >= configuration.getUploadMinute() ) {
      //  lastCloudUploadTime = millis();
      //  SerialMon.printf("%s(%d): Upload to cloud and update NTP (not implemented yet)\n", pcTaskName, core);
      //}
      if (configuration.isUploadToCloudEnabled()){
        SerialMon.println("Cloud: Checking if reconnection is needed.");
        if (thingSpeakClient.reconnect(true)) {
          SerialMon.println("Cloud: Updating ThingSpeak.");
          SerialMon.printf("   %s | %s\n", measuresTopicStr, measuresMsgStr);
          thingSpeakClient.mqttPublishFeed(measuresTopicStr, measuresMsgStr);
          hadRelayOnSinceLastCloudPublish=false;
          
          sprintf(debugMsgStr, "field1=%d&field2=%d&field3=%d", (int)(millis()/1000), ESP.getFreeHeap(), ESP.getMaxAllocHeap());
          SerialMon.printf("   %s | %s\n", debugTopicStr, debugMsgStr);
          thingSpeakClient.mqttPublishFeed(debugTopicStr, debugMsgStr);
        }
      }
    }

#ifdef DEBUG_MUTEX
    SerialMon.println("<<< relayManagementTask OUT");
#endif
    //Sleep a bit
    vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 100 ));
  }
}

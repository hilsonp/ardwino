/* Read several times all sensors
    -> Compute StdDev https://forum.arduino.cc/index.php?topic=497072.0
    -> Compute Avg
    -> Update TempSensor objects (Avg StdDev)

    Notes:
    - Delayed Task: http://tvaira.free.fr/esp32/esp32-freertos.html
    - https://lastminuteengineers.com/multiple-ds18b20-arduino-tutorial/
*/
void sensorManagementTask(void *pvParameters)
{
  const char *pcTaskName = "SensorMngt";
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  char identificationStr[DEFAULT_BUFFER_LEN];
  DeviceAddress addr;
  DeviceAddressStr addrChar;
  TempSensor * tempSensorPtr;
  float tempC;
  uint8_t core;
  
  for(;;) {
    core = xPortGetCoreID();
    SerialMon.printf("%s(%d): Manage sensors\n", pcTaskName, core);

    for (uint8_t bus_index=0;bus_index<BUS_COUNT;bus_index++){
      //Asking a refresh on the bus
      SerialMon.printf("%s(%d):    Asking bus %d refresh\n", pcTaskName, core, bus_index+1);
      sensorsBuses[bus_index].requestTemperatures();
      //https://forum.arduino.cc/index.php?topic=61840.msg448278#msg448278
      while (oneWire[bus_index].read() == 0) { vTaskDelay( pdMS_TO_TICKS( 100 ) ); }
      
      //vTaskDelay( pdMS_TO_TICKS( 4000 ) ); //Waiting for the conversions to happen
      
      for (uint8_t sensor_index=0;sensor_index<MAX_SENSORS_PER_BUS;sensor_index++){
        //SerialMon.printf("%s(%d):       bus:%d sensor:%d\n", pcTaskName, core, bus_index+1, sensor_index+1);
        tempSensorPtr = configuration.getTempSensor(bus_index, sensor_index);
        if (tempSensorPtr != NULL) {
          tempSensorPtr->getIdentificationStr(identificationStr);
          SerialMon.printf("%s(%d):       Reading %s: ", pcTaskName, core, identificationStr);
          tempSensorPtr->getAddress(addr);
          tempC = sensorsBuses[bus_index].getTempC(addr);
          if (tempC == -127.00) {
            SerialMon.println("Error");
            tempSensorPtr->setTemperature(-127.00);
          }
          else {
            SerialMon.println(tempC);
            tempSensorPtr->setUncorrectedTemperature(tempC);
          }
        }
      }
    }
    
    //vTaskDelay( pdMS_TO_TICKS( 500 ) );
    vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10000 ) ); // toutes les 10000 ms
  }
}

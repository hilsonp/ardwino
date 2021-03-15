void loop() {
  uint8_t screen_idx = 0;
  // SerialDebug handle
  // Notes: if in inactive mode (until receive anything from serial),
  // it show only messages of always or errors level type
  // And the overhead during inactive mode is very low
  // Only if not DEBUG_DISABLED
  ///debugHandle();

  ///printlnI(F("This is a message of debug level INFO"));
  //updateTime();

//  button.check(); // acebutton check, rotary is on ISR
//  nav.doInput(); // menu check
  unsigned int idleScreenRefreshMsec = 500;
  // https://github.com/neu-rah/ArduinoMenu/wiki/Idling
//  SerialMon.println("++++++");
//  SerialMon.println(millis());
//  SerialMon.println(lastIdleScreenRefresh);
//  SerialMon.println(millis()-lastIdleScreenRefresh);
//  SerialMon.println(idleScreenRefreshSec);
//  SerialMon.println(idleScreenRefreshSec*1000);  
//  SerialMon.println((millis()-lastIdleScreenRefresh) > (idleScreenRefreshSec*1000));
//  if (nav.sleepTask && ((millis()-lastIdleScreenRefresh) > (idleScreenRefreshSec*1000))) {
  if ((millis()-lastIdleScreenRefresh) > (idleScreenRefreshMsec)) {
    //SerialMon.println("+++ Refresh");
    lastIdleScreenRefresh = millis();
//    SerialMon.println(lastIdleScreenRefresh);
    refreshM5Screen(initScreen, screen_idx);
    initScreen=true;
//    u8g2.firstPage();
//    do {
//      refreshIdleScreen();
//    } while ( u8g2.nextPage() );
//  } else {
//    if (nav.changed(0)) {//only draw if menu changed for gfx device
//      u8g2.firstPage();
//      do nav.doOutput(); while(u8g2.nextPage());
//    }
  }
  M5.update();
  if (M5.BtnA.wasReleased()) {
    screen_idx--;
  }
  if (M5.BtnC.wasReleased()) {
    screen_idx++;
  }
  delay(1);
  //delay(1000);
}

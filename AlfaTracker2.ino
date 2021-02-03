#include "Adafruit_MCP9808.h"
#include <DS3232RTC.h>
#include <Wire.h>
#include <avr/sleep.h>
#include "Adafruit_FONA.h"


/******************* temp sensor init *****************/
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();
float tempC = 0;

/******************* LTE setup *****************/
#define SIMCOM_7000
#define FONA_PWRKEY 6
#define FONA_RST 7
#define FONA_TX 10 // Microcontroller RX
#define FONA_RX 11 // Microcontroller TX
 
// this is a large buffer for replies
char replybuffer[255];

#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

Adafruit_FONA_LTE fona = Adafruit_FONA_LTE();

float latitude, longitude, speed_kph, heading, altitude, seconds;
uint8_t type;
char imei[16] = {0};
uint16_t vbat = 0;

char latBuff[12], longBuff[12], locBuff[50], speedBuff[12],
     altBuff[12], tempBuff[12], battBuff[12];

char URL[200];  // Make sure this is long enough for your request URL
int ttff=0;
int retry=0;

/******************* Thingspeak setup *****************/

const char * myWriteAPIKey = "THINGSPEAKKEY";

void setup() {
  //CLKPR = 0x80; // (1000 0000) enable change in clock frequency
  //CLKPR = 0x01; // (0000 0001) use clock division factor 2 to reduce the frequency from 16 MHz to 8 MHz
  digitalWrite(13,LOW);//turning LED off
  Serial.begin(115200);
  Serial.println(F("Alfa Tracker Beginning"));

  /******************* temp sensor setup *****************/
  tempsensor.begin(0x18);

  /******************* RTC setup *****************/
  pinMode(2, INPUT_PULLUP); //alarm wakeup interrupt
  RTC.setAlarm(ALM1_MATCH_DATE, 0, 0, 0, 1);
  RTC.setAlarm(ALM2_MATCH_DATE, 0, 0, 0, 1);
  RTC.alarm(ALARM_1);
  RTC.alarm(ALARM_2);
  RTC.alarmInterrupt(ALARM_1, false);
  RTC.alarmInterrupt(ALARM_2, false);
  RTC.squareWave(SQWAVE_NONE);
  
  tmElements_t tm;
  tm.Hour = 00;               // set the RTC to an arbitrary time
  tm.Minute = 00;
  tm.Second = 00;
  tm.Day = 4;
  tm.Month = 2;
  tm.Year = 2018 - 1970;      // tmElements_t.Year is the offset from 1970
  RTC.write(tm);              // set the RTC from the tm structure

  pinMode(FONA_RST, OUTPUT);
  digitalWrite(FONA_RST, HIGH); // Default state
  pinMode(FONA_PWRKEY, OUTPUT);
  powerOn();
  moduleSetup();
  Serial.println(F("SIM7000/SIM7500 LTE Demo"));
  while (!netStatus()) {
    Serial.println(F("Failed to connect to cell network, retrying..."));
    delay(2000); // Retry every 2s
  }
}

void loop() {

  /******************* temp sensor reading *****************/
  tempsensor.wake(); // Wake up the MCP9808 if it was sleeping
  tempC = tempsensor.readTempC();
  Serial.print(F("Temp: ")); Serial.print(tempC); Serial.println(F("*C\t")); 
  tempsensor.shutdown(); // In this mode the MCP9808 draws only about 0.1uA

  fona.getBattVoltage(&vbat);
  Serial.print("Vbat: "); Serial.println(vbat);
  retry = 0;
  while (!fona.enableGPS(true)) {
    Serial.println(F("Failed to turn on GPS, retrying..."));
    delay(2000); // Retry every 2s
    retry++;
    if(retry>10)break;
  }
  Serial.println(F("Turned on GPS!"));
  latitude = 0.0;
  longitude = 0.0;
  
  altitude=-1.0;
  ttff=0;
  while (!fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude)) {
    Serial.print(F("Failed to get GPS location, retrying... Waited: "));
    Serial.println(ttff);
    delay(5000); // Retry every 2s
    ttff+=5;
    if(ttff>180)break;
  }
  retry=0;
  while (!fona.enableGPRS(true)) {
    Serial.println(F("Failed to enable GPRS, retrying..."));
    delay(2000); // Retry every 2s
    retry++;
    if(retry>10)break;
  }
  Serial.println(F("Enabled GPRS!"));
  
  //latitude = 10;
  //longitude = 11;
  //altitude = 12;
  Serial.println(F("Found 'eeeeem!"));
  Serial.println(F("---------------------"));
  Serial.print(F("Latitude: ")); Serial.println(latitude, 6);
  Serial.print(F("Longitude: ")); Serial.println(longitude, 6);
  Serial.print(F("Altitude: ")); Serial.println(altitude);
  
  dtostrf(latitude, 1, 6, latBuff);
  dtostrf(longitude, 1, 6, longBuff);
  dtostrf(altitude, 1, 1, altBuff);
  dtostrf(tempC, 1, 2, tempBuff); // float_val, min_width, digits_after_decimal, char_buffer
  dtostrf(vbat, 1, 0, battBuff);
  uint16_t statuscode;
            int16_t length;
  sprintf(URL, "http://api.thingspeak.com/update?api_key=%s&field1=%s&field2=%s&field3=%s&field4=%s&field5=%s", myWriteAPIKey, altBuff, tempBuff, battBuff, latBuff, longBuff);
  Serial.println(URL);
  // Turn on GPRS
  retry = 0;
  while (!netStatus()) {
    Serial.println(F("Failed to connect to cell network, retrying..."));
    delay(2000); // Retry every 2s
    retry++;
    if(retry>10)break;
  }

  if (!fona.HTTP_GET_start(URL, &statuscode, (uint16_t *)&length)) 
  {
      Serial.println(F("Get Failed!"));
  } 
  fona.HTTP_GET_end();
  // Disable GPRS
  // Note that you might not want to check if this was successful, but just run it
  // since the next command is to turn off the module anyway
  if (!fona.enableGPRS(false)) Serial.println(F("Failed to disable GPRS!"));

  // Turn off GPS
  if (!fona.enableGPS(false)) Serial.println(F("Failed to turn off GPS!"));
  delay(1000);

  if(!fona.powerDown())
  {
    powerOff();
  }
  delay(1000);
  SetAlarm(15);
  Going_To_Sleep();
  powerOn();
  moduleSetup();
}

void Going_To_Sleep(){
    sleep_enable();//Enabling sleep mode
    attachInterrupt(0, wakeUp, LOW);//attaching a interrupt to pin d2
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);//Setting the sleep mode, in our case full sleep
    digitalWrite(LED_BUILTIN,LOW);//turning LED off
    //time_t t;// creates temp time variable
    //t=RTC.get(); //gets current time from rtc
    //Serial.println("Sleep  Time: "+String(hour(t))+":"+String(minute(t))+":"+String(second(t)));//prints time stamp on serial monitor
    delay(1000); //wait a   `second to allow the led to be turned off before going to sleep
    sleep_cpu();//activating sleep mode
    Serial.println("just woke up!");//next line of code executed after the interrupt 
    //digitalWrite(LED_BUILTIN,HIGH);//turning LED on
    RTC.alarm(ALARM_2);
    //t=RTC.get();
    //Serial.println("WakeUp Time: "+String(hour(t))+":"+String(minute(t))+":"+String(second(t)));//Prints time stamp 
  }

void SetAlarm(byte increment)
{
  time_t t; //create a temporary time variable so we can set the time and read the time from the RTC
  t=RTC.get();//Gets the current time of the RTC
  int minute_t;
  minute_t = minute(t)+increment;
  if(minute_t>=60)
  {
    minute_t-=60;
  }
  RTC.setAlarm(ALM2_MATCH_MINUTES , 0, minute_t, 0, 0);// Setting alarm 2 to go off 5 minutes from now
  // clear the alarm flag
  RTC.alarm(ALARM_2);
  // configure the INT/SQW pin for "interrupt" operation (disable square wave output)
  RTC.squareWave(SQWAVE_NONE);
  // enable interrupt output for Alarm 1
  RTC.alarmInterrupt(ALARM_2, true);
}

void wakeUp(){
  //Serial.println("Interrrupt Fired");//Print message to serial monitor
   sleep_disable();//Disable sleep mode
  detachInterrupt(0); //Removes the interrupt from pin 2;
}
void moduleSetup() {
  // Note: The SIM7000A baud rate seems to reset after being power cycled (SIMCom firmware thing)
  // SIM7000 takes about 3s to turn on but SIM7500 takes about 15s
  // Press reset button if the module is still turning on and the board doesn't find it.
  // When the module is on it should communicate right after pressing reset
  fonaSS.begin(115200); // Default SIM7000 shield baud rate
  fonaSS.println("AT+IPR=9600"); // Set baud rate
  delay(100); // Short pause to let the command run
  fonaSS.begin(9600);
  Serial.println(F("Powered On"));
  retry = 0;
  while (! fona.begin(fonaSS)) {
    Serial.println(F("Couldn't find FONA"));
    retry++;
    if(retry>5)break;
  }
  Serial.println(F("Began"));
  type = fona.type();
  Serial.println(F("FONA is OK"));
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {      
    Serial.print(F("Module IMEI: ")); Serial.println(imei);
  }
  fona.setFunctionality(1);
  fona.setNetworkSettings(F("hologram"));
  delay(1000);
  fona.setPreferredMode(38); // Use LTE only, not 2G
  fona.setPreferredLTEMode(1);

  //fona.enableRTC(true);
  
  //fona.enableSleepMode(true);
  //fona.set_eDRX(1, 4, "0010");
  //fona.enablePSM(true, 0, 0);
  //while (!fona.enableGPRS(true)) {
  //  Serial.println(F("Failed to enable GPRS, retrying..."));
  //  delay(2000); // Retry every 2s
  //}

  //type = fona.type();
  
  // Print module IMEI number.
  //uint8_t imeiLen = fona.getIMEI(imei);
}

bool netStatus() {
  int n = fona.getNetworkStatus();
  
  Serial.print(F("Network status ")); Serial.print(n); Serial.print(F(": "));
  if (n == 0) Serial.println(F("Not registered"));
  if (n == 1) Serial.println(F("Registered (home)"));
  if (n == 2) Serial.println(F("Not registered (searching)"));
  if (n == 3) Serial.println(F("Denied"));
  if (n == 4) Serial.println(F("Unknown"));
  if (n == 5) Serial.println(F("Registered roaming"));

  if (!(n == 1 || n == 5)) return false;
  else return true;
}

// Power on the module
void powerOn() {
  digitalWrite(FONA_PWRKEY, LOW);
  delay(100);  
  digitalWrite(FONA_PWRKEY, HIGH);
}

void powerOff() {
  digitalWrite(FONA_PWRKEY, LOW);
  delay(1500);  
  digitalWrite(FONA_PWRKEY, HIGH);
}

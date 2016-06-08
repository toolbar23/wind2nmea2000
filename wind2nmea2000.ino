#define N2k_SPI_CS_PIN 10
#define SDCARD_CHIP_SELECT 53
#define LED_PIN 13

#define CALIBRATE_PIN 3

#define BATTERY_RX_PIN 64 // A10
#define SOLAR_RX_PIN 63 // A10

#define BLUETOOTH_RX_PIN 33
#define BLUETOOTH_TX_PIN 31


#define CMPS11_GET_ANGLE_16_BIT 0x13

#define ENABLE_WIND_INPUT 1
#define ENABLE_BATTERY 1
#define ENABLE_SOLAR 1
#define ENABLE_TEMP 1
#define ENABLE_BLUETOOTH 1

#define DEBUG_SOLAR 0
#define DEBUG_BATTERY 0
#define DEBUG_WIND 1
#define DEBUG_ANGLE 1
#define DEBUG_NMEA2000 0
#define DEBUG_TEMP 0

#define TEMP_SEND_EVERY_NTH 8

#define CALIBRATE_PIN_NEEDS_ROUNDS 100 // every round ~ 50ms

#define NUMBER_OF_AVG_MAST 10


#include <SoftwareSerial.h>
#include <SD.h>
#include <mcp_can_dfs.h>
#include <mcp_can.h>
#include <NMEA2000_mcp.h>
#include <Arduino.h>
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <N2kMessages.h>
#include <EEPROM.h>


// for averaging the mast-rotation over the last few seconds
double mastvalues[NUMBER_OF_AVG_MAST];
int mastIdx = 0;

// flags wether the solar/battery data has been updated (updates
// are received in small bits, nmea2000 packet contains many 
// attributes in one go, so sending out is batched)
int SolarHasUpdate=0;
int BatteryHasUpdate=0;
int BatteryHasUpdateVA=0;

// for logging
File sdCardDataFile ;

// buffers to collect complete lines from the different connected ttys
String windInputBuffer = "";
String solarInputBuffer = "";
String batteryInputBuffer = "";
String bluetoothInputBuffer = "";

float lastWindAngleUncorrected;
bool debugViaBluetooth  = true;

SoftwareSerial batterySerial(BATTERY_RX_PIN, NULL); // RX, TX
SoftwareSerial solarSerial(SOLAR_RX_PIN, NULL); // RX, TX
SoftwareSerial bluetoothSerial(BLUETOOTH_RX_PIN, BLUETOOTH_TX_PIN);


// ID of the settings block
#define CONFIG_VERSION "ws2"
#define CONFIG_START 32

#define SERIAL_WAIT_MILLIS 40
#define SERIAL_WAIT_STEP_MILLIS 1


/*
 * 
 * data structure for configuration storage
 * 
 */
 
struct ConfigurationStoreStruct {
  // This is for mere detection if they are your settings
  char version[4];
  // The variables of your settings
  double offsetMastSensors;
} storage = {
  CONFIG_VERSION,
  0
};

typedef struct {
  unsigned long PGN;
  void (*Handler)(const tN2kMsg &N2kMsg); 
} tNMEA2000Handler;



void loadConfig() {
  // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
  Serial.println("Loading Config2");
  if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] &&
      EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] &&
      EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2]) {
    for (unsigned int t = 0; t < sizeof(storage); t++)
      *((char*)&storage + t) = EEPROM.read(CONFIG_START + t);
    Serial.println("Loaded");
  }

}

void saveConfig() {
  for (unsigned int t = 0; t < sizeof(storage); t++)
    EEPROM.write(CONFIG_START + t, *((char*)&storage + t));
}











/*
 * LOGGING
 */
 
void llog(String s) {
  //Serial.print(s);
  sdCardDataFile.print(s);
}

void llog(double s) {
  //Serial.print(s);
  sdCardDataFile.print(s);
}


void llogln(String s) {
  //Serial.println(s);
  sdCardDataFile.println(s);
  sdCardDataFile.flush();
}






/***
 * Handler for updates from/to NMEA2000 network
 * I want to log some stuff to the sd card for later analysis
 * (polar creation)
 ***/

void SystemDateTimeHandler(const tN2kMsg &N2kMsg);
void WindHandler(const tN2kMsg &N2kMsg);
void BoatSpeedHandler(const tN2kMsg &N2kMsg);

tNMEA2000Handler NMEA2000Handlers[]={
  {126992L,&SystemDateTimeHandler},
  {130306L,&WindHandler},
  {128259L,&BoatSpeedHandler},
  {0,0}
};


void SystemDateTimeHandler(const tN2kMsg &N2kMsg) {
  
  uint16_t SystemDate;
  unsigned char SID;
  double SystemTime;
  tN2kTimeSource TimeSource;
                     
  if (ParseN2kPGN126992(N2kMsg,SID,SystemDate,SystemTime,TimeSource) ) {
      llog("PGN126992");
      if (SystemDate>0) {
        llog("|SystemDate:");
        llog(SystemDate);
      }
      if (SystemTime>0) {
        llog("|SystemTime:");
        llog(SystemTime);
      }
      if (TimeSource>0) {
        llog("|TimeSource:");
        llog(TimeSource);
      }
      llogln("");
    }
}

void WindHandler(const tN2kMsg &N2kMsg) {
  
  unsigned char SID;
  double WindSpeed;
  double WindAngle;
  tN2kWindReference WindReference;
                     
  if ( ParseN2kPGN130306(N2kMsg, SID, WindSpeed, WindAngle, WindReference) ) {
      llog("PGN130306");
     
        llog("|WindSpeed:");
        llog(WindSpeed);
   
        llog("|WindAngle:");
        llog(WindAngle);
   
        llog("|WindReference:");
        llog(WindReference);
     
      llogln("");
    }
}


void BoatSpeedHandler(const tN2kMsg &N2kMsg) {
  
  unsigned char SID;
  double WaterRefereced;
  double GroundReferenced;
  tN2kSpeedWaterReferenceType SWRT;

  if ( ParseN2kPGN128259(N2kMsg, SID, WaterRefereced, GroundReferenced, SWRT) ) {
      llog("PGN130306");
      if (WaterRefereced>0) {
        llog("|WaterReferenced:");
        llog(WaterRefereced);
      }
      if (GroundReferenced>0) {
        llog("|GroundReferenced:");
        llog(GroundReferenced);
      }
      if (SWRT>0) {
        llog("|SWRT:");
        llog(SWRT);
      }
      llogln("");
    }
}
 


//NMEA 2000 message handler
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  int iHandler;
  // Find handler
  for (iHandler=0; NMEA2000Handlers[iHandler].PGN!=0 && !(N2kMsg.PGN==NMEA2000Handlers[iHandler].PGN); iHandler++);
  if (NMEA2000Handlers[iHandler].PGN!=0) {
    NMEA2000Handlers[iHandler].Handler(N2kMsg); 
  }
}






void setup() {

  // Console
  Serial.begin(115200);
  loadConfig();

  // Set Product information
  NMEA2000.SetProductInformation("00000004", // Manufacturer's Model serial code
                                 105, // Manufacturer's product code
                                 "Wind System 2",  // Manufacturer's Model ID
                                 "1.0.0.11 (2016-02-10)",  // Manufacturer's Software version code
                                 "1.0.0.0 (2016-02-10)" // Manufacturer's Model version
                                );
  NMEA2000.SetDeviceInformation(1231, // Unique number. Use e.g. Serial number.
                                130, // Device function=Temperature. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                85, // Device class=Sensor Communication Interface. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                1976 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                               );

  // Rotation
  Serial1.begin(9600, SERIAL_8N2);
  Serial2.begin(9600, SERIAL_8N2);
  //Serial1.begin(4800,SERIAL_8N1);
  //Serial2.begin(4800,SERIAL_8N1);

  // NMEA0183 Wind
  Serial3.begin(4800, SERIAL_8N1);
  windInputBuffer.reserve(200);

  // Battery
  solarSerial.begin(19200);
  batterySerial.begin(19200);
  bluetoothSerial.begin(9600);  //
  batteryInputBuffer.reserve(200);
  solarInputBuffer.reserve(200);
  bluetoothInputBuffer.reserve(200);

  // NMEA2000 CAN-Bus
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.
  NMEA2000.SetForwardOwnMessages();
  if ( DEBUG_NMEA2000 ) {
    NMEA2000.EnableForward(true);
  } else {
    NMEA2000.EnableForward(false);
  }
  
  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  //NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 24);//NORMAL
  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, 24);
  
  Serial.println("debug");
  NMEA2000.Open();

  Serial.println("opened");

  // Info-LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // SDCard
  Serial.print("Initializing SD card...");
  pinMode(SDCARD_CHIP_SELECT, OUTPUT);
  if (!SD.begin(SDCARD_CHIP_SELECT)) {
    Serial.println("Card failed, or not present");
  }
 Serial.println("card initialized.");
 
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  sdCardDataFile = SD.open("datalog.txt", FILE_WRITE);


  

}


int resendInitializationEveryMillis = 10000;
int resendInformationCounter = resendInitializationEveryMillis;

int calibratePinCounter = CALIBRATE_PIN_NEEDS_ROUNDS;

void loop() {
  NMEA2000.ParseMessages();

  int calibrationButtonValue = digitalRead(CALIBRATE_PIN);

  resendInformationCounter--;
  
  if ( resendInformationCounter <= 0 ) {
    
    NMEA2000.SendIsoAddressClaim(0xff,0);
    NMEA2000.SendProductInformation(0);
    NMEA2000.SendConfigurationInformation(0);
    resendInformationCounter = resendInitializationEveryMillis;
    NMEA2000.Open();  
    
    SolarHasUpdate=1;
    BatteryHasUpdate=1;
    BatteryHasUpdateVA=1;

    sendBatteryStatus();
    sendSolarStatus();
    
  } 
  
  else if ( batterySerial.available() && ENABLE_BATTERY ) {
      processBatteryCharacter(batterySerial.read());
  } 

  else  if ( solarSerial.available() && ENABLE_SOLAR ) {
      processSolarCharacter(solarSerial.read());
  } 
  
  else  if ( bluetoothSerial.available() && ENABLE_BLUETOOTH ) {
      processBluetoothCharacter(bluetoothSerial.read());
  } 
  else if ( calibrationButtonValue == HIGH ) {

    delay(50);
    if ( calibratePinCounter-- == 0 ) {
      calibrateAngles();
    }    
    
 } else {
    calibratePinCounter = CALIBRATE_PIN_NEEDS_ROUNDS;
    resendInformationCounter -= 50;
    delay(50);
  }
  

}








// Blinking lights for informationspun


void signalLong() {
  Serial.println("LONG");
  digitalWrite(LED_PIN, HIGH);
  delay(5000);
  digitalWrite(LED_PIN, LOW);
}

void signalStart() {
  Serial.println("START");
  for ( int i = 0; i < 10; i++ ) {
    Serial.println(i);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}

void signalBlinkShort(int millis = 200) {
  for ( int i = 0; i < millis; i += 100 ) {
    Serial.println(i);
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
    delay(50);
  }
}






// Calibration

int calibrateAnglesMinus() {

  storage.offsetMastSensors =  storage.offsetMastSensors - 1;
  Serial.print("Update Mast-Correction to ");
  Serial.print(storage.offsetMastSensors);
  Serial.println(".");
  saveConfig();
  
  
}

int calibrateAnglesPlus() {

  storage.offsetMastSensors =  storage.offsetMastSensors - 1;
  Serial.print("Update Mast-Correction to ");
  Serial.print(storage.offsetMastSensors);
  Serial.println(".");
  saveConfig();
  
}

int calibrateAngles() {

  signalStart();
  Serial.println("Start calibration");

  float offsetSum = 0;
  int offsetCount = 0;

  for ( int i = 0; i < 10; i++ ) {

    float mastAngle = readMastAngle();
    debug("got mastAngle ");
    debugln(mastAngle);

    float windAngle  = lastWindAngleUncorrected;
    debug(" and windAngle ");
    debugln(windAngle);

    float offset = - windAngle - mastAngle;
    debug(" = offset ");
    debugln(offset);

    offsetCount++;
    offsetSum += offset;

    signalBlinkShort(500);
  }

  float offsetAverage = offsetSum / offsetCount;
  debug(" = offsetAverage ");
  debugln(offsetAverage);

  
  storage.offsetMastSensors = offsetAverage;

  saveConfig();
  debugln("Stored");

  signalLong();

}





/*
   Rotation Sensing
*/




int waitForInputSerial1(int timer = SERIAL_WAIT_MILLIS) {

  while ( timer > 0 && Serial1.available() == 0 ) {
      if ( DEBUG_ANGLE ) { 
        debugln("waiting for angle1");
      }

    delay(SERIAL_WAIT_STEP_MILLIS);
    timer -= SERIAL_WAIT_STEP_MILLIS;
  }
  return Serial1.available();
}

int waitForInputSerial2(int timer = SERIAL_WAIT_MILLIS) {

  while ( timer > 0 && Serial2.available() == 0 ) {
    if ( DEBUG_ANGLE ) { 
      debugln("waiting for angle2");
    }
    delay(SERIAL_WAIT_STEP_MILLIS);
    timer -= SERIAL_WAIT_STEP_MILLIS;
  }
  return Serial2.available();
}


double readAngle1() {
  
  Serial1.write(CMPS11_GET_ANGLE_16_BIT);
  if ( waitForInputSerial1() ) {
    int byteHigh1 = Serial1.read();


    if ( waitForInputSerial1() ) {

      int byteLow1 = Serial1.read();

      double angle = ((byteHigh1 << 8 ) + byteLow1) / 10.0;
      if ( DEBUG_ANGLE ) { 
        debug("received angle1 ");
        debugln(angle);
      }

      return angle;

    }
  }
  if ( DEBUG_ANGLE ) { 
    debugln("no input from angle1");
  }
  return -1;
}

double readAngle2() {
  Serial2.write(CMPS11_GET_ANGLE_16_BIT);
  if ( waitForInputSerial2() ) {
    int byteHigh1 = Serial2.read();
    if ( waitForInputSerial2() ) {
      int byteLow1 = Serial2.read();
      double angle = ((byteHigh1 << 8 ) + byteLow1) / 10;
      if ( DEBUG_ANGLE ) { 
        debug("received angle2 ");
        debugln(angle);
      }
      return angle;
    }
  }
  if ( DEBUG_ANGLE ) { 
    debugln("no input from angle2");
  }
  return -1;

}

double readMastAngle() {

  double angle1 = readAngle1();
  double angle2 = readAngle2();

  
  if ( angle1 > -1 && angle2 > -1 ) {
    double mastAngle = angle1 - angle2;
    if ( DEBUG_ANGLE ) {
      debug("got angle ");
      debugln(mastAngle);
    }
    return mastAngle;
  } else {
    if ( DEBUG_ANGLE ) {
      debugln("at least one angle is missing");
    }
    return -1;
  }

}





/*
    Input from the main tty
*/

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();

    if ( inChar == 'c' ) {
      calibrateAngles();
    }
    else if ( inChar == 'p' ) {
      calibrateAnglesPlus();
    }
    else if ( inChar == 'm' ) {
      calibrateAnglesMinus();
    }
  }
}

/*
    Input from the bluetooth tty
*/


void processBluetoothCharacter(char inChar) {

  // add it to the windInputBuffer:
  bluetoothInputBuffer += inChar;
  // if the incoming character is a newline, set a flag
  // so the main loop can do something about it:

  int ascii = inChar;


  if ( inChar == 'c' ) {
    calibrateAngles();
  }
  else if ( inChar == 'p' ) {
    calibrateAnglesPlus();
  }
  else if ( inChar == 'm' ) {
    calibrateAnglesMinus();
  }
  else if ( inChar == 'd' ) {
    debugViaBluetooth = debugViaBluetooth ? false : true;
  }

  if (inChar == '\n' || inChar == '\r') {
    //processSolarLine(solarInputBuffer);
    bluetoothInputBuffer  = "";
  }

}




/*
    Wind Input
*/

void serialEvent3() {

  while (Serial3.available()) {
    // get the new byte:
    char inChar = (char)Serial3.read();
    //Serial.print(inChar);
    processWindCharacter(inChar);
  }
}

void processWindCharacter(char inChar) {

  // add it to the windInputBuffer:
  windInputBuffer += inChar;
  
  // if the incoming character is a newline, set a flag
  // so the main loop can do something about it:

  int ascii = inChar;

  if (inChar == '\n' || inChar == '\r' ) {
    processNMEA0183Line(windInputBuffer);
    windInputBuffer  = "";
  }

}


double avgMastvalues(double newVal) {

  mastvalues[mastIdx++] = newVal;
  if ( mastIdx == NUMBER_OF_AVG_MAST ) {
      mastIdx = 0;
  }
  double sum = 0;
  for (int i=0; i<NUMBER_OF_AVG_MAST; i++ ) {
    sum += mastvalues[i];
    if(i == mastIdx-1) { debug("*"); }
    debug("mv ");
    debug(i);
    debug(" ");
    debug(mastvalues[i]);
    debugln("");
  }
  return sum / NUMBER_OF_AVG_MAST;
}


int tempReceiveCounter = 0;

void processNMEA0183Line(String s) {

  if ( ENABLE_TEMP && getValue(s, ',', 0).equals("$WIXDR") && tempReceiveCounter-- == 0 ) { // for temp-lines

    tempReceiveCounter = TEMP_SEND_EVERY_NTH;
    
    String tempString = getValue(s, ',', 2);
    double temp = StringToDouble(tempString);
    
    if ( DEBUG_TEMP ) {
      debug("got temp ");
      debugln(temp);
    }  

    if ( DEBUG_TEMP || DEBUG_NMEA2000 ) {
        debug("nmea-send temp, temp ");
        debugln(temp);
    }
    
    tN2kMsg N2kMsg;
    SetN2kTemperature(N2kMsg, 1, 1, N2kts_OutsideTemperature, CToKelvin(temp));
    NMEA2000.SendMsg(N2kMsg);

  }
  
  else if ( ENABLE_WIND_INPUT && getValue(s, ',', 0).equals("$IIMWV") ) { // for wind-lines

    String windAngleString = getValue(s, ',', 1);
    String windSpeedString = getValue(s, ',', 3);

    double windAngle = StringToDouble(windAngleString);
    lastWindAngleUncorrected = windAngle;

    double windSpeed = StringToDouble(windSpeedString);

    double mastAngle = avgMastvalues(readMastAngle());


    if ( DEBUG_WIND ) {
      debug("got wind, speed ");
      debug(windSpeed);
      debug(", angle uncorrected ");
      debug(windAngle);
      debug(", mastAngle ");
      debug(mastAngle);
      debug(", offset ");
      debug(storage.offsetMastSensors); 
    }

    windAngle += mastAngle + storage.offsetMastSensors;
    windAngle = fmod(windAngle, 360.0);

    windAngle = round(windAngle);
    if (windAngle < 0 ) {
      windAngle = 360 + windAngle;
    }

    if ( DEBUG_WIND ) {
      debug(", calculated ");
      debugln(windAngle); 
    }

    tN2kMsg N2kMsg;

    double windAngleInRad = DegToRad(windAngle);
    if  (  windSpeed > -1 ) {
     
      if ( DEBUG_WIND || DEBUG_NMEA2000 ) {
        debug("nmea-send wind, speed ");
        debug(windSpeed);
        debug(", angle ");
        debugln(windAngle);
      }
 
      SetN2kWindSpeed(N2kMsg, 1, KnotsToMetersPerSecond(windSpeed), windAngleInRad, N2kWind_Apprent);
      NMEA2000.SendMsg(N2kMsg);
      WindHandler(N2kMsg);
      
    }

  }

}


double KnotsToMetersPerSecond(double knots) {
  return knots * 0.514444;
}






/*
    Helper
*/



double StringToDouble (String s) {
  char buf[15]; //make this the size of the String
  s.toCharArray(buf, 15);

  char *endptr;
  float num;
  num = strtod(buf, &endptr);
 // if (*endptr != '\0')
  //  return -1;
  return num;
}


String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = { 0, -1  };
  int maxIndex = data.length() - 1;
  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}


void debugln(String s) {
  Serial.println(s);
  if ( debugViaBluetooth ) {
    bluetoothSerial.println(s);
  }
}


void debugln(double s) {
  Serial.println(s);
  if ( debugViaBluetooth ) {
    bluetoothSerial.println(s);
  }
}


void debug(String s) {
  Serial.print(s);
  if ( debugViaBluetooth ) {
    bluetoothSerial.print(s);
  }
}


void debug(double s) {
  Serial.print(s);
  if ( debugViaBluetooth ) {
    bluetoothSerial.print(s);
  }

}





/*
 * 
 * solar and battery updates from Victron 
 * 
 */




double SolarVolt; // SOC
double SolarAmp; // SOC

void updateSolarVolt(double value){
  SolarVolt=value;
  SolarHasUpdate=1;
}
void updateSolarAmp(double value){
  SolarAmp=value;
  SolarHasUpdate=1;
}


void sendSolarStatus(){

  if ( SolarHasUpdate){
    tN2kMsg N2kMsg;
    SetN2kPGN127508(N2kMsg, 2, SolarVolt, SolarAmp, 0);
    if ( DEBUG_SOLAR || DEBUG_NMEA2000 ) {
      debug("nmea-send solar status 127508, volt ");
      debug(SolarVolt);
      debug(", amp ");
      debugln(SolarAmp);
    }
    NMEA2000.SendMsg(N2kMsg);

    SolarHasUpdate=0;
  }
  
}


double BatteryVolt=0; // SOC
double BatteryAmp=0; // SOC

double BatteryStateOfCharge=0; // SOC
double BatteryTimeToGo=0; // TTG


void updateBatteryStateOfCharge(double value){
  BatteryStateOfCharge=value;
  BatteryHasUpdate=1;
}
void updateBatteryVolt(double value){
  BatteryVolt=value;
  BatteryHasUpdateVA=1;
}
void updateBatteryAmp(double value){
  BatteryAmp=value;
  BatteryHasUpdateVA=1;
}
void updateBatteryTimeToGo(double value){
  BatteryTimeToGo=value;
  BatteryHasUpdate=1;
}


void sendBatteryStatus(){
  if  ( BatteryHasUpdateVA){
    tN2kMsg N2kMsg;
    SetN2kPGN127508(N2kMsg, 1, BatteryVolt, BatteryAmp, 0);
    if ( DEBUG_BATTERY || DEBUG_NMEA2000 ) {
      debug("nmea-send battery status 127508, volt ");
      debug(BatteryVolt);
      debug(", amp ");
      debugln(BatteryAmp);
    }
    NMEA2000.SendMsg(N2kMsg);
    BatteryHasUpdateVA=0;
  }
  if  ( BatteryHasUpdate){
    tN2kMsg N2kMsg2;
    SetN2kPGN127506(N2kMsg2, 1, 1, N2kDCt_Battery, BatteryStateOfCharge, 100, BatteryTimeToGo, 0);
    if ( DEBUG_BATTERY || DEBUG_NMEA2000 ) {
      debug("nmea-send battery status 127506, soc ");
      debug(BatteryStateOfCharge);
      debug(", ttg ");
      debugln(BatteryTimeToGo);
    }
    NMEA2000.SendMsg(N2kMsg2);
    
    BatteryHasUpdate=0;
  }  
}


void processBatteryCharacter(char inChar) {

  // add it to the windInputBuffer:
  batteryInputBuffer += inChar;
  // if the incoming character is a newline, set a flag
  // so the main loop can do something about it:

  int ascii = inChar;

  if (inChar == '\n' || inChar == '\r') {
    processBatteryLine(batteryInputBuffer);
    batteryInputBuffer  = "";
  }

}


void processSolarCharacter(char inChar) {

  // add it to the windInputBuffer:
  solarInputBuffer += inChar;
  // if the incoming character is a newline, set a flag
  // so the main loop can do something about it:

  int ascii = inChar;

  if (inChar == '\n' || inChar == '\r') {
    processSolarLine(solarInputBuffer);
    solarInputBuffer  = "";
  }

}



#define VICTRON_SEPARATOR '\t'

void processBatteryLine(String s) {

  String valueString= getValue(s, VICTRON_SEPARATOR, 1);
  double value = StringToDouble(valueString);

  if ( getValue(s, VICTRON_SEPARATOR, 0).equals("V") ) {
    value /= 1000;
    updateBatteryVolt(value);
    if ( DEBUG_BATTERY ) {
      debug("got battery volt ");
      debugln(value);
    }
  }
  else if ( getValue(s, VICTRON_SEPARATOR, 0).equals("I") ) {
    value /= 1000;
    updateBatteryAmp(value);
    if ( DEBUG_BATTERY ) {
      debug("got battery amp ");
      debugln(value);
    }
  }
  else if ( getValue(s, VICTRON_SEPARATOR, 0).equals("SOC") ) {
    value /= 10;
    updateBatteryStateOfCharge(value);
    if ( DEBUG_BATTERY ) {
      debug("got battery soc ");
      debugln(value);
    }
  }
  else if ( getValue(s, VICTRON_SEPARATOR, 0).equals("TTG") ) {
    if ( value < 0 ) {
      value = 14400;
    }
    updateBatteryTimeToGo(value);
    if ( DEBUG_BATTERY ) {
      debug("got battery ttg ");
      debugln(value);
    }

  }
}


void processSolarLine(String s) {

  String valueString= getValue(s, VICTRON_SEPARATOR, 1);
  double value = StringToDouble(valueString);

  if ( getValue(s, VICTRON_SEPARATOR, 0).equals("VPV") ) {
    value /= 1000;
    updateSolarVolt(value);
    if ( DEBUG_SOLAR ) {
      debug("got solar volt ");
      debugln(value);
    }
  }
  else if ( getValue(s, VICTRON_SEPARATOR, 0).equals("I") ) {
    value /= 1000;
    updateSolarAmp(value);
    if ( DEBUG_SOLAR ) {
      debug("got solar amp ");
      debugln(value);
    }
  }
}



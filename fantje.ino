#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <TTN_esp32.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <U8x8lib.h>

/***************************************************************************
 *  Go to your TTN console register a device then the copy fields
 *  and replace the CHANGE_ME strings below
 ****************************************************************************/
// ABP
const char* devAddr = "CHANGE ME"; // Change to TTN Device Address
const char* nwkSKey = "CHANGE ME"; // Change to TTN Network Session Key
const char* appSKey = "CHANGE ME"; // Change to TTN Application Session Key
// OTAA
const char* devEui = "CHANGE_ME"; // Change to TTN Device EUI
const char* appEui = "CHANGE_ME"; // Change to TTN Application EUI
const char* appKey = "CHANGE_ME"; // Chaneg to TTN Application Key
int8_t   retries    = 10;
uint32_t retryDelay = 10000;

TTN_esp32 ttn;
Adafruit_BMP085 bmp;

const boolean useOTAA = false;
const unsigned long sentwait= 15000;
const unsigned long gpswait = 1000;
const float minimumDistance = 1.0;
unsigned long lastsent;
const float mindist = 250.0;

/***********
 * GPS stuff
 */
TinyGPSPlus gps;
TinyGPSCustom viewGPS(gps, "GPGSV", 3);
TinyGPSCustom viewGAL(gps, "GAGSV", 3);
TinyGPSCustom viewGLO(gps, "GLGSV", 3);
TinyGPSCustom viewBD2(gps, "BDGSV", 3);

#define GPS_RX 36
#define GPS_TX 37
HardwareSerial GPSSerial(1);

/**
 * Screen and LEDs
 */
#define BUILTIN_LED 25
// the OLED used
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

/**********************************
 * Packing the variables into bytes
 */
typedef union {
  float f[6];                // Assigning fVal.f will also populate fVal.bytes;
  unsigned char bytes[24];   // Both fVal.f and fVal.bytes share the same 4 bytes of memory.
} floatArr2Val;
floatArr2Val packed_message;

struct gnssLocation {
  float latitude;
  float longitude;
  float altitude;
  float hdop;
};
struct gnssLocation lastLocation = {0.0, 0.0, 0.0, 1000.0};
struct gnssLocation currentLocation;

/**********************************
 * 
 */
void message(const uint8_t* payload, size_t size, int rssi)
{
  Serial.println("-- MESSAGE");
  Serial.print("Received " + String(size) + " bytes RSSI=" + String(rssi) + "db");
  for (int i = 0; i < size; i++) {
    Serial.print(" " + String(payload[i]));
    // Serial.write(payload[i]);
  }
  Serial.println();
}

/*****************************************
 * Get the coordinates from the gps stream
 **/

boolean get_coords (unsigned long ms) {
  // Feed loop
  unsigned long start = millis();
  while (GPSSerial.available() && (millis()-start)<ms) gps.encode(GPSSerial.read());
  
  uint8_t year  = gps.date.year();
  uint8_t month = gps.date.month();
  uint8_t day   = gps.date.day();
  uint8_t hour  = gps.time.hour();
  uint8_t min   = gps.time.minute();
  uint8_t sec   = gps.time.second();
 
  float latitude  = gps.location.lat();
  float longitude = gps.location.lng();
  float altitude  = gps.altitude.meters();
  float speed     = gps.speed.mps();
  float course    = gps.course.deg();
  float hdop      = gps.hdop.value()/100.0;

  Serial.print("Location (");
  Serial.print(latitude);
  Serial.print(", ");
  Serial.print(longitude);
  Serial.print(") HDOP ");
  Serial.println(hdop);

  int ngps = atoi(viewGPS.value());
  int nbd2 = atoi(viewBD2.value());
  int ngal = atoi(viewGAL.value());
  int nglo = atoi(viewGLO.value());
    
  Serial.print("GPS ");
  Serial.print(ngps);
  Serial.print(", Galileo ");
  Serial.print(ngal);
  Serial.print(", Glonass ");
  Serial.print(nglo);
  Serial.print(", Beidou ");
  Serial.println(nbd2);
    
  // if (latitude  != 0.0 && latitude  != packed_message.f[0] && 
  //     longitude != 0.0 && longitude != packed_message.f[1]) {

  if (latitude && longitude) {
    packed_message.f[0] = latitude;
    packed_message.f[1] = longitude;
    packed_message.f[2] = altitude;
    packed_message.f[3] = hdop;
    
    floatDisplay(1,"Lat",latitude);
    floatDisplay(2,"Lon",longitude);
    floatDisplay(3,"DOP",hdop);
    timeDisplay(6,hour, min, sec);
       
    return true;  
  } else {
    intDisplay(2,0,"GPS",ngps);
    intDisplay(2,7,"GAL",ngal);
    intDisplay(3,0,"GLO",nglo);
    intDisplay(3,7,"BD2",nbd2);
    timeDisplay(6,hour, min, sec);
    Serial.println("No new location");
    return false;
  }
}

/******************************************************
 * BMP180 pressure sensor
 */
boolean get_pressure() {
  float prs = bmp.readPressure()/100.0;
  float tmp = bmp.readTemperature();
  floatDisplay(4,"P",prs);
  floatDisplay(5,"T",tmp);
  packed_message.f[4] = prs; 
  packed_message.f[5] = tmp;  
}

/******************************************************
 * Show a float value (preceded by text) on the display
 */
void floatDisplay (uint8_t l, char *t, float f) {
  char s[16]; // used to sprintf for OLED display
  u8x8.setCursor(0, l);
  //u8x8.print(t);
  //u8x8.setCursor(sizeof(t),l);
  //sprintf(s, "%f", f);
  sprintf(s,"%s %.2f", t, f);
  u8x8.print(s);
}

/*********************************************************
 * Show an integer value (preceded by text) on the display
 */
void intDisplay (uint8_t l, uint8_t c, char *t, uint8_t i) {
  char s[16]; // used to sprintf for OLED display
  u8x8.setCursor(c, l);
  //u8x8.print(t);
  //u8x8.setCursor(sizeof(t),l);
  //sprintf(s, "%d", i);
  sprintf(s,"%s %02d", t, i);
  u8x8.print(s);
}


/******************************
 * Show the time on the display
 */
void timeDisplay(uint8_t l, uint8_t hr, uint8_t mn, uint8_t sc) {
  char s[16]; // used to sprintf for OLED display
  u8x8.setCursor(0, l);
  sprintf(s, "T %.2d:%.2d:%.2d", hr, mn, sc);
  u8x8.print(s);
}

/******************************
 * Distance from cosine rule
 */
float distance(gnssLocation one, gnssLocation two) {
  float pi = 4*atan(1.0);
  float rad = pi / 180.0;
  float d=6378139.0*acos(sin(rad*one.latitude)*sin(rad*two.latitude)+
               cos(rad*one.longitude)*cos(rad*two.longitude)*cos(rad*(one.longitude-two.longitude)));
  return d;
}

/**********************************************************************
 * Setup of the board, open GPS stram, connect to sensors, init screen.
 * Try to connect to TTN through OTAA, if this fails use ABP.
 */
void setup()
{
  Serial.begin(115200);
  Serial.println("Starting");

  pinMode(BUILTIN_LED, OUTPUT);

  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  GPSSerial.setTimeout(2);

  // Set Galileo support by sending out a magic UBX-CFG-GNSS string
  // GPSSerial.write();
   
  if (!bmp.begin()) {
    Serial.println("Could not find the BMP180 sensor.");
  }
  
  u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.drawString(0, 1, "Fantje TTN"); 
  u8x8.drawString(0, 7, "Setup");
    
  /** 
   * Try to connect through OTAA for a minute. 
   */
  ttn.begin();
  ttn.onMessage(message); // Declare callback function for handling downlink messages from server

  if (useOTAA) {
    Serial.println("Joining TTN ");
    u8x8.drawString(0, 7, "Joining TTN");
    ttn.join(devEui, appEui, appKey, 0, retries, retryDelay);
    int startmillis = millis();
    while (!ttn.isJoined() && (millis()-startmillis)<(retries*retryDelay)) {
      Serial.print(".");
      delay(500);
    }
  }
  if (useOTAA && ttn.isJoined()) {
    Serial.println("\njoined !");
    u8x8.drawString(0, 7, "TTN OTAA       ");
  } else {
    Serial.println("Using ABP");
    ttn.personalize(devAddr, nwkSKey, appSKey);
    u8x8.drawString(0, 7, "TTN ABP        ");
  }
  lastsent = millis();
  ttn.showStatus(); 
  Serial.println("Setup done");
}

/************************************************************************
 * Main loop, try to get a new location, if so send out the packet to TTN
 */
void loop() {
  char* senttxt;
  
  // Get sensor values here
  get_pressure();
  unsigned long  now = millis();
  if ((now-lastsent)>sentwait) {
    if (get_coords(gpswait)) {
      double d = distance(lastLocation, currentLocation);
      Serial.print("Distance to previous point ");
      Serial.println(d);
      // if (d > minimumDistance) {
        if (ttn.sendBytes(packed_message.bytes, sizeof(packed_message.bytes))) {
          lastLocation = currentLocation;
          floatDisplay(7,"Sent",d);
          Serial.print("Sent ");
          Serial.print(sizeof(packed_message.bytes));
          Serial.println(" bytes");
          digitalWrite(BUILTIN_LED, HIGH); delay(1000); digitalWrite(BUILTIN_LED, LOW);
       // } else {
       //   floatDisplay(7,"Dist",d);
        }
      // } 
    } 
    lastsent=now;
  } else {
    int dt = (sentwait-now+lastsent)/1000;
    intDisplay(7,0,"Skip",dt);
  }
}

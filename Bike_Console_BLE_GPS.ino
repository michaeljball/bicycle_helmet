
#include <M5Stack.h> 
#include "utility/MPU9250.h"
#include "utility/quaternionFilters.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>
#include <TinyGPS++.h>


Preferences preferences;
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;


// See the following for generating UUIDs:
// https://www.uuidgenerator.net/


#define TRUE  1
#define FALSE  0

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;


// The serial connection to the GPS device
HardwareSerial ss(2);

#define TFT_GREY 0x5AEB

#define LOOP_PERIOD 35 // Display updates every 35 ms
#define BLINK_PERIOD 500  // Spped of signal blink  half second

// States of operation 
#define FORWARD 0
#define LEFT 1
#define RIGHT 2
#define STOP 3


// Boolean state of switches to send
union {
  uint8_t STATE =0;    // All off
  struct {
    uint8_t  s1 : 1; // bit position 0
    uint8_t  s2 : 2; // bit positions 1..2
    uint8_t  s3 : 3; // bit positions 3..5
    uint8_t  s4 : 2; // bit positions 6..7 
    // total # of bits just needs to add up to the uint8_t size
  } current;
} state;

int timezone = -4;
int leftButton;
int rightButton;
int stopButton;
int brakes = 0;

float ltx = 0;    // Saved x coord of bottom of needle
uint16_t osx = 120, osy = 120; // Saved x & y coords
uint32_t nextTime = 0;       // time for next update
uint32_t blinkDelay = 0;     // time for next update
int blinkValue = 0;

#define sound 1               // Enable beeps 

int old_analog =  -999; // Value last displayed
int old_digital = -999; // Value last displayed

uint8_t nValue = 0;
int value[6] = {0, 0, 0, 0, 0, 0};
int old_value[6] = { -1, -1, -1, -1, -1, -1};
int d = 0;

int pre_HD = 0;
float HD = 5;                                   // Heading new
float HD_old = 360;                             // Heading old
uint16_t cx, cy;                                // center x,y
const uint16_t rad = 40;                        // radius of length
const float deg = 2 * PI / 360;                 // radian to degree

int altitude = 0;                 // Current altitude from sea level
float trip = 0;                   // Current trip meter
float temperature = 0;            // Current temperature
int curSpeed;                     // Current Speed

float startLAT, startLNG;         // Starting Latitude & Longitude

int GuageX = 30;                   // Position of Speedometer in screen
int GuageY = 60;
int SignalsX = 120;                // Position of Signal Indicators on screen
int SignalsY = 36;

MPU9250 IMU;
// Kalman kalmanX, kalmanY, kalmanZ; // Create the Kalman instances


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};




void setup(void) {
  M5.begin();
  Wire.begin();
  ss.begin(GPSBaud);

  // Create the BLE Device
  BLEDevice::init("MyHel");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pServer->getAdvertising()->addServiceUUID(SERVICE_UUID);
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
  
  byte c = IMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);

    Serial.println("MPU9250 is online...");

    // Start by performing self test and reporting values
    IMU.MPU9250SelfTest(IMU.SelfTest);
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(IMU.SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(IMU.SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(IMU.SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(IMU.SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(IMU.SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(IMU.SelfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);

        IMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

        IMU.tempCount = IMU.readTempData();  // Read the adc values
        // Temperature in degrees Centigrade
        IMU.temperature = ((float) IMU.tempCount) / 333.87 + 21.0;
        temperature = IMU.temperature;
        // Print temperature in degrees Centigrade
        Serial.print("Temperature is ");  Serial.print(temperature, 1);
        Serial.println(" degrees C");
        Serial.println("");


  
  // M5.Lcd.setRotation(2);
  M5.Lcd.fillScreen(TFT_BLACK);
  
  M5.Lcd.setTextColor(TFT_WHITE);
 
//  M5.Lcd.drawRightString("4:15 PM", 200, 4, 4);                // Draw Altitude
  while (ss.available() > 0)   gps.encode(ss.read());            // Get GPS data

  startLNG = gps.location.lng();
  startLAT = gps.location.lat();  
    
  drawSpeedometer(GuageX,GuageY); // Draw analogue meter
  drawCompass(40,37);

  drawSignals(SignalsX,SignalsY);
  
  nextTime = millis(); // Next update time
}


void loop() {
    
  while (ss.available() > 0)   gps.encode(ss.read());  // Get GPS data
  
  if (nextTime <= millis()) {
    nextTime = millis() + LOOP_PERIOD;

    HD = gps.course.deg();

    updateTime(140,4);

    updateThermometer(260,0);                       // Place the current temperature in the top right corner

    updateSpeedometer(gps.speed.kmph(), 0, GuageX,GuageY);   // Update speedometer needle position

    updateCompass(40,40);

    
     M5.update();
     updateState();
     updateSignals();

    // notify changed value
    if (deviceConnected) {
        pCharacteristic->setValue(&state.STATE, 1);
        pCharacteristic->notify();
        nValue++;
        smartDelay(10); // bluetooth stack will go into congestion, if too many packets are sent

    }
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        smartDelay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }   
  }
    if (millis() > 5000 && gps.charsProcessed() < 10)
    M5.Lcd.println(F("No GPS data received: check wiring"));
  
}


//****************************

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}


uint8_t music[1000]; 
void tone_volume(uint16_t frequency, uint32_t duration) { 
  float interval=0.001257*float(frequency);  
  float phase=0;
  for (int i=0;i<1000;i++) {
   music[i]=127+126*sin(phase);  
   phase+=interval;
  }   
  music[999]=0;
  int remains=duration;
  for (int i=0;i<duration;i+=200) {   
   if (remains<200) {
    music[remains*999/200]=0;
   }
   M5.Speaker.playMusic(music,5000);  
   remains-=200;
  } 
}
//****************************
void beep(int leng) {
  if (sound) M5.Speaker.setVolume(1); else M5.Speaker.setVolume(0);            
  tone_volume(1000,leng); 
}  


void updateState() {
String CurrentState = "STOP"; 
      if (M5.BtnA.wasPressed()) {
        state.current.s1 = TRUE;
        String CurrentState = "LEFT";
      }
      if (M5.BtnC.wasPressed()) {
        state.current.s2 = TRUE;
      }
       if (M5.BtnB.wasPressed()) {
        state.current.s3 = TRUE;
        brakes = 1;
      }   
 
      if (M5.BtnA.wasReleased()) {
        state.current.s1 = FALSE;
      }
      if (M5.BtnC.wasReleased()) {
        state.current.s2 = FALSE;
      }
       if (M5.BtnB.wasReleased()) {
        state.current.s3 = FALSE;
        brakes = 0;
      }   
                
}


// #########################################################################
//  Draw the current time on the screen
// #########################################################################
void updateTime(int posx, int posy)
{
static int temp_minute=0;

  if((int)gps.time.minute() != temp_minute){
    char sz[32];
    temp_minute=gps.time.minute();

        
    if((gps.time.hour()+timezone) > 12) sprintf(sz, "%02d:%02d PM", gps.time.hour()+timezone-12, gps.time.minute());
    else if((gps.time.hour()+timezone) <1) sprintf(sz, "%02d:%02d PM", 12 -gps.time.hour()+timezone, gps.time.minute());
    else sprintf(sz, "%02d:%02d AM", gps.time.hour()+timezone, gps.time.minute());
    
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(posx,posy);
    M5.Lcd.fillRect(posx,posy,posx+20,posy+20, TFT_BLACK);   // Blank out previous


    M5.Lcd.drawString(sz, posx, posy, 2);                 // Draw Temperature 
    Serial.println(sz); // Hour (0-23) (u8)

  }
}      

// #########################################################################
//  Draw the current temperature on the screen
// #########################################################################
void updateThermometer(int posx, int posy)
{
    IMU.tempCount = IMU.readTempData();  // Read the adc values Temperature in degrees Centigrade
    IMU.temperature = ((float) IMU.tempCount) / 333.87 + 21.0;

    if (IMU.temperature != temperature) {
      temperature = IMU.temperature;
      char buf[8]; dtostrf(temperature, 4, 0, buf);
      strcat(buf, " c");
   
      M5.Lcd.fillRect(posx, posy, posx+48, posy+32, TFT_BLACK);
      M5.Lcd.setTextColor(TFT_WHITE);  // Text colour
     // M5.Lcd.setTextSize(2);
      M5.Lcd.drawString(buf, posx, posy+4, 2);                 // Draw Temperature 
    }

  
}

// #########################################################################
//  Draw the analogue speedometer on the screen
// #########################################################################
void drawSpeedometer(int posx, int posy)
{
  // Meter outline
  M5.Lcd.fillRect(0+posx, 0+posy, 239+posx, 126+posy, TFT_GREY);
  M5.Lcd.fillRect(5+posx, 3+posy, 230+posx, 119+posy, TFT_BLACK);

  M5.Lcd.setTextColor(TFT_WHITE);  // Text colour

  // Draw ticks every 5 degrees from -50 to +50 degrees (100 deg. FSD swing)
  for (int i = -90; i < 91; i += 5) {
    // Long scale tick length
    int tl = 15;

    // Coodinates of tick to draw
    float sx = cos((i - 90) * 0.0174532925);
    float sy = sin((i - 90) * 0.0174532925);
    uint16_t x0 = sx * (100 + tl) + 136;
    uint16_t y0 = sy * (100 + tl) + 140;
    uint16_t x1 = sx * 100 + 136;
    uint16_t y1 = sy * 100 + 140;

    // Coordinates of next tick for zone fill
    float sx2 = cos((i + 5 - 90) * 0.0174532925);
    float sy2 = sin((i + 5 - 90) * 0.0174532925);
    int x2 = sx2 * (100 + tl) + 136;
    int y2 = sy2 * (100 + tl) + 140;
    int x3 = sx2 * 100 + 136;
    int y3 = sy2 * 100 + 140;

    // YGreen zone limits
    if (i >= -90 && i < 0) {
    M5.Lcd.fillTriangle(x0+posx, y0+posy, x1+posx, y1+posy, x2+posx, y2+posy, TFT_DARKGREEN);
      M5.Lcd.fillTriangle(x1+posx, y1+posy, x2+posx, y2+posy, x3+posx, y3+posy, TFT_DARKGREEN);
    }

    // Yellow zone limits
    if (i >= 0 && i < 45) {
      M5.Lcd.fillTriangle(x0+posx, y0+posy, x1+posx, y1+posy, x2+posx, y2+posy, TFT_YELLOW);
      M5.Lcd.fillTriangle(x1+posx, y1+posy, x2+posx, y2+posy, x3+posx, y3+posy, TFT_YELLOW);
    }

    // Orange zone limits
    if (i >= 45 && i < 90) {
      M5.Lcd.fillTriangle(x0+posx, y0+posy, x1+posx, y1+posy, x2+posx, y2+posy, TFT_RED);
      M5.Lcd.fillTriangle(x1+posx, y1+posy, x2+posx, y2+posy, x3+posx, y3+posy, TFT_RED);
    }

    // Short scale tick length
    if (i % 25 != 0) tl = 8;

    // Recalculate coords incase tick lenght changed
    x0 = sx * (100 + tl) + 136;
    y0 = sy * (100 + tl) + 140;
    x1 = sx * 100 + 136;
    y1 = sy * 100 + 140;

    // Draw tick
    M5.Lcd.drawLine(x0+posx, y0+posy, x1+posx, y1+posy, TFT_WHITE);

    // Check if labels should be drawn, with position tweaks
    if (i % 25 == 0) {
      // Calculate label positions
      x0 = sx * (100 + tl + 10) + 136;
      y0 = sy * (100 + tl + 10) + 140;
      switch (i / 25) {
        case -4: M5.Lcd.drawCentreString("0", x0+posx, y0+posy - 18, 2); break;
        case -3: M5.Lcd.drawCentreString("5", x0+posx, y0+posy - 15, 2); break;
        case -2: M5.Lcd.drawCentreString("10", x0+posx, y0+posy - 12, 2); break;
        case -1: M5.Lcd.drawCentreString("15", x0+posx, y0+posy - 9, 2); break;
        case 0: M5.Lcd.drawCentreString("20", x0+posx, y0+posy - 6, 2); break;
        case 1: M5.Lcd.drawCentreString("25", x0+posx, y0+posy - 9, 2); break;
        case 2: M5.Lcd.drawCentreString("30", x0+posx, y0+posy - 12, 2); break;
        case 3: M5.Lcd.drawCentreString("35", x0+posx, y0+posy - 15, 2); break;
        case 4: M5.Lcd.drawCentreString("40", x0+posx, y0+posy - 18, 2); break;      }
    }

    // Now draw the arc of the scale
    sx = cos((i + 5 - 90) * 0.0174532925);
    sy = sin((i + 5 - 90) * 0.0174532925);
    x0 = sx * 100 + 136;
    y0 = sy * 100 + 140;
    // Draw scale arc, don't draw the last part
    if (i < 90) M5.Lcd.drawLine(x0+posx, y0+posy, x1+posx, y1+posy, TFT_WHITE);
  }


  M5.Lcd.drawRect(5+posx, 3+posy, 230+posx, 115+posy, TFT_WHITE);   // Draw bezel line

  updateSpeedometer(0, 0, GuageX,GuageY); // Put meter needle at 0
}

// #########################################################################
// Update needle position
// This function is blocking while needle moves, time depends on ms_delay
// 10ms minimises needle flicker if text is drawn within needle sweep area
// Smaller values OK if text not in sweep area, zero for instant movement but
// does not look realistic... (note: 100 increments for full scale deflection)
// #########################################################################
void updateSpeedometer(int value, byte ms_delay, int posx, int posy)
{
  char buf[12]; 
  if (value < -10) value = -10; // Limit value to emulate needle end stops
  if (value > 110) value = 110;

  // Move the needle util new value reached
  while (!(value == old_analog)) {
    if (old_analog < value) old_analog++;
    else old_analog--;

    if (ms_delay == 0) old_analog = value; // Update immediately id delay is 0

    float sdeg = map(old_analog, -10, 110, -180, -0); // Map value to angle
 //   float sdeg = map(old_analog,-2, 102, -180, -0); // Map value to angle
    // Calcualte tip of needle coords
    float sx = cos(sdeg * 0.0174532925);
    float sy = sin(sdeg * 0.0174532925);

    // Calculate x delta of needle start (does not start at pivot point)
    float tx = tan((sdeg + 90) * 0.0174532925);

    // Erase old needle image
    M5.Lcd.drawLine(136 + 20 * ltx - 1+posx, 140 - 20 +posy, osx - 1 +posx, osy+posy, TFT_BLACK);
    M5.Lcd.drawLine(136 + 20 * ltx+posx, 140 - 20+posy, osx+posx, osy+posy, TFT_BLACK);
    M5.Lcd.drawLine(136 + 20 * ltx + 1+posx, 140 - 20+posy, osx + 1+posx, osy+posy, TFT_BLACK);

    // Re-plot text under needle
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    dtostrf(value*4/10, 4, 0, buf);                         
    M5.Lcd.drawRightString(buf, 150+posx, 70+posy, 4);
    M5.Lcd.drawString("KM/H", 154+posx, 90+posy, 2); // Units at bottom right

    snprintf(buf, 12, "Trip: %3.2f", trip);
    M5.Lcd.fillRect(120+posx, 144+posy, 180+posx, 164+posy, TFT_BLACK);
    M5.Lcd.drawRightString(buf, 120+posx, 144+posy, 2);

    snprintf(buf, 12, "Alt: %3.2f", gps.altitude.meters());
    M5.Lcd.fillRect(280+posx, 144+posy, 320+posx, 164+posy, TFT_BLACK);
    M5.Lcd.drawRightString(buf, 280+posx, 144+posy, 2);
    
    // Store new needle end coords for next erase
    ltx = tx;
    osx = sx * 98 + 136;
    osy = sy * 98 + 140;

    // Draw the needle in the new postion, magenta makes needle a bit bolder
    // draws 3 lines to thicken needle
    M5.Lcd.drawLine(136 + 20 * ltx - 1+posx, 140 - 20 +posy, osx - 1 +posx, osy+posy, TFT_RED);
    M5.Lcd.drawLine(136 + 20 * ltx+posx, 140 - 20+posy, osx+posx, osy+posy, TFT_MAGENTA);
    M5.Lcd.drawLine(136 + 20 * ltx + 1+posx, 140 - 20+posy, osx + 1+posx, osy+posy, TFT_RED);


    // Slow needle down slightly as it approaches new postion
    if (abs(old_analog - value) < 10) ms_delay += ms_delay / 5;

    

    // Wait before next update
    delay(ms_delay);
  }
}


// #########################################################################
//  Draw the analogue compass on the screen
// #########################################################################
void drawCompass(int cx, int cy)
{
  uint16_t xd,yd;

  M5.Lcd.fillCircle(cx, cy, rad-2, TFT_BLUE);        // BLUE  Ring
  M5.Lcd.fillCircle(cx, cy, rad-5, TFT_BLACK);        // BLACK

  M5.Lcd.setTextColor(TFT_WHITE);
  xd = cos(-90 * deg) * (rad - 11) + cx;
  yd = sin(-90 * deg) * (rad - 11) + cy;
  
  M5.Lcd.drawCentreString("N", xd, yd-2, 2);
 
  xd = cos(  0 * deg) * (rad - 11) + cx;
  yd = sin(  0 * deg) * (rad - 11) + cy;
  M5.Lcd.drawCentreString("E", xd-2, yd-4, 2);

  xd = cos( 90 * deg) * (rad - 11) + cx;
  yd = sin( 90 * deg) * (rad - 11) + cy;
  M5.Lcd.drawCentreString("S", xd, yd-7, 2);
  
  xd = cos(180 * deg) * (rad - 11) + cx;
  yd = sin(180 * deg) * (rad - 11) + cy;
  M5.Lcd.drawCentreString("W", xd-1, yd-4, 2);

  for (int i = 0; i < 360; i += 30) {           // 12 point
    xd = cos((i - 90) * deg) * (rad - 5) + cx;
    yd = sin((i - 90) * deg) * (rad - 5) + cy;
    if ( i ==   0 ){ xd = xd + 1;}
    if ( i == 180 ){ xd = xd + 1;yd = yd + 1;}
    if ( i == 270 ){ yd = yd + 1;}
    M5.Lcd.drawPixel(xd, yd, 0xFFFF);              // pixel ring
  }

}

void updateCompass(int cx, int cy){
  int cr = 21; float r = M_PI/180;
  float h1 = HD_old;
  M5.Lcd.fillTriangle(                              // analog clear
    cx-cr*cos(r* h1   ),cy+cr*sin(r* h1   ),
    cx+cr*cos(r*(h1+5)),cy-cr*sin(r*(h1+5)),
    cx+cr*cos(r*(h1-5)),cy-cr*sin(r*(h1-5)),
    TFT_BLACK);
  float h2 = HD;
  M5.Lcd.fillTriangle(                              // analog draw
    cx-cr*cos(r* h2   ),cy+cr*sin(r* h2   ),     // x0,y0
    cx+cr*cos(r*(h2+5)),cy-cr*sin(r*(h2+5)),     // x1,y1
    cx+cr*cos(r*(h2-5)),cy-cr*sin(r*(h2-5)),     // x2,y2
    TFT_RED);
  float d1,d2;                                   // deg decode
  if ((h1<=360)&&(h1>=90)){d1=90*((h1/90)-3);}   // 90-270:-180-  0
  if ((h2<=360)&&(h2>=90)){d2=90*((h2/90)-3);}   // 90-270:-180-  0 
  if ((h1<= 90)&&(h1>= 0)){d1=90*((h1/90)+1);}   //  0- 90:+180- 90
  if ((h2<= 90)&&(h2>= 0)){d2=90*((h2/90)+1);}   //  0- 90:+180- 90
  int xa = 135;
//  set_text(xa, 1, String(int(d1)), 0x0000, 1);  // digital clear
//  set_text(xa, 1, String(int(d2)), 0x5FCC, 1);  // digital draw
  M5.Lcd.fillCircle(cx, cy, 5, TFT_RED);         // Circle r = 6
  M5.Lcd.fillCircle(cx, cy, 3, TFT_BLACK);

  if(HD != HD_old){
    indication_Dot(cx, cy);                           // dot      indication
    HD_old = HD;                                // old HD save
  }
}

void indication_Dot(int cx, int cy){
  uint16_t xa,ya,xb,yb;
  int h1 = HD_old - 90;
  int h2 = HD - 90;
  xa = sin(h1 * deg) * (rad - 8) + cx;          // ring old x
  ya = cos(h1 * deg) * (rad - 8) + cy;          // ring old y
  xb = sin(h2 * deg) * (rad - 8) + cx;          // ring new x
  yb = cos(h2 * deg) * (rad - 8) + cy;          // ring new y
  M5.Lcd.fillRect(xa, ya-1, 4, 4, TFT_BLACK);         // 9 dot clear
  M5.Lcd.fillRect(xb, yb-1, 4, 4, TFT_RED);         // 9 dot draw
}

void indication_Triangle(int cx, int cy){
  int cr = 21; float r = M_PI/180;
  float h1 = HD_old;
  M5.Lcd.fillTriangle(                              // analog clear
    cx-cr*cos(r* h1   ),cy+cr*sin(r* h1   ),
    cx+cr*cos(r*(h1+5)),cy-cr*sin(r*(h1+5)),
    cx+cr*cos(r*(h1-5)),cy-cr*sin(r*(h1-5)),
    TFT_BLACK);
  float h2 = HD;
  M5.Lcd.fillTriangle(                              // analog draw
    cx-cr*cos(r* h2   ),cy+cr*sin(r* h2   ),     // x0,y0
    cx+cr*cos(r*(h2+5)),cy-cr*sin(r*(h2+5)),     // x1,y1
    cx+cr*cos(r*(h2-5)),cy-cr*sin(r*(h2-5)),     // x2,y2
    TFT_RED);
  float d1,d2;                                   // deg decode
  if ((h1<=360)&&(h1>=90)){d1=90*((h1/90)-3);}   // 90-270:-180-  0
  if ((h2<=360)&&(h2>=90)){d2=90*((h2/90)-3);}   // 90-270:-180-  0 
  if ((h1<= 90)&&(h1>= 0)){d1=90*((h1/90)+1);}   //  0- 90:+180- 90
  if ((h2<= 90)&&(h2>= 0)){d2=90*((h2/90)+1);}   //  0- 90:+180- 90
  int xa = 80;
  set_text(xa, 1, String(int(d1)), 0x0000, 1);  // digital clear
  set_text(xa, 1, String(int(d2)), 0x5FCC, 1);  // digital draw
  M5.Lcd.fillCircle(cx, cy, 5, TFT_RED);         // Circle r = 6
  M5.Lcd.fillCircle(cx, cy, 3, TFT_BLACK);
} 

void set_text(int x,int y,String text,uint16_t color,byte size){
  M5.Lcd.setTextSize(size);  
  M5.Lcd.setCursor(x,y);
  M5.Lcd.setTextColor(color);
  M5.Lcd.print(text);
}
 

void drawSignals(int cx,int cy)
{

  M5.Lcd.fillTriangle(cx, cy+8, cx + 22, cy, cx+22, cy+16, TFT_RED);            // Left Signal indicator
  M5.Lcd.fillTriangle(cx+70, cy, cx + 92, cy+8, cx+70, cy + 16, TFT_RED);       // Right Signal indicator
  M5.Lcd.fillRect(cx+30, cy, 32, 16, TFT_RED);                             // Brake Signal indicator
  M5.Lcd.fillRect(cx+32, cy +2, 28, 12, TFT_BLACK); 
  
}


void updateSignals() {
  int cx = SignalsX; int cy = SignalsY;
    if (blinkDelay <= millis()) {
      blinkDelay = millis() + BLINK_PERIOD;
      blinkValue = !blinkValue;

      if (state.STATE == 0) {  
        M5.Lcd.fillTriangle(cx, cy+8, cx + 22, cy, cx+22, cy+16, TFT_RED);            // Left Signal indicator
        M5.Lcd.fillTriangle(cx+70, cy, cx + 92, cy+8, cx+70, cy + 16, TFT_RED);       // Right Signal indicator
      }  
      if (state.current.s1) {                                                         // Left signal indicator
        if(blinkValue == 0) {     
           M5.Lcd.fillTriangle(cx+70, cy, cx + 92, cy+8, cx+70, cy + 16, TFT_RED);  // Ensure right indicator is off
           M5.Lcd.fillTriangle(cx, cy+8, cx + 22, cy, cx+22, cy+16, TFT_WHITE);
        } else M5.Lcd.fillTriangle(cx, cy+8, cx + 22, cy, cx+22, cy+16, TFT_RED); 
      } 
      else if (state.current.s2) {                                           // Left signal indicator
        if(blinkValue == 0) {     
          M5.Lcd.fillTriangle(cx, cy+8, cx + 22, cy, cx+22, cy+16, TFT_RED);       // Ensure left indicator is off
          M5.Lcd.fillTriangle(cx+70, cy, cx + 92, cy+8, cx+70, cy + 16, TFT_WHITE);   
        }  else M5.Lcd.fillTriangle(cx+70, cy, cx + 92, cy+8, cx+70, cy + 16, TFT_RED);    
      }  
      if (state.current.s3) {                                                            // brake signal indicator
        if(blinkValue == 0) { 
           M5.Lcd.fillRect(cx+30, cy, 32, 16, TFT_RED);                             // Toggle brake Signal ON
           M5.Lcd.fillRect(cx+32, cy +2, 28, 12, TFT_WHITE); 
        }  
        else { 
           M5.Lcd.fillRect(cx+30, cy, 32, 16, TFT_RED);                             // Toggle brake Signal OFF
           M5.Lcd.fillRect(cx+32, cy +2, 28, 12, TFT_BLACK); 
        } 
      } else  M5.Lcd.fillRect(cx+32, cy +2, 28, 12, TFT_BLACK);  
    }
}

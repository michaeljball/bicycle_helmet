#include <Adafruit_NeoPixel.h>
#include "BLEDevice.h"
//#include "BLEScan.h"

#define PIN 17

#define NUM_LEDS 64
#define BRIGHTNESS 20

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, PIN, NEO_GRBW + NEO_KHZ800);


// Incoming message from BLE remote
union {
  uint8_t STATE =0;    // All off
  struct {
    uint8_t  s1 : 1; // bit position 0        Left Signal
    uint8_t  s2 : 2; // bit positions 1..2    Right Signal
    uint8_t  s3 : 3; // bit positions 3..5    Stop Signal
    uint8_t  s4 : 2; // bit positions 6..7 
    // total # of bits just needs to add up to the uint8_t size
  } current;
} state;



// The remote service we wish to connect to.
static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
// The characteristic of the remote service we are interested in.
static BLEUUID    charUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");

static BLEAddress *pServerAddress;
static boolean doConnect = false;
static boolean connected = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData, size_t length,  bool isNotify) {
    Serial.print("Notify callback for characteristic ");
    for (int i = 0; i < length; i++) {
      Serial.print(pData[i]);
      Serial.print(" ");
    }
    Serial.println();
}

bool connectToServer(BLEAddress pAddress) {
    Serial.print("Forming a connection to ");
    Serial.println(pAddress.toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    // Connect to the remove BLE Server.
    pClient->connect(pAddress);
    Serial.println(" - Connected to server");

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      return false;
    }
    Serial.println(" - Found our service");


    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID.toString().c_str());
      return false;
    }
    Serial.println(" - Found our characteristic");

    // Read the value of the characteristic.
    std::string value = pRemoteCharacteristic->readValue();
    Serial.print("The characteristic value was: ");
    Serial.println(value.c_str());

    pRemoteCharacteristic->registerForNotify(notifyCallback);
}
/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());


    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.getServiceUUID().equals(serviceUUID)) {
    
      // 
      Serial.print("Found our device!  address: "); 
      advertisedDevice.getScan()->stop();

      pServerAddress = new BLEAddress(advertisedDevice.getAddress());
      doConnect = true;

    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks




void setup() {

  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 30 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30);


  strip.setBrightness(BRIGHTNESS);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer(*pServerAddress)) {
      Serial.println("We are now connected to the BLE Server.");
      connected = true;
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }

  // If we are connected to a peer BLE Server, update the characteristic each time we are reached
  // with the current time since boot.
  if (connected) {
    String newValue = "Time since boot: " + String(millis()/1000);
    Serial.println("Setting new characteristic value to \"" + newValue + "\"");
    
    // Set the characteristic's value to be the array of bytes that is actually a string.
    pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
  }
  


  
}

void loop() {
uint16_t j;
uint8_t oldstate = 0; 


 if (connected) {
      // Read the value of the characteristic.
    std::string value = pRemoteCharacteristic->readValue();
    byte buf[64]= {0};
    memcpy(buf,value.c_str(),value.length());
    Serial.print("The characteristic value was: ");
 
      Serial.print(buf[0],HEX);
      Serial.print(" ");

      switch(buf[0]) {
        case 0:
          if(oldstate !=0) {
            fullOff(); 
            Serial.println("Standby");
            oldstate = 0;
          }  
          break;
          
         case 1:
          if(oldstate !=1) {
            LeftTurn(strip.Color(255, 80, 0), 20);
            Serial.println("Turning Left");
            oldstate = 1;
          }  
          break;
          
         case 2:
          if(oldstate !=2) {
            RightTurn(strip.Color(255, 80, 0), 20);
            Serial.println("Turning Right");
            oldstate = 2;
          }  
          break;
          
          case 8:
          if(oldstate !=8) {
            StopLight(strip.Color(255, 0, 0), 20); 
            Serial.println("Stopping");
            oldstate = 8;
          }  
          break;

          default:
            fullOff(); 
            break;
                 
      }
    
  }


 
}

void LeftTurn(uint32_t c, uint8_t wait) {

  for(uint16_t i = strip.numPixels()/2; i<strip.numPixels(); i++) {    // Start halfway and scan left
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
  for(uint16_t i = strip.numPixels()/2; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0));
    strip.show();
    delay(wait);
  }  
  
}

void RightTurn(uint32_t c, uint8_t wait) {

  for(int16_t i = strip.numPixels()/2; i>-1; i--) {    // Start halfway and scan right
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
    for(int16_t i = strip.numPixels()/2; i>-1; i--) {    // Start halfway and scan right
    strip.setPixelColor(i, strip.Color(0, 0, 0));
    strip.show();
    delay(wait);
  }
  
}




void StopLight(uint32_t c, uint8_t wait) {

uint16_t center= strip.numPixels()/2;
  
  for(uint16_t i = 0; i<20; i++) {    // Scan both ways from center
    strip.setPixelColor(center+i, c);
    strip.setPixelColor(center-i, c);
    strip.show();
    delay(wait);
  }
  for(uint16_t i = 0; i<20; i++) {    // Scan both ways from center
    strip.setPixelColor(center+i, strip.Color(0, 0, 0));
    strip.setPixelColor(center-i, strip.Color(0, 0, 0));
    strip.show();
    delay(wait);
  }
}



// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}



void fullWhite() {
  
    for(uint16_t i=0; i<strip.numPixels(); i++) {
        strip.setPixelColor(i, strip.Color(0,0,0, 255 ) );
    }
      strip.show();
}

void fullOff() {
  
    for(uint16_t i=0; i<strip.numPixels(); i++) {
        strip.setPixelColor(i, strip.Color(0,0,0,0 ) );
    }
      strip.show();
}



uint8_t red(uint32_t c) {
  return (c >> 16);
}
uint8_t green(uint32_t c) {
  return (c >> 8);
}
uint8_t blue(uint32_t c) {
  return (c);
}

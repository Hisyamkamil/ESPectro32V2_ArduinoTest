#include <Arduino.h>
#include <ESPectro32_Board.h>
#include <ESPectro32_RGBLED_Animation.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <freertos/task.h>
#include <freertos/FreeRTOS.h>
#include <AloraSensorKit.h>
#include <LSM6DSL.h>
#include <driver/adc.h>

#include <NMEAGPS.h>
#include <Streamers.h>
#include <HardwareSerial.h>
#include <i2c_SI7060.h>
//#include "mic_i2s.h"
#include <Wire.h>
const uint8_t NEO_M8P_ADDR = 0x42;
const uint8_t NEO_M8P_DATA = 0xFF;
NMEAGPS gps;

HardwareSerial gpsSerial(1);
HardwareSerial loraShieldSerial(2);

RgbLedColor_t aCol(200, 0, 80);
ESPectro32_RGBLED_FadeInOutAnimation fadeAnim(ESPectro32.RgbLed(), aCol);
AloraSensorKit sensorKit;

LSM6DSL imu(LSM6DSL_MODE_I2C, 0x6B);
SI7060 si7060_01(1);
float temp2;

static const uint8_t PROGMEM LED_MATRIX_SQUARE[] =
{ 0xFF,
  0xFF,
  0xFF,
  0xFF,
  0xFF,
  0xFF,
  0xFF
};

#include <ESPectro32_LedMatrix_Animation.h>
ESPectro32_LedMatrix_Animation ledMatrixAnim;

void testSDCard();
void testLoRaShield();

int touchThreshold = 40;
static bool touchpadATouched = false;
static bool touchpadBTouched = false;

void gotTouchA(){
    touchpadATouched = true;
}

void gotTouchB(){
    touchpadBTouched = true;
}

void touchInterrupTask(void* data) {
    while (true) {
        if (touchpadATouched) {
            Serial.println("[TOUCH A] Touched");
            touchpadATouched = false;
        }
    
        if (touchpadBTouched) {
            Serial.println("[TOUCH B] Touched");
            touchpadBTouched = false;
        }

        delay(50);
    }
}

void testIMU(){
    if (!imu.begin()) {
        Serial.println("Failed initializing LSM6DSL");
    }
    for(int i = 0; i<10; i++){
        Serial.println("\nAccelerometer:");
        Serial.print("X = ");
        Serial.println(imu.readFloatAccelX(), 4);
        Serial.print("Y = ");
        Serial.println(imu.readFloatAccelY(), 4);
        Serial.print("Z = ");
        Serial.println(imu.readFloatAccelZ(), 4);

        Serial.println("\nGyroscope:");
        Serial.print("X = ");
        Serial.println(imu.readFloatGyroX(), 4);
        Serial.print("Y = ");
        Serial.println(imu.readFloatGyroY(), 4);
        Serial.print("Z = ");
        Serial.println(imu.readFloatGyroZ(), 4);

        Serial.println("\nThermometer:");
        Serial.print(" Degrees C = ");
        Serial.println(imu.readTemperatureC(), 4);
        Serial.print(" Degrees F = ");
        Serial.println(imu.readTemperatureF(), 4);
        delay(1000);
    }
}


void test_si7060(){
    for (int i = 0; i<10; i++){
        si7060_01.prepare();
        temp2 = si7060_01.readTemperature();
        Serial.print("read temp2 ");
        Serial.print(temp2);
        Serial.println("°C  !");
        delay(500);
    }
}
/*
void testMic_i2sCodec(){
    example_i2s_init();
    example_i2s_adc_dac(NULL);
    delay(1000);

}
*/

void i2c_scanner(){
      byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
 
  delay(2000);           // wait 5 seconds for next scan
}

void setup() {
    pinMode(16, OUTPUT);
    digitalWrite(16, HIGH);
    Wire.begin(21,22);
    Serial.begin(9600);
    ESPectro32.begin();
    for (int i=0; i<3; i++){
        i2c_scanner();
    }

    delay(3000);
    testSDCard();

    Serial.println("[LED ANIMATION] Will start animation on built-in LED");
    ESPectro32.LED().setAnimation(ESPectro_LED_Animation_Fading, 3000, 3);

    fadeAnim.start(3000, 3);

    ESPectro32.ButtonA().onButtonUp([]() {
        Serial.println("[BUTTON A] up event detected");
    });

    ESPectro32.ButtonA().onButtonDown([]() {
        Serial.println("[BUTTON A] down event detected");
    });

    ESPectro32.ButtonA().onPressed([]() {
        Serial.println("[BUTTON A] press event detected");
    });

    ESPectro32.ButtonA().onLongPressed([]() {
        Serial.println("[BUTTON A] long press event detected");
    });

    ESPectro32.ButtonA().onDoublePressed([]() {
        Serial.println("[BUTTON A] double press event detected");
    });

    ESPectro32.ButtonB().onButtonUp([]() {
        Serial.println("[BUTTON B] up event detected");
    });

    ESPectro32.ButtonB().onButtonDown([]() {
        Serial.println("[BUTTON B] down event detected");
    });

    ESPectro32.ButtonB().onPressed([]() {
        Serial.println("[BUTTON B] press event detected");
    });

    ESPectro32.ButtonB().onLongPressed([]() {
        Serial.println("[BUTTON B] long press event detected");
    });

    ESPectro32.ButtonB().onDoublePressed([]() {
        Serial.println("[BUTTON B] up event detected");
    });

    Serial.println("[LED MATRIX] Begin LED matrix bitmap drawing");
    ESPectro32.LedMatrix().drawBitmap(0, 0, LED_MATRIX_SQUARE, 7, 7, 200);
    delay(8000);

    Serial.println("[LED MATRIX] Begin LED matrix animation");
    
    ESPectro32.LedMatrix().drawBitmap(0, 0, LED_MATRIX_ICON_HEART, 7, 7, 200);
    ledMatrixAnim.setLedMatrix(ESPectro32.LedMatrix());
    ledMatrixAnim.addFrameWithData((uint8_t*)LED_MATRIX_ICON_HEART);
    ledMatrixAnim.addFrameWithData((uint8_t*)LED_MATRIX_ICON_HEART_OUTLINE);
    ledMatrixAnim.addFrameWithData((uint8_t*)LED_MATRIX_ICON_HEART);
    ledMatrixAnim.addFrameWithData((uint8_t*)LED_MATRIX_ICON_HEART_OUTLINE);
    ledMatrixAnim.addFrameWithDataCallback([](ESPectro32_LedMatrix &ledM) {
        ledM.drawCircle(3, 3, 3, 200);
    });
    ledMatrixAnim.addFrameWithDataCallback([](ESPectro32_LedMatrix &ledM) { 
        ledM.fillCircle(3, 3, 3, 200);
    });

    ledMatrixAnim.start(3000);

    Serial.println("[PHOTO TRANSISTOR] Will start reading");
    for (int i = 0; i < 20; i++) {
        int trVal = ESPectro32.readPhotoTransistorValue();
        Serial.printf("[PHOTO TRANSISTOR] Value: %d %f volt\n", trVal, ESPectro32.readPhotoTransistorVoltage());

        delay(400);
    
    }
    testIMU();
    test_si7060();    
//    testMic_i2sCodec();
    
    Serial.println("finished");
}

void loop() {
}

void testSDCard() {
    File myFile;

    Serial.println("[SD_CARD] Begin SD card testing");

    if (!SD.begin(33)) {
        Serial.println("[SD_CARD] Failed initializing SD card!");
        return;
    }

    myFile = SD.open("/test.txt", FILE_WRITE);
    
    // if the file opened okay, write to it:
    if (myFile) {
        Serial.print("[SD_CARD] Successfully opened /test.txt");
        myFile.println("testing 1, 2, 3.");
        myFile.close();
        Serial.println("[SD_CARD] Successfully wrote /test.txt file");
    } else {
        // if the file didn't open, print an error:
        Serial.println("[SD_CARD] Error opening test.txt");
        return;
    }

    // re-open the file for reading:
    myFile = SD.open("/test.txt");
    if (myFile) {
        Serial.println("[SD_CARD] Successfully re-open the /test.txt text");

    // read from the file until there's nothing else in it:
        while (myFile.available()) {
            Serial.write(myFile.read());
        }
    // close the file:
        myFile.close();
    } else {
    // if the file didn't open, print an error:
        Serial.println("[SD_CARD] Error opening test.txt");
        return;
    }

    Serial.println("[SD_CARD] OK!");
}

void testLoRaShield() {
    loraShieldSerial.begin(9600, SERIAL_8N1, 33, 23);

    Serial.println("[LORA SHIELD] Test AT command is started");
    loraShieldSerial.print("AT\r\n");
    delay(50);

    Serial.println("[LORA SHIELD] Waiting for response...");
    String result = "";

    while (loraShieldSerial.available()) {
        result += String(loraShieldSerial.read());
    }

    Serial.print("[LORA SHIELD] Response: ");
    Serial.println(result);

    if (result.length() > 0) {
        Serial.println("[LORA SHIELD] OK!");
    }
}
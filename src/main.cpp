#include <Arduino.h>
#include <ArduinoBLE.h>

// variables for button
// const int speedPin = 2;
// int oldButtonState = LOW;

// Define constants
#define BLE_UUID "19B10000-E8F2-537E-4F6C-D104768A1214"
#define POT_PIN A0

int potValue = 0; // variable to store the value read
int newSpeed = 0;
int oldSpeed = 0;


// Function Prototypes
void controlLed(BLEDevice peripheral);

void setup()
{
  Serial.begin(9600);

  // initialize the BLE hardware
  BLE.begin();

  Serial.println("BLE Central - skateboard control");

  // start scanning for peripherals
  BLE.scanForUuid(BLE_UUID);
}

void loop()
{
  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();

  if (peripheral)
  {
    // discovered a peripheral, print out address, local name, and advertised service
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();

    if (peripheral.localName() != "SkateBoard")
    {
      return;
    }

    // stop scanning
    BLE.stopScan();

    controlLed(peripheral);

    // peripheral disconnected, start scanning again
    BLE.scanForUuid(BLE_UUID);
  }
}

void controlLed(BLEDevice peripheral)
{
  // connect to the peripheral
  Serial.println("Connecting ...");

  if (peripheral.connect())
  {
    Serial.println("Connected");
  }
  else
  {
    Serial.println("Failed to connect!");
    return;
  }

  // discover peripheral attributes
  Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes())
  {
    Serial.println("Attributes discovered");
  }
  else
  {
    Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }

  // retrieve the LED characteristic
  BLECharacteristic speedCharacteristic = peripheral.characteristic(BLE_UUID);

  if (!speedCharacteristic)
  {
    Serial.println("Peripheral does not have skateboard speed characteristic!");
    peripheral.disconnect();
    return;
  }
  else if (!speedCharacteristic.canWrite())
  {
    Serial.println("Peripheral does not have a writable skateboard speed characteristic!");
    peripheral.disconnect();
    return;
  }
  int oldPot = 0;
  while (peripheral.connected())
  {
    // while the peripheral is connected

    // read the pot pin
    potValue = analogRead(POT_PIN);
    int newSpeed = float(potValue / 1023.0) * 100;
    newSpeed = (newSpeed - 50) * 2;

    // if (newSpeed >= 55) {
    //   newSpeed = (newSpeed - 50) * 2;
    // } else if (newSpeed <= 45) {
    //   newSpeed = (newSpeed - 50) * 2;
    // }

    if (potValue > (oldPot + 20) || potValue < (oldPot - 20)) {
      oldPot = potValue;
      Serial.print("Pot Value: ");
      Serial.println(potValue);
      Serial.print("New Speed: ");
      Serial.println(newSpeed);
    }
    
    if (newSpeed != oldSpeed)
    {
      // button changed
      oldSpeed = newSpeed;
      int success = speedCharacteristic.writeValue((int8_t)newSpeed);

      if (!success) {
        Serial.println("Error: Pot Value Not Written");
      }
      // if (success) {
      //   Serial.println("Pot Value Written");
      // } else {
      //   Serial.println("Error: Pot Value Not Written");
      // }

      // if (buttonState)
      // {
      //   Serial.println("button pressed");

      //   // button is pressed, write 0x01 to turn the LED on
      // speedCharacteristic.writeValue((byte)0x01);
      // }
      // else
      // {
      //   Serial.println("button released");

      //   // button is released, write 0x00 to turn the LED off
      //   ledCharacteristic.writeValue((byte)0x00);
      // }
    }
  }

  Serial.println("Peripheral disconnected");
}
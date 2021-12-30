#include <Arduino.h>
#include <ArduinoBLE.h>

// variables for button
// const int speedPin = 2;
// int oldButtonState = LOW;

// Define constants
#define BLE_UUID "19B10000-E8F2-537E-4F6C-D104768A1214"
#define SPEED_UUID "19B10000-E8F2-537E-4F6C-D104776A1219"
#define DIR_UUID "19B10000-E8F2-537E-4F6C-D604168C1217"
#define POT_PIN A0

int potValue = 0; // variable to store the value read
int newSpeed = 0;
int oldSpeed = 0;


// Function Prototypes
void controlLed(BLEDevice peripheral);

void setup()
{
  Serial.begin(9600);
  // while (!Serial);

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

  // retrieve the speed characteristic
  BLECharacteristic speedCharacteristic = peripheral.characteristic(SPEED_UUID);
  // retrieve the direction characteristic
  BLECharacteristic forwardDirectionCharacteristic = peripheral.characteristic(DIR_UUID);

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
  if (!forwardDirectionCharacteristic)
  {
    Serial.println("Peripheral does not have skateboard direction characteristic!");
    peripheral.disconnect();
    return;
  }
  else if (!forwardDirectionCharacteristic.canWrite())
  {
    Serial.println("Peripheral does not have a writable skateboard direction characteristic!");
    peripheral.disconnect();
    return;
  }
  int oldPot = 0;
  int newSpeed = 0;
  bool forward = true;
  while (peripheral.connected())
  {
    // while the peripheral is connected

    // read the pot pin
    potValue = analogRead(POT_PIN);
    newSpeed = float(potValue / 1023.0) * 100;
    newSpeed = (newSpeed - 50) * 2;

    if (potValue > (oldPot + 20) || potValue < (oldPot - 20)) {
      oldPot = potValue;
      Serial.print("Pot Value: ");
      Serial.println(potValue);
      Serial.print("New Speed: ");
      Serial.println(newSpeed);
    }
    
    if (newSpeed != oldSpeed)
    {
      oldSpeed = newSpeed;
      if (newSpeed < 0)
        forward = false;
      else
        forward = true;


      int speedSuccess = speedCharacteristic.writeValue((int8_t)abs(newSpeed));
      int dirSuccess = forwardDirectionCharacteristic.writeValue((int8_t)forward);

      if (!speedSuccess || !dirSuccess) {
        Serial.println("Error: Value Write Failure!");
      }
    }
  }

  Serial.println("Peripheral disconnected");
}
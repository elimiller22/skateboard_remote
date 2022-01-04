#include <ArduinoBLE.h>

// Define constants
#define BLE_UUID "19B10000-E8F2-537E-4F6C-D104768A1214"
#define SPEED_UUID "19B10000-E8F2-537E-4F6C-D104776A1219"
#define DIR_UUID "19B10000-E8F2-537E-4F6C-D604168C1217"
#define POT_PIN A0

// Function Prototypes
void controlSkateboard(BLEDevice peripheral,
                       BLECharacteristic speedCharacteristic,
                       BLECharacteristic forwardDirectionCharacteristic);
bool getCharacteristics(BLEDevice peripheral,
                        BLECharacteristic &speedCharacteristic,
                        BLECharacteristic &forwardDirectionCharacteristic);

/**
 * @brief Function to run setup on arduino.
 *
 */
void setup() {
  Serial.begin(9600);
  // while (!Serial);

  // initialize the BLE hardware
  BLE.begin();

  Serial.println("BLE Central - skateboard controller");

  // start scanning for peripherals
  BLE.scanForUuid(BLE_UUID);
}

/**
 * @brief Loop function on Arduino
 *
 */
void loop() {
  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();
  BLECharacteristic speedCharacteristic;
  BLECharacteristic forwardDirectionCharacteristic;

  if (peripheral) {
    // discovered a peripheral, print out address, local name, and advertised
    // service
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();

    if (peripheral.localName() != "SkateBoard") {
      return;
    }

    // stop scanning
    BLE.stopScan();

    if (peripheral.connect()) {
      Serial.println("Connected to Skateboard");
    } else {
      Serial.println("Failed to connect to Skateboard!");
      return;
    }

    bool characteristicsCreated = getCharacteristics(
        peripheral, speedCharacteristic, forwardDirectionCharacteristic);
    if (characteristicsCreated)
      controlSkateboard(peripheral, speedCharacteristic,
                        forwardDirectionCharacteristic);

    // peripheral disconnected, start scanning again
    BLE.scanForUuid(BLE_UUID);
  }
}

/**
 * @brief Function to retrieve and return the speed and direction
 * characteristics.
 *
 * @param peripheral - The ble peripheral object
 * @param speedCharacteristic - Var for speed characteristic.
 * @param forwardDirectionCharacteristic - Var for direction characteristic.
 * @return bool - A flag for if the function completed successfully.
 */
bool getCharacteristics(BLEDevice peripheral,
                        BLECharacteristic &speedCharacteristic,
                        BLECharacteristic &forwardDirectionCharacteristic) {
  // discover peripheral attributes
  if (!peripheral.discoverAttributes()) {
    Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return false;
  }

  // retrieve the speed characteristic
  speedCharacteristic = peripheral.characteristic(SPEED_UUID);
  // retrieve the direction characteristic
  forwardDirectionCharacteristic = peripheral.characteristic(DIR_UUID);

  if (!speedCharacteristic) {
    Serial.println("Peripheral does not have skateboard speed characteristic!");
    peripheral.disconnect();
    return false;
  } else if (!speedCharacteristic.canWrite()) {
    Serial.println(
        "Peripheral does not have a writable skateboard speed characteristic!");
    peripheral.disconnect();
    return false;
  }
  if (!forwardDirectionCharacteristic) {
    Serial.println(
        "Peripheral does not have skateboard direction characteristic!");
    peripheral.disconnect();
    return false;
  } else if (!forwardDirectionCharacteristic.canWrite()) {
    Serial.println("Peripheral does not have a writable skateboard direction "
                   "characteristic!");
    peripheral.disconnect();
    return false;
  }
  return true;
}

/**
 * @brief Function to control the skateboard via ble.
 *
 * @param peripheral - The ble peripheral object
 * @param speedCharacteristic - Speed characteristic object.
 * @param forwardDirectionCharacteristic - Direction characteristic object.
 */
void controlSkateboard(BLEDevice peripheral,
                       BLECharacteristic speedCharacteristic,
                       BLECharacteristic forwardDirectionCharacteristic) {
  int potValue = 0; // variable to store the value read
  int speedSensitivity = 5;
  int oldSpeed = 0;
  int newSpeed = 0;
  bool forward = true;
  while (peripheral.connected()) {
    // while the peripheral is connected

    // read the pot pin
    potValue = analogRead(POT_PIN);
    newSpeed = float(potValue / 1023.0) * 100;
    newSpeed = (newSpeed - 50) * 2;

    // Tune the top and bottom end values
    if (newSpeed >= (100 - speedSensitivity))
      newSpeed = 100;
    else if (newSpeed <= speedSensitivity && newSpeed >= -speedSensitivity)
      newSpeed = 0;
    else if (newSpeed <= -(100 - speedSensitivity))
      newSpeed = -100;

    // If the speed has changed more than the specified sensitivity, then write
    // the new speed.
    if (newSpeed > (oldSpeed + speedSensitivity) ||
        newSpeed < (oldSpeed - speedSensitivity)) {
      Serial.print("Pot Value: ");
      Serial.println(potValue);
      Serial.print("New Speed: ");
      Serial.println(newSpeed);
      oldSpeed = newSpeed;
      if (newSpeed < 0)
        forward = false;
      else
        forward = true;

      int speedSuccess = speedCharacteristic.writeValue((int8_t)abs(newSpeed));
      int dirSuccess =
          forwardDirectionCharacteristic.writeValue((int8_t)forward);

      if (!speedSuccess || !dirSuccess) {
        Serial.println("Error: Value Write Failure!");
      }
    }
  }

  Serial.println("Peripheral disconnected");
}
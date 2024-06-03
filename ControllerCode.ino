#include <ArduinoBLE.h>

#define MAX_SLAVES 5 // 최대 슬레이브 수

BLEDevice slaveDevices[MAX_SLAVES];
int numSlaves = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // BLE 초기화
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1);
  }

  Serial.println("BLE Central - Looking for peripherals...");

  BLE.scanForUuid("1809"); // 온도 서비스 UUID로 스캔 시작
}

void loop() {
  // 새로운 슬레이브 장치 찾기
  BLEDevice peripheral = BLE.available();

  if (peripheral) {
    if (numSlaves < MAX_SLAVES) {
      Serial.print("Found peripheral: ");
      Serial.println(peripheral.address());

      if (peripheral.hasService("1809")) {
        Serial.println("Found temperature service");

        // 연결 시도
        if (peripheral.connect()) {
          Serial.println("Connected to peripheral");
          
          // 슬레이브 장치 목록에 추가
          slaveDevices[numSlaves] = peripheral;
          numSlaves++;
        } else {
          Serial.println("Failed to connect to peripheral");
        }
      }
    } else {
      Serial.println("Max number of slaves reached");
    }
  }

  // 각 슬레이브 장치의 온도 값 읽기
  for (int i = 0; i < numSlaves; i++) {
    if (slaveDevices[i].connected()) {
      BLECharacteristic tempCharacteristic = slaveDevices[i].characteristic("2A6E");
      
      if (tempCharacteristic.canRead()) {
        float temperature;
        tempCharacteristic.readValue(temperature);
        
        Serial.print("Slave ");
        Serial.print(i);
        Serial.print(" Temperature: ");
        Serial.println(temperature);
      }
    } else {
      Serial.print("Slave ");
      Serial.print(i);
      Serial.println(" disconnected");
      
      // 슬레이브 장치 리스트에서 제거
      for (int j = i; j < numSlaves - 1; j++) {
        slaveDevices[j] = slaveDevices[j + 1];
      }
      numSlaves--;
      i--; // 현재 인덱스를 다시 처리
    }
  }








  #include <ArduinoBLE.h>

BLEService sensorService("180C"); // Custom service

// BLE Characteristics
BLEStringCharacteristic sensorDataCharacteristic("2A56", BLERead | BLENotify, 32);

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }

  // Start scanning for BLE peripherals
  BLE.scan();

  Serial.println("BLE Central - scanning for peripherals...");
}

void loop() {
  // Check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();

  if (peripheral) {
    // Found a peripheral, connect to it
    if (peripheral.localName() == "Nano33BLE_Sense") {
      BLE.stopScan();

      Serial.println("Connecting ...");
      if (peripheral.connect()) {
        Serial.println("Connected to peripheral");
      } else {
        Serial.println("Failed to connect!");
        BLE.scan();
        return;
      }

      // Discover the service
      if (peripheral.discoverService(sensorService)) {
        Serial.println("Service discovered");

        // Discover the characteristic
        if (peripheral.discoverCharacteristic(sensorDataCharacteristic)) {
          Serial.println("Characteristic discovered");
          sensorDataCharacteristic.subscribe();
        }
      }
    }
  }

  // Check if we are connected to a peripheral
  if (peripheral && peripheral.connected()) {
    // Check if new sensor data is available
    if (sensorDataCharacteristic.valueUpdated()) {
      String sensorData = sensorDataCharacteristic.value();
      Serial.print("Received sensor data: ");
      Serial.println(sensorData);
    }
  } else {
    BLE.scan();
  }
}


  delay(1000);
}








#include <ArduinoBLE.h>

#define MAX_DEVICES 5

BLEDevice peripherals[MAX_DEVICES];
BLEService sensorService("180C"); // Custom service

// BLE Characteristics
BLEStringCharacteristic sensorDataCharacteristics[MAX_DEVICES] = {
  BLEStringCharacteristic("2A56", BLERead | BLENotify, 32),
  BLEStringCharacteristic("2A56", BLERead | BLENotify, 32),
  BLEStringCharacteristic("2A56", BLERead | BLENotify, 32),
  BLEStringCharacteristic("2A56", BLERead | BLENotify, 32),
  BLEStringCharacteristic("2A56", BLERead | BLENotify, 32)
};

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }

  // Start scanning for BLE peripherals
  BLE.scan();

  Serial.println("BLE Central - scanning for peripherals...");
}

void loop() {
  // Check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();

  if (peripheral) {
    for (int i = 0; i < MAX_DEVICES; i++) {
      if (!peripherals[i] && peripheral.localName() == "Nano33BLE_Sense") {
        peripherals[i] = peripheral;

        BLE.stopScan();

        Serial.print("Connecting to ");
        Serial.println(peripheral.address());

        if (peripherals[i].connect()) {
          Serial.println("Connected to peripheral");
          if (peripherals[i].discoverService(sensorService)) {
            Serial.println("Service discovered");
            if (peripherals[i].discoverCharacteristic(sensorDataCharacteristics[i])) {
              Serial.println("Characteristic discovered");
              sensorDataCharacteristics[i].subscribe();
            }
          }
        } else {
          Serial.println("Failed to connect!");
        }

        BLE.scan();
        break;
      }
    }
  }

  // Check if we are connected to any peripherals
  for (int i = 0; i < MAX_DEVICES; i++) {
    if (peripherals[i] && peripherals[i].connected()) {
      // Check if new sensor data is available
      if (sensorDataCharacteristics[i].valueUpdated()) {
        String sensorData = sensorDataCharacteristics[i].value();
        Serial.print("Received data from ");
        Serial.print(peripherals[i].address());
        Serial.print(": ");
        Serial.println(sensorData);
      }
    }
  }
}

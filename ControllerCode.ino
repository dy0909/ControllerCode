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

  delay(1000);
}

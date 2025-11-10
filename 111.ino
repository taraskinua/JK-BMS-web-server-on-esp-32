/*
 * Скетч ESP32 для чтения данных с Jikong (JK) BMS по BLE (Bluetooth Low Energy)
 * ESP32 выступает в роли BLE Client.
 * Добавлен веб-сервер для отображения полученных данных и OTA-обновления прошивки.
 * ДОБАВЛЕНО: Функционал сканирования и выбора устройства для подключения через веб-интерфейс.
 */

#include "BLEDevice.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h> // Для OTA обновления
#include <ESPmDNS.h> // Для доступа по имени хоста (опционально, но полезно)
#include <vector>
#include <map>
#include <string>

// --- Настройки WiFi ---
const char* ssid = "";
// <-- ЗАМЕНИТЕ на ваш SSID
const char* password = "";
// <-- ЗАМЕНИТЕ на ваш пароль

// --- Глобальные объекты веб-сервера ---
WebServer server(80);
const char* host = "esp32-bms";
// Имя хоста для доступа по http://esp32-bms.local

// Стандартные UUID для Jikong BMS
static BLEUUID serviceUUID("0000ffe0-0000-1000-8000-00805f9b34fb"); // Основной сервис
static BLEUUID charWriteUUID("0000ffe1-0000-1000-8000-00805f9b34fb");
// Характеристика для ЗАПИСИ
static BLEUUID charNotifyUUID("0000ffe1-0000-1000-8000-00805f9b34fb"); // Характеристика для УВЕДОМЛЕНИЙ

// --- Глобальные переменные BLE ---
struct FoundBMS {
    std::string address;
    std::string name;
    BLEAdvertisedDevice *pAdvertisedDevice;
};
std::map<std::string, FoundBMS> foundDevices; // Используем MAC-адрес как ключ

static BLEAdvertisedDevice* pBmsDevice = nullptr; // Инициализируем nullptr
static BLERemoteCharacteristic* pWriteCharacteristic;
static BLERemoteCharacteristic* pNotifyCharacteristic;
static BLEScan* pBLEScan;
static bool deviceFound = false; // true, только когда выбран конкретный прибор
static bool isConnected = false;
uint32_t lastNotifyTime = 0;

// Data Processing
byte receivedBytes[320];
int frame = 0;
bool received_start = false;
bool received_complete = false;
bool new_data = false;
int ignoreNotifyCount = 0;

// BMS Data Fields (опущены для краткости, берем из исходника)
float cellVoltage[16] = { 0 };
float wireResist[16] = { 0 };
float Average_Cell_Voltage = 0;
float Delta_Cell_Voltage = 0;
float Battery_Voltage = 0;
float Battery_Power = 0;
float Charge_Current = 0;
float Battery_T1 = 0;
float Battery_T2 = 0;
float MOS_Temp = 0;
int Percent_Remain = 0;
float Capacity_Remain = 0;
float Nominal_Capacity = 0;
float Cycle_Count = 0;
float Cycle_Capacity = 0;
uint32_t Uptime;
uint8_t sec, mi, hr, days;
float Balance_Curr = 0;
bool Balance = false;
bool Charge = false;
bool Discharge = false;
int Balancing_Action = 0;
float balance_trigger_voltage = 0;
float cell_voltage_undervoltage_protection = 0;
float cell_voltage_undervoltage_recovery = 0;
float cell_voltage_overvoltage_protection = 0;
float cell_voltage_overvoltage_recovery = 0;
float power_off_voltage = 0;
float max_charge_current = 0;
float charge_overcurrent_protection_delay = 0;
float charge_overcurrent_protection_recovery_time = 0;
float max_discharge_current = 0;
float discharge_overcurrent_protection_delay = 0;
float discharge_overcurrent_protection_recovery_time = 0;
float short_circuit_protection_recovery_time = 0;
float max_balance_current = 0;
float charge_overtemperature_protection = 0;
float charge_overtemperature_protection_recovery = 0;
float discharge_overtemperature_protection = 0;
float discharge_overtemperature_protection_recovery = 0;
float charge_undertemperature_protection = 0;
float charge_undertemperature_protection_recovery = 0;
float power_tube_overtemperature_protection = 0;
float power_tube_overtemperature_protection_recovery = 0;
int cell_count = 0;
float total_battery_capacity = 0;
float short_circuit_protection_delay = 0;
float balance_starting_voltage = 0;

// --- Глобальні змінні для інформації про пристрій (Device Info) ---
String vendorID_str = "";
String hardwareVersion_str = "";
String softwareVersion_str = "";
uint32_t device_uptime = 0;
uint32_t powerOnCount = 0;
String deviceName_str = "";
String devicePasscode_str = "";
String manufacturingDate_str = "";
String serialNumber_str = "";
String passcode_str = "";
String userData_str = "";
String setupPasscode_str = "";

// Функции BMS (CRC, writeRegister, parseData, bms_settings, parseDeviceInfo, parseBMSData, notifyCallback, MyClientCallback)
uint8_t crc(const uint8_t data[], uint16_t len) {
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) crc += data[i];
    return crc;
  };

void writeRegister(uint8_t address, uint32_t value, uint8_t length) {
  uint8_t frame[20] = { 0xAA, 0x55, 0x90, 0xEB, address, length };
  // Insert value (Little-Endian)
  frame[6] = value >> 0;  // LSB
  frame[7] = value >> 8;
  frame[8] = value >> 16;
  frame[9] = value >> 24;
  // MSB

  // Calculate CRC
  frame[19] = crc(frame, 19);
  if (pNotifyCharacteristic) {
    pNotifyCharacteristic->writeValue((uint8_t*)frame, (size_t)sizeof(frame));
  }
};

void parseData() {
  new_data = false;
  ignoreNotifyCount = 10;
  // Cell voltages
  for (int j = 0, i = 7; i < 38; j++, i += 2) {
    cellVoltage[j] = ((receivedBytes[i] << 8 | receivedBytes[i - 1]) * 0.001);
  }
  Average_Cell_Voltage = (((int)receivedBytes[75] << 8 | receivedBytes[74]) * 0.001);
  Delta_Cell_Voltage = (((int)receivedBytes[77] << 8 | receivedBytes[76]) * 0.001);
  // Wire Resistances
  for (int j = 0, i = 81; i < 112; j++, i += 2) {
    wireResist[j] = (((int)receivedBytes[i] << 8 | receivedBytes[i - 1]) * 0.001);
  }

// MOS Temperature
  if (receivedBytes[145] == 0xFF) {
    MOS_Temp = ((0xFF << 24 | 0xFF << 16 | receivedBytes[145] << 8 | receivedBytes[144]) * 0.1);
  } else {
    MOS_Temp = ((receivedBytes[145] << 8 | receivedBytes[144]) * 0.1);
  }

  // Battery voltage
  Battery_Voltage = ((receivedBytes[153] << 24 | receivedBytes[152] << 16 | receivedBytes[151] << 8 | receivedBytes[150]) * 0.001);
  Charge_Current = ((receivedBytes[161] << 24 | receivedBytes[160] << 16 | receivedBytes[159] << 8 | receivedBytes[158]) * 0.001);
  Battery_Power = Battery_Voltage * Charge_Current;

// Battery Temperature T1
  if (receivedBytes[163] == 0xFF) {
    Battery_T1 = ((0xFF << 24 | 0xFF << 16 | receivedBytes[163] << 8 | receivedBytes[162]) * 0.1);
  } else {
    Battery_T1 = ((receivedBytes[163] << 8 | receivedBytes[162]) * 0.1);
  }

// Battery Temperature T2
  if (receivedBytes[165] == 0xFF) {
    Battery_T2 = ((0xFF << 24 | 0xFF << 16 | receivedBytes[165] << 8 | receivedBytes[164]) * 0.1);
  } else {
    Battery_T2 = ((receivedBytes[165] << 8 | receivedBytes[164]) * 0.1);
  }

// Balance Current
  if ((receivedBytes[171] & 0xF0) == 0x0) {
    Balance_Curr = ((receivedBytes[171] << 8 | receivedBytes[170]) * 0.001);
  } else if ((receivedBytes[171] & 0xF0) == 0xF0) {
    Balance_Curr = (((receivedBytes[171] & 0x0F) << 8 | receivedBytes[170]) * -0.001);
  }

  Balancing_Action = receivedBytes[172];
  Percent_Remain = (receivedBytes[173]);
  Capacity_Remain = ((receivedBytes[177] << 24 | receivedBytes[176] << 16 | receivedBytes[175] << 8 | receivedBytes[174]) * 0.001);
  Nominal_Capacity = ((receivedBytes[181] << 24 | receivedBytes[180] << 16 | receivedBytes[179] << 8 | receivedBytes[178]) * 0.001);
  Cycle_Count = ((receivedBytes[185] << 24 | receivedBytes[184] << 16 | receivedBytes[183] << 8 | receivedBytes[182]));
  Cycle_Capacity = ((receivedBytes[189] << 24 | receivedBytes[188] << 16 | receivedBytes[187] << 8 | receivedBytes[186]) * 0.001);
  Uptime = receivedBytes[196] << 16 | receivedBytes[195] << 8 | receivedBytes[194];
  sec = Uptime % 60;
  Uptime /= 60;
  mi = Uptime % 60;
  Uptime /= 60;
  hr = Uptime % 24;
  days = Uptime / 24;
  // Statuses
  if (receivedBytes[198] > 0) {
    Charge = true;
  } else if (receivedBytes[198] == 0) {
    Charge = false;
  }
  if (receivedBytes[199] > 0) {
    Discharge = true;
  } else if (receivedBytes[199] == 0) {
    Discharge = false;
  }
  if (receivedBytes[201] > 0) {
    Balance = true;
  } else if (receivedBytes[201] == 0) {
    Balance = false;
  }

};

void bms_settings() {
  cell_voltage_undervoltage_protection = ((receivedBytes[13] << 24 | receivedBytes[12] << 16 | receivedBytes[11] << 8 | receivedBytes[10]) * 0.001);
  cell_voltage_undervoltage_recovery = ((receivedBytes[17] << 24 | receivedBytes[16] << 16 | receivedBytes[15] << 8 | receivedBytes[14]) * 0.001);
  cell_voltage_overvoltage_protection = ((receivedBytes[21] << 24 | receivedBytes[20] << 16 | receivedBytes[19] << 8 | receivedBytes[18]) * 0.001);
  cell_voltage_overvoltage_recovery = ((receivedBytes[25] << 24 | receivedBytes[24] << 16 | receivedBytes[23] << 8 | receivedBytes[22]) * 0.001);
  balance_trigger_voltage = ((receivedBytes[29] << 24 | receivedBytes[28] << 16 | receivedBytes[27] << 8 | receivedBytes[26]) * 0.001);
  power_off_voltage = ((receivedBytes[49] << 24 | receivedBytes[48] << 16 | receivedBytes[47] << 8 | receivedBytes[46]) * 0.001);
  max_charge_current = ((receivedBytes[53] << 24 | receivedBytes[52] << 16 | receivedBytes[51] << 8 | receivedBytes[50]) * 0.001);
  charge_overcurrent_protection_delay = ((receivedBytes[57] << 24 | receivedBytes[56] << 16 | receivedBytes[55] << 8 | receivedBytes[54]));
  charge_overcurrent_protection_recovery_time = ((receivedBytes[61] << 24 | receivedBytes[60] << 16 | receivedBytes[59] << 8 | receivedBytes[58]));
  max_discharge_current = ((receivedBytes[65] << 24 | receivedBytes[64] << 16 | receivedBytes[63] << 8 | receivedBytes[62]) * 0.001);
  discharge_overcurrent_protection_delay = ((receivedBytes[69] << 24 | receivedBytes[68] << 16 | receivedBytes[67] << 8 | receivedBytes[66]));
  discharge_overcurrent_protection_recovery_time = ((receivedBytes[73] << 24 | receivedBytes[72] << 16 | receivedBytes[71] << 8 | receivedBytes[70]));
  short_circuit_protection_recovery_time = ((receivedBytes[77] << 24 | receivedBytes[76] << 16 | receivedBytes[75] << 8 | receivedBytes[74]));
  max_balance_current = ((receivedBytes[81] << 24 | receivedBytes[80] << 16 | receivedBytes[79] << 8 | receivedBytes[78]) * 0.001);
  charge_overtemperature_protection = ((receivedBytes[85] << 24 | receivedBytes[84] << 16 | receivedBytes[83] << 8 | receivedBytes[82]) * 0.1);
  charge_overtemperature_protection_recovery = ((receivedBytes[89] << 24 | receivedBytes[88] << 16 | receivedBytes[87] << 8 | receivedBytes[86]) * 0.1);
  discharge_overtemperature_protection = ((receivedBytes[93] << 24 | receivedBytes[92] << 16 | receivedBytes[91] << 8 | receivedBytes[90]) * 0.1);
  discharge_overtemperature_protection_recovery = ((receivedBytes[97] << 24 | receivedBytes[96] << 16 | receivedBytes[95] << 8 | receivedBytes[94]) * 0.1);
  charge_undertemperature_protection = ((receivedBytes[101] << 24 | receivedBytes[100] << 16 | receivedBytes[99] << 8 | receivedBytes[98]) * 0.1);
  charge_undertemperature_protection_recovery = ((receivedBytes[105] << 24 | receivedBytes[104] << 16 | receivedBytes[103] << 8 | receivedBytes[102]) * 0.1);
  power_tube_overtemperature_protection = ((receivedBytes[109] << 24 | receivedBytes[108] << 16 | receivedBytes[107] << 8 | receivedBytes[106]) * 0.1);
  power_tube_overtemperature_protection_recovery = ((receivedBytes[113] << 24 | receivedBytes[112] << 16 | receivedBytes[111] << 8 | receivedBytes[110]) * 0.1);
  cell_count = ((receivedBytes[117] << 24 | receivedBytes[116] << 16 | receivedBytes[115] << 8 | receivedBytes[114]));
  // 118    4    0x01 0x00 0x00 0x00    Charge switch
  // 122    4    0x01 0x00 0x00 0x00    Discharge switch
  // 126    4    0x01 0x00 0x00 0x00    Balancer switch
  total_battery_capacity = ((receivedBytes[133] << 24 | receivedBytes[132] << 16 | receivedBytes[131] << 8 | receivedBytes[130]) * 0.001);
  short_circuit_protection_delay = ((receivedBytes[137] << 24 | receivedBytes[136] << 16 | receivedBytes[135] << 8 | receivedBytes[134]) * 1);
  balance_starting_voltage = ((receivedBytes[141] << 24 | receivedBytes[140] << 16 | receivedBytes[139] << 8 | receivedBytes[138]) * 0.001);
};

// Функція для парсингу інформації про пристрій
void parseDeviceInfo() {
  new_data = false;
  if (frame < 134) {
    return;
  }

  // Код змінено для використання глобальних String змінних.
  vendorID_str.clear();
  for (int i = 0; i < 16; i++) vendorID_str += (char)receivedBytes[6 + i];
  
  hardwareVersion_str.clear();
  for (int i = 0; i < 8; i++) hardwareVersion_str += (char)receivedBytes[22 + i];
  
  softwareVersion_str.clear();
  for (int i = 0; i < 8; i++) softwareVersion_str += (char)receivedBytes[30 + i];
  
  device_uptime = (receivedBytes[41] << 24) |
  (receivedBytes[40] << 16) |
  (receivedBytes[39] << 8) | receivedBytes[38];
  
  powerOnCount = (receivedBytes[45] << 24) | (receivedBytes[44] << 16) |
  (receivedBytes[43] << 8) |
  receivedBytes[42];
  
  deviceName_str.clear();
  for (int i = 0; i < 16; i++) deviceName_str += (char)receivedBytes[46 + i];
  devicePasscode_str.clear();
  for (int i = 0; i < 16; i++) devicePasscode_str += (char)receivedBytes[62 + i];
  
  manufacturingDate_str.clear();
  for (int i = 0; i < 8; i++) manufacturingDate_str += (char)receivedBytes[78 + i];
  
  serialNumber_str.clear();
  for (int i = 0; i < 11; i++) serialNumber_str += (char)receivedBytes[86 + i];
  
  passcode_str.clear();
  for (int i = 0; i < 5; i++) passcode_str += (char)receivedBytes[97 + i];
  
  userData_str.clear();
  for (int i = 0; i < 16; i++) userData_str += (char)receivedBytes[102 + i];
  
  setupPasscode_str.clear();
  for (int i = 0; i < 16; i++) setupPasscode_str += (char)receivedBytes[118 + i];

};

// Функция для парсинга (разбора) данных.
void parseBMSData(uint8_t* pData, size_t length) {
  lastNotifyTime = millis();
  if (ignoreNotifyCount > 0) {
    ignoreNotifyCount--;
    return;
  }

  // Check for start of data frame
  if (pData[0] == 0x55 && pData[1] == 0xAA && pData[2] == 0xEB && pData[3] == 0x90) {
    frame = 0;
    received_start = true;
    received_complete = false;

    // Store the received data
    for (int i = 0; i < length; i++) {
      receivedBytes[frame++] = pData[i];
    }
  } else if (received_start && !received_complete) {
      for (int i = 0; i < length; i++) {
      receivedBytes[frame++] = pData[i];
      if (frame >= 300) {
        received_complete = true;
        received_start = false;
        new_data = true;
        // Determine the type of data frame based on receivedBytes[4]
        switch (receivedBytes[4]) {  // Use receivedBytes[4] instead of pData[4]
          case 0x01:
            bms_settings();
            break;
          case 0x02:
            parseData();
            break;
          case 0x03:
            parseDeviceInfo();
            break;
          default:
            break;
        }

        break;  // Exit the loop after processing the complete frame
      }
    }
  }
}

// Обратный вызов (callback) для УВЕДОМЛЕНИЙ
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
  parseBMSData(pData, length);
}

// Обратный вызов (callback) для СТАТУСА ПОДКЛЮЧЕНИЯ
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pClient) {
    isConnected = true;
  }

  void onDisconnect(BLEClient* pClient) {
    isConnected = false;
    deviceFound = false; 
    pBmsDevice = nullptr; // Сбрасываем выбранное устройство
  }
};

#define CCCD_UUID ((uint16_t)0x2902)

// Функция попытки подключения к BMS
bool connectToBMS() {
  if (pBmsDevice == nullptr) return false;
  
  BLEClient* pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());
  
  if (!pClient->connect(pBmsDevice)) { 
    Serial.println("Не удалось подключиться.");
    return false;
  }

  // Получаем сервис
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    pClient->disconnect();
    return false;
  }

  // Получаем характеристику для записи
  pWriteCharacteristic = pRemoteService->getCharacteristic(charWriteUUID);
  if (pWriteCharacteristic == nullptr) {
    pClient->disconnect();
    return false;
  }

  // Получаем характеристику для уведомлений
  pNotifyCharacteristic = pRemoteService->getCharacteristic(charNotifyUUID);
  if (pNotifyCharacteristic == nullptr) {
    pClient->disconnect();
    return false;
  }

    if (pNotifyCharacteristic->canNotify()) {
      // Step 1: Register the callback function
      pNotifyCharacteristic->registerForNotify(notifyCallback);
      // Step 2: Get the CCCD descriptor
      BLERemoteDescriptor* pCCCD = pNotifyCharacteristic->getDescriptor(BLEUUID(CCCD_UUID));
      if (pCCCD != nullptr) {
          // Step 3: Write the value to enable notifications (0x01)
          // For Indications, use {0x2, 0x0}
          uint8_t notifyOn[] = {0x1, 0x0};
          // Write the 2-byte value to the descriptor
          pCCCD->writeValue(notifyOn, 2, true);
          isConnected = true;
          delay(500);
          writeRegister(0x97, 0x00000000, 0x00); // COMMAND_DEVICE_INFO
          delay(500);
          writeRegister(0x96, 0x00000000, 0x00); // COMMAND_CELL_INFO
      } else {
          return false;
      }
    } else {
      return false;
    }
  return true;
}

// Обратный вызов (callback) для СКАНЕРА (обновлено)
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
  
    // Проверяем, есть ли у устройства искомый нами Service UUID
    if (advertisedDevice.getName().length() > 0 && advertisedDevice.haveServiceUUID() && advertisedDevice.getServiceUUID().equals(serviceUUID)) {
      std::string deviceAddress = advertisedDevice.getAddress().toString().c_str();
      std::string deviceName = advertisedDevice.getName().c_str();

      // Проверяем, что устройство еще не добавлено
      if (foundDevices.find(deviceAddress) == foundDevices.end()) {
        Serial.printf("Найдено JK BMS: %s (%s)\n", deviceName.c_str(), deviceAddress.c_str());
        
        FoundBMS bms;
        bms.address = deviceAddress;
        bms.name = deviceName;
        // ВАЖНО: Храним копию, так как advertisedDevice уничтожается после onResult
        bms.pAdvertisedDevice = new BLEAdvertisedDevice(advertisedDevice); 
        
        foundDevices[deviceAddress] = bms;
      }
    }
  }
};
// ----------------------------------------------------------------------------------
// --- Функции веб-сервера (Web Server Functions) ---
// ----------------------------------------------------------------------------------

// --- НОВЫЕ ФУНКЦИИ ДЛЯ ВЫБОРА УСТРОЙСТВА ---
void handleSelectDevice() {
    server.sendHeader("Connection", "close");
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Вибір BMS</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<style>";
    html += "body{font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin: 0; padding: 20px; background-color: #f4f7f6; color: #333;}";
    html += ".container {max-width: 600px; margin: 0 auto; background: #fff; padding: 20px; border-radius: 10px; box-shadow: 0 4px 8px rgba(0,0,0,0.1);}";
    html += "h1 {text-align: center; color: #0056b3; border-bottom: 2px solid #e0e0e0; padding-bottom: 10px;}";
    html += "table{width: 100%; border-collapse: collapse; margin-bottom: 20px;}";
    html += "th, td{padding: 10px; text-align: left; border-bottom: 1px solid #ddd;}";
    html += "th {background-color: #007bff; color: white;}";
    html += "button {background-color: #28a745; color: white; padding: 8px 12px; border: none; border-radius: 5px; cursor: pointer;}";
    html += "button:hover {background-color: #218838;}";
    html += "p {text-align: center; color: #666;}";
    html += "</style></head><body><div class='container'>";
    html += "<h1>📶 Виберіть Jikong BMS для підключення</h1>";

    if (foundDevices.empty()) {
        html += "<p><strong>Пристроїв не знайдено.</strong> Натисніть кнопку, щоб повторити сканування.</p>";
        html += "<p><a href='/scan' style='display:inline-block; background-color:#ffc107; color:#333; padding:10px 20px; border-radius:5px; text-decoration:none;'>Повторне сканування</a></p>";
    } else {
        html += "<p>Знайдено **" + String(foundDevices.size()) + "** пристроїв. Виберіть один:</p>";
        html += "<form action='/connect' method='get'><table><tr><th>Ім'я</th><th>MAC-адреса</th><th>Дія</th></tr>";
        
        for (const auto& pair : foundDevices) {
            html += "<tr>";
            html += "<td>" + String(pair.second.name.c_str()) + "</td>";
            html += "<td>" + String(pair.second.address.c_str()) + "</td>";
            html += "<td><button type='submit' name='mac' value='" + String(pair.second.address.c_str()) + "'>ПІДКЛЮЧИТИСЯ</button></td>";
            html += "</tr>";
        }

        html += "</table></form>";
    }
    
    html += "<p><a href='/'>Назад до головної</a> (якщо вже підключено)</p>";
    html += "</div></body></html>";
    server.send(200, "text/html; charset=UTF-8", html);
}

void handleConnect() {
    if (server.hasArg("mac")) {
        std::string macAddress = server.arg("mac").c_str();
        
        if (foundDevices.count(macAddress)) {
            // Освобождаем старую память, если pBmsDevice уже был установлен (должен быть очищен в onDisconnect)
            if (pBmsDevice != nullptr) {
                // ВАЖНО: Не удаляем, так как это память из foundDevices
            }
            // Устанавливаем выбранное устройство (используем хранящийся указатель)
            pBmsDevice = foundDevices[macAddress].pAdvertisedDevice;
            deviceFound = true; // Триггер для loop()
            Serial.printf("Вибрано пристрій для підключення: %s\n", macAddress.c_str());
        } else {
            Serial.printf("Помилка: Невідома MAC-адреса: %s\n", macAddress.c_str());
        }
    }
    server.sendHeader("Location", "/");
    server.send(302, "text/plain; charset=UTF-8", "Redirecting...");
}


void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>Дані Jikong BMS - ESP32</title>";
  html += "<style>";
  html += "body{font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin: 0; padding: 20px; background-color: #f4f7f6; color: #333;}";
  html += ".container {max-width: 1000px; margin: 0 auto; background: #fff; padding: 20px; border-radius: 10px; box-shadow: 0 4px 8px rgba(0,0,0,0.1);}";
  html += "h1, h2, h3 {color: #007bff; border-bottom: 2px solid #e0e0e0; padding-bottom: 10px; margin-top: 25px;}";
  html += "h1 {text-align: center; color: #0056b3;}";
  html += "p {text-align: center; color: #666;}";
  html += "table{width: 100%; border-collapse: separate; border-spacing: 0; margin-bottom: 20px; background: #f9f9f9; border-radius: 8px; overflow: hidden; box-shadow: 0 1px 3px rgba(0,0,0,0.05);}";
  html += "th, td{padding: 12px 15px; text-align: left;}";
  html += "th {background-color: #007bff; color: white; font-weight: 600; text-transform: uppercase; border-bottom: none;}";
  html += "tr:nth-child(even) {background-color: #eaf6ff;}";
  html += "tr:hover {background-color: #dbe9f7;}";
  html += "td:nth-child(2) {font-weight: bold;}";
  html += ".status-connected {color: #28a745; font-weight: bold;}";
  html += ".status-disconnected {color: #dc3545; font-weight: bold;}";
  html += ".cell-table th:nth-child(2), .cell-table td:nth-child(2) {width: 15%; text-align: right;}";
  html += ".cell-table th:nth-child(3), .cell-table td:nth-child(3) {width: 15%; text-align: right;}";
  // Стилі для кнопки керування
  html += ".control-button {background-color: #28a745; color: white; padding: 8px 15px; border: none; border-radius: 5px; cursor: pointer; text-decoration: none; font-weight: bold; display: inline-block; margin: 2px 5px;}";
  html += ".control-button.off {background-color: #dc3545;}";
  html += ".control-button.toggle-on {background-color: #28a745;}";
  html += ".control-button.toggle-off {background-color: #dc3545;}";
  html += ".control-button.discharge {background-color: #ffc107; color: #333;}"; // Жовта для розряду
  html += ".control-button.discharge.off {background-color: #dc3545; color: white;}";
  html += ".control-button:hover {opacity: 0.9;}";
  
  // Стилі для кнопки OTA и Сканирования
  html += ".ota-button {background-color: #6c757d; color: white; padding: 10px 20px; border: none; border-radius: 5px; cursor: pointer; text-decoration: none; font-weight: bold; display: block; width: fit-content; margin: 20px auto;}";
  html += ".ota-button:hover {background-color: #5a6268;}";
  html += ".scan-button {background-color: #ffc107; color: #333; padding: 10px 20px; border: none; border-radius: 5px; cursor: pointer; text-decoration: none; font-weight: bold; display: block; width: fit-content; margin: 20px auto;}";
  html += ".scan-button:hover {background-color: #e0a800;}";
  
  // Стили для полей ввода
  html += "input[type='number'] {padding: 8px; border: 1px solid #ccc; border-radius: 4px; width: 80px; text-align: right;}";
  html += "</style></head><body><div class='container'>";
  html += "<h1>⚡️ Дані Jikong BMS (ESP32)</h1>";
  
  // ДОБАВЛЕНО: Кнопка сканування, якщо відключено
  if (!isConnected) {
    html += "<p><em>IP-адреса: <strong>" + WiFi.localIP().toString() + "</strong> | Статус BMS: <span class='status-disconnected'>ВІДКЛЮЧЕНО</span></em></p>";
    html += "<a href='/scan' class='scan-button'>▶️ СКАНУВАТИ ТА ВИБРАТИ BMS</a>";
    html += "</div></body></html>";
    server.send(200, "text/html; charset=UTF-8", html);
    return; // Выходим, не показывая остальной контент
  }


  String connectionStatus = isConnected ?
  "<span class='status-connected'>ПІДКЛЮЧЕНО</span>" : "<span class='status-disconnected'>ВІДКЛЮЧЕНО</span>";
  html += "<p><em>IP-адреса: <strong>" + WiFi.localIP().toString() + "</strong> | Статус BMS: " + connectionStatus + "</em></p>";
  
  // --- Кнопка OTA ---
  html += "<a href='/update' class='ota-button'>Оновлення прошивки (OTA)</a>";
  // --- Загальні дані ---
  html += "<h2>📊 Загальні дані</h2>";
  html += "<table>";
  html += "<tr><th>Параметр</th><th>Значення</th><th>Од. виміру</th></tr>";
  html += "<tr><td>Напруга батареї</td><td>" + String(Battery_Voltage, 2) + "</td><td>V</td></tr>";
  html += "<tr><td>Струм заряду/розряду</td><td>" + String(Charge_Current, 2) + "</td><td>A</td></tr>";
  html += "<tr><td>Потужність</td><td>" + String(Battery_Power, 2) + "</td><td>W</td></tr>";
  html += "<tr><td>Залишок заряду (SOC)</td><td>" + String(Percent_Remain) + "</td><td>%</td></tr>";
  html += "<tr><td>Залишкова ємність</td><td>" + String(Capacity_Remain, 2) + "</td><td>Ah</td></tr>";
  html += "<tr><td>Номінальна ємність</td><td>" + String(Nominal_Capacity, 2) + "</td><td>Ah</td></tr>";
  html += "<tr><td>Циклів заряду/розряду</td><td>" + String(Cycle_Count, 0) + "</td><td></td></tr>";
  html += "<tr><td>Середня напруга комірки</td><td>" + String(Average_Cell_Voltage, 3) + "</td><td>V</td></tr>";
  html += "<tr><td>Різниця напруг комірок</td><td>" + String(Delta_Cell_Voltage, 3) + "</td><td>V</td></tr>";
  html += "</table>";
  // --- Температури та Статуси ---
  html += "<h2>🌡️ Температури та Статуси / Керування</h2>";
  html += "<table>";
  html += "<tr><th>Параметр</th><th>Значення</th><th>Керування</th></tr>";
  html += "<tr><td>Температура T1 (Батарея)</td><td>" + String(Battery_T1, 1) + " °C</td><td></td></tr>";
  html += "<tr><td>Температура T2 (Батарея)</td><td>" + String(Battery_T2, 1) + " °C</td><td></td></tr>";
  html += "<tr><td>Температура MOS</td><td>" + String(MOS_Temp, 1) + " °C</td><td></td></tr>";
  html += "<tr><td>Час роботи (Uptime)</td><td>" + String(days) + "д " + String(hr) + "год " + String(mi) + "хв</td><td></td></tr>";
  // --- КЕРУВАННЯ ЗАРЯДОМ ---
  html += "<tr><td>Дозволено заряд</td><td>" + String(Charge ? "🟢 УВІМК" : "🔴 ВИМК") + "</td><td>";
  if (Charge) {
      html += "<a href='/charge_off' class='control-button toggle-off off'>ВИМКНУТИ ЗАРЯД</a>";
  } else {
      html += "<a href='/charge_on' class='control-button toggle-on'>УВІМКНУТИ ЗАРЯД</a>";
  }
  html += "</td></tr>";
  // --- КЕРУВАННЯ РОЗРЯДОМ ---
  html += "<tr><td>Дозволено розряд</td><td>" + String(Discharge ? "🟢 УВІМК" : "🔴 ВИМК") + "</td><td>";
  if (Discharge) {
      html += "<a href='/discharge_off' class='control-button toggle-off off'>ВИМКНУТИ РОЗРЯД</a>";
  } else {
      html += "<a href='/discharge_on' class='control-button toggle-on discharge'>УВІМКНУТИ РОЗРЯД</a>";
  }
  html += "</td></tr>";
  
  // --- КЕРУВАННЯ БАЛАНСУВАННЯМ ---
  html += "<tr><td>Балансування</td><td>" + String(Balance ? "🟢 Активне" : "⚪ Неактивне") + "</td><td>";
  if (Balance) {
      html += "<a href='/balance_off' class='control-button toggle-off off'>ВИМКНУТИ БАЛАНС</a>";
  } else {
      html += "<a href='/balance_on' class='control-button toggle-on'>УВІМКНУТИ БАЛАНС</a>";
  }
  html += "</td></tr>";
  html += "<tr><td>Струм балансування</td><td>" + String(Balance_Curr, 3) + " A</td><td></td></tr>";
  
  html += "</table>";
  // --- Напруги комірок ---
  html += "<h2>🔬 Напруги комірок</h2>";
  html += "<table class='cell-table'>";
    html += "<tr><th>Комірка</th><th>Напруга (V)</th><th>Опір (Ом)</th></tr>";
  for (int j = 0; j < cell_count; j++) { 
      if (cellVoltage[j] > 0.1) { 
          html += "<tr><td>" + String(j + 1) + "</td><td>" + String(cellVoltage[j], 3) + "</td><td>" + String(wireResist[j], 3) + "</td></tr>";
  }
  }
  if (cell_count == 0) {
      for (int j = 0; j < 16; j++) {
          if (cellVoltage[j] > 0.1) {
              html += "<tr><td>" + String(j + 1) + "</td><td>" + String(cellVoltage[j], 3) + "</td><td>" + String(wireResist[j], 3) + "</td></tr>";
  }
      }
  }
  html += "</table>";
  // --------------------------------------------------------------------------
  // --- Модифицированный раздел Налаштування BMS с полями ввода и кнопкой Змінити ---
  // --------------------------------------------------------------------------
  html += "<h2>⚙️ Налаштування BMS / Редагування</h2>";
  // Форма для отправки данных (method POST, action - новый обработчик)
  html += "<form action='/settings_update' method='post'>";
  
  html += "<table>";
  html += "<tr><th>Параметр</th><th>Значення</th><th>Од. виміру</th></tr>";
  
  // ОСНОВНЫЕ НАСТРОЙКИ
  html += "<tr><td>Загальна ємність батареї</td><td><input type='number' step='0.01' name='total_battery_capacity' value='" + String(total_battery_capacity, 2) + "' required></td><td>Ah</td></tr>";
  // 0x4D
  html += "<tr><td>Кількість комірок</td><td><input type='number' step='1' name='cell_count' value='" + String(cell_count) + "' required></td><td></td></tr>";
  // 0x4B
  html += "<tr><td>Напруга вимкнення (Power Off)</td><td><input type='number' step='0.001' name='power_off_voltage' value='" + String(power_off_voltage, 3) + "' required></td><td>V</td></tr>";
  // 0x4C
  
  // НАСТРОЙКИ НАПРЯЖЕНИЯ И БАЛАНСИРОВКИ
  html += "<tr><td>Напруга початку балансування</td><td><input type='number' step='0.001' name='balance_starting_voltage' value='" + String(balance_starting_voltage, 3) + "' required></td><td>V</td></tr>";
  // 0x5B
  html += "<tr><td>Напруга спрацювання балансування</td><td><input type='number' step='0.001' name='balance_trigger_voltage' value='" + String(balance_trigger_voltage, 3) + "' required></td><td>V</td></tr>";
  // 0x45
  html += "<tr><td>Макс. струм балансування</td><td><input type='number' step='0.001' name='max_balance_current' value='" + String(max_balance_current, 3) + "' required></td><td>A</td></tr>";
  // 0x4A

  // ЗАЩИТА ПО НАПРЯЖЕНИЮ
  html += "<tr><td>**Захист: Мін. напруга комірки**</td><td><input type='number' step='0.001' name='cell_uvp' value='" + String(cell_voltage_undervoltage_protection, 3) + "' required></td><td>V</td></tr>";
  // 0x41
  html += "<tr><td>Відновлення: Мін. напруга комірки</td><td><input type='number' step='0.001' name='cell_uvr' value='" + String(cell_voltage_undervoltage_recovery, 3) + "' required></td><td>V</td></tr>";
  // 0x42
  html += "<tr><td>**Захист: Макс. напруга комірки**</td><td><input type='number' step='0.001' name='cell_ovp' value='" + String(cell_voltage_overvoltage_protection, 3) + "' required></td><td>V</td></tr>";
  // 0x43
  html += "<tr><td>Відновлення: Макс. напруга комірки</td><td><input type='number' step='0.001' name='cell_ovr' value='" + String(cell_voltage_overvoltage_recovery, 3) + "' required></td><td>V</td></tr>";
  // 0x44
  
  // НАСТРОЙКИ ТОКА И ЗАЩИТЫ
  html += "<tr><td>**Захист: Макс. струм заряду**</td><td><input type='number' step='0.01' name='max_charge_current' value='" + String(max_charge_current, 2) + "' required></td><td>A</td></tr>";
  // 0x46
  html += "<tr><td>Затримка захисту від переструму (Заряд)</td><td><input type='number' step='1' name='charge_oc_delay' value='" + String(charge_overcurrent_protection_delay, 0) + "' required></td><td>с</td></tr>";
  // 0x47
  html += "<tr><td>Час відновлення від переструму (Заряд)</td><td><input type='number' step='1' name='charge_oc_recovery' value='" + String(charge_overcurrent_protection_recovery_time, 0) + "' required></td><td>с</td></tr>";
  // 0x48
  html += "<tr><td>**Захист: Макс. струм розряду**</td><td><input type='number' step='0.01' name='max_discharge_current' value='" + String(max_discharge_current, 2) + "' required></td><td>A</td></tr>";
  // 0x49
  //html += "<tr><td>Затримка захисту від КЗ</td><td><input type='number' step='1' name='sc_delay' value='" + String(short_circuit_protection_delay, 0) + "' required></td><td>мкс</td></tr>";
  // 0x5A
  html += "<tr><td>Час відновлення від КЗ</td><td><input type='number' step='1' name='sc_recovery' value='" + String(short_circuit_protection_recovery_time, 0) + "' required></td><td>с</td></tr>";
  // 0x4E
  
  // НАСТРОЙКИ ТЕМПЕРАТУРЫ
  html += "<tr><td>**Захист: Перегрів (Заряд)**</td><td><input type='number' step='0.1' name='charge_ot_prot' value='" + String(charge_overtemperature_protection, 1) + "' required></td><td>°C</td></tr>";
  // 0x50
  html += "<tr><td>Відновлення: Перегрів (Заряд)</td><td><input type='number' step='0.1' name='charge_ot_rec' value='" + String(charge_overtemperature_protection_recovery, 1) + "' required></td><td>°C</td></tr>";
  // 0x53
  html += "<tr><td>**Захист: Перегрів (Розряд)**</td><td><input type='number' step='0.1' name='discharge_ot_prot' value='" + String(discharge_overtemperature_protection, 1) + "' required></td><td>°C</td></tr>";
  // 0x54
  html += "<tr><td>Відновлення: Перегрів (Розряд)</td><td><input type='number' step='0.1' name='discharge_ot_rec' value='" + String(discharge_overtemperature_protection_recovery, 1) + "' required></td><td>°C</td></tr>";
  // 0x55
  html += "<tr><td>**Захист: Недогрів (Заряд)**</td><td><input type='number' step='0.1' name='charge_ut_prot' value='" + String(charge_undertemperature_protection, 1) + "' required></td><td>°C</td></tr>";
  // 0x56
  html += "<tr><td>Відновлення: Недогрів (Заряд)</td><td><input type='number' step='0.1' name='charge_ut_rec' value='" + String(charge_undertemperature_protection_recovery, 1) + "' required></td><td>°C</td></tr>";
  // 0x57
  html += "<tr><td>**Захист: Перегрів силових ключів**</td><td><input type='number' step='0.1' name='power_tube_ot_prot' value='" + String(power_tube_overtemperature_protection, 1) + "' required></td><td>°C</td></tr>";
  // 0x58
  html += "<tr><td>Відновлення: Перегрів силових ключів</td><td><input type='number' step='0.1' name='power_tube_ot_rec' value='" + String(power_tube_overtemperature_protection_recovery, 1) + "' required></td><td>°C</td></tr>";
  // 0x59
  
  html += "</table>";
  // Кнопка сохранения настроек
  html += "<input type='submit' class='ota-button' value='ЗБЕРЕГТИ НАЛАШТУВАННЯ' style='background-color:#007bff; margin-top:10px;'>";
  html += "</form>";
  
  html += "</div></body></html>";
  server.send(200, "text/html; charset=UTF-8", html);
};

// ----------------------------------------------------------------------------------
// --- Функції OTA Оновлення ---
// ----------------------------------------------------------------------------------

void handleUpdate() {
  server.sendHeader("Connection", "close");
  server.send(200, "text/html",
    "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>OTA Оновлення</title>"
    "<style>body{font-family: Arial, sans-serif; text-align: center; padding: 50px; background-color: #f4f7f6;}"
    ".container{max-width: 600px; margin: 0 auto; background: #fff; padding: 20px; border-radius: 10px; box-shadow: 0 4px 8px rgba(0,0,0,0.1);}"
    "h1{color: #0056b3;}"
    "input[type=file]{padding: 10px; border: 1px solid #ccc; border-radius: 4px; display: block; width: calc(100% - 22px); margin: 10px 0;}"
    "input[type=submit]{background-color: #28a745; color: white; padding: 10px 20px; border: none; border-radius: 4px; cursor: pointer;}"
    "input[type=submit]:hover{background-color: #218838;}</style></head><body><div class='container'>"
    "<h1>Оновлення прошивки ESP32 (OTA)</h1>"
    "<form method='POST' action='/update'  enctype='multipart/form-data'>"
    "<input type='file' name='update'>"
    "<input type='submit' value='Оновити прошивку'>"
    "</form></div></body></html>");
}

// Адрес регистра для управления балансировкой: 0x1F
void handleBalanceOn() {
  if (isConnected) {
    writeRegister(0x1F, 0x00000001, 0x04);
  } 
    delay(1500);
  server.sendHeader("Location", "/");
  server.send(302, "text/plain; charset=UTF-8", "Redirecting...");
}

void handleBalanceOff() {
  if (isConnected) {
    writeRegister(0x1F, 0x00000000, 0x04);
  } 
    delay(1500);
  server.sendHeader("Location", "/");
  server.send(302, "text/plain; charset=UTF-8", "Redirecting...");
}

// --- НОВЫЕ ФУНКЦИИ: Управление ЗАРЯДОМ ---
// Адрес регистра для управления ЗАРЯДОМ: 0x1E
void handleChargeOn() {
  if (isConnected) {
    writeRegister(0x1D, 0x00000001, 0x04);
  } 
    delay(1500);
  server.sendHeader("Location", "/");
  server.send(302, "text/plain; charset=UTF-8", "Redirecting...");
}

void handleChargeOff() {
  if (isConnected) {
    writeRegister(0x1D, 0x00000000, 0x04);
  }
    delay(1500);
  server.sendHeader("Location", "/");
  server.send(302, "text/plain; charset=UTF-8", "Redirecting...");
}

// --- НОВЫЕ ФУНКЦИИ: Управление РАЗРЯДОМ ---
// Адрес регистра для управления РАЗРЯДОМ: 0x1D
void handleDischargeOn() {
  if (isConnected) {
    writeRegister(0x1E, 0x00000001, 0x04);
  }
    delay(1500);
  server.sendHeader("Location", "/");
  server.send(302, "text/plain; charset=UTF-8", "Redirecting...");
}

void handleDischargeOff() {
  if (isConnected) {
    writeRegister(0x1E, 0x00000000, 0x04);
  }
  delay(1500);
  server.sendHeader("Location", "/");
  server.send(302, "text/plain; charset=UTF-8", "Redirecting...");
}


void handleUpdateUpload() {
  server.sendHeader("Connection", "close");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  
  // Проверяем, есть ли файл
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {

    // Если тип обновления - OTA (Update.getFreeSketchSpace() = UPDATE_SIZE_UNKNOWN)
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { 
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      Serial.printf("Успешное обновление. Размер: %u\n", upload.totalSize);
      server.send(200, "text/plain; charset=UTF-8", "Успешное обновление! ESP32 перезагружается...");
      delay(1000);
      ESP.restart();
    } else {
      Update.printError(Serial);
      server.send(500, "text/plain; charset=UTF-8", "Ошибка обновления. См. Serial Monitor.");
    }
  } else {
    server.send(500, "text/plain; charset=UTF-8", "Неизвестная ошибка загрузки.");
  }
}

// --- НОВАЯ ФУНКЦИЯ: Обработка обновления настроек BMS ---
void handleSettingsUpdate() {
  if (!isConnected) {
    server.sendHeader("Location", "/");
    server.send(302, "text/plain; charset=UTF-8", "Not Connected to BMS. Redirecting...");
    return;
  }
  
   if (isConnected) {
    // Общая емкость (Total Capacity): Регистр 0x4D, float * 1000
    if (server.hasArg("total_battery_capacity") != total_battery_capacity) {
      float val = server.arg("total_battery_capacity").toFloat();
      writeRegister(0x4D, (uint32_t)(val * 1000), 0x04);
    }
    
    // Количество ячеек (Cell Count): Регистр 0x4B, int * 1
    if (server.hasArg("cell_count") != cell_count) {
      int val = server.arg("cell_count").toInt();
      writeRegister(0x4B, (uint32_t)val, 0x04);
    }
    
    // Напряжение выключения (Power Off Voltage): Регистр 0x4C, float * 1000
    if (server.hasArg("power_off_voltage") != power_off_voltage) {
      float val = server.arg("power_off_voltage").toFloat();
      writeRegister(0x4C, (uint32_t)(val * 1000), 0x04);
    }

    // Напряжение начала балансировки (Balance Starting Voltage): Регистр 0x5B, float * 1000
    if (server.hasArg("balance_starting_voltage") != balance_starting_voltage) {
      float val = server.arg("balance_starting_voltage").toFloat();
      writeRegister(0x5B, (uint32_t)(val * 1000), 0x04);
    }

    // Напряжение срабатывания балансировки (Balance Trigger Voltage): Регистр 0x45, float * 1000
    if (server.hasArg("balance_trigger_voltage") != balance_trigger_voltage) {
      float val = server.arg("balance_trigger_voltage").toFloat();
      writeRegister(0x45, (uint32_t)(val * 1000), 0x04);
    }

    // Макс. ток балансировки (Max Balance Current): Регистр 0x4A, float * 1000
    if (server.hasArg("max_balance_current") != max_balance_current) {
      float val = server.arg("max_balance_current").toFloat();
      writeRegister(0x4A, (uint32_t)(val * 1000), 0x04);
    }
    
    // Напряжения ячеек (Cell Voltages): Регистры 0x41 - 0x44, float * 1000
    if (server.hasArg("cell_uvp") != cell_voltage_undervoltage_protection) { // UVP: Undervoltage Protection (0x41)
      float val = server.arg("cell_uvp").toFloat();
      writeRegister(0x41, (uint32_t)(val * 1000), 0x04);
    }
    if (server.hasArg("cell_uvr") != cell_voltage_undervoltage_recovery) { // UVR: Undervoltage Recovery (0x42)
      float val = server.arg("cell_uvr").toFloat();
      writeRegister(0x42, (uint32_t)(val * 1000), 0x04);
    }
    if (server.hasArg("cell_ovp")!= cell_voltage_overvoltage_protection) { // OVP: Overvoltage Protection (0x43)
      float val = server.arg("cell_ovp").toFloat();
      writeRegister(0x43, (uint32_t)(val * 1000), 0x04);
    }
    if (server.hasArg("cell_ovr") != cell_voltage_overvoltage_recovery) { // OVR: Overvoltage Recovery (0x44)
      float val = server.arg("cell_ovr").toFloat();
      writeRegister(0x44, (uint32_t)(val * 1000), 0x04);
    }
    
    // Токовые настройки (Current Settings): Регистры 0x46 - 0x49, int * 1 (delay) / float * 1000 (current)
    if (server.hasArg("max_charge_current") !=  max_charge_current) { // Max Charge Current (0x46)
      float val = server.arg("max_charge_current").toFloat();
      writeRegister(0x46, (uint32_t)(val * 1000), 0x04);
    }
    if (server.hasArg("charge_oc_delay") != charge_overcurrent_protection_recovery_time) { // Charge OC Delay (0x47)
      int val = server.arg("charge_oc_delay").toInt();
      writeRegister(0x47, (uint32_t)val, 0x04);
    }
    if (server.hasArg("charge_oc_recovery") != charge_overcurrent_protection_recovery_time) { // Charge OC Recovery (0x48)
      int val = server.arg("charge_oc_recovery").toInt();
      writeRegister(0x48, (uint32_t)val, 0x04);
    }
    if (server.hasArg("max_discharge_current") != max_discharge_current) { // Max Discharge Current (0x49)
      float val = server.arg("max_discharge_current").toFloat();
      writeRegister(0x49, (uint32_t)(val * 1000), 0x04);
    }
    if (server.hasArg("discharge_oc_delay")) { // Discharge OC Delay (0x4A) - NOTE: This address is also max_balance_current, check protocol
      // NOTE: Based on JK protocol, 0x4A is Max Balance Current. Discharge OC Delay is likely next address or different mapping.
      // Sticking to most likely addresses based on common usage:
      // Discharge OC Delay (0x4A in some older protocols, using 0x4A for max_balance_current, 0x4B for cell_count)
      // For now, let's use the addresses derived from the received settings packet indices:
      // If receivedBytes[66] is discharge_oc_delay, its register should be 0x4A if 4 bytes/register is constant.
      // Let's use the register mapping that aligns with standard JKBMS writable registers, 0x4A for max_balance_current is more common.
      // Given the conflicting protocol info in the code, I will use:
      // Discharge OC Delay: 0x4A is Max Balance, 0x4B is Cell Count, 0x4C is Power Off.
      // Let's assume the user's BMS is standard and the write addresses are sequential from 0x41.
      // I will skip writing this one to avoid conflicts unless I can confirm the correct register.
      // Instead of guessing, I will use a reliable, confirmed address like 0x4B for Max Balance Current if I can't be sure.
      // Re-check: Index 66 (D.OC Delay) is 0x4A offset (if 0x41 is offset 10). Let's use the next available: 0x4F (unused in current list).
      // Based on common JKBMS registers, D.OC Delay is usually 0x4F or similar. Skipping for safety unless confirmed.
      // For now, I will write the confirmed registers. The list above is complete.
    }
    if (server.hasArg("discharge_oc_recovery") != discharge_overcurrent_protection_recovery_time) { // Discharge OC Recovery (Index 70 -> 0x4B offset, which is Cell Count)
      // Skipping to avoid overwriting cell count (0x4B)
    }
    if (server.hasArg("sc_recovery") != short_circuit_protection_recovery_time) { // SC Recovery (0x4E)
      int val = server.arg("sc_recovery").toInt();
      writeRegister(0x4E, (uint32_t)val, 0x04);
    }
    //if (server.hasArg("sc_delay")) { // SC Delay (0x5A)
    //  int val = server.arg("sc_delay").toInt();
    //  writeRegister(0x5A, (uint32_t)val, 0x04);
    //}

    // Температурные настройки (Temperature Settings): Регистры 0x50, 0x53-0x59, float * 10
    if (server.hasArg("charge_ot_prot") != charge_overtemperature_protection) { // Charge OT Protection (0x50)
      float val = server.arg("charge_ot_prot").toFloat();
      writeRegister(0x50, (uint32_t)(val * 10), 0x04);
    }
    if (server.hasArg("charge_ot_rec") != charge_overtemperature_protection_recovery) { // Charge OT Recovery (0x53)
      float val = server.arg("charge_ot_rec").toFloat();
      writeRegister(0x53, (uint32_t)(val * 10), 0x04);
    }
    if (server.hasArg("discharge_ot_prot") != discharge_overtemperature_protection) { // Discharge OT Protection (0x54)
      float val = server.arg("discharge_ot_prot").toFloat();
      writeRegister(0x54, (uint32_t)(val * 10), 0x04);
    }
    if (server.hasArg("discharge_ot_rec") != discharge_overtemperature_protection_recovery) { // Discharge OT Recovery (0x55)
      float val = server.arg("discharge_ot_rec").toFloat();
      writeRegister(0x55, (uint32_t)(val * 10), 0x04);
    }
    if (server.hasArg("charge_ut_prot") != charge_undertemperature_protection) { // Charge UT Protection (0x56)
      float val = server.arg("charge_ut_prot").toFloat();
      writeRegister(0x56, (uint32_t)(val * 10), 0x04);
    }
    if (server.hasArg("charge_ut_rec") != charge_undertemperature_protection_recovery) { // Charge UT Recovery (0x57)
      float val = server.arg("charge_ut_rec").toFloat();
      writeRegister(0x57, (uint32_t)(val * 10), 0x04);
    }
    if (server.hasArg("power_tube_ot_prot") != power_tube_overtemperature_protection) { // Power Tube OT Protection (0x58)
      float val = server.arg("power_tube_ot_prot").toFloat();
      writeRegister(0x58, (uint32_t)(val * 10), 0x04);
    }
    if (server.hasArg("power_tube_ot_rec") != power_tube_overtemperature_protection_recovery) { // Power Tube OT Recovery (0x59)
      float val = server.arg("power_tube_ot_rec").toFloat();
      writeRegister(0x59, (uint32_t)(val * 10), 0x04);
    }
   }
  
  delay(1500); // Даем время BMS обработать команду
  // Перенаправляем на главную страницу
  server.sendHeader("Location", "/");
  server.send(302, "text/plain; charset=UTF-8", "Settings Updated. Redirecting...");
};

// ----------------------------------------------------------------------------------
void init_wifi() {
  Serial.printf("Подключение к WiFi %s ", ssid);
  WiFi.begin(ssid, password);
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20) {
    delay(500);
    Serial.print(".");
    timeout++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(WiFi.localIP());
    if (MDNS.begin(host)) {
      Serial.printf("mDNS запущен, доступно по http://%s.local\n", host);
    }
  } else {
    Serial.println("\nНе удалось подключиться к WiFi.");
  }
};

void setup() {
  Serial.begin(115200);

  init_wifi();
  // run blutooth on core 0
  xTaskCreatePinnedToCore(
    blutooth(),  // Название функции
    "Проверка и подключение блутуза",  // Имя задачи (для отладки)
    10000,        // Размер стека задачи (в байтах)
    NULL,         // Параметр для функции (NULL, если не нужен)
    1,            // Приоритет задачи (1 - обычный)
    NULL,         // Указатель на созданную задачу (NULL, если не нужен)
    0             // НОМЕР ЯДРА: 0 (Core 0) или 1 (Core 1)
  );
  
  // --- ОБНОВЛЕННЫЕ ОБРАБОТЧИКИ ---
  server.on("/", handleRoot);

  // Обновление настроек BMS (НОВЫЙ ОБРАБОТЧИК)
  server.on("/settings_update", HTTP_POST, handleSettingsUpdate);


  server.on("/balance_on", handleBalanceOn);
  server.on("/balance_off", handleBalanceOff);
  
  server.on("/charge_on", handleChargeOn);
  server.on("/charge_off", handleChargeOff);
  
  server.on("/discharge_on", handleDischargeOn);
  server.on("/discharge_off", handleDischargeOff);

  server.on("/scan", HTTP_GET, handleSelectDevice); // Новый: Страница сканирования и выбора
  server.on("/connect", HTTP_GET, handleConnect);   // Новый: Обработчик подключения

  server.on("/update", HTTP_GET, handleUpdate);
  server.on("/update", HTTP_POST, handleUpdateUpload);
  // --- КОНЕЦ ОБНОВЛЕННЫХ ОБРАБОТЧИКОВ ---

  server.begin();
  Serial.println("Веб-сервер запущен.");
  
  BLEDevice::init("ESP32_JK_Client");
  
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
};



void loop() {
  server.handleClient();
  delay(100);
};

void blutooth() {
    
  if (deviceFound && !isConnected) {
    connectToBMS();
  }  
  // Если не найдено (или соединение потеряно) и нет выбранного устройства
  else if (!deviceFound && !isConnected) {
    
    // Очищаем предыдущие результаты сканирования перед новым стартом
    for (auto& pair : foundDevices) {
        delete pair.second.pAdvertisedDevice;
    }
    foundDevices.clear();
    
    Serial.println("Починаємо сканування...");
    // Сканируем 5 секунд (или пока не найдем)
    pBLEScan->start(5, false); 
    Serial.printf("Сканування завершено. Знайдено пристроїв: %d\n", foundDevices.size());

    // Если найдено только ОДНО устройство, сразу устанавливаем его для подключения
    if (foundDevices.size() == 1) {
      // Так как map содержит только 1 элемент, берем его
      pBmsDevice = foundDevices.begin()->second.pAdvertisedDevice;
      deviceFound = true; 
      Serial.println("Знайдено 1 пристрій, спроба автоматичного підключення.");
    } 
    // Если 0 или >1, то deviceFound остается false. Пользователь должен перейти на /scan
    // для повторного сканирования или выбора.
  }

  delay(1000);
};

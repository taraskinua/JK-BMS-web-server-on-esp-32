/*
 * Скетч ESP32 для читання даних з Jikong (JK) BMS по BLE (Bluetooth Low Energy)
 * ESP32 виступає в ролі BLE Client.
 * Доданий веб-сервер для відображення отриманих даних та OTA-оновлення прошивки.
 * ДОДАНО: Функціонал сканування та вибору пристрою для підключення через веб-інтерфейс.
 */

// -----------------------------------------------------------------------------
// --- 1. INCLUDE LIBRARIES ----------------------------------------------------
// -----------------------------------------------------------------------------
#include "BLEDevice.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>  // Для OTA оновлення
#include <vector>
#include <map>
#include <string>
#include <cmath>     // Для abs() в порівнянні float

// -----------------------------------------------------------------------------
// --- 2. WIFI & WEB SERVER CONFIGURATION --------------------------------------
// -----------------------------------------------------------------------------

const char* ssid = "homewifi";                // <-- ЗАМЕНІТЬ на ваш SSID
const char* password = "homewifi1234567890";  // <-- ЗАМЕНІТЬ на ваш пароль

WebServer server(80);

// -----------------------------------------------------------------------------
// --- 3. BLE UUIDS & CONSTANTS ------------------------------------------------
// -----------------------------------------------------------------------------

static BLEUUID serviceUUID("0000ffe0-0000-1000-8000-00805f9b34fb");        // Основний сервіс
static BLEUUID charWriteUUID("0000ffe1-0000-1000-8000-00805f9b34fb");      // Характеристика для ЗАПИСУ
static BLEUUID charNotifyUUID("0000ffe1-0000-1000-8000-00805f9b34fb");     // Характеристика для СПОВІЩЕНЬ
#define CCCD_UUID ((uint16_t)0x2902)
#define FLOAT_TOLERANCE 0.001 // Допуск для порівняння float в налаштуваннях

// -----------------------------------------------------------------------------
// --- 4. BLE GLOBAL STATE & DEVICE MANAGEMENT ---------------------------------
// -----------------------------------------------------------------------------

struct FoundBMS {
  std::string address;
  std::string name;
  BLEAdvertisedDevice* pAdvertisedDevice;
};

// Використовуємо MAC-адресу як ключ для знайдених пристроїв
std::map<std::string, FoundBMS> foundDevices;

static BLEClient* pClient = nullptr;
static BLEAdvertisedDevice* pBmsDevice = nullptr;
static BLERemoteCharacteristic* pWriteCharacteristic = nullptr;
static BLERemoteCharacteristic* pNotifyCharacteristic = nullptr;
static BLEScan* pBLEScan = nullptr;

static bool deviceFound = false;     // true, тільки коли вибраний конкретний прилад
static bool isConnected = false;

// -----------------------------------------------------------------------------
// --- 5. DATA PROCESSING & FRAME STATE ----------------------------------------
// -----------------------------------------------------------------------------

byte receivedBytes[320];
int frame = 0;
bool received_start = false;
bool received_complete = false;
bool new_data = false;
int ignoreNotifyCount = 0;

// -----------------------------------------------------------------------------
// --- 6. BMS DATA STRUCTURES --------------------------------------------------
// -----------------------------------------------------------------------------

// Структура для динамічних даних (телеметрія)
struct BMS_Data {
  float cellVoltage[24] = { 0 };
  float wireResist[24] = { 0 };
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
  int cell_count = 0;
} G_data;


// Структура для налаштувань (параметри захисту та ліміти)
struct BMS_Settings {
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
  float total_battery_capacity = 0;
  float short_circuit_protection_delay = 0;
  float balance_starting_voltage = 0;
} G_settings;

// Структура для інформації про прилад
struct Device_Info {
  std::string vendorID = "Unknown Vendor";
  std::string hardwareVersion = "V0.0";
  std::string softwareVersion = "V0.0";
  std::string deviceName = "Unknown Device";
  std::string devicePasscode = "0000000000000000";
  std::string manufacturingDate = "00000000";
  std::string serialNumber = "00000000000";
  std::string passcode = "00000";
  std::string userData = "No user data";
  std::string setupPasscode = "0000000000000000";
  uint32_t uptime = 0;
  uint32_t powerOnCount = 0;
} G_info;


// -----------------------------------------------------------------------------
// --- 7. UTILITY FUNCTIONS (CRC & WRITE REGISTER) -----------------------------
// -----------------------------------------------------------------------------

uint8_t crc(const uint8_t data[], uint16_t len) {
  uint8_t crc = 0;
  for (uint16_t i = 0; i < len; i++)
    crc += data[i];
  return crc;
}

void writeRegister(uint8_t address, uint32_t value, uint8_t length) {
  if (pNotifyCharacteristic == nullptr) {
    Serial.println("Write Register Error: Notify Characteristic not available.");
    return;
  }

  uint8_t frame[20] = { 0xAA, 0x55, 0x90, 0xEB, address, length };

  // Вставка значення (Little-Endian)
  frame[6] = value >> 0;   // LSB
  frame[7] = value >> 8;
  frame[8] = value >> 16;
  frame[9] = value >> 24;  // MSB

  // Розрахунок CRC
  frame[19] = crc(frame, 19);

  pNotifyCharacteristic->writeValue((uint8_t*)frame, (size_t)sizeof(frame));
}


// -----------------------------------------------------------------------------
// --- 8. DATA PARSING LOGIC ---------------------------------------------------
// -----------------------------------------------------------------------------

void parseBMSData() {
  new_data = true;

  // Напруги комірок
  for (int j = 0, i = 7; i < 38; j++, i += 2) {
    G_data.cellVoltage[j] = ((receivedBytes[i] << 8 | receivedBytes[i - 1]) * 0.001);
  }

  // Опори дротів
  for (int j = 0, i = 81; i < 112; j++, i += 2) {
    G_data.wireResist[j] = (((int)receivedBytes[i] << 8 | receivedBytes[i - 1]) * 0.001);
  }

  G_data.Average_Cell_Voltage = (((int)receivedBytes[75] << 8 | receivedBytes[74]) * 0.001);
  G_data.Delta_Cell_Voltage = (((int)receivedBytes[77] << 8 | receivedBytes[76]) * 0.001);

  // Температура MOS (Signed 16-bit, converted to float * 0.1)
  if (receivedBytes[145] == 0xFF) {
    G_data.MOS_Temp = ((0xFF << 24 | 0xFF << 16 | receivedBytes[145] << 8 | receivedBytes[144]) * 0.1);
  } else {
    G_data.MOS_Temp = ((receivedBytes[145] << 8 | receivedBytes[144]) * 0.1);
  }

  // Напруга батареї (32-bit, Little-Endian * 0.001)
  G_data.Battery_Voltage = ((receivedBytes[153] << 24 | receivedBytes[152] << 16 | receivedBytes[151] << 8 | receivedBytes[150]) * 0.001);

  // Струм заряду/розряду (32-bit, Little-Endian * 0.001)
  G_data.Charge_Current = ((receivedBytes[161] << 24 | receivedBytes[160] << 16 | receivedBytes[159] << 8 | receivedBytes[158]) * 0.001);
  G_data.Battery_Power = G_data.Battery_Voltage * G_data.Charge_Current;

  // Температура батареї T1 (Signed 16-bit, converted to float * 0.1)
  if (receivedBytes[163] == 0xFF) {
    G_data.Battery_T1 = ((0xFF << 24 | 0xFF << 16 | receivedBytes[163] << 8 | receivedBytes[162]) * 0.1);
  } else {
    G_data.Battery_T1 = ((receivedBytes[163] << 8 | receivedBytes[162]) * 0.1);
  }

  // Температура батареї T2 (Signed 16-bit, converted to float * 0.1)
  if (receivedBytes[165] == 0xFF) {
    G_data.Battery_T2 = ((0xFF << 24 | 0xFF << 16 | receivedBytes[165] << 8 | receivedBytes[164]) * 0.1);
  } else {
    G_data.Battery_T2 = ((receivedBytes[165] << 8 | receivedBytes[164]) * 0.1);
  }

  // Струм балансування (Signed 16-bit, converted to float * 0.001)
  if ((receivedBytes[171] & 0xF0) == 0x0) {
    G_data.Balance_Curr = ((receivedBytes[171] << 8 | receivedBytes[170]) * 0.001);
  } else if ((receivedBytes[171] & 0xF0) == 0xF0) {
    G_data.Balance_Curr = (((receivedBytes[171] & 0x0F) << 8 | receivedBytes[170]) * -0.001);
  }

  G_data.Balancing_Action = receivedBytes[172];
  G_data.Percent_Remain = (receivedBytes[173]);
  G_data.Capacity_Remain = ((receivedBytes[177] << 24 | receivedBytes[176] << 16 | receivedBytes[175] << 8 | receivedBytes[174]) * 0.001);
  G_data.Nominal_Capacity = ((receivedBytes[181] << 24 | receivedBytes[180] << 16 | receivedBytes[179] << 8 | receivedBytes[178]) * 0.001);
  G_data.Cycle_Count = ((receivedBytes[185] << 24 | receivedBytes[184] << 16 | receivedBytes[183] << 8 | receivedBytes[182]));
  G_data.Cycle_Capacity = ((receivedBytes[189] << 24 | receivedBytes[188] << 16 | receivedBytes[187] << 8 | receivedBytes[186]) * 0.001);

  // Uptime (3-байти, Little-Endian)
  G_data.Uptime = receivedBytes[196] << 16 | receivedBytes[195] << 8 | receivedBytes[194];
  G_data.sec = G_data.Uptime % 60;
  G_data.Uptime /= 60;
  G_data.mi = G_data.Uptime % 60;
  G_data.Uptime /= 60;
  G_data.hr = G_data.Uptime % 24;
  G_data.days = G_data.Uptime / 24;

  // Статуси (Charge/Discharge/Balance)
  G_data.Charge = (receivedBytes[198] > 0);
  G_data.Discharge = (receivedBytes[199] > 0);
  G_data.Balance = (receivedBytes[201] > 0);
}

void parseBMSSettings() {
  G_settings.cell_voltage_undervoltage_protection = ((receivedBytes[13] << 24 | receivedBytes[12] << 16 | receivedBytes[11] << 8 | receivedBytes[10]) * 0.001);
  G_settings.cell_voltage_undervoltage_recovery = ((receivedBytes[17] << 24 | receivedBytes[16] << 16 | receivedBytes[15] << 8 | receivedBytes[14]) * 0.001);
  G_settings.cell_voltage_overvoltage_protection = ((receivedBytes[21] << 24 | receivedBytes[20] << 16 | receivedBytes[19] << 8 | receivedBytes[18]) * 0.001);
  G_settings.cell_voltage_overvoltage_recovery = ((receivedBytes[25] << 24 | receivedBytes[24] << 16 | receivedBytes[23] << 8 | receivedBytes[22]) * 0.001);
  G_settings.balance_trigger_voltage = ((receivedBytes[29] << 24 | receivedBytes[28] << 16 | receivedBytes[27] << 8 | receivedBytes[26]) * 0.001);
  G_settings.power_off_voltage = ((receivedBytes[49] << 24 | receivedBytes[48] << 16 | receivedBytes[47] << 8 | receivedBytes[46]) * 0.001);
  G_settings.max_charge_current = ((receivedBytes[53] << 24 | receivedBytes[52] << 16 | receivedBytes[51] << 8 | receivedBytes[50]) * 0.001);
  G_settings.charge_overcurrent_protection_delay = ((receivedBytes[57] << 24 | receivedBytes[56] << 16 | receivedBytes[55] << 8 | receivedBytes[54]));
  G_settings.charge_overcurrent_protection_recovery_time = ((receivedBytes[61] << 24 | receivedBytes[60] << 16 | receivedBytes[59] << 8 | receivedBytes[58]));
  G_settings.max_discharge_current = ((receivedBytes[65] << 24 | receivedBytes[64] << 16 | receivedBytes[63] << 8 | receivedBytes[62]) * 0.001);
  G_settings.discharge_overcurrent_protection_delay = ((receivedBytes[69] << 24 | receivedBytes[68] << 16 | receivedBytes[67] << 8 | receivedBytes[66]));
  G_settings.discharge_overcurrent_protection_recovery_time = ((receivedBytes[73] << 24 | receivedBytes[72] << 16 | receivedBytes[71] << 8 | receivedBytes[70]));
  G_settings.short_circuit_protection_recovery_time = ((receivedBytes[77] << 24 | receivedBytes[76] << 16 | receivedBytes[75] << 8 | receivedBytes[74]));
  G_settings.max_balance_current = ((receivedBytes[81] << 24 | receivedBytes[80] << 16 | receivedBytes[79] << 8 | receivedBytes[78]) * 0.001);
  G_settings.charge_overtemperature_protection = ((receivedBytes[85] << 24 | receivedBytes[84] << 16 | receivedBytes[83] << 8 | receivedBytes[82]) * 0.1);
  G_settings.charge_overtemperature_protection_recovery = ((receivedBytes[89] << 24 | receivedBytes[88] << 16 | receivedBytes[87] << 8 | receivedBytes[86]) * 0.1);
  G_settings.discharge_overtemperature_protection = ((receivedBytes[93] << 24 | receivedBytes[92] << 16 | receivedBytes[91] << 8 | receivedBytes[90]) * 0.1);
  G_settings.discharge_overtemperature_protection_recovery = ((receivedBytes[97] << 24 | receivedBytes[96] << 16 | receivedBytes[95] << 8 | receivedBytes[94]) * 0.1);
  G_settings.charge_undertemperature_protection = ((receivedBytes[101] << 24 | receivedBytes[100] << 16 | receivedBytes[99] << 8 | receivedBytes[98]) * 0.1);
  G_settings.charge_undertemperature_protection_recovery = ((receivedBytes[105] << 24 | receivedBytes[104] << 16 | receivedBytes[103] << 8 | receivedBytes[102]) * 0.1);
  G_settings.power_tube_overtemperature_protection = ((receivedBytes[109] << 24 | receivedBytes[108] << 16 | receivedBytes[107] << 8 | receivedBytes[106]) * 0.1);
  G_settings.power_tube_overtemperature_protection_recovery = ((receivedBytes[113] << 24 | receivedBytes[112] << 16 | receivedBytes[111] << 8 | receivedBytes[110]) * 0.1);
  G_data.cell_count = ((receivedBytes[117] << 24 | receivedBytes[116] << 16 | receivedBytes[115] << 8 | receivedBytes[114]));
  G_settings.total_battery_capacity = ((receivedBytes[133] << 24 | receivedBytes[132] << 16 | receivedBytes[131] << 8 | receivedBytes[130]) * 0.001);
  G_settings.short_circuit_protection_delay = ((receivedBytes[137] << 24 | receivedBytes[136] << 16 | receivedBytes[135] << 8 | receivedBytes[134]) * 1);
  G_settings.balance_starting_voltage = ((receivedBytes[141] << 24 | receivedBytes[140] << 16 | receivedBytes[139] << 8 | receivedBytes[138]) * 0.001);
}

void parseDeviceInfo() {
  // Перевірка, що мінімальна довжина фрейму для Device Info отримана (приблизно 134 байти)
  if (frame < 134) {
    return;
  }
  
  // Парсинг строкових даних
  G_info.vendorID.assign(receivedBytes + 6, receivedBytes + 6 + 16);
  G_info.hardwareVersion.assign(receivedBytes + 22, receivedBytes + 22 + 8);
  G_info.softwareVersion.assign(receivedBytes + 30, receivedBytes + 30 + 8);
  G_info.deviceName.assign(receivedBytes + 46, receivedBytes + 46 + 16);
  G_info.devicePasscode.assign(receivedBytes + 62, receivedBytes + 62 + 16);
  G_info.manufacturingDate.assign(receivedBytes + 78, receivedBytes + 78 + 8);
  G_info.serialNumber.assign(receivedBytes + 86, receivedBytes + 86 + 11);
  G_info.passcode.assign(receivedBytes + 97, receivedBytes + 97 + 5);
  G_info.userData.assign(receivedBytes + 102, receivedBytes + 102 + 16);
  G_info.setupPasscode.assign(receivedBytes + 118, receivedBytes + 118 + 16);

  // Парсинг uint32_t (Little-Endian)
  G_info.uptime = (receivedBytes[41] << 24) | (receivedBytes[40] << 16) | (receivedBytes[39] << 8) | receivedBytes[38];
  G_info.powerOnCount = (receivedBytes[45] << 24) | (receivedBytes[44] << 16) | (receivedBytes[43] << 8) | receivedBytes[42];
}

// Функція для парсингу (розбору) даних, отримуваних по частинах
void processBMSFrame(uint8_t* pData, size_t length) {
  // Перевірка на початок кадру даних (AA 55 EB 90)
  if (pData[0] == 0x55 && pData[1] == 0xAA && pData[2] == 0xEB && pData[3] == 0x90) {
    frame = 0;
    received_start = true;
    received_complete = false;

    // Зберігаємо отримані дані
    for (int i = 0; i < length; i++) {
      receivedBytes[frame++] = pData[i];
    }
  } else if (received_start && !received_complete) {
    for (int i = 0; i < length; i++) {
      receivedBytes[frame++] = pData[i];

      // Очікуємо максимальну довжину кадру 300 байтів
      if (frame >= 300) {
        received_complete = true;
        received_start = false;

        // Визначаємо тип кадру даних за receivedBytes[4]
        switch (receivedBytes[4]) {
          case 0x01: // Налаштування
            parseBMSSettings();
            break;
          case 0x02: // Дані телеметрії
            parseBMSData();
            break;
          case 0x03: // Інформація про пристрій
            parseDeviceInfo();
            break;
          default:
            break;
        }

        break;  // Виходимо з циклу після обробки повного кадру
      }
    }
  }
}

// Зворотний виклик (callback) для СПОВІЩЕНЬ
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {

  if (length < 10) {
    return;
  }
  processBMSFrame(pData, length);
}

// -----------------------------------------------------------------------------
// --- 9. BLE CLIENT CALLBACKS & SCAN LOGIC ------------------------------------
// -----------------------------------------------------------------------------

// Зворотний виклик (callback) для СТАТУСУ ПІДКЛЮЧЕННЯ
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pClient) {
    isConnected = true;
  }

  void onDisconnect(BLEClient* pClient) {
    isConnected = false;
    deviceFound = false;
    pBmsDevice = nullptr; // Скидаємо вибраний пристрій
    Serial.println("Disconnected from BMS.");
  }
};

// Зворотний виклик (callback) для СКАНЕРА
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // Перевіряємо, чи має пристрій потрібний Service UUID
    if (advertisedDevice.getName().length() > 0 && advertisedDevice.haveServiceUUID() && advertisedDevice.getServiceUUID().equals(serviceUUID)) {
      std::string deviceAddress = advertisedDevice.getAddress().toString().c_str();
      std::string deviceName = advertisedDevice.getName().c_str();

      // Перевіряємо, що пристрій ще не додано
      if (foundDevices.find(deviceAddress) == foundDevices.end()) {
        Serial.printf("Знайдено JK BMS: %s (%s)\n", deviceName.c_str(), deviceAddress.c_str());
        FoundBMS bms;
        bms.address = deviceAddress;
        bms.name = deviceName;
        // ВАЖЛИВО: Зберігаємо копію, оскільки advertisedDevice знищується після onResult
        bms.pAdvertisedDevice = new BLEAdvertisedDevice(advertisedDevice);
        foundDevices[deviceAddress] = bms;
      }
    }
  }
};

void bleScanTask() {
  // Очищаємо попередні результати
  for (auto& pair : foundDevices) {
    delete pair.second.pAdvertisedDevice;
  }
  foundDevices.clear();
  Serial.println("Починаємо асинхронне сканування...");

  // Скануємо 5 секунд. 'false' означає, що ми використовуємо Callbacks.
  pBLEScan->start(5, false);

  Serial.printf("Сканування завершено. Знайдено пристроїв: %d\n", foundDevices.size());
}

bool connectToServer(BLEAddress address) {
  Serial.printf("Спроба підключення до %s...\n", address.toString().c_str());

  if (pClient != nullptr) {
    // Очищаємо старий клієнт, якщо він існує, для перепідключення
    if (pClient->isConnected()) {
      pClient->disconnect();
    }
    delete pClient;
    pClient = nullptr;
  }

  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());

  if (!pClient->connect(address)) {
    Serial.println("Помилка: Не вдалося підключитися.");
    return false;
  }

  Serial.println("Підключення встановлено.");

  // Отримуємо сервіс BMS
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.println("Помилка: Сервіс не знайдено.");
    pClient->disconnect();
    return false;
  }

  // Отримуємо характеристику для ЗАПИСУ
  pWriteCharacteristic = pRemoteService->getCharacteristic(charWriteUUID);
  if (pWriteCharacteristic == nullptr) {
    Serial.println("Помилка: Характеристику запису не знайдено.");
    pClient->disconnect();
    return false;
  }

  // Отримуємо характеристику для СПОВІЩЕНЬ
  pNotifyCharacteristic = pRemoteService->getCharacteristic(charNotifyUUID);
  if (pNotifyCharacteristic == nullptr) {
    Serial.println("Помилка: Характеристику сповіщень не знайдено.");
    pClient->disconnect();
    return false;
  }

  // Включаємо сповіщення (Notify)
  if (pNotifyCharacteristic->canNotify()) {
    pNotifyCharacteristic->registerForNotify(notifyCallback);
    BLERemoteDescriptor* pCCCD = pNotifyCharacteristic->getDescriptor(BLEUUID(CCCD_UUID));
    if (pCCCD != nullptr) {
      uint8_t notifyOn[] = { 0x1, 0x0 };
      pCCCD->writeValue(notifyOn, 2, true);
      Serial.println("Сповіщення включені.");
    }
  }

  // Відправка команд на отримання даних і налаштувань (запит регістрів 0x97, 0x96, 0x95)
  delay(100);
  writeRegister(0x97, 0x00000000, 0x00); // Запит інфо про пристрій
  delay(100);
  writeRegister(0x96, 0x00000000, 0x00); // Запит налаштувань
  delay(100);
  writeRegister(0x95, 0x00000000, 0x00); // Запит даних

  Serial.println("Успішно підключено та ініціалізовано.");
  return true;
}


// -----------------------------------------------------------------------------
// --- 10. WEB SERVER HANDLERS -------------------------------------------------
// -----------------------------------------------------------------------------

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>Дані Jikong BMS - ESP32</title>";
  html += "<style>";
  html += "body { font-family: Arial, sans-serif; background-color: #f4f4f9; color: #333; margin: 0; padding: 0; }";
  html += ".container { max-width: 800px; margin: 20px auto; padding: 20px; background-color: #fff; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }";
  html += "h1 { color: #007bff; text-align: center; }";
  html += "h2 { border-bottom: 2px solid #ccc; padding-bottom: 5px; margin-top: 20px; color: #555; }";
  html += "table { width: 100%; border-collapse: collapse; margin-top: 10px; }";
  html += "th, td { border: 1px solid #ddd; padding: 8px; text-align: left; }";
  html += "th { background-color: #007bff; color: white; }";
  html += "tr:nth-child(even) { background-color: #f2f2f2; }";
  html += ".scan-button, .ota-button, .control-button, .disconnect-button { display: inline-block; padding: 10px 15px; margin-top: 10px; border-radius: 5px; text-decoration: none; color: white; font-weight: bold; text-align: center; }";
  html += ".scan-button { background-color: #28a745; margin-bottom: 20px; }";
  html += ".ota-button { background-color: #ffc107; margin-bottom: 20px; }";
  html += ".disconnect-button { background-color: #dc3545; margin-left: 10px; }";
  html += ".control-button { padding: 5px 10px; font-size: 0.9em; margin: 2px; }";
  html += ".toggle-on { background-color: #28a745; }";
  html += ".toggle-off { background-color: #dc3545; }";
  html += ".status-connected { color: green; font-weight: bold; }";
  html += ".status-disconnected { color: red; font-weight: bold; }";
  html += ".cell-table td { font-size: 0.9em; }";
  html += "</style></head><body><div class='container'>";
  html += "<h1>⚡️ Дані Jikong BMS (ESP32)</h1>";

  // Кнопка сканування, якщо відключено
  if (!isConnected) {
    html += "<p><em>IP-адреса: <strong>" + WiFi.localIP().toString() + "</strong> | Статус BMS: <span class='status-disconnected'>ВІДКЛЮЧЕНО</span></em></p>";
    html += "<a href='/scan' class='scan-button'>▶️ СКАНУВАТИ ТА ВИБРАТИ BMS</a>";
    html += "</div></body></html>";
    server.send(200, "text/html; charset=UTF-8", html);
    return; // Виходимо, не показуючи решту контенту
  } else {
    String connectionStatus = isConnected ? "<span class='status-connected'>ПІДКЛЮЧЕНО</span>" : "<span class='status-disconnected'>ВІДКЛЮЧЕНО</span>";
    html += "<p style='text-align: center;' ><em>IP-адреса: <strong>" + WiFi.localIP().toString() + "</strong> | Статус BMS: " + connectionStatus + "</em></p>";
    html += "<p style='text-align: center;' ><em><a href='/disconnect' class='disconnect-button'>❌ ВІДКЛЮЧИТИСЯ</a>";
  }

  // --- Кнопка OTA ---
  html += "<a href='/update' class='ota-button'>(OTA)</a></em></p>";

  // --- Загальні дані ---
  html += "<h2>📊 Загальні дані</h2>";
  html += "<table>";
  html += "<tr><th>Параметр</th><th>Значення</th><th>Од. виміру</th></tr>";

  // *** ЗБЕРЕЖЕНО ОРИГІНАЛЬНИЙ СПОСІБ ПЕРЕТВОРЕННЯ В СТРОКУ ***
  html += "<tr><td>Напруга батареї</td><td>" + String(G_data.Battery_Voltage, 2) + "</td><td>V</td></tr>";
  html += "<tr><td>Струм заряду/розряду</td><td>" + String(G_data.Charge_Current, 2) + "</td><td>A</td></tr>";
  html += "<tr><td>Потужність</td><td>" + String(G_data.Battery_Power, 2) + "</td><td>W</td></tr>";
  html += "<tr><td>Залишок заряду (SOC)</td><td>" + String(G_data.Percent_Remain) + "</td><td>%</td></tr>";
  html += "<tr><td>Залишкова ємність</td><td>" + String(G_data.Capacity_Remain, 3) + "</td><td>Ah</td></tr>";
  html += "<tr><td>Номінальна ємність</td><td>" + String(G_data.Nominal_Capacity, 3) + "</td><td>Ah</td></tr>";
  html += "<tr><td>Середня напруга комірки</td><td>" + String(G_data.Average_Cell_Voltage, 3) + "</td><td>V</td></tr>";
  html += "<tr><td>Дельта напруг комірок</td><td>" + String(G_data.Delta_Cell_Voltage, 3) + "</td><td>V</td></tr>";
  html += "<tr><td>Температура MOS</td><td>" + String(G_data.MOS_Temp, 1) + "</td><td>°C</td></tr>";
  html += "<tr><td>Температура T1</td><td>" + String(G_data.Battery_T1, 1) + "</td><td>°C</td></tr>";
  html += "<tr><td>Температура T2</td><td>" + String(G_data.Battery_T2, 1) + "</td><td>°C</td></tr>";
  html += "<tr><td>Час роботи</td><td>" + String(G_data.days) + "d " + String(G_data.hr) + "h " + String(G_data.mi) + "m " + String(G_data.sec) + "s</td><td></td></tr>";
  html += "<tr><td>Цикли/Ємність циклу</td><td>" + String(G_data.Cycle_Count) + " / " + String(G_data.Cycle_Capacity, 3) + " Ah</td><td></td></tr>";
  // --- КЕРУВАННЯ ЗАРЯДОМ ---
  html += "<tr><td>Дозволено заряд</td><td>" + String(G_data.Charge ? "🟢 УВІМК" : "🔴 ВИМК") + "</td><td>";
  if (G_data.Charge) {
    html += "<a href='/charge_off' class='control-button toggle-off off'>ВИМКНУТИ ЗАРЯД</a>";
  } else {
    html += "<a href='/charge_on' class='control-button toggle-on'>УВІМКНУТИ ЗАРЯД</a>";
  }
  html += "</td></tr>";
  // --- КЕРУВАННЯ РОЗРЯДОМ ---
  html += "<tr><td>Дозволено розряд</td><td>" + String(G_data.Discharge ? "🟢 УВІМК" : "🔴 ВИМК") + "</td><td>";
  if (G_data.Discharge) {
    html += "<a href='/discharge_off' class='control-button toggle-off off'>ВИМКНУТИ РОЗРЯД</a>";
  } else {
    html += "<a href='/discharge_on' class='control-button toggle-on discharge'>УВІМКНУТИ РОЗРЯД</a>";
  }
  html += "</td></tr>";
  // --- КЕРУВАННЯ БАЛАНСУВАННЯМ ---
  html += "<tr><td>Балансування</td><td>" + String(G_data.Balance ? "🟢 Активне" : "⚪ Неактивне") + "</td><td>";
  if (G_data.Balance) {
    html += "<a href='/balance_off' class='control-button toggle-off off'>ВИМКНУТИ БАЛАНС</a>";
  } else {
    html += "<a href='/balance_on' class='control-button toggle-on'>УВІМКНУТИ БАЛАНС</a>";
  }
  html += "</td></tr>";
  html += "<tr><td>Струм балансування</td><td>" + String(G_data.Balance_Curr, 3) + " A</td><td></td></tr>";
  html += "</table>";

  // --- Напруги комірок ---
  html += "<h2>🔬 Напруги комірок</h2>";
  html += "<table class='cell-table'>";
  html += "<tr><th>Комірка</th><th>Напруга (V)</th><th>Опір (Ом)</th></tr>";
  for (int j = 0; j < G_data.cell_count; j++) {
    if (G_data.cellVoltage[j] > 0.1) {
      html += "<tr><td>" + String(j + 1) + "</td><td>" + String(G_data.cellVoltage[j], 3) + "</td><td>" + String(G_data.wireResist[j], 3) + "</td></tr>";
    }
  }
  // Fallback для комірок, якщо count = 0
  if (G_data.cell_count == 0) {
    for (int j = 0; j < 16; j++) {
      if (G_data.cellVoltage[j] > 0.1) {
        html += "<tr><td>" + String(j + 1) + "</td><td>" + String(G_data.cellVoltage[j], 3) + "</td><td>" + String(G_data.wireResist[j], 3) + "</td></tr>";
      }
    }
  }
  html += "</table>";

  // --- Інфо про пристрій ---
  html += "<h2>⚙️ Інформація про Пристрій BMS</h2>";
  html += "<table>";
  html += "<tr><th>Параметр</th><th>Значення</th></tr>";
  html += "<tr><td>Ім'я пристрою</td><td>" + String(G_info.deviceName.c_str()) + "</td></tr>";
  html += "<tr><td>Серійний номер</td><td>" + String(G_info.serialNumber.c_str()) + "</td></tr>";
  html += "<tr><td>Версія HW/SW</td><td>" + String(G_info.hardwareVersion.c_str()) + " / " + String(G_info.softwareVersion.c_str()) + "</td></tr>";
  html += "<tr><td>Vendor ID</td><td>" + String(G_info.vendorID.c_str()) + "</td></tr>";
  html += "<tr><td>Дата виготовлення</td><td>" + String(G_info.manufacturingDate.c_str()) + "</td></tr>";
  html += "<tr><td>Час роботи (вкл)</td><td>" + String(G_info.uptime) + " сек</td></tr>";
  html += "<tr><td>Кількість увімкнень</td><td>" + String(G_info.powerOnCount) + "</td></tr>";
  html += "</table>";

  // --- Налаштування ---
  html += "<h2>⚙️ Налаштування BMS / Редагування</h2>";
  html += "<form action='/settings_update' method='post'>";
  html += "<table>";
  html += "<tr><th>Параметр</th><th>Поточне значення</th><th>Нове значення</th><th>Од. виміру</th></tr>";
  
  // Приклад поля для редагування (Загальна ємність)
  html += "<tr><td>Загальна ємність батареї</td><td>" + String(G_settings.total_battery_capacity, 3) + "</td>";
  html += "<td><input type='number' step='0.001' name='total_battery_capacity' value='" + String(G_settings.total_battery_capacity, 3) + "'></td><td>Ah</td></tr>";
  
  // Додайте інші поля налаштувань за потреби:
  html += "<tr><td>UVP Напруга комірки (V)</td><td>" + String(G_settings.cell_voltage_undervoltage_protection, 3) + "</td>";
  html += "<td><input type='number' step='0.001' name='cell_voltage_undervoltage_protection' value='" + String(G_settings.cell_voltage_undervoltage_protection, 3) + "'></td><td>V</td></tr>";

  html += "</table>";
  html += "<input type='submit' value='Зберегти Налаштування' class='scan-button'>";
  html += "</form>";
  
  html += "</div></body></html>";
  server.sendHeader("Connection", "close");
  server.send(200, "text/html; charset=UTF-8", html);
}

void handleScan() {
  // Виконуємо сканування
  bleScanTask(); 
  
  // Формуємо сторінку з результатами
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Сканування BMS</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<style>";
  html += "body { font-family: Arial, sans-serif; background-color: #f4f4f9; color: #333; margin: 0; padding: 0; }";
  html += ".container { max-width: 800px; margin: 20px auto; padding: 20px; background-color: #fff; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }";
  html += "h1 { color: #007bff; text-align: center; }";
  html += "table { width: 100%; border-collapse: collapse; margin-top: 20px; }";
  html += "th, td { border: 1px solid #ddd; padding: 8px; text-align: left; }";
  html += "th { background-color: #007bff; color: white; }";
  html += "tr:nth-child(even) { background-color: #f2f2f2; }";
  html += ".connect-button { display: inline-block; padding: 5px 10px; margin: 2px; border-radius: 5px; text-decoration: none; color: white; font-weight: bold; background-color: #28a745; }";
  html += "</style></head><body><div class='container'>";
  html += "<h1>🔍 Результати сканування BMS</h1>";
  html += "<table>";
  html += "<tr><th>Ім'я пристрою</th><th>MAC-адреса</th><th>Дія</th></tr>";

  if (foundDevices.empty()) {
    html += "<tr><td colspan='3'>Пристроїв Jikong BMS не знайдено. Спробуйте оновити сторінку.</td></tr>";
  } else {
    for (const auto& pair : foundDevices) {
      std::string address = pair.first;
      std::string name = pair.second.name;
      html += "<tr>";
      html += "<td>" + String(name.c_str()) + "</td>";
      html += "<td>" + String(address.c_str()) + "</td>";
      html += "<td><a href='/connect?mac=" + String(address.c_str()) + "' class='connect-button'>Підключитися</a></td>";
      html += "</tr>";
    }
  }

  html += "</table>";
  html += "<a href='/' style='display: block; margin-top: 20px;'>&#9664; На головну</a>";
  html += "</div></body></html>";
  server.sendHeader("Connection", "close");
  server.send(200, "text/html; charset=UTF-8", html);
}

void handleSelectDevice() {
    // Цей обробник потрібен для відображення результатів, але ми об'єднали його логіку з handleScan
    handleScan();
}

void handleConnect() {
  if (server.hasArg("mac")) {
    std::string mac_addr = server.arg("mac").c_str();

    auto it = foundDevices.find(mac_addr);
    if (it != foundDevices.end()) {
      pBmsDevice = it->second.pAdvertisedDevice;
      deviceFound = true;
      Serial.printf("Вибрано пристрій: %s\n", mac_addr.c_str());

      // Спроба підключення
      if (connectToServer(pBmsDevice->getAddress())) {
        server.sendHeader("Location", "/");
        server.send(302, "text/plain; charset=UTF-8", "Redirecting...");
        return;
      }
    }
  }
  server.send(500, "text/plain; charset=UTF-8", "Помилка: Не вдалося підключитися або пристрій не знайдено.");
}

void handleDisconnect() {
  if (pClient != nullptr && pClient->isConnected()) {
    pClient->disconnect();
  }
  // isConnected і deviceFound будуть скинуті в MyClientCallback::onDisconnect
  server.sendHeader("Location", "/");
  server.send(302, "text/plain; charset=UTF-8", "Redirecting...");
}

// --- OTA Handlers (залишені як були) ---

void handleUpdate() {
  server.send(200, "text/html",
    "<!DOCTYPE html><html><head><title>OTA Update</title><meta charset='UTF-8'>"
    "<style>body{font-family: Arial; text-align: center;} input{padding: 10px; margin: 5px; border-radius: 5px;} .file-upload{border: 1px solid #ccc; padding: 20px; width: 300px; margin: 50px auto;}</style></head>"
    "<body><h1>OTA Update</h1><p>Оберіть файл прошивки (.bin):</p>"
    "<form method='POST' action='/update' enctype='multipart/form-data' class='file-upload'>"
    "<input type='file' name='firmware'><input type='submit' value='Оновити'></form>"
    "<a href='/'>&#9664; На головну</a></body></html>"
  );
}

void handleUpdateUpload() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("Отримання файлу прошивки: %s\n", upload.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      Serial.printf("Оновлення успішне: %u байт\nПерезавантаження...\n", upload.totalSize);
      server.send(200, "text/plain", "Оновлення успішне. Перезавантаження...");
      delay(100);
      ESP.restart();
    } else {
      Update.printError(Serial);
      server.send(500, "text/plain", "Помилка оновлення.");
    }
  }
}

void handleSettingsUpdate() {
  if (!isConnected) {
    server.send(403, "text/plain; charset=UTF-8", "Error: Not connected to BMS.");
    return;
  }

  // Обробка Загальної ємності (Total Battery Capacity): Регістр 0x4D, float * 1000
  if (server.hasArg("total_battery_capacity")) {
    float val = server.arg("total_battery_capacity").toFloat();
    // Перевіряємо, чи є значна зміна перед записом
    if (std::abs(val - G_settings.total_battery_capacity) > FLOAT_TOLERANCE) {
      writeRegister(0x4D, (uint32_t)(val * 1000), 0x04);
      G_settings.total_battery_capacity = val; // Оновлюємо локальну копію
      Serial.printf("Оновлено 0x4D: %.3f V\n", val);
      delay(100);
    }
  }

  // Обробка UVP (Undervoltage Protection)
  if (server.hasArg("cell_voltage_undervoltage_protection")) {
    float val = server.arg("cell_voltage_undervoltage_protection").toFloat();
    if (std::abs(val - G_settings.cell_voltage_undervoltage_protection) > FLOAT_TOLERANCE) {
      writeRegister(0x41, (uint32_t)(val * 1000), 0x04); // Регістр 0x41
      G_settings.cell_voltage_undervoltage_protection = val;
      Serial.printf("Оновлено 0x41: %.3f V\n", val);
      delay(100);
    }
  }

  // Запит налаштувань для перевірки (0x96)
  writeRegister(0x96, 0x00000000, 0x00);
  
  delay(1500); // Даємо час BMS обробити команду
  
  // Перенаправляємо на головну сторінку
  server.sendHeader("Location", "/");
  server.send(302, "text/plain; charset=UTF-8", "Settings Updated. Redirecting...");
}


// --- Handlers для включення/виключення ---

void handleChargeOn() {
  if (isConnected) {
    writeRegister(0x1D, 0x00000001, 0x04); // Регістр 0x1D для Charge ON
  }
  delay(1500);
  server.sendHeader("Location", "/");
  server.send(302, "text/plain; charset=UTF-8", "Redirecting...");
}

void handleChargeOff() {
  if (isConnected) {
    writeRegister(0x1D, 0x00000000, 0x04); // Регістр 0x1D для Charge OFF
  }
  delay(1500);
  server.sendHeader("Location", "/");
  server.send(302, "text/plain; charset=UTF-8", "Redirecting...");
}

void handleDischargeOn() {
  if (isConnected) {
    writeRegister(0x1E, 0x00000001, 0x04); // Регістр 0x1E для Discharge ON
  }
  delay(1500);
  server.sendHeader("Location", "/");
  server.send(302, "text/plain; charset=UTF-8", "Redirecting...");
}

void handleDischargeOff() {
  if (isConnected) {
    writeRegister(0x1E, 0x00000000, 0x04); // Регістр 0x1E для Discharge OFF
  }
  delay(1500);
  server.sendHeader("Location", "/");
  server.send(302, "text/plain; charset=UTF-8", "Redirecting...");
}

void handleBalanceOn() {
  if (isConnected) {
    writeRegister(0x1F, 0x00000001, 0x04); // Регістр 0x1F для Balance ON
  }
  delay(1500);
  server.sendHeader("Location", "/");
  server.send(302, "text/plain; charset=UTF-8", "Redirecting...");
}

void handleBalanceOff() {
  if (isConnected) {
    writeRegister(0x1F, 0x00000000, 0x04); // Регістр 0x1F для Balance OFF
  }
  delay(1500);
  server.sendHeader("Location", "/");
  server.send(302, "text/plain; charset=UTF-8", "Redirecting...");
}


// -----------------------------------------------------------------------------
// --- 11. WIFI & TASK SETUP ---------------------------------------------------
// -----------------------------------------------------------------------------

void init_wifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Підключення до WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi підключено. IP: ");
  Serial.println(WiFi.localIP());
}

// Функція задачі для Веб-сервера, запущена в окремому ядрі
void webServerTask(void* parameter) {
  server.on("/", handleRoot);
  server.on("/charge_on", handleChargeOn);
  server.on("/charge_off", handleChargeOff);
  server.on("/discharge_on", handleDischargeOn);
  server.on("/discharge_off", handleDischargeOff);
  server.on("/balance_on", handleBalanceOn); 
  server.on("/balance_off", handleBalanceOff); 
  server.on("/connect", HTTP_GET, handleConnect);
  server.on("/update", HTTP_GET, handleUpdate);
  // ВИПРАВЛЕНО: Правильний синтаксис для обробки завантаження файлу (OTA)
  server.on("/update", HTTP_POST, handleUpdate, handleUpdateUpload);
  server.on("/settings_update", HTTP_POST, handleSettingsUpdate); 
  server.on("/scan", HTTP_GET, handleScan);
  server.on("/select_device", HTTP_GET, handleSelectDevice);
  server.on("/disconnect", HTTP_GET, handleDisconnect);

  server.begin();
  Serial.println("Веб-сервер запущений.");

  // БЕЗКІНЕЧНИЙ ЦИКЛ ЗАДАЧІ
  for (;;) {
    server.handleClient();
    vTaskDelay(300 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);

  init_wifi();
  
  // 1. Ініціалізація BLE
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); 
  
  // 2. Створення та прив'язка задачі для Веб-сервера (запуск на Core 1)
  xTaskCreatePinnedToCore(
    webServerTask,
    "WebServer",
    10000,    // Розмір стека
    NULL,
    10,       // Пріоритет
    NULL,
    0         // Core 1
  );
}

void loop() {
  // loop() залишається порожнім, оскільки вся логіка перенесена у задачі FreeRTOS.
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

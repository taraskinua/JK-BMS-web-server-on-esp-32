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
#include <cmath>  // Для abs() в порівнянні float

// -----------------------------------------------------------------------------
// --- 2. WIFI & WEB SERVER CONFIGURATION --------------------------------------
// -----------------------------------------------------------------------------

const char* ssid = "homewifi";                // <-- ЗАМЕНІТЬ на ваш SSID
const char* password = "homewifi1234567890";  // <-- ЗАМЕНІТЬ на ваш пароль

WebServer server(80);

// -----------------------------------------------------------------------------
// --- 3. BLE UUIDS & CONSTANTS ------------------------------------------------
// -----------------------------------------------------------------------------

static BLEUUID serviceUUID("0000ffe0-0000-1000-8000-00805f9b34fb");     // Основний сервіс
static BLEUUID charWriteUUID("0000ffe1-0000-1000-8000-00805f9b34fb");   // Характеристика для ЗАПИСУ
static BLEUUID charNotifyUUID("0000ffe1-0000-1000-8000-00805f9b34fb");  // Характеристика для СПОВІЩЕНЬ
#define CCCD_UUID ((uint16_t)0x2902)
#define FLOAT_TOLERANCE 0.001  // Допуск для порівняння float в налаштуваннях

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

static bool deviceFound = false;  // true, тільки коли вибраний конкретний прилад
static bool isConnected = false;
static bool parsedata = false;
static bool parsesettings = false;
static bool parseinfo = false;

// -----------------------------------------------------------------------------
// --- 5. DATA PROCESSING & FRAME STATE ----------------------------------------
// -----------------------------------------------------------------------------

byte receivedBytes[385];
int frame = 0;
bool received_start = false;
bool received_complete = false;

// -----------------------------------------------------------------------------
// --- 6. BMS DATA STRUCTURES --------------------------------------------------
// -----------------------------------------------------------------------------

// Структура для динамічних даних (телеметрія)
struct BMS_Data {
  float cellVoltage[32] = { 0 };
  float wireResist[32] = { 0 };
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
  int real_cell_count = 0;
} G_data;


struct BMS_Settings {
  // Cell Voltage Settings (Units: V, Data scaled by 0.001)
  float cell_voltage_undervoltage_protection = 0;  // Index 10
  float cell_voltage_undervoltage_recovery = 0;    // Index 14
  float cell_voltage_overvoltage_protection = 0;   // Index 18
  float cell_voltage_overvoltage_recovery = 0;     // Index 22
  float balance_trigger_voltage = 0;               // Index 26
  float power_off_voltage = 0;                     // Index 46
  float balance_starting_voltage = 0;              // Index 138
  float full_charge_voltage = 0;                   // Index 30 (from later code)
  float nominal_voltage = 0;                       // Index 34 (from later code)
  float float_charge_voltage = 0;                  // Index 38 (from later code)
  float balance_precision = 0;                     // Index 42 (from later code, scaled by 0.001)

  // Current Limits (Units: A, Data scaled by 0.001)
  float max_charge_current = 0;     // Index 50
  float max_discharge_current = 0;  // Index 62
  float max_balance_current = 0;    // Index 78

  // Timing/Delay Settings (Units: Unknown, Data is direct 4-byte integer)
  long charge_overcurrent_protection_delay = 0;             // Index 54 (no scaling)
  long charge_overcurrent_protection_recovery_time = 0;     // Index 58 (no scaling)
  long discharge_overcurrent_protection_delay = 0;          // Index 66 (no scaling)
  long discharge_overcurrent_protection_recovery_time = 0;  // Index 70 (no scaling)
  long short_circuit_protection_recovery_time = 0;          // Index 74 (no scaling)
  long short_circuit_protection_delay = 0;                  // Index 134 (no scaling, * 1)

  // Temperature Settings (Units: °C, Data scaled by 0.1)
  float charge_overtemperature_protection = 0;               // Index 82
  float charge_overtemperature_protection_recovery = 0;      // Index 86
  float discharge_overtemperature_protection = 0;            // Index 90
  float discharge_overtemperature_protection_recovery = 0;   // Index 94
  float charge_undertemperature_protection = 0;              // Index 98
  float charge_undertemperature_protection_recovery = 0;     // Index 102
  float power_tube_overtemperature_protection = 0;           // Index 106
  float power_tube_overtemperature_protection_recovery = 0;  // Index 110
  float fan_on_temperature = 0;                              // Index 122 (from later code, scaled by 0.1)
  float fan_off_temperature = 0;                             // Index 126 (from later code, scaled by 0.1)

  // Other Settings
  long cell_count = 0;               // Index 114 (no scaling)
  float total_battery_capacity = 0;  // Index 130 (Units: Ah, scaled by 0.001)
  long bms_mode = 0;                 // Index 118 (from later code, no scaling)
} G_settings;


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
  frame[6] = value >> 0;  // LSB
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
  G_data.real_cell_count = 0;

  // Напруги комірок
  for (int j = 0, i = 7; i < 38; j++, i += 2) {
    G_data.cellVoltage[j] = ((receivedBytes[i] << 8 | receivedBytes[i - 1]) * 0.001);
    if (G_data.cellVoltage[j] > 0.1) {
      G_data.real_cell_count += 1;
    }
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

  parsedata = true;
}

void parseBMSSettings() {
  G_settings.cell_voltage_undervoltage_protection = (((uint32_t)receivedBytes[13] << 24 | (uint32_t)receivedBytes[12] << 16 | (uint32_t)receivedBytes[11] << 8 | (uint32_t)receivedBytes[10]) * 0.001);
  G_settings.cell_voltage_undervoltage_recovery = (((uint32_t)receivedBytes[17] << 24 | (uint32_t)receivedBytes[16] << 16 | (uint32_t)receivedBytes[15] << 8 | (uint32_t)receivedBytes[14]) * 0.001);
  G_settings.cell_voltage_overvoltage_protection = (((uint32_t)receivedBytes[21] << 24 | (uint32_t)receivedBytes[20] << 16 | (uint32_t)receivedBytes[19] << 8 | (uint32_t)receivedBytes[18]) * 0.001);
  G_settings.cell_voltage_overvoltage_recovery = (((uint32_t)receivedBytes[25] << 24 | (uint32_t)receivedBytes[24] << 16 | (uint32_t)receivedBytes[23] << 8 | (uint32_t)receivedBytes[22]) * 0.001);
  G_settings.balance_trigger_voltage = (((uint32_t)receivedBytes[29] << 24 | (uint32_t)receivedBytes[28] << 16 | (uint32_t)receivedBytes[27] << 8 | (uint32_t)receivedBytes[26]) * 0.001);
  G_settings.full_charge_voltage = (((uint32_t)receivedBytes[33] << 24 | (uint32_t)receivedBytes[32] << 16 | (uint32_t)receivedBytes[31] << 8 | (uint32_t)receivedBytes[30]) * 0.001);
  G_settings.nominal_voltage = (((uint32_t)receivedBytes[37] << 24 | (uint32_t)receivedBytes[36] << 16 | (uint32_t)receivedBytes[35] << 8 | (uint32_t)receivedBytes[34]) * 0.001);
  G_settings.float_charge_voltage = (((uint32_t)receivedBytes[41] << 24 | (uint32_t)receivedBytes[40] << 16 | (uint32_t)receivedBytes[39] << 8 | (uint32_t)receivedBytes[38]) * 0.001);
  G_settings.balance_precision = (((uint32_t)receivedBytes[45] << 24 | (uint32_t)receivedBytes[44] << 16 | (uint32_t)receivedBytes[43] << 8 | (uint32_t)receivedBytes[42]) * 0.001);
  G_settings.power_off_voltage = (((uint32_t)receivedBytes[49] << 24 | (uint32_t)receivedBytes[48] << 16 | (uint32_t)receivedBytes[47] << 8 | (uint32_t)receivedBytes[46]) * 0.001);
  G_settings.max_charge_current = (((uint32_t)receivedBytes[53] << 24 | (uint32_t)receivedBytes[52] << 16 | (uint32_t)receivedBytes[51] << 8 | (uint32_t)receivedBytes[50]) * 0.001);
  G_settings.charge_overcurrent_protection_delay = (((uint32_t)receivedBytes[57] << 24 | (uint32_t)receivedBytes[56] << 16 | (uint32_t)receivedBytes[55] << 8 | (uint32_t)receivedBytes[54]));
  G_settings.charge_overcurrent_protection_recovery_time = (((uint32_t)receivedBytes[61] << 24 | (uint32_t)receivedBytes[60] << 16 | (uint32_t)receivedBytes[59] << 8 | (uint32_t)receivedBytes[58]));
  G_settings.max_discharge_current = (((uint32_t)receivedBytes[65] << 24 | (uint32_t)receivedBytes[64] << 16 | (uint32_t)receivedBytes[63] << 8 | (uint32_t)receivedBytes[62]) * 0.001);
  G_settings.discharge_overcurrent_protection_delay = (((uint32_t)receivedBytes[69] << 24 | (uint32_t)receivedBytes[68] << 16 | (uint32_t)receivedBytes[67] << 8 | (uint32_t)receivedBytes[66]));
  G_settings.discharge_overcurrent_protection_recovery_time = (((uint32_t)receivedBytes[73] << 24 | (uint32_t)receivedBytes[72] << 16 | (uint32_t)receivedBytes[71] << 8 | (uint32_t)receivedBytes[70]));
  G_settings.short_circuit_protection_recovery_time = (((uint32_t)receivedBytes[77] << 24 | (uint32_t)receivedBytes[76] << 16 | (uint32_t)receivedBytes[75] << 8 | (uint32_t)receivedBytes[74]));
  G_settings.max_balance_current = (((uint32_t)receivedBytes[81] << 24 | (uint32_t)receivedBytes[80] << 16 | (uint32_t)receivedBytes[79] << 8 | (uint32_t)receivedBytes[78]) * 0.001);
  G_settings.charge_overtemperature_protection = (((uint32_t)receivedBytes[85] << 24 | (uint32_t)receivedBytes[84] << 16 | (uint32_t)receivedBytes[83] << 8 | (uint32_t)receivedBytes[82]) * 0.1);
  G_settings.charge_overtemperature_protection_recovery = (((uint32_t)receivedBytes[89] << 24 | (uint32_t)receivedBytes[88] << 16 | (uint32_t)receivedBytes[87] << 8 | (uint32_t)receivedBytes[86]) * 0.1);
  G_settings.discharge_overtemperature_protection = (((uint32_t)receivedBytes[93] << 24 | (uint32_t)receivedBytes[92] << 16 | (uint32_t)receivedBytes[91] << 8 | (uint32_t)receivedBytes[90]) * 0.1);
  G_settings.discharge_overtemperature_protection_recovery = (((uint32_t)receivedBytes[97] << 24 | (uint32_t)receivedBytes[96] << 16 | (uint32_t)receivedBytes[95] << 8 | (uint32_t)receivedBytes[94]) * 0.1);
  G_settings.charge_undertemperature_protection = (((uint32_t)receivedBytes[101] << 24 | (uint32_t)receivedBytes[100] << 16 | (uint32_t)receivedBytes[99] << 8 | (uint32_t)receivedBytes[98]) * 0.1);
  G_settings.charge_undertemperature_protection_recovery = (((uint32_t)receivedBytes[105] << 24 | (uint32_t)receivedBytes[104] << 16 | (uint32_t)receivedBytes[103] << 8 | (uint32_t)receivedBytes[102]) * 0.1);
  G_settings.power_tube_overtemperature_protection = (((uint32_t)receivedBytes[109] << 24 | (uint32_t)receivedBytes[108] << 16 | (uint32_t)receivedBytes[107] << 8 | (uint32_t)receivedBytes[106]) * 0.1);
  G_settings.power_tube_overtemperature_protection_recovery = (((uint32_t)receivedBytes[113] << 24 | (uint32_t)receivedBytes[112] << 16 | (uint32_t)receivedBytes[111] << 8 | (uint32_t)receivedBytes[110]) * 0.1);
  G_settings.cell_count = (((uint32_t)receivedBytes[117] << 24 | (uint32_t)receivedBytes[116] << 16 | (uint32_t)receivedBytes[115] << 8 | (uint32_t)receivedBytes[114]));
  G_settings.bms_mode = (((uint32_t)receivedBytes[121] << 24 | (uint32_t)receivedBytes[120] << 16 | (uint32_t)receivedBytes[119] << 8 | (uint32_t)receivedBytes[118]));  // * 1
  G_settings.fan_on_temperature = (((uint32_t)receivedBytes[125] << 24 | (uint32_t)receivedBytes[124] << 16 | (uint32_t)receivedBytes[123] << 8 | (uint32_t)receivedBytes[122]) * 0.1);
  G_settings.fan_off_temperature = (((uint32_t)receivedBytes[129] << 24 | (uint32_t)receivedBytes[128] << 16 | (uint32_t)receivedBytes[127] << 8 | (uint32_t)receivedBytes[126]) * 0.1);
  G_settings.total_battery_capacity = (((uint32_t)receivedBytes[133] << 24 | (uint32_t)receivedBytes[132] << 16 | (uint32_t)receivedBytes[131] << 8 | (uint32_t)receivedBytes[130]) * 0.001);
  G_settings.short_circuit_protection_delay = (((uint32_t)receivedBytes[137] << 24 | (uint32_t)receivedBytes[136] << 16 | (uint32_t)receivedBytes[135] << 8 | (uint32_t)receivedBytes[134]) * 1);
  G_settings.balance_starting_voltage = (((uint32_t)receivedBytes[141] << 24 | (uint32_t)receivedBytes[140] << 16 | (uint32_t)receivedBytes[139] << 8 | (uint32_t)receivedBytes[138]) * 0.001);

  parsesettings = true;
}

void parseDeviceInfo() {
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

  parseinfo = true;  // Устанавливаем флаг завершения парсинга
}

// Функція для парсингу (розбору) даних, отримуваних по частинах
void processBMSFrame(uint8_t* pData, size_t length) {
  // Перевірка на початок кадру даних (AA 55 EB 90)
  if (pData[0] == 0x55 && pData[1] == 0xAA && pData[2] == 0xEB && pData[3] == 0x90) {

    if (received_complete && frame > 100) {
      if (!parseinfo && receivedBytes[4] == 0x03) {
        parseDeviceInfo();
      } else if (!parsesettings && receivedBytes[4] == 0x01) {
        parseBMSSettings();
      } else if (receivedBytes[4] == 0x02) {
        parseBMSData();
      }
    }
    frame = 0;
    received_complete = false;
    received_start = true;

    // Зберігаємо отримані дані
    for (int i = 0; i < length; i++) {
      receivedBytes[frame++] = pData[i];
    }
  } else if (received_start) {
    for (int i = 0; i < length; i++) {
      receivedBytes[frame++] = pData[i];
    }
    received_complete = true;
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
    pBmsDevice = nullptr;  // Скидаємо вибраний пристрій
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
  delay(500);
  writeRegister(0x97, 0x00000000, 0x00);  // Запит інфо про пристрій
  delay(500);
  writeRegister(0x96, 0x00000000, 0x00);  // Запит налаштувань
  delay(500);
  writeRegister(0x95, 0x00000000, 0x00);  // Запит даних
  delay(1000);
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
  html += ".settings-button { background-color: #007bff; margin-bottom: 20px; margin-left: 10px; }";  // НОВА КНОПКА
  html += ".control-button { padding: 5px 10px; font-size: 0.9em; margin: 2px; }";
  html += ".toggle-on { background-color: #28a745; }";
  html += ".toggle-off { background-color: #dc3545; }";
  html += ".status-connected { color: green; font-weight: bold; }";
  html += ".status-disconnected { color: red; font-weight: bold; }";
  html += ".cell-table td { font-size: 0.9em; }";
  html += ".group-header th { background-color: #4CAF50; }";  // Додано стиль для заголовків груп налаштувань
  html += ".save-button { display: block; width: 100%; padding: 10px; background-color: #007bff; color: white; border: none; border-radius: 5px; cursor: pointer; font-size: 1.1em; margin-top: 20px; }";
  html += "</style></head><body><div class='container'>";
  html += "<h1>⚡️ Дані Jikong BMS (ESP32)</h1>";

  // Кнопка сканування, якщо відключено
  if (!isConnected) {
    html += "<p style='text-align: center;'><em>IP-адреса: <strong>" + WiFi.localIP().toString() + "</strong> | Статус BMS: <span class='status-disconnected'>ВІДКЛЮЧЕНО</span></em></p>";
    html += "<p style='text-align: center;' >";  // Початок блоку кнопок
    html += "<a href='/update' class='settings-button'>(OTA)</a>";
    html += "<a href='/scan' class='settings-button'>▶️ СКАНУВАТИ ТА ВИБРАТИ BMS</a></p>";
    html += "</div></body></html>";
    server.send(200, "text/html; charset=UTF-8", html);
    return;  // Виходимо, не показуючи решту контенту
  } else {
    String connectionStatus = isConnected ? "<span class='status-connected'>ПІДКЛЮЧЕНО</span>" : "<span class='status-disconnected'>ВІДКЛЮЧЕНО</span>";
    html += "<p style='text-align: center;' ><em>IP-адреса: <strong>" + WiFi.localIP().toString() + "</strong> | Статус BMS: " + connectionStatus + "</em></p>";
    html += "<p style='text-align: center;' >";  // Початок блоку кнопок
    html += "<a href='/update' class='settings-button'>(OTA)</a>";
    html += "<a href='/settings' class='settings-button'>⚙️</a>";
    html += "<a href='/info' class='settings-button' style='background-color: #6c757d;'>ℹ️</a>";
    html += "<a href='/disconnect' class='settings-button'>❌</a></em></p>";
  }

  // --- Загальні дані ---
  html += "<h2>📊 Загальні дані</h2>";
  html += "<table>";
  html += "<tr><th>Параметр</th><th>Значення</th><th>Од. виміру</th></tr>";
  html += "<tr><td>Напруга батареї</td><td>" + String(G_data.Battery_Voltage, 2) + "</td><td>V</td></tr>";
  html += "<tr><td>Струм заряду/розряду</td><td>" + String(G_data.Charge_Current, 2) + "</td><td>A</td></tr>";
  html += "<tr><td>Потужність</td><td>" + String(G_data.Battery_Power, 2) + "</td><td>W</td></tr>";
  html += "<tr><td>Залишок заряду (SOC)</td><td>" + String(G_data.Percent_Remain) + "</td><td>%</td></tr>";
  html += "<tr><td>Дельта напруг комірок</td><td>" + String(G_data.Delta_Cell_Voltage, 3) + "</td><td>V</td></tr>";

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

  // Fallback для комірок, якщо count = 0
  if (G_data.real_cell_count == 0) {
    for (int j = 0; j < 16; j++) {
      if (G_data.cellVoltage[j] > 0.1) {
        html += "<tr><td>" + String(j + 1) + "</td><td>" + String(G_data.cellVoltage[j], 3) + "</td><td>" + String(G_data.wireResist[j], 3) + "</td></tr>";
      }
    }
  } else {
    for (int j = 0; j < G_data.real_cell_count; j++) {
      html += "<tr><td>" + String(j + 1) + "</td><td>" + String(G_data.cellVoltage[j], 3) + "</td><td>" + String(G_data.wireResist[j], 3) + "</td></tr>";
    }
  }
  html += "</table>";
  html += "</div></body></html>";

  server.sendHeader("Connection", "close");
  server.send(200, "text/html; charset=UTF-8", html);
}



void handleInfo() {
  // Начало HTML-страницы и стили
  String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>"
                "<title>Device Info</title>"
                "<style>"
                "body{font-family:Arial;background-color:#f4f4f9;color:#333;margin:20px;}"
                ".container{max-width:600px;margin:auto;background:#fff;padding:20px;border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,0.1);}"
                "h2{color:#007bff;border-bottom:2px solid #007bff;padding-bottom:10px;}"
                "table{width:100%;border-collapse:collapse;margin-top:20px;}"
                "th, td{padding:10px;text-align:left;border-bottom:1px solid #ddd;}"
                "th{background-color:#f8f9fa;color:#333;font-weight:bold;}"
                ".numeric-val{text-align:right;font-weight:bold;color:#28a745;}"
                "</style>"
                "</head><body><div class='container'><h2>&#x1F6E0; Информация об устройстве</h2>";
  // --- Отображение строковых данных ---
  html += "<table>";
  html += "<tr><th>Параметр</th><th>Значення</th></tr>";
  html += "<tr><td>Ім'я пристрою</td><td>" + String(G_info.deviceName.c_str()) + "</td></tr>";
  html += "<tr><td>Серійний номер</td><td>" + String(G_info.serialNumber.c_str()) + "</td></tr>";
  html += "<tr><td>Версія HW/SW</td><td>" + String(G_info.hardwareVersion.c_str()) + " / " + String(G_info.softwareVersion.c_str()) + "</td></tr>";
  html += "<tr><td>Vendor ID</td><td>" + String(G_info.vendorID.c_str()) + "</td></tr>";
  html += "<tr><td>Дата виготовлення</td><td>" + String(G_info.manufacturingDate.c_str()) + "</td></tr>";
  html += "<tr><td>Device Passcode</td><td>" + String(G_info.devicePasscode.c_str()) + "</td></tr>";
  html += "<tr><td>Setup Passcode</td><td>" + String(G_info.setupPasscode.c_str()) + "</td></tr>";
  html += "<tr><td>Passcode</td><td>" + String(G_info.passcode.c_str()) + "</td></tr>";
  html += "<tr><td>Дані користувача</td><td>" + String(G_info.userData.c_str()) + "</td></tr>";
  html += "</table>";
  // --- Кнопка повернення ---
  html += "<p style='text-align: center;'><a href='/' class='back-button'>◀️ НА ГОЛОВНУ</a></p>";
  // Конец таблицы и HTML-страницы
  html += "</div></body></html>";
  server.sendHeader("Connection", "close");
  server.send(200, "text/html; charset=UTF-8", html);
}

void handleSettings() {
  // Перевірка, чи підключено до BMS
  if (!isConnected) {
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "Redirecting to root...");
    return;
  }

  // 1. Сообщаем браузеру, что будем слать данные частями
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", ""); // Отправляем только заголовки

  String chunk = "";

  // --- ЧАСТЬ 1: HEAD и STYLES ---
  chunk += "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  chunk += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  chunk += "<title>Налаштування Jikong BMS</title>";
  chunk += "<style>";
  chunk += "body { font-family: Arial, sans-serif; background-color: #f4f4f9; color: #333; margin: 0; padding: 0; }";
  chunk += ".container { max-width: 800px; margin: 20px auto; padding: 20px; background-color: #fff; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }";
  chunk += "h1 { color: #007bff; text-align: center; }";
  chunk += "h2 { border-bottom: 2px solid #ccc; padding-bottom: 5px; margin-top: 20px; color: #555; }";
  chunk += "table { width: 100%; border-collapse: collapse; margin-top: 10px; }";
  chunk += "th, td { border: 1px solid #ddd; padding: 8px; text-align: left; }";
  chunk += "th { background-color: #007bff; color: white; }";
  chunk += "tr:nth-child(even) { background-color: #f2f2f2; }";
  chunk += ".group-header th { background-color: #4CAF50; }";
  chunk += "input[type='number'] { width: 90%; padding: 5px; border: 1px solid #ccc; border-radius: 4px; box-sizing: border-box; }";
  chunk += ".save-button { display: block; width: 100%; padding: 15px; background-color: #007bff; color: white; border: none; border-radius: 5px; cursor: pointer; font-size: 1.2em; margin-top: 20px; }";
  chunk += ".back-button { display: inline-block; padding: 8px 15px; background-color: #6c757d; color: white; text-decoration: none; border-radius: 5px; margin-bottom: 15px; }";
  chunk += "</style></head><body><div class='container'>";
  chunk += "<h1>⚙️ Налаштування Jikong BMS</h1>";
  chunk += "<a href='/' class='back-button'>⬅️ На головну</a>";
  
  server.sendContent(chunk); // ОТПРАВКА ЧАСТИ 1
  chunk = "";

  // --- ЧАСТЬ 2: НАЧАЛО ФОРМЫ ---
  chunk += "<h2>⚙️ Параметри BMS</h2>";
  chunk += "<form method='POST' action='/settings_update' class='settings-form'>";
  chunk += "<table>";
  chunk += "<tr><th>Параметр</th><th>Поточне значення</th><th>Нове значення</th></tr>";
  chunk += "<tr class='group-header'><th colspan='3'>Основні налаштування</th></tr>";
  // total_battery_capacity
  chunk += "<tr><td>Номінальна ємність (Ah)</td><td>" + String(G_settings.total_battery_capacity, 3) + "</td><td><input type='number' name='total_battery_capacity' value='" + String(G_settings.total_battery_capacity, 3) + "' step='0.001' min='0.001' max='5000' required></td></tr>";
  // cell_count
  chunk += "<tr><td>Кількість комірок</td><td>" + String(G_settings.cell_count) + "</td><td><input type='number' name='cell_count' value='" + String(G_settings.cell_count) + "' step='1' min='1' max='32' required></td></tr>";
  // bms_mode
  chunk += "<tr><td>Режим BMS</td><td>" + String(G_settings.bms_mode) + "</td><td><input type='number' name='bms_mode' value='" + String(G_settings.bms_mode) + "' step='1' min='0' max='255' required></td></tr>";

  server.sendContent(chunk); // ОТПРАВКА ЧАСТИ 2
  chunk = "";

  // --- ЧАСТЬ 3: НАПРЯЖЕНИЯ ---
  chunk += "<tr class='group-header'><th colspan='3'>Напруги комірок (V)</th></tr>";
  chunk += "<tr><td>Захист від перенапруги (ОВР)</td><td>" + String(G_settings.cell_voltage_overvoltage_protection, 3) + "</td><td><input type='number' name='cell_voltage_overvoltage_protection' value='" + String(G_settings.cell_voltage_overvoltage_protection, 3) + "' step='0.001' min='0' max='5' required></td></tr>";
  chunk += "<tr><td>Відновлення ОВР</td><td>" + String(G_settings.cell_voltage_overvoltage_recovery, 3) + "</td><td><input type='number' name='cell_voltage_overvoltage_recovery' value='" + String(G_settings.cell_voltage_overvoltage_recovery, 3) + "' step='0.001' min='0' max='5' required></td></tr>";
  chunk += "<tr><td>Захист від низької напруги (УВР)</td><td>" + String(G_settings.cell_voltage_undervoltage_protection, 3) + "</td><td><input type='number' name='cell_voltage_undervoltage_protection' value='" + String(G_settings.cell_voltage_undervoltage_protection, 3) + "' step='0.001' min='0' max='5' required></td></tr>";
  chunk += "<tr><td>Відновлення УВР</td><td>" + String(G_settings.cell_voltage_undervoltage_recovery, 3) + "</td><td><input type='number' name='cell_voltage_undervoltage_recovery' value='" + String(G_settings.cell_voltage_undervoltage_recovery, 3) + "' step='0.001' min='0' max='5' required></td></tr>";
  chunk += "<tr><td>Напруга вимкнення</td><td>" + String(G_settings.power_off_voltage, 3) + "</td><td><input type='number' name='power_off_voltage' value='" + String(G_settings.power_off_voltage, 3) + "' step='0.001' min='0' max='5' required></td></tr>";

  server.sendContent(chunk); // ОТПРАВКА ЧАСТИ 3
  chunk = "";

  // --- ЧАСТЬ 4: ЗАРЯДНЫЕ НАПРЯЖЕНИЯ ---
  chunk += "<tr class='group-header'><th colspan='3'>Зарядні / Номінальні напруги (V)</th></tr>";
  chunk += "<tr><td>Повна напруга заряду</td><td>" + String(G_settings.full_charge_voltage, 3) + "</td><td><input type='number' name='full_charge_voltage' value='" + String(G_settings.full_charge_voltage, 3) + "' step='0.001' min='0' max='5' required></td></tr>";
  chunk += "<tr><td>Номінальна напруга</td><td>" + String(G_settings.nominal_voltage, 3) + "</td><td><input type='number' name='nominal_voltage' value='" + String(G_settings.nominal_voltage, 3) + "' step='0.001' min='0' max='5' required></td></tr>";
  chunk += "<tr><td>Підтримуюча напруга заряду</td><td>" + String(G_settings.float_charge_voltage, 3) + "</td><td><input type='number' name='float_charge_voltage' value='" + String(G_settings.float_charge_voltage, 3) + "' step='0.001' min='0' max='5' required></td></tr>";
  
  server.sendContent(chunk); // ОТПРАВКА ЧАСТИ 4
  chunk = "";

  // --- ЧАСТЬ 5: БАЛАНСИРОВКА И ТОКИ ---
  chunk += "<tr class='group-header'><th colspan='3'>Балансування та Струми</th></tr>";
  chunk += "<tr><td>Напруга запуску балансування (V)</td><td>" + String(G_settings.balance_trigger_voltage, 3) + "</td><td><input type='number' name='balance_trigger_voltage' value='" + String(G_settings.balance_trigger_voltage, 3) + "' step='0.001' min='0' max='5' required></td></tr>";
  chunk += "<tr><td>Мінімальна напруга запуску (V)</td><td>" + String(G_settings.balance_starting_voltage, 3) + "</td><td><input type='number' name='balance_starting_voltage' value='" + String(G_settings.balance_starting_voltage, 3) + "' step='0.001' min='0' max='5' required></td></tr>";
  chunk += "<tr><td>Макс. струм балансування (A)</td><td>" + String(G_settings.max_balance_current, 3) + "</td><td><input type='number' name='max_balance_current' value='" + String(G_settings.max_balance_current, 3) + "' step='0.001' min='0' max='1' required></td></tr>";
  chunk += "<tr><td>Напруга зупинки балансування (V)</td><td>" + String(G_settings.balance_precision, 3) + "</td><td><input type='number' name='balance_precision' value='" + String(G_settings.balance_precision, 3) + "' step='0.001' min='0' max='5' required></td></tr>";
  chunk += "<tr><td>Макс. струм заряду (A)</td><td>" + String(G_settings.max_charge_current) + "</td><td><input type='number' name='max_charge_current' value='" + String(G_settings.max_charge_current) + "' step='1' min='0' max='500' required></td></tr>";
  chunk += "<tr><td>Макс. струм розряду (A)</td><td>" + String(G_settings.max_discharge_current) + "</td><td><input type='number' name='max_discharge_current' value='" + String(G_settings.max_discharge_current) + "' step='1' min='0' max='500' required></td></tr>";

  server.sendContent(chunk); // ОТПРАВКА ЧАСТИ 5
  chunk = "";

  // --- ЧАСТЬ 6: ЗАДЕРЖКИ ---
  chunk += "<tr class='group-header'><th colspan='3'>Затримки захисту (мс)</th></tr>";
  chunk += "<tr><td>Затримка надструму заряду</td><td>" + String(G_settings.charge_overcurrent_protection_delay) + "</td><td><input type='number' name='charge_overcurrent_protection_delay' value='" + String(G_settings.charge_overcurrent_protection_delay) + "' step='1' min='0' max='65535' required></td></tr>";
  chunk += "<tr><td>Затримка надструму розряду</td><td>" + String(G_settings.discharge_overcurrent_protection_delay) + "</td><td><input type='number' name='discharge_overcurrent_protection_delay' value='" + String(G_settings.discharge_overcurrent_protection_delay) + "' step='1' min='0' max='65535' required></td></tr>";
  chunk += "<tr><td>Затримка захисту від КЗ</td><td>" + String(G_settings.short_circuit_protection_delay) + "</td><td><input type='number' name='short_circuit_protection_delay' value='" + String(G_settings.short_circuit_protection_delay) + "' step='1' min='0' max='65535' required></td></tr>";

  server.sendContent(chunk); // ОТПРАВКА ЧАСТИ 6
  chunk = "";

  // --- ЧАСТЬ 7: ТЕМПЕРАТУРЫ ---
  chunk += "<tr class='group-header'><th colspan='3'>Температурний захист (°C)</th></tr>";
  chunk += "<tr><td>Захист від перегріву (Заряд)</td><td>" + String(G_settings.charge_overtemperature_protection, 1) + "</td><td><input type='number' name='charge_overtemperature_protection' value='" + String(G_settings.charge_overtemperature_protection, 1) + "' step='0.1' min='0' max='150' required></td></tr>";
  chunk += "<tr><td>Захист від перегріву (Розряд)</td><td>" + String(G_settings.discharge_overtemperature_protection, 1) + "</td><td><input type='number' name='discharge_overtemperature_protection' value='" + String(G_settings.discharge_overtemperature_protection, 1) + "' step='0.1' min='0' max='150' required></td></tr>";
  chunk += "<tr><td>MOSFET перегрів</td><td>" + String(G_settings.power_tube_overtemperature_protection, 1) + "</td><td><input type='number' name='power_tube_overtemperature_protection' value='" + String(G_settings.power_tube_overtemperature_protection, 1) + "' step='0.1' min='0' max='150' required></td></tr>";

  // --- ПОДВАЛ ---
  chunk += "</table>";
  chunk += "<button type='submit' class='save-button'>💾 ЗБЕРЕГТИ ВСІ НАЛАШТУВАННЯ</button>";
  chunk += "</form></div></body></html>";

  server.sendContent(chunk); // ОТПРАВКА ФИНАЛЬНОЙ ЧАСТИ
  server.sendContent("");    // ЗАВЕРШЕНИЕ ПЕРЕДАЧИ
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
              "<a href='/'>&#9664; На головну</a></body></html>");
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

  // Всі налаштування записуються як 4-байтні (0x04) значення.
  // Адреси регістрів (Index / 2) -> (Reg Address)

  // --- VOLTAGE SETTINGS (Scale: 0.001 -> Multiplier: 1000) ---
  // Регістри 0x0A - 0x14 (Індекси 10, 14, 18, 22, 26, 30, 34, 38, 42, 46)
  if (server.hasArg("cell_voltage_undervoltage_protection")) {
    float val = server.arg("cell_voltage_undervoltage_protection").toFloat();
    if (std::abs(val - G_settings.cell_voltage_undervoltage_protection) > FLOAT_TOLERANCE) {
      writeRegister(0x0A, (uint32_t)(val * 1000), 0x04); // Index 10 -> 0x0A
      G_settings.cell_voltage_undervoltage_protection = val;
    }
  }
  if (server.hasArg("cell_voltage_undervoltage_recovery")) {
    float val = server.arg("cell_voltage_undervoltage_recovery").toFloat();
    if (std::abs(val - G_settings.cell_voltage_undervoltage_recovery) > FLOAT_TOLERANCE) {
      writeRegister(0x0B, (uint32_t)(val * 1000), 0x04); // Index 14 -> 0x0B
      G_settings.cell_voltage_undervoltage_recovery = val;
    }
  }
  if (server.hasArg("cell_voltage_overvoltage_protection")) {
    float val = server.arg("cell_voltage_overvoltage_protection").toFloat();
    if (std::abs(val - G_settings.cell_voltage_overvoltage_protection) > FLOAT_TOLERANCE) {
      writeRegister(0x0C, (uint32_t)(val * 1000), 0x04); // Index 18 -> 0x0C
      G_settings.cell_voltage_overvoltage_protection = val;
    }
  }
  if (server.hasArg("cell_voltage_overvoltage_recovery")) {
    float val = server.arg("cell_voltage_overvoltage_recovery").toFloat();
    if (std::abs(val - G_settings.cell_voltage_overvoltage_recovery) > FLOAT_TOLERANCE) {
      writeRegister(0x0D, (uint32_t)(val * 1000), 0x04); // Index 22 -> 0x0D
      G_settings.cell_voltage_overvoltage_recovery = val;
    }
  }
  if (server.hasArg("balance_trigger_voltage")) {
    float val = server.arg("balance_trigger_voltage").toFloat();
    if (std::abs(val - G_settings.balance_trigger_voltage) > FLOAT_TOLERANCE) {
      writeRegister(0x0E, (uint32_t)(val * 1000), 0x04); // Index 26 -> 0x0E
      G_settings.balance_trigger_voltage = val;
    }
  }
  if (server.hasArg("full_charge_voltage")) {
    float val = server.arg("full_charge_voltage").toFloat();
    if (std::abs(val - G_settings.full_charge_voltage) > FLOAT_TOLERANCE) {
      writeRegister(0x0F, (uint32_t)(val * 1000), 0x04); // Index 30 -> 0x0F
      G_settings.full_charge_voltage = val;
    }
  }
  if (server.hasArg("nominal_voltage")) {
    float val = server.arg("nominal_voltage").toFloat();
    if (std::abs(val - G_settings.nominal_voltage) > FLOAT_TOLERANCE) {
      writeRegister(0x10, (uint32_t)(val * 1000), 0x04); // Index 34 -> 0x10
      G_settings.nominal_voltage = val;
    }
  }
  if (server.hasArg("float_charge_voltage")) {
    float val = server.arg("float_charge_voltage").toFloat();
    if (std::abs(val - G_settings.float_charge_voltage) > FLOAT_TOLERANCE) {
      writeRegister(0x11, (uint32_t)(val * 1000), 0x04); // Index 38 -> 0x11
      G_settings.float_charge_voltage = val;
    }
  }
  if (server.hasArg("balance_precision")) {
    float val = server.arg("balance_precision").toFloat();
    if (std::abs(val - G_settings.balance_precision) > FLOAT_TOLERANCE) {
      writeRegister(0x12, (uint32_t)(val * 1000), 0x04); // Index 42 -> 0x12
      G_settings.balance_precision = val;
    }
  }
  if (server.hasArg("power_off_voltage")) {
    float val = server.arg("power_off_voltage").toFloat();
    if (std::abs(val - G_settings.power_off_voltage) > FLOAT_TOLERANCE) {
      writeRegister(0x14, (uint32_t)(val * 1000), 0x04); // Index 46 -> 0x14
      G_settings.power_off_voltage = val;
    }
  }
  if (server.hasArg("balance_starting_voltage")) {
    float val = server.arg("balance_starting_voltage").toFloat();
    if (std::abs(val - G_settings.balance_starting_voltage) > FLOAT_TOLERANCE) {
      writeRegister(0x45, (uint32_t)(val * 1000), 0x04); // Index 138 -> 0x45
      G_settings.balance_starting_voltage = val;
    }
  }

  // --- CURRENT & CAPACITY SETTINGS (Scale: 0.001 -> Multiplier: 1000) ---
  // Регістри 0x15, 0x18, 0x1C (Індекси 50, 62, 78) та 0x4D (Індекс 130)
  if (server.hasArg("max_charge_current")) {
    float val = server.arg("max_charge_current").toFloat();
    if (std::abs(val - G_settings.max_charge_current) > FLOAT_TOLERANCE) {
      writeRegister(0x15, (uint32_t)(val * 1000), 0x04); // Index 50 -> 0x15
      G_settings.max_charge_current = val;
    }
  }
  if (server.hasArg("max_discharge_current")) {
    float val = server.arg("max_discharge_current").toFloat();
    if (std::abs(val - G_settings.max_discharge_current) > FLOAT_TOLERANCE) {
      writeRegister(0x18, (uint32_t)(val * 1000), 0x04); // Index 62 -> 0x18
      G_settings.max_discharge_current = val;
    }
  }
  if (server.hasArg("max_balance_current")) {
    float val = server.arg("max_balance_current").toFloat();
    if (std::abs(val - G_settings.max_balance_current) > FLOAT_TOLERANCE) {
      writeRegister(0x1C, (uint32_t)(val * 1000), 0x04); // Index 78 -> 0x1C
      G_settings.max_balance_current = val;
    }
  }
  // Total Battery Capacity
  if (server.hasArg("total_battery_capacity")) {
    float val = server.arg("total_battery_capacity").toFloat();
    if (std::abs(val - G_settings.total_battery_capacity) > FLOAT_TOLERANCE) {
      writeRegister(0x41, (uint32_t)(val * 1000), 0x04); // Index 130 -> 0x41 (Виправлено: 130/2 = 65 = 0x41. У вашому коді було 0x4D, але згідно індексу 130 має бути 0x41)
      G_settings.total_battery_capacity = val;
    }
  }


  // --- TIME/DELAY SETTINGS (Scale: 1 -> Multiplier: 1, long type) ---
  // Регістри 0x16, 0x17, 0x19, 0x1A, 0x1B, 0x43 (Індекси 54, 58, 66, 70, 74, 134)
  if (server.hasArg("charge_overcurrent_protection_delay")) {
    long val = server.arg("charge_overcurrent_protection_delay").toInt();
    if (val != G_settings.charge_overcurrent_protection_delay) {
      writeRegister(0x16, (uint32_t)val, 0x04); // Index 54 -> 0x16
      G_settings.charge_overcurrent_protection_delay = val;
    }
  }
  if (server.hasArg("charge_overcurrent_protection_recovery_time")) {
    long val = server.arg("charge_overcurrent_protection_recovery_time").toInt();
    if (val != G_settings.charge_overcurrent_protection_recovery_time) {
      writeRegister(0x17, (uint32_t)val, 0x04); // Index 58 -> 0x17
      G_settings.charge_overcurrent_protection_recovery_time = val;
    }
  }
  if (server.hasArg("discharge_overcurrent_protection_delay")) {
    long val = server.arg("discharge_overcurrent_protection_delay").toInt();
    if (val != G_settings.discharge_overcurrent_protection_delay) {
      writeRegister(0x19, (uint32_t)val, 0x04); // Index 66 -> 0x19
      G_settings.discharge_overcurrent_protection_delay = val;
    }
  }
  if (server.hasArg("discharge_overcurrent_protection_recovery_time")) {
    long val = server.arg("discharge_overcurrent_protection_recovery_time").toInt();
    if (val != G_settings.discharge_overcurrent_protection_recovery_time) {
      writeRegister(0x1A, (uint32_t)val, 0x04); // Index 70 -> 0x1A
      G_settings.discharge_overcurrent_protection_recovery_time = val;
    }
  }
  if (server.hasArg("short_circuit_protection_recovery_time")) {
    long val = server.arg("short_circuit_protection_recovery_time").toInt();
    if (val != G_settings.short_circuit_protection_recovery_time) {
      writeRegister(0x1B, (uint32_t)val, 0x04); // Index 74 -> 0x1B
      G_settings.short_circuit_protection_recovery_time = val;
    }
  }
  if (server.hasArg("short_circuit_protection_delay")) {
    long val = server.arg("short_circuit_protection_delay").toInt();
    if (val != G_settings.short_circuit_protection_delay) {
      writeRegister(0x43, (uint32_t)val, 0x04); // Index 134 -> 0x43
      G_settings.short_circuit_protection_delay = val;
    }
  }

  // --- TEMPERATURE SETTINGS (Scale: 0.1 -> Multiplier: 10) ---
  // Регістри 0x1D - 0x22, 0x25, 0x26, 0x3E, 0x3F (Індекси 82, 86, 90, 94, 98, 102, 106, 110, 122, 126)
  if (server.hasArg("charge_overtemperature_protection")) {
    float val = server.arg("charge_overtemperature_protection").toFloat();
    if (std::abs(val - G_settings.charge_overtemperature_protection) > FLOAT_TOLERANCE) {
      writeRegister(0x1D, (uint32_t)(val * 10), 0x04); // Index 82 -> 0x1D
      G_settings.charge_overtemperature_protection = val;
    }
  }
  if (server.hasArg("charge_overtemperature_protection_recovery")) {
    float val = server.arg("charge_overtemperature_protection_recovery").toFloat();
    if (std::abs(val - G_settings.charge_overtemperature_protection_recovery) > FLOAT_TOLERANCE) {
      writeRegister(0x1E, (uint32_t)(val * 10), 0x04); // Index 86 -> 0x1E
      G_settings.charge_overtemperature_protection_recovery = val;
    }
  }
  if (server.hasArg("discharge_overtemperature_protection")) {
    float val = server.arg("discharge_overtemperature_protection").toFloat();
    if (std::abs(val - G_settings.discharge_overtemperature_protection) > FLOAT_TOLERANCE) {
      writeRegister(0x1F, (uint32_t)(val * 10), 0x04); // Index 90 -> 0x1F
      G_settings.discharge_overtemperature_protection = val;
    }
  }
  if (server.hasArg("discharge_overtemperature_protection_recovery")) {
    float val = server.arg("discharge_overtemperature_protection_recovery").toFloat();
    if (std::abs(val - G_settings.discharge_overtemperature_protection_recovery) > FLOAT_TOLERANCE) {
      writeRegister(0x20, (uint32_t)(val * 10), 0x04); // Index 94 -> 0x20
      G_settings.discharge_overtemperature_protection_recovery = val;
    }
  }
  if (server.hasArg("charge_undertemperature_protection")) {
    float val = server.arg("charge_undertemperature_protection").toFloat();
    if (std::abs(val - G_settings.charge_undertemperature_protection) > FLOAT_TOLERANCE) {
      writeRegister(0x21, (uint32_t)(val * 10), 0x04); // Index 98 -> 0x21
      G_settings.charge_undertemperature_protection = val;
    }
  }
  if (server.hasArg("charge_undertemperature_protection_recovery")) {
    float val = server.arg("charge_undertemperature_protection_recovery").toFloat();
    if (std::abs(val - G_settings.charge_undertemperature_protection_recovery) > FLOAT_TOLERANCE) {
      writeRegister(0x22, (uint32_t)(val * 10), 0x04); // Index 102 -> 0x22
      G_settings.charge_undertemperature_protection_recovery = val;
    }
  }
  if (server.hasArg("power_tube_overtemperature_protection")) {
    float val = server.arg("power_tube_overtemperature_protection").toFloat();
    if (std::abs(val - G_settings.power_tube_overtemperature_protection) > FLOAT_TOLERANCE) {
      writeRegister(0x25, (uint32_t)(val * 10), 0x04); // Index 106 -> 0x25
      G_settings.power_tube_overtemperature_protection = val;
    }
  }
  if (server.hasArg("power_tube_overtemperature_protection_recovery")) {
    float val = server.arg("power_tube_overtemperature_protection_recovery").toFloat();
    if (std::abs(val - G_settings.power_tube_overtemperature_protection_recovery) > FLOAT_TOLERANCE) {
      writeRegister(0x26, (uint32_t)(val * 10), 0x04); // Index 110 -> 0x26
      G_settings.power_tube_overtemperature_protection_recovery = val;
    }
  }
  if (server.hasArg("fan_on_temperature")) {
    float val = server.arg("fan_on_temperature").toFloat();
    if (std::abs(val - G_settings.fan_on_temperature) > FLOAT_TOLERANCE) {
      writeRegister(0x3E, (uint32_t)(val * 10), 0x04); // Index 122 -> 0x3E
      G_settings.fan_on_temperature = val;
    }
  }
  if (server.hasArg("fan_off_temperature")) {
    float val = server.arg("fan_off_temperature").toFloat();
    if (std::abs(val - G_settings.fan_off_temperature) > FLOAT_TOLERANCE) {
      writeRegister(0x3F, (uint32_t)(val * 10), 0x04); // Index 126 -> 0x3F
      G_settings.fan_off_temperature = val;
    }
  }

  // --- OTHER SETTINGS (Scale: 1 -> Multiplier: 1, long type) ---
  // Регістри 0x27, 0x28 (Індекси 114, 118)
  if (server.hasArg("cell_count")) {
    long val = server.arg("cell_count").toInt();
    if (val != G_settings.cell_count) {
      writeRegister(0x27, (uint32_t)val, 0x04); // Index 114 -> 0x27
      G_settings.cell_count = val;
    }
  }
  if (server.hasArg("bms_mode")) {
    long val = server.arg("bms_mode").toInt();
    if (val != G_settings.bms_mode) {
      writeRegister(0x28, (uint32_t)val, 0x04); // Index 118 -> 0x28
      G_settings.bms_mode = val;
    }
  }


  Serial.println("Оновлення налаштувань BMS завершено.");
  // Запит налаштувань для перевірки (0x96)
  writeRegister(0x96, 0x00000000, 0x00);

  delay(500);

  // Перенаправляємо на головну сторінку
  server.sendHeader("Location", "/");
  server.send(302, "text/plain; charset=UTF-8", "Settings Updated. Redirecting...");
}


// --- Handlers для включення/виключення ---

void handleChargeOn() {
  if (!isConnected) {
    server.send(403, "text/plain; charset=UTF-8", "Error: Not connected to BMS.");
    return;
  }
  writeRegister(0x1D, 0x00000001, 0x04); 
  delay(1500);
  server.sendHeader("Location", "/");
  server.send(302, "text/plain; charset=UTF-8", "Redirecting...");
}

void handleChargeOff() {
  if (!isConnected) {
    server.send(403, "text/plain; charset=UTF-8", "Error: Not connected to BMS.");
    return;
  }
  writeRegister(0x1D, 0x00000000, 0x04); 

  delay(1500);
  server.sendHeader("Location", "/");
  server.send(302, "text/plain; charset=UTF-8", "Redirecting...");
}

void handleDischargeOn() {
  if (!isConnected) {
    server.send(403, "text/plain; charset=UTF-8", "Error: Not connected to BMS.");
    return;
  }

  writeRegister(0x1E, 0x00000001, 0x04);  

  delay(1500);
  server.sendHeader("Location", "/");
  server.send(302, "text/plain; charset=UTF-8", "Redirecting...");
}

void handleDischargeOff() {
  if (!isConnected) {
    server.send(403, "text/plain; charset=UTF-8", "Error: Not connected to BMS.");
    return;
  }

  writeRegister(0x1E, 0x00000000, 0x04);

  delay(1500);
  server.sendHeader("Location", "/");
  server.send(302, "text/plain; charset=UTF-8", "Redirecting...");
}

void handleBalanceOn() {
  if (!isConnected) {
    server.send(403, "text/plain; charset=UTF-8", "Error: Not connected to BMS.");
    return;
  }

  writeRegister(0x1F, 0x00000001, 0x04);

  delay(1500);
  server.sendHeader("Location", "/");
  server.send(302, "text/plain; charset=UTF-8", "Redirecting...");
}

void handleBalanceOff() {
  if (!isConnected) {
    server.send(403, "text/plain; charset=UTF-8", "Error: Not connected to BMS.");
    return;
  }

  writeRegister(0x1F, 0x00000000, 0x04);

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

void setup() {
  Serial.begin(115200);

  init_wifi();

  server.on("/", handleRoot);
  server.on("/info", handleInfo);  // ДОДАТИ ЦЕЙ РЯДОК
  server.on("/settings", handleSettings);
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

  // 1. Ініціалізація BLE
  BLEDevice::init("JKBMS service");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);

  bleScanTask();
}

void loop() {
  // loop() залишається порожнім, оскільки вся логіка перенесена у задачі FreeRTOS.
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  server.handleClient();
}

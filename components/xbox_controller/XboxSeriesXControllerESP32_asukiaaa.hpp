#pragma once

#include <NimBLEDevice.h>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <XboxControllerNotificationParser.h>

#include <XboxSeriesXHIDReportBuilder_asukiaaa.hpp>

// #define XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL Serial
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
const unsigned long printInterval = 100UL;
#endif

namespace XboxSeriesXControllerESP32_asukiaaa {

static NimBLEUUID uuidServiceGeneral("1801");
static NimBLEUUID uuidServiceBattery("180f");
static NimBLEUUID uuidServiceHid("1812");
static NimBLEUUID uuidCharaReport("2a4d");
static NimBLEUUID uuidCharaPnp("2a50");
static NimBLEUUID uuidCharaHidInformation("2a4a");
static NimBLEUUID uuidCharaPeripheralAppearance("2a01");
static NimBLEUUID uuidCharaPeripheralControlParameters("2a04");

static std::atomic<const NimBLEAdvertisedDevice*> advDevice{nullptr};
static std::atomic<NimBLEClient*> pConnectedClient{nullptr};

static const uint16_t controllerAppearance = 964;
static const String controllerManufacturerDataNormal = "060000";
static const String controllerManufacturerDataSearching = "0600030080";

enum class ConnectionState : uint8_t {
  Connected = 0,
  WaitingForFirstNotification = 1,
  Found = 2,
  Scanning = 3,
};

// No BLE calls, logging, allocation, or blocking work while this guard is held.
class StateGuard {
 public:
  explicit StateGuard(portMUX_TYPE& mux) : mux(mux) { portENTER_CRITICAL(&mux); }
  ~StateGuard() { portEXIT_CRITICAL(&mux); }
  StateGuard(const StateGuard&) = delete;
  StateGuard& operator=(const StateGuard&) = delete;
 private:
  portMUX_TYPE& mux;
};

class Core;
class ClientCallbacks : public NimBLEClientCallbacks {
 public:
  explicit ClientCallbacks(Core* owner) : owner(owner) {}
  void onConnect(NimBLEClient* pClient) override;
  void onDisconnect(NimBLEClient* pClient, int reason) override;

 private:
  Core* owner;
 public:
  void onPassKeyEntry(NimBLEConnInfo& connInfo) override {
    NimBLEDevice::injectPassKey(connInfo, 0);
  }

  void onConfirmPasskey(NimBLEConnInfo& connInfo, uint32_t pass_key) override {
    // Serial.print("The passkey YES/NO number: ");
    // Serial.println(pass_key);
    NimBLEDevice::injectConfirmPasskey(connInfo, true);
  }

  void onAuthenticationComplete(NimBLEConnInfo& connInfo) override {
    if (!connInfo.isEncrypted()) {
      // Serial.printf("Encrypt connection failed - disconnecting\n");
      /** Find the client with the connection handle provided in connInfo */
      NimBLEDevice::getClientByHandle(connInfo.getConnHandle())->disconnect();
      return;
    }
  }
};

/** Define a class to handle the callbacks when advertisments are received */
class ScanCallbacks : public NimBLEScanCallbacks {
 public:
  ScanCallbacks(String strTargetDeviceAddress,
                std::atomic<ConnectionState>* pConnectionState) {
    if (strTargetDeviceAddress != "") {
      this->targetDeviceAddress =
          new NimBLEAddress(strTargetDeviceAddress.c_str(), 0);
    }
    this->pConnectionState = pConnectionState;
  }

  bool ignoreTargetAddressToConnectToUnkownSearchingDevice = false;

  void setTargetAddress(ble_addr_t address) {
    if (targetDeviceAddress != nullptr) {
      *targetDeviceAddress = address;
    } else {
      targetDeviceAddress = new NimBLEAddress(address);
    }
  }

  void clearTargetAddress() {
    if (targetDeviceAddress != nullptr) {
      delete targetDeviceAddress;
    }
  }

 private:
  NimBLEAddress* targetDeviceAddress = nullptr;
  std::atomic<ConnectionState>* pConnectionState;
  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
    XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.print("Advertised Device found: ");
    XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println(
        advertisedDevice->toString().c_str());
    XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.printf(
        "name:%s, address:%s\n", advertisedDevice->getName().c_str(),
        advertisedDevice->getAddress().toString().c_str());
    XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.printf(
        "uuidService:%s\n",
        advertisedDevice->haveServiceUUID()
            ? advertisedDevice->getServiceUUID().toString().c_str()
            : "none");
#endif
    auto pHex = NimBLEUtils::dataToHexString(
        (uint8_t*)advertisedDevice->getManufacturerData().data(),
        advertisedDevice->getManufacturerData().length());
    if ((targetDeviceAddress != nullptr &&
         advertisedDevice->getAddress().equals(*targetDeviceAddress)) ||
        (targetDeviceAddress != nullptr &&
         ignoreTargetAddressToConnectToUnkownSearchingDevice &&
         strcmp(pHex.c_str(), controllerManufacturerDataSearching.c_str()) ==
             0) ||
        (targetDeviceAddress == nullptr &&
         advertisedDevice->getAppearance() == controllerAppearance &&
         (strcmp(pHex.c_str(), controllerManufacturerDataNormal.c_str()) == 0 ||
          strcmp(pHex.c_str(), controllerManufacturerDataSearching.c_str()) ==
              0) &&
         advertisedDevice->getServiceUUID().equals(uuidServiceHid)))
    // if (advertisedDevice->isAdvertisingService(uuidServiceHid))
    {
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
      XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println("Found target");
#endif
      /** stop scan before connecting */
      // NimBLEDevice::getScan()->stop();
      /** Save the device reference in a global for the client to use*/
      ConnectionState expected = ConnectionState::Scanning;
      if (pConnectionState->compare_exchange_strong(expected, ConnectionState::Found)) {
        advDevice = advertisedDevice;
      }
    }
  }

  void onScanEnd(const NimBLEScanResults& scanResults, int reason) override {
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
    XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println("Scan Ended");
#endif
  }
};

class Core {
 public:
  Core(String targetDeviceAddress = "") {
    this->scanCBs = new ScanCallbacks(targetDeviceAddress, &connectionState);
    this->clientCBs = new ClientCallbacks(this);
  }

  ScanCallbacks* scanCBs;
  ClientCallbacks* clientCBs;
  uint8_t battery = 0;
  static const int deviceAddressLen = 6;
  uint8_t deviceAddressArr[deviceAddressLen];

  void setIgnoreTargetAddressToConnectToUnkownSearchingDevice(bool allow) {
    scanCBs->ignoreTargetAddressToConnectToUnkownSearchingDevice = allow;
  }

  void setTargetAddress(ble_addr_t address) {
    scanCBs->setTargetAddress(address);
  }

  void clearTargetAddress() { scanCBs->clearTargetAddress(); }

  bool begin() {
    initialized = NimBLEDevice::init("");
    if (!initialized) return false;
    NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_PUBLIC);
    NimBLEDevice::setSecurityAuth(true, false, false);
    NimBLEDevice::setPower(ESP_PWR_LVL_P9); /* +9db */
    return true;
  }

  void writeHIDReport(uint8_t* dataArr, size_t dataLen) {
    if (pConnectedClient == nullptr) {
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
      XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println("no connnected client");
#endif
      return;
    }
    NimBLEClient* pClient = pConnectedClient;
    auto pService = pClient->getService(uuidServiceHid);
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
    XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println(pService->toString().c_str());
#endif
    for (auto pChara : pService->getCharacteristics()) {
      if (pChara->canWrite()) {
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
        XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println(
            "canWrite " + String(pChara->canWrite()));
        XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println(
            pChara->toString().c_str());
        writeWithComment(pChara, dataArr, dataLen);
#else
        pChara->writeValue(dataArr, dataLen, false);
#endif
      }
    }
  }

  void writeHIDReport(
      const XboxSeriesXHIDReportBuilder_asukiaaa::ReportBase& repo) {
    writeHIDReport((uint8_t*)repo.arr8t, repo.arr8tLen);
  }

  void writeHIDReport(
      const XboxSeriesXHIDReportBuilder_asukiaaa::ReportBeforeUnion&
          repoBeforeUnion) {
    XboxSeriesXHIDReportBuilder_asukiaaa::ReportBase repo;
    repo.v = repoBeforeUnion;
    writeHIDReport((uint8_t*)repo.arr8t, repo.arr8tLen);
  }

  void onLoop() {
    if (!initialized) return;
    if (!isConnected()) {
      const auto advertised = advDevice.exchange(nullptr);
      if (advertised != nullptr) {
        auto connectionResult = connectToServer(advertised);
        if (!connectionResult || !isConnected()) {
          NimBLEDevice::deleteBond(advertised->getAddress());
          ++countFailedConnection;
          // reset();
          connectionState = ConnectionState::Scanning;
        } else {
          countFailedConnection = 0;
        }
      } else if (!isScanning()) {
        // reset();
        startScan();
      }
    }
  }

  String buildDeviceAddressStr() {
    char buffer[18];
    auto addr = deviceAddressArr;
    snprintf(buffer, sizeof(buffer), "%02x:%02x:%02x:%02x:%02x:%02x", addr[5],
             addr[4], addr[3], addr[2], addr[1], addr[0]);
    return String(buffer);
  }

  void startScan() {
    connectionState = ConnectionState::Scanning;
    auto pScan = NimBLEDevice::getScan();
    // pScan->clearResults();
    // pScan->clearDuplicateCache();
    pScan->setDuplicateFilter(false);
    pScan->setScanCallbacks(scanCBs);
    // pScan->setActiveScan(true);
    pScan->setInterval(97);
    pScan->setWindow(97);
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
    XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println("Start scan");
#endif
    pScan->start(msScanTime);
  }

  // Legacy single-thread access only; tasks must use getSnapshot().
  XboxControllerNotificationParser xboxNotif;

  struct Snapshot {
    XboxControllerNotificationParser notification;
    uint8_t battery;
    bool connected;
    uint32_t receivedNotificationAt;
    uint32_t receivedBatteryAt;
    uint32_t notificationSequence;
    uint32_t disconnectGeneration;
  };

  Snapshot getSnapshot() {
    StateGuard guard(stateMux);
    return {xboxNotif, battery, connectionState == ConnectionState::Connected,
            receivedNotificationAt, receivedBatteryAt, notificationSequence,
            disconnectGeneration};
  }

  bool isWaitingForFirstNotification() {
    return connectionState == ConnectionState::WaitingForFirstNotification;
  }
  bool isConnected() {
    return connectionState == ConnectionState::WaitingForFirstNotification ||
           connectionState == ConnectionState::Connected;
  }
  unsigned long getReceiveNotificationAt() { return getSnapshot().receivedNotificationAt; }
  uint8_t getCountFailedConnection() { return countFailedConnection; }
  unsigned long getReceiveBatteryAt() { return getSnapshot().receivedBatteryAt; }

 private:
  friend class ClientCallbacks;
  portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
  std::atomic<ConnectionState> connectionState{ConnectionState::Scanning};
  uint32_t receivedNotificationAt = 0;
  uint32_t receivedBatteryAt = 0;
  uint32_t notificationSequence = 0;
  uint32_t disconnectGeneration = 0;

  void clearInputLocked() {
    xboxNotif = XboxControllerNotificationParser();
    battery = 0;
    receivedNotificationAt = receivedBatteryAt = 0;
  }
  bool initialized = false;
  uint32_t msScanTime = 4000; /** 0 = scan forever */
  uint8_t countFailedConnection = 0;
  uint8_t retryCountInOneConnection = 3;
  unsigned long retryIntervalMs = 100;
  NimBLEClient* pClient = nullptr;

#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
  static void writeWithComment(NimBLERemoteCharacteristic* pChara,
                               uint8_t* data, size_t len) {
    Serial.println("send(print from addr 0) ");
    for (int i = 0; i < len; ++i) {
      Serial.print(data[i]);
      Serial.print(" ");
    }
    if (pChara->writeValue(data, len, true)) {
      XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println("suceeded in writing");
    } else {
      XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println("failed writing");
    }
  }
#endif

  static void readAndPrint(NimBLERemoteCharacteristic* pChara) {
    auto str = pChara->readValue();
    if (str.size() == 0) {
      str = pChara->readValue();
    }
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
    printValue(str);
#endif
  }

  bool isScanning() { return NimBLEDevice::getScan()->isScanning(); }

  // void reset() {
  //   NimBLEDevice::deinit(true);
  //   delay(500);
  //   begin();
  //   delay(500);
  // }

  /** Handles the provisioning of clients and connects / interfaces with the
   * server */
  bool connectToServer(const NimBLEAdvertisedDevice* advDevice) {
    NimBLEClient* pClient = nullptr;

    /** Check if we have a client we should reuse first **/
    if (NimBLEDevice::getCreatedClientCount()) {
      pClient = NimBLEDevice::getClientByPeerAddress(advDevice->getAddress());
      if (pClient) {
        pClient->connect();
      }
    }

    /** No client to reuse? Create a new one. */
    if (!pClient) {
      if (NimBLEDevice::getCreatedClientCount() >= NIMBLE_MAX_CONNECTIONS) {
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
        XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println(
            "Max clients reached - no more connections available");
#endif
        return false;
      }

      pClient = NimBLEDevice::createClient();

#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
      XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println("New client created");
#endif

      // default values
      // pClient->setConnectionParams(
      //     BLE_GAP_INITIAL_CONN_ITVL_MIN, BLE_GAP_INITIAL_CONN_ITVL_MAX,
      //     BLE_GAP_INITIAL_CONN_LATENCY, BLE_GAP_INITIAL_SUPERVISION_TIMEOUT,
      //     16, 16);
      // pClient->setConnectionParams(
      //     BLE_GAP_INITIAL_CONN_ITVL_MIN, BLE_GAP_INITIAL_CONN_ITVL_MAX,
      //     BLE_GAP_INITIAL_CONN_LATENCY, BLE_GAP_INITIAL_SUPERVISION_TIMEOUT,
      //     100, 100);
      pClient->setClientCallbacks(clientCBs, true);
      pClient->connect(advDevice, true);
    }

    int retryCount = retryCountInOneConnection;
    while (!pClient->isConnected()) {
      if (retryCount <= 0) {
        return false;
      } else {
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
        XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println("Try connection. left: " +
                                                      String(retryCount));
#endif
      }

      // NimBLEDevice::getScan()->stop();
      // pClient->disconnect();
      delay(retryIntervalMs);
      // Serial.println(pClient->toString().c_str());
      pClient->connect(true);
      --retryCount;
    }
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
    XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.print("Connected to: ");
    XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println(
        pClient->getPeerAddress().toString().c_str());
    XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.print("RSSI: ");
    XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println(pClient->getRssi());
#endif

    // pClient->discoverAttributes();

    bool result = afterConnect(pClient);
    if (!result) {
      return result;
    }

#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
    XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println("Done with this device!");
#endif
    pConnectedClient = pClient;
    return true;
  }

  bool afterConnect(NimBLEClient* pClient) {
    memcpy(deviceAddressArr, pClient->getPeerAddress().getBase()->val,
           deviceAddressLen);
    for (auto pService : pClient->getServices(true)) {
      auto sUuid = pService->getUUID();
      if (!sUuid.equals(uuidServiceHid) && !sUuid.equals(uuidServiceBattery)) {
        continue;  // skip
      }
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
      XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println(
          pService->toString().c_str());
#endif
      for (auto pChara : pService->getCharacteristics(true)) {
        charaHandle(pChara);
        charaSubscribeNotification(pChara);
      }
    }

    return true;
  }

#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
  void charaPrintId(NimBLERemoteCharacteristic* pChara) {
    XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.printf(
        "s:%s c:%s h:%d",
        pChara->getRemoteService()->getUUID().toString().c_str(),
        pChara->getUUID().toString().c_str(), pChara->getHandle());
  }

  static void printValue(std::__cxx11::string str) {
    XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.printf("str: %s\n", str.c_str());
    XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.printf("hex:");
    for (auto v : str) {
      XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.printf(" %02x", v);
    }
    XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println("");
  }
#endif

  void charaHandle(NimBLERemoteCharacteristic* pChara) {
    if (pChara->canWrite()) {
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
      charaPrintId(pChara);
      XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println(" canWrite");
#endif
    }
    if (pChara->canRead()) {
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
      charaPrintId(pChara);
      XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println(" canRead");
#endif
      // Reading value is required for subscribe
      auto str = pChara->readValue();
      if (str.size() == 0) {
        str = pChara->readValue();
      }
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
      printValue(str);
#endif
    }
  }

  void charaSubscribeNotification(NimBLERemoteCharacteristic* pChara) {
    if (pChara->canNotify()) {
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
      charaPrintId(pChara);
      XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println(" canNotify ");
#endif
      if (pChara->subscribe(
              true,
              std::bind(&Core::notifyCB, this, std::placeholders::_1,
                        std::placeholders::_2, std::placeholders::_3,
                        std::placeholders::_4),
              true)) {
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
        XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println(
            "succeeded in subscribing");
#endif
      } else {
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
        XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println("failed subscribing");
#endif
      }
    }
  }

  void notifyCB(NimBLERemoteCharacteristic* pRemoteCharacteristic,
                uint8_t* pData, size_t length, bool isNotify) {
    if (!pRemoteCharacteristic || !pRemoteCharacteristic->getRemoteService() ||
        !pData) return;
    auto sUuid = pRemoteCharacteristic->getRemoteService()->getUUID();
    if (sUuid.equals(uuidServiceHid) &&
        pRemoteCharacteristic->getUUID().equals(uuidCharaReport)) {
      StateGuard guard(stateMux);
      if (!isConnected() || xboxNotif.update(pData, length) != 0) return;
      ++notificationSequence;
      receivedNotificationAt = millis();
      connectionState = ConnectionState::Connected;
    } else if (sUuid.equals(uuidServiceBattery) &&
               pRemoteCharacteristic->getUUID().equals(NimBLEUUID("2a19")) &&
               length == 1 && pData[0] <= 100) {
      StateGuard guard(stateMux);
      if (!isConnected()) return;
      battery = pData[0];
      receivedBatteryAt = millis();
    }
  }
};

inline void ClientCallbacks::onConnect(NimBLEClient* pClient) {
  StateGuard guard(owner->stateMux);
  owner->clearInputLocked();
  owner->connectionState = ConnectionState::WaitingForFirstNotification;
}

inline void ClientCallbacks::onDisconnect(NimBLEClient* pClient, int reason) {
  StateGuard guard(owner->stateMux);
  owner->clearInputLocked();
  ++owner->disconnectGeneration;
  owner->connectionState = ConnectionState::Scanning;
  pConnectedClient = nullptr;
}

};  // namespace XboxSeriesXControllerESP32_asukiaaa

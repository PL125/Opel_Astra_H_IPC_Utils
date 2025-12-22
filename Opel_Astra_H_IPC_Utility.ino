#include <SPI.h>


class M35080 {
  private:
    // M35080 Properties
    static constexpr uint8_t  PAGE_SIZE                     = {0x20u};  // 32 bytes
    static constexpr uint16_t MEMORY_SIZE                   = {0x400u}; // 1024 bytes => 8 Kbits
    static constexpr uint8_t  PAGE_COUNT                    = {MEMORY_SIZE / PAGE_SIZE};
    static constexpr uint8_t  WRITE_CYCLE_TIMEOUT_MS        = {0x0Au};  // 10 Milliseconds
    static constexpr uint16_t INCREMENTAL_ADDRESS_START     = {0x0000u};
    static constexpr uint16_t INCREMENTAL_ADDRESS_END       = {0x001Fu};
    static constexpr uint16_t NON_INCREMENTAL_ADDRESS_START = {0x0020u};
    static constexpr uint16_t NON_INCREMENTAL_ADDRESS_END   = {0x03FFu};

    // M35080 Instruction Set
    static constexpr uint8_t WREN   = 0b00000110u;  // Write Enable
    static constexpr uint8_t WRDI   = 0b00000100u;  // Write Disable
    static constexpr uint8_t RDSR   = 0b00000101u;  // Read Status Register
    static constexpr uint8_t WRSR   = 0b00000001u;  // Write Status Register
    static constexpr uint8_t READ   = 0b00000011u;  // Read Data from Memory Array
    static constexpr uint8_t WRITE  = 0b00000010u;  // Write Data to Memory Array
    static constexpr uint8_t WRINC  = 0b00000111u;  // Write Data to Secure Array

    // M35080 Status Register Bits
    static constexpr uint8_t SRWD_FLAG = 0b10000000u; // Status Register Write Disable (SRWD) --> Read/Write bit
    static constexpr uint8_t UV_FLAG   = 0b01000000u; // (UV)                                 --> Read only bit
    static constexpr uint8_t X_FLAG    = 0b00100000u; // (X)
    static constexpr uint8_t INC_FLAG  = 0b00010000u; // (INC)                                --> Read only bit
    static constexpr uint8_t BP1_FLAG  = 0b00001000u; // Block Protect 1 (BP1)                --> Read/Write bit
    static constexpr uint8_t BP0_FLAG  = 0b00000100u; // Block Protect 0 (BP0)                --> Read/Write bit
    static constexpr uint8_t WEL_FLAG  = 0b00000010u; // Write Enable Latch (WEL)             --> Read only bit
    static constexpr uint8_t WIP_FLAG  = 0b00000001u; // Write In Progress (WIP)              --> Read only bit 


    SPIClass m_SPI;
    const uint8_t m_WP;   // Write Protect Pin  (~W)
    const uint8_t m_CS;   // Chip Select Pin    (~S)
  //const uint8_t m_SCLK; // Serial Clock Pin   (C)
  //const uint8_t m_MISO; // Serial Output Pin  (Q)
  //const uint8_t m_MOSI; // Serial Input Pin   (D)
  //static inline const SPISettings SPI_Cfg{5000000, MSBFIRST, SPI_MODE0};

    // Delete default constructor
    M35080() = delete;

    // Delete move constructor
    M35080(M35080 &&) = delete;

    // Delete copy constructor
    M35080(const M35080 &) = delete;

    // Delete assignment operators
    M35080& operator=(M35080 &&) = delete;
    M35080& operator=(const M35080 &) = delete;

    constexpr bool isEvenAddress(const uint16_t address) const {
      return (0u == (address & uint16_t{0x0001u}));
    }

    constexpr bool isIncrementalAddress(const uint16_t address) const {
      return ((this->INCREMENTAL_ADDRESS_END >= address) /*&& (this->INCREMENTAL_ADDRESS_START <= address)*/);
    }

    void beginSPI() {
    //this->m_SPI.beginTransaction(this->SPI_Cfg);
      this->m_SPI.beginTransaction(SPISettings{5000000, MSBFIRST, SPI_MODE0});
    }

    void endSPI() {
      this->m_SPI.endTransaction();
    }

    // Select the EEPROM
    void chipSelect() {
      digitalWrite(this->m_CS, LOW);  // Set Chip Select pin low
    }

    // Deselect the EEPROM
    void chipDeselect() {
      digitalWrite(this->m_CS, HIGH); // Set Chip Select pin high
      delay(1);                       // Wait "Deselect Time" (tSHSL)
    }

    // Check incremental write operation
    bool wasIncrementalWriteOk() {
      const uint8_t currentStatusReg = readStatusRegister(0);
      if (0u == (INC_FLAG & currentStatusReg)) {
        return true;  // Incremental write success
      }
      return false;
    }

    // Enable hardware write protection, forbids writing to Status Register bits (BP0, BP1 and SRWD)
    bool enableHardwareProtectedMode() {
      if (this->m_WP > 0u) {
        const uint8_t currentStatusReg = readStatusRegister(0);
        if (0u == (SRWD_FLAG & currentStatusReg)) {
          digitalWrite(this->m_WP, HIGH);                                 // Set Write Protect pin high
          const uint8_t desiredStatusReg = SRWD_FLAG | currentStatusReg;  // Set "Status Register Write Disable" (SRWD) bit
          writeStatusRegister(desiredStatusReg);                          // Write 1 to "Status Register Write Disable" (SRWD) bit
          digitalWrite(this->m_WP, LOW);                                  // Set Write Protect pin low
        }
        return true;
      }
      return false;
    }

    // Disable hardware write protection, allows writing to Status Register bits (BP0, BP1 and SRWD)
    bool disableHardwareProtectedMode() {
      if (this->m_WP > 0u) {
        const uint8_t currentStatusReg = readStatusRegister(0);
        if (0u != (SRWD_FLAG & currentStatusReg)) {
          digitalWrite(this->m_WP, HIGH);                                 // Set Write Protect pin high
          const uint8_t desiredStatusReg = ~SRWD_FLAG & currentStatusReg; // Clear "Status Register Write Disable" (SRWD) bit
          writeStatusRegister(desiredStatusReg);                          // Write 0 to "Status Register Write Disable" (SRWD) bit
          digitalWrite(this->m_WP, LOW);                                  // Set Write Protect pin low
        }
        return true;
      }
      return false;
    }

    // Enable write access to the EEPROM
    void sendWriteEnable() {
      beginSPI();
      chipSelect();
      this->m_SPI.transfer(WREN); // Send "Write Enable" instruction
      chipDeselect();
      endSPI();
    }

    // Disable write access to the EEPROM
    void sendWriteDisable() {
      beginSPI();
      chipSelect();
      this->m_SPI.transfer(WRDI); // Send "Write Disable" instruction
      chipDeselect();
      endSPI();
    }

    // Wait "Write Enable Latch" (WEL) bit is set
    bool waitWriteReady(const uint32_t timeout = 1000u) {
      bool ret = false;
      const uint32_t start = millis();
      beginSPI();
      chipSelect();
      this->m_SPI.transfer(RDSR); // Send "Read Status Register" instruction
      do {  // Poll WEL bit
        const uint8_t currentStatusReg = this->m_SPI.transfer(0x00u);
        if (0u != (WEL_FLAG & currentStatusReg)) {
          ret = true;
          break;
        }
        delay(1);
      } while (timeout > (millis() - start));
      chipDeselect();
      endSPI();
      return ret;
    }

    // Wait "Write In Progress" (WIP) bit to clear
    bool waitWriteComplete(const uint32_t timeout = 1000u) {  // TODO: Use WRITE_CYCLE_TIMEOUT_MS as timeout
      bool ret = false;
      const uint32_t start = millis();
      beginSPI();
      chipSelect();
      this->m_SPI.transfer(RDSR); // Send "Read Status Register" instruction
      do {  // Poll WIP bit
        const uint8_t currentStatusReg = this->m_SPI.transfer(0x00u);
        if (0u == (WIP_FLAG & currentStatusReg)) {
          ret = true;
          break;
        }
        delay(1);
      } while (timeout > (millis() - start));
      chipDeselect();
      endSPI();
      return ret;
    }

    // Read EEPROM Status Register
    uint8_t readStatusRegister(const int) {
      beginSPI();
      chipSelect();
      this->m_SPI.transfer(RDSR); // Send "Read Status Register" instruction
      const uint8_t ret = this->m_SPI.transfer(0x00u);
      chipDeselect();
      endSPI();
      return ret;
    }

    // Write EEPROM Status Register
    void writeStatusRegister(const uint8_t value) {
      sendWriteEnable();
      if (waitWriteReady() == true) {
        beginSPI();
        chipSelect();
        this->m_SPI.transfer(WRSR); // Send "Write Status Register" instruction
        this->m_SPI.transfer(value);
        chipDeselect();
        endSPI();
        (void)waitWriteComplete();
      }
      sendWriteDisable();
    }


  public:
    M35080(const uint8_t CS, const uint8_t WP = 0) :
      m_WP{WP},
      m_CS{CS}
    {
      if (this->m_WP > 0u) {
        pinMode(this->m_WP, OUTPUT);
        digitalWrite(this->m_WP, LOW);
      } // else { pinMode(this->m_WP, INPUT_PULLUP); }

      pinMode(this->m_CS, OUTPUT);
      digitalWrite(this->m_CS, HIGH);   // Start with chip not selected

      this->m_SPI = SPIClass(SPI);
      this->m_SPI.begin();
    }

    ~M35080() {
      this->m_SPI.end();
    }

    static constexpr uint16_t getMemorySize() {
      return M35080::MEMORY_SIZE;
    }

    uint8_t readStatusRegister() {
      return readStatusRegister(0);
    }

    // Test EEPROM connections
    bool testConnections() {
      const uint8_t currentStatusReg = readStatusRegister(0);
      if (0u == (WEL_FLAG & currentStatusReg)) {  // The "Write Enable Latch" (WEL) is a volatile bit, at power on it must be cleared
        sendWriteEnable();
        if (waitWriteReady() == true) {           // The "Write Enable Latch" (WEL) bit was set, EEPROM connections are good
          sendWriteDisable();
          return true;
        }
      }
      return false; 
    }

    void printStatusRegister() {
      const uint8_t statusRegister = readStatusRegister(0);
      struct {
        const __FlashStringHelper *pStatusRegisterStr;
        const uint8_t statusRegisterFlag;
      } const statusRegisters[] = {
        { F("Status Register Write Disable (SRWD) -->"), SRWD_FLAG },
        { F("(UV)                                 -->"), UV_FLAG   },
        { F("(X)                                  -->"), X_FLAG    },
        { F("(INC)                                -->"), INC_FLAG  },
        { F("Block Protect 1 (BP1)                -->"), BP1_FLAG  },
        { F("Block Protect 0 (BP0)                -->"), BP0_FLAG  },
        { F("Write Enable Latch (WEL)             -->"), WEL_FLAG  },
        { F("Write In Progress (WIP)              -->"), WIP_FLAG  }
      };

      Serial.println(F("\n----------------------------------------------"));
      Serial.print(F(" Current Status Register value --> "));
      Serial.println(statusRegister, BIN);
      for (const auto &statusReg : statusRegisters) {
        Serial.print(F("  "));
        Serial.print(statusReg.pStatusRegisterStr);
        Serial.print(F(" "));
        Serial.println((0u != (statusReg.statusRegisterFlag & statusRegister)) ? 1u : 0u);
      }
      Serial.println(F("----------------------------------------------\n"));
    }

    uint8_t readByte(const uint16_t address) {
      beginSPI();
      chipSelect();
      this->m_SPI.transfer(READ); // Send "Read Data from Memory Array" instruction
      this->m_SPI.transfer16(address);
      const uint8_t value = this->m_SPI.transfer(0x00u);
      chipDeselect();
      endSPI();
      return value;
    }

    uint16_t readWord(const uint16_t address) {
      beginSPI();
      chipSelect();
      this->m_SPI.transfer(READ); // Send "Read Data from Memory Array" instruction
      this->m_SPI.transfer16(address);
      const uint16_t value = this->m_SPI.transfer16(0x0000u);
      chipDeselect();
      endSPI();
      return value;
    }

    bool readBytes(const uint16_t startAddress, uint8_t *pBuffer, const uint16_t length) {
      bool ret = false;
      if ((nullptr != pBuffer) && (0u < length)) {
        beginSPI();
        chipSelect();
        this->m_SPI.transfer(READ); // Send "Read Data from Memory Array" instruction
        this->m_SPI.transfer16(startAddress);
        for (uint16_t i = 0u; i < length; ++i) {
          *pBuffer++ = this->m_SPI.transfer(0x00u);
        }
        chipDeselect();
        endSPI();
        ret = true;
      }
      return ret;
    }

    bool writeByte(const uint16_t address, const uint8_t value) {
      bool ret = false;
      if (isIncrementalAddress(address) == false) {
        sendWriteEnable();
        if (waitWriteReady() == true) {
          beginSPI();
          chipSelect();
          this->m_SPI.transfer(WRITE);  // Send "Write Data to Memory Array" instruction
          this->m_SPI.transfer16(address);
          this->m_SPI.transfer(value);
          chipDeselect();
          endSPI();
          ret = waitWriteComplete();
        }
      }
      return ret;
    }

    // Write a word (two bytes) to the EEPROM incremental area
    bool writeIncremental(const uint8_t address, const uint8_t value1, const uint8_t value2) {
      bool ret = false;
      if (isIncrementalAddress(address) == true) {
        if (isEvenAddress(address) == false) {
          Serial.println(F("Error: Incremental write must respect even address boundaries!"));
          return ret;
        }
        sendWriteEnable();
        if (waitWriteReady() == true) {
          beginSPI();
          chipSelect();
          this->m_SPI.transfer(WRINC);  // Send "Write Data to Secure Array" instruction
          this->m_SPI.transfer16(address);
          this->m_SPI.transfer(value1); // Send Data Byte 1
          this->m_SPI.transfer(value2); // Send Data Byte 2
          chipDeselect();
          endSPI();
          if (waitWriteComplete() == true) {
            ret = wasIncrementalWriteOk();
          }
        }
      }
      return ret;
    }

    // Write one or more bytes to the EEPROM in a single WRITE cycle (Page Write Sequence)
    bool writeBytes(const uint16_t startAddress, const uint8_t *pBuffer, const uint8_t length) {
      bool ret = false;
      if ((nullptr != pBuffer) && (0u < length) && (isIncrementalAddress(startAddress) == false)) {
        if (this->PAGE_SIZE < length) {
          Serial.println(F("Error: Length exceeds page size!"));
          return ret;
        }
        if (this->PAGE_SIZE < ((startAddress % this->PAGE_SIZE) + length)) {
          Serial.println(F("Error: Write operation crosses page boundary!"));
          return ret;
        }
      //if (0u != (startAddress % this->PAGE_SIZE)) {
      //  Serial.println(F("Info: Address is not page aligned!"));
      //}
      //Serial.print(F("Info: Writing to page "));
      //Serial.println(startAddress / this->PAGE_SIZE);
        sendWriteEnable();
        if (waitWriteReady() == true) {
          beginSPI();
          chipSelect();
          this->m_SPI.transfer(WRITE);  // Send "Write Data to Memory Array" instruction
          this->m_SPI.transfer16(startAddress);
          for (uint8_t i = 0u; i < length; ++i) {
            this->m_SPI.transfer(*pBuffer++);
          }
          chipDeselect();
          endSPI();
          ret = waitWriteComplete();
        }
      }
      return ret;
    }
};









class Astra_H_Zafira_B_IPC_Utils {
  private:
    constexpr inline bool isDigit(const char c) {
      return ('0' <= c && '9' >= c);
    }

  public:
    static constexpr uint8_t SECURITY_CODE_LENGTH    = 4;
    static constexpr uint8_t VIN_NUMBER_LENGTH       = 17;
    static constexpr uint8_t CODE_IDENT_LENGTH       = 2;
    static constexpr uint8_t CODE_INDEX_LENGTH       = 3;
    static constexpr uint8_t SOFTWARE_VERSION_LENGTH = 5;

    Astra_H_Zafira_B_IPC_Utils() = default; // Default no-param constructor

    int32_t getKilometers(const uint8_t (&eeprom)[16]) {  // Offset 0x000-0x00F
      uint16_t prevVal = 0;
      uint8_t diffCount = 0;
      int32_t kilometrage = 0;

      for (uint8_t i = 0; i < 16; i += 2) {
        const uint8_t MSByte = eeprom[i + 0];
        const uint8_t LSByte = eeprom[i + 1];
        uint16_t curVal = 0;

        curVal |= static_cast<uint16_t>(MSByte) << 8;
        curVal |= static_cast<uint16_t>(LSByte) << 0;

        if ((i > 0) && (curVal != prevVal)) {
          ++diffCount;

          if (diffCount > 1) {
            return -1;
          }
        }

        kilometrage += curVal;
        prevVal = curVal;
      }

      return (kilometrage * 2) | diffCount;
    }

    bool getVinNumber(const uint8_t (&eeprom)[VIN_NUMBER_LENGTH], char (&vinNumberStr)[VIN_NUMBER_LENGTH + 1]) {  // Offset 0x1CE-0x1DE
      bool ret = false;

      if (strncmp(reinterpret_cast<const char*>(eeprom), "W0L0AH", 6) == 0) {
        ret = true;
      }

      snprintf(vinNumberStr, VIN_NUMBER_LENGTH + 1, "%s", (ret == true ? reinterpret_cast<const char*>(eeprom) : ""));

      return ret;
    }

    int16_t getSecurityCode(const uint8_t (&eeprom)[SECURITY_CODE_LENGTH], char (*pSecurityCodeStr)[SECURITY_CODE_LENGTH + 1] = nullptr) {  // Offset 0x234-0x237
      int16_t securityCode = 0;

      for (uint8_t i = 0; i < SECURITY_CODE_LENGTH; ++i) {
        if (!isDigit(eeprom[i])) {
          securityCode = -1;
          break;
        }

        const uint8_t curDigit = (eeprom[i] - '0');
        securityCode = securityCode * 10 + curDigit;
      }

      if (pSecurityCodeStr != nullptr) {
        volatile const size_t dstSize = SECURITY_CODE_LENGTH + 1;
        snprintf(*pSecurityCodeStr, dstSize, "%04" PRIi16, securityCode);
        //snprintf(*pSecurityCodeStr, dstSize, "%0*" PRIi16, SECURITY_CODE_LENGTH, securityCode);
      }

      return securityCode;
    }

    bool getAdditionalCodes(const uint8_t (&eeprom)[5], uint8_t &codeIndex, uint8_t &codeVersion) { // Offset 0x020-0x024
      codeIndex = 0;
      codeVersion = 0;

      for (uint8_t i = 0; i < 5; ++i) {
        if (!isDigit(eeprom[i])) {
          return false;
        }

        const uint8_t curDigit = (eeprom[i] - '0');
        if (i < CODE_INDEX_LENGTH) {
          codeIndex = codeIndex * 10 + curDigit;
        } else {
          codeVersion = codeVersion * 10 + curDigit;
        }
      }

      return true;
    }

    bool getCodeIdent(const uint8_t (&eeprom)[CODE_IDENT_LENGTH], char (&codeIdentStr)[CODE_IDENT_LENGTH + 1]) {  // Offset 0x0A0-0x0A1
      bool ret = true;
      auto isUpper = [](const uint8_t c) -> bool
      {
        return ('A' <= c && 'Z' >= c);
      };

      for (uint8_t i = 0; i < CODE_IDENT_LENGTH; ++i) {
        if (!isUpper(eeprom[i])) {
          ret = false;
          break;
        }
      }

      snprintf(codeIdentStr, CODE_IDENT_LENGTH + 1, "%s", (ret == true ? reinterpret_cast<const char*>(eeprom) : ""));

      return ret;
    }

    bool getSoftwareVersion(const uint8_t (&eeprom)[SOFTWARE_VERSION_LENGTH], char (&SoftwareVersionStr)[SOFTWARE_VERSION_LENGTH + 1]) {  // Offset 0x150-0x154
      bool ret = true;

      if (strncmp(reinterpret_cast<const char*>(eeprom), (const char[]){0x00, 0x00, 0x00, 0x00, 0x00}, SOFTWARE_VERSION_LENGTH) == 0) {
        ret = false;
      }

      snprintf(SoftwareVersionStr, SOFTWARE_VERSION_LENGTH + 1, "%s", (ret == true ? reinterpret_cast<const char*>(eeprom) : ""));

      return ret;
    }

    bool getCodeIndexStr(const uint8_t codIndex, char (&codeIndexStr)[40]) {
      struct {
        const uint8_t codeIndex;
        const __FlashStringHelper *pCodeIndexStr;
      } const codeIndexIPC[] = {
        { 1, F("001 - Astra H, linear resistance")     },
        { 3, F("003 - Zafira B")                       },
        { 4, F("004 - Zafira B")                       },
        { 5, F("005 - Astra H, linear voltage type B") },
        { 6, F("006 - Zafira B")                       },
        { 7, F("007 - Zafira B")                       }
      };

      for (const auto &lookupTbl : codeIndexIPC) {
        if (lookupTbl.codeIndex == codIndex) {
          strlcpy_P(codeIndexStr, reinterpret_cast<PGM_P>(lookupTbl.pCodeIndexStr), sizeof(codeIndexStr));
          //snprintf_P(codeIndexStr, sizeof(codeIndexStr), reinterpret_cast<PGM_P>( F("%s") ), reinterpret_cast<PGM_P>(lookupTbl.pCodeIndexStr));
          return true;
        }
      }

      return false;
    }

    bool getCodeVersionStr(const uint8_t codeVersion, char (&codeVersionStr)[3]) {
      snprintf(codeVersionStr, sizeof(codeVersionStr), "%02" PRIu8, codeVersion);

      return true;
    }

    void parseEEprom(const uint8_t (&eeprom)[0x400]) {
      uint8_t codeIdentBuffer[CODE_IDENT_LENGTH] = {};
      uint8_t vinNumberBuffer[VIN_NUMBER_LENGTH] = {};
      uint8_t softVersionBuffer[SOFTWARE_VERSION_LENGTH] = {};
      uint8_t kilometrageBuffer[16] = {};
      uint8_t securityCodeBuffer[SECURITY_CODE_LENGTH] = {};
      uint8_t additionalCodesBuffer[5] = {};

      memcpy(&kilometrageBuffer, &eeprom[0x000], sizeof(kilometrageBuffer));          // Offset 0x000-0x00F
      memcpy(&additionalCodesBuffer, &eeprom[0x020], sizeof(additionalCodesBuffer));  // Offset 0x020-0x024
      memcpy(&codeIdentBuffer, &eeprom[0x0A0], sizeof(codeIdentBuffer));              // Offset 0x0A0-0x0A1
      memcpy(&softVersionBuffer, &eeprom[0x150], sizeof(softVersionBuffer));          // Offset 0x150-0x154
      memcpy(&vinNumberBuffer, &eeprom[0x1CE], sizeof(vinNumberBuffer));              // Offset 0x1CE-0x1DE
      memcpy(&securityCodeBuffer, &eeprom[0x234], sizeof(securityCodeBuffer));        // Offset 0x234-0x237

      uint8_t codeIndex, codeVersion;
      char securityCodeStr[SECURITY_CODE_LENGTH + 1] = "";
      char vinNumber[VIN_NUMBER_LENGTH + 1] = "";
      char codeIdent[CODE_IDENT_LENGTH + 1] = "";
      char softwareVersion[SOFTWARE_VERSION_LENGTH + 1] = "";
      char codeIndexStr[40] = "";
      char codeVersionStr[3] = "";
      const int16_t securityCode = getSecurityCode(securityCodeBuffer, &securityCodeStr);
      const int32_t kilometrage = getKilometers(kilometrageBuffer);

      if (getAdditionalCodes(additionalCodesBuffer, codeIndex, codeVersion)) {
        getCodeIndexStr(codeIndex, codeIndexStr);
        getCodeVersionStr(codeVersion, codeVersionStr);
      }

      Serial.println((String)F("\n--------------------------------------------------------------------------"));
      Serial.println((String)F("EEPROM: M35080"));
      Serial.println((String)F("Security Code: ")    + (securityCode >= 0                                      ? securityCodeStr     : "Invalid eeprom content"));
      Serial.println((String)F("Kilometrage: ")      + (kilometrage >= 0                                       ? (String)kilometrage : "Invalid eeprom content"));
      Serial.println((String)F("VIN: ")              + (getVinNumber(vinNumberBuffer, vinNumber)               ? vinNumber           : "Invalid eeprom content"));
      Serial.println((String)F("Software Version: ") + (getSoftwareVersion(softVersionBuffer, softwareVersion) ? softwareVersion     : "Unsupported software version or invalid eeprom content"));
      Serial.println((String)F("Ident: ")            + (getCodeIdent(codeIdentBuffer, codeIdent)               ? codeIdent           : "Invalid eeprom content"));
      Serial.println((String)F("Code Version: ")     + (strnlen(codeVersionStr, sizeof(codeVersionStr))        ? codeVersionStr      : "Invalid eeprom content"));
      Serial.println((String)F("Code Index: ")       + (strnlen(codeIndexStr, sizeof(codeIndexStr))            ? codeIndexStr        : "Invalid eeprom content"));
      Serial.println((String)F("--------------------------------------------------------------------------\n"));
    }
};










//#define M35080_WRITE_ENABLE

constexpr uint32_t SERIAL_SPEED = 115200u;
constexpr uint16_t M35080_MEMORY_SIZE = M35080::getMemorySize();

// Pin definitions
constexpr uint8_t M35080_CS_PIN = 10; // Chip Select Pin
constexpr uint8_t M35080_WP_PIN =     // Write Protect Pin
#if !defined(M35080_WRITE_ENABLE)
  0;
#else
  9;
#endif


/********************************************************************************************
| M35080 Pin |      Description     | Arduino Uno Pin | Arduino Pro Micro Pin | Description |
|------------|----------------------|-----------------|-----------------------|-------------|
|     1      | VSS - Ground         |       GND       |          GND          |             |
|     2      | ~S  - Chip Select    |       10        |          10           |    SS       |
|     3      | ~W  - Write Protect  |       9         |          9            |    GPIO9    |
|     4      | Q   - Data Output    |       12        |          14           |    MISO     |
|     5      | NC  - Not Connected  |                 |                       |             |
|     6      | C   - Clock          |       13        |          15           |    SCK      |
|     7      | D   - Data Input     |       11        |          16           |    MOSI     |
|     8      | VCC - Supply Voltage |       5V        |          VCC          |             |
********************************************************************************************/



uint8_t EEPROM[M35080_MEMORY_SIZE];
Astra_H_Zafira_B_IPC_Utils AstraIpcUtils;
M35080 M35080(M35080_CS_PIN, M35080_WP_PIN);


void hexdump(const uint8_t (&eeprom)[M35080_MEMORY_SIZE], const bool printOffset = true, const bool printAscii = true) {
  constexpr uint16_t EEPROM_SIZE = sizeof(eeprom) / sizeof(eeprom[0]);
  char buffer[7];

  for (uint16_t offset = 0u; offset < EEPROM_SIZE; offset += 16u) {
    if (printOffset) {
      snprintf(buffer, sizeof(buffer), "0x%04X", offset);
      Serial.print(buffer);
      Serial.print(F("  "));
    }

    for (uint16_t byte = 0u; byte < 16u; ++byte) {
      if (EEPROM_SIZE <= offset + byte) {
        break;
      }
      snprintf(buffer, sizeof(buffer), "%02X", eeprom[offset + byte]);
      Serial.print(buffer);
      Serial.print(F(" "));
    }

    if (printAscii) {
      Serial.print(F(" "));
      for (uint16_t byte = 0u; byte < 16u; ++byte) {
        if (EEPROM_SIZE <= offset + byte) {
          break;
        }
        const unsigned char aux = static_cast<unsigned char>(eeprom[offset + byte]);
        if (isprint(aux)) {
          const char c = static_cast<char>(aux);
          Serial.print(c);
        } else {
          Serial.print(F("."));
        }
      }
    }
    Serial.println();
  }
  Serial.println();
}


void setup() {
  Serial.begin(SERIAL_SPEED);
  while ((Serial == false) || (millis() < 5000)); // Wait
  Serial.println();

  if (M35080.testConnections() == false) {
    Serial.println(F("Check connections, and try again"));
    return;
  }

  M35080.printStatusRegister();

  // Read EEPROM
  M35080.readBytes(0x0000u, &EEPROM[0x0000u], M35080_MEMORY_SIZE);

  // Print EEPROM
  hexdump(EEPROM, true, true);

  // Print decoded EEPROM
  AstraIpcUtils.parseEEprom(EEPROM);
}


void loop() {
  ;
}

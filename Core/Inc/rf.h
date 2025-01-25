/*
 * rf.h
 *
 *  Created on: Dec 18, 2024
 *      Author: batum
 */

#ifndef INC_RF_H_
#define INC_RF_H_

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>

#define MAX_FREQ_VAL 0xFF
#define MIN_FREQ_VAL 0x00

#define MAX_TX_BUFFER_SIZE 58L
#define MAX_TX_BUFFER_SIZE_CRC MAX_TX_BUFFER_SIZE - 1
#define MAX_TX_BUFFER_SIZE_FIXED_CRC MAX_TX_BUFFER_SIZE - 4

#define RF_Operating_Config() {HAL_GPIO_WritePin(this->_pinConfs.M0Pin.handle, this->_pinConfs.M0Pin.pin, GPIO_PIN_SET); HAL_GPIO_WritePin(this->_pinConfs.M1Pin.handle, this->_pinConfs.M1Pin.pin, GPIO_PIN_SET);}
#define RF_Operating_Normal() {HAL_GPIO_WritePin(this->_pinConfs.M0Pin.handle, this->_pinConfs.M0Pin.pin, GPIO_PIN_RESET); HAL_GPIO_WritePin(this->_pinConfs.M1Pin.handle, this->_pinConfs.M1Pin.pin, GPIO_PIN_RESET);}
#define RF_WaitAUX(__HANDLE__) {if(__waitAUX(__HANDLE__, __HANDLE__->_paramConfs.auxTimeout) == E32_Timeout)return E32_Timeout;}
#define RF_PackageTimerCheck(__HANDLE__) {do{if(__packageTimerCheck(__HANDLE__) == E32_NoPackageTime)return E32_NoPackageTime;}while(0);}

typedef enum Error_Status{
    E32_Success = 1,
    E32_Timeout,
    E32_CrcBroken,
    E32_FailureMode,
    E32_NoMessage,
    E32_BigPacket,
    E32_BrokenGetSet,
    E32_NoPackageTime,
    E32_OutOfLimit,
	E32_CloseFailure
} Status;

typedef enum RF_FREQ{
    FREQ_410 = 0x00,
    FREQ_411 = 0x01,
    FREQ_412 = 0x02,
    FREQ_413 = 0x03,
    FREQ_414 = 0x04,
    FREQ_415 = 0x05,
    FREQ_416 = 0x06,
    FREQ_417 = 0x07,
    FREQ_418 = 0x08,
    FREQ_419 = 0x09,
    FREQ_420 = 0x0A,
    FREQ_421 = 0x0B,
    FREQ_422 = 0x0C,
    FREQ_423 = 0x0D,
    FREQ_424 = 0x0E,
    FREQ_425 = 0x0F,
    FREQ_426 = 0x10,
    FREQ_427 = 0x11,
    FREQ_428 = 0x12,
    FREQ_429 = 0x13,
    FREQ_430 = 0x14,
    FREQ_431 = 0x15,
    FREQ_432 = 0x16,
    FREQ_433 = 0x17,
    FREQ_434 = 0x18,
    FREQ_435 = 0x19,
    FREQ_436 = 0x1A,
    FREQ_437 = 0x1B,
    FREQ_438 = 0x1C,
    FREQ_439 = 0x1D,
    FREQ_440 = 0x1E,
    FREQ_441 = 0x1F
}RF_FREQ;

typedef enum RF_DEBUGGER_UART_PARITY{
    DEBUGGER_UART_PARITY_8N1 = UART_PARITY_NONE,
    DEBUGGER_UART_PARITY_8O1 = UART_PARITY_ODD,
    DEBUGGER_UART_PARITY_8E1 = UART_PARITY_EVEN
}RF_DEBUGGER_UART_PARITY;

/*
------------------------
 Ayar Öntanim Makrolari
------------------------
*/
// SPED 7, 6 bit
typedef enum RF_UART_PARITY{
    UARTPARITY_8N1 = 0b00,
    UARTPARITY_8O1 = 0b01,
    UARTPARITY_8E1 = 0b10,
}RF_UART_PARITY;

// SPED 5, 4, 3 bit
typedef enum RF_UART_BAUD{
    UARTBAUDRATE_1200 = 0b000,
    UARTBAUDRATE_2400 = 0b001,
    UARTBAUDRATE_4800 = 0b010,
    UARTBAUDRATE_9600 = 0b011,
    UARTBAUDRATE_19200 = 0b100,
    UARTBAUDRATE_38400 = 0b101,
    UARTBAUDRATE_57600 = 0b110,
    UARTBAUDRATE_115200 = 0b111,
}RF_UART_BAUD;

// SPED 2, 1, 0
typedef enum RF_AIR_DATA{
    AIRDATARATE_03k = 0b000,
    AIRDATARATE_12k = 0b001,
    AIRDATARATE_24k = 0b010,
    AIRDATARATE_48k = 0b011,
    AIRDATARATE_96k = 0b100,
    AIRDATARATE_192k = 0b101,
}RF_AIR_DATA;

// OPTION 7 bit
typedef enum RF_TRANS_MODE{
    TRANSPARENTMODE = 0b0,
    FIXEDMODE = 0b1,
    BROADCASTMODE = 0b11111,
}RF_TRANS_MODE;

// OPTION 6 bit
typedef enum RF_IO_MODE{
    IO_PUSHPULL = 0b0,
    IO_OPENDRAIN = 0b1,
}RF_IO_MODE;

// OPTION 5, 4, 3 bit
typedef enum RF_WIRELESS{
    WIRELESSWAKEUP_250 = 0b000,
    WIRELESSWAKEUP_500 = 0b001,
    WIRELESSWAKEUP_750 = 0b010,
    WIRELESSWAKEUP_1000 = 0b011,
    WIRELESSWAKEUP_1250 = 0b100,
    WIRELESSWAKEUP_1500 = 0b101,
    WIRELESSWAKEUP_1750 = 0b110,
    WIRELESSWAKEUP_2000 = 0b111,
}RF_WIRELESS;

// OPTION 2 bit
typedef enum RF_FEC{
    FEC_OFF = 0b0,
    FEC_ON = 0b1,
}RF_FEC;

// OPTION 1, 0 bit
typedef enum RF_TRANS_POWER{
    TRANSMISSIONPOWER_30 = 0b00,
    TRANSMISSIONPOWER_27 = 0b01,
    TRANSMISSIONPOWER_24 = 0b10,
    TRANSMISSIONPOWER_21 = 0b11,
}RF_TRANS_POWER;

/*
--------------------
 Struct Tanimlari
--------------------
*/
typedef struct RF_GPIOPinData{
	GPIO_TypeDef *handle;
	uint16_t pin;
	char set;
}RF_GPIOPinData;

typedef struct RF_GPIOPins{
    	RF_GPIOPinData M0Pin;
    	RF_GPIOPinData M1Pin;
    	RF_GPIOPinData AUXPin;
}RF_GPIOPins;

typedef struct Sped{
    RF_UART_PARITY UARTParity; // UARTPARITY_8N1
    RF_UART_BAUD UARTBaud; // UARTBAUDRATE_9600
    RF_AIR_DATA AirDataRate; // AIRDATARATE_24k
}Sped;

typedef struct Option{
    RF_TRANS_MODE TransmissionMode; //TRANSPARENTMODE
    RF_IO_MODE IODriver; // IO_PUSHPULL
    RF_WIRELESS WirelessWakeUp; // WIRELESSWAKEUP_250
    RF_FEC FECset; // FEC_ON
    RF_TRANS_POWER TransmissionPower; // TRANSMISSIONPOWER_30
}Option;

typedef struct ConfigRF{
    struct Sped RFSped;
    struct Option RFOption;
    uint8_t Channel;
    uint8_t AddressHigh;
    uint8_t AddressLow;
}ConfigRF;

typedef struct E32_433T30D E32_433T30D;

// Private Fonksiyonlar
void __changeRFUARTBaudRate(UART_HandleTypeDef *port, uint32_t newBaudRate, uint32_t newParity);
void __debugPrintf(E32_433T30D* this, const char *format, ...);
void __clearSerialBuffer(E32_433T30D* this);
Status __waitAUX(E32_433T30D* this, uint32_t timeout);
void __managedDelay(E32_433T30D* this, uint32_t timeout);
Status __setSettings(E32_433T30D* this);
Status __getSettings(E32_433T30D* this);

char* __getTranmissionPower(const E32_433T30D* this);
char* __getFECFilter(const E32_433T30D* this);
char* __getWirelessWakeup(const E32_433T30D* this);
char* __getIOMode(const E32_433T30D* this);
char* __getTransmissionType(const E32_433T30D* this);
char* __getAirData(const E32_433T30D* this);
char* __getUARTParity(const E32_433T30D* this);
char* __getUARTBaudRate(const E32_433T30D* this);

float __airDataRateEnum2Value(const E32_433T30D* this);
uint32_t __UARTRateEnum2Value(const RF_UART_BAUD baud);
uint32_t __UARTParityEnum2Value(const RF_UART_PARITY parity);
uint64_t __calculatePacketSendTime(E32_433T30D* this, const size_t packetSize);
Status __setPinConfig(E32_433T30D* this, const RF_GPIOPinData m0, const RF_GPIOPinData m1, const RF_GPIOPinData aux);
Status __ChangeSettings(E32_433T30D* this, const uint8_t HighAddress, const uint8_t LowAddress, const uint8_t channel,
                 const RF_UART_PARITY parity, const RF_UART_BAUD baud, const RF_AIR_DATA airdata,
                 const RF_TRANS_MODE transmode, const RF_IO_MODE IOmode, const RF_WIRELESS wirelesswake,
                 const RF_FEC fecmode, const RF_TRANS_POWER transpower);
Status __RFStart();
Status __packageTimerCheck(E32_433T30D* this);
Status __tempConftoDevice(E32_433T30D* this);

// Function Pointer Fonksiyonları
uint8_t __calculateCRC8(E32_433T30D* this, const uint8_t *data, const size_t length);

Status __receiveSingleData(E32_433T30D* this, uint8_t *data);
Status __receiveDataPacket(E32_433T30D* this, uint8_t *data, const size_t length);

Status __sendFixedSingleData(E32_433T30D* this, const uint8_t AddressHigh, const uint8_t AddressLow, const uint8_t Channel, const uint8_t* data);
Status __sendTransparentSingleData(E32_433T30D* this, const uint8_t* data);

Status __sendFixedDataPacket(E32_433T30D* this, const uint8_t AddressHigh, const uint8_t AddressLow, const uint8_t Channel, const uint8_t *data, const size_t size);
Status __sendBroadcastDataPacket(E32_433T30D* this, const uint8_t Channel, const uint8_t *data, const size_t size);
Status __sendTransparentDataPacket(E32_433T30D* this, uint8_t *data, const size_t size);

Status __setTransmissionMode(E32_433T30D* this, const RF_TRANS_MODE Mode);
Status __setAddresses(E32_433T30D* this, const uint8_t AddHigh, const uint8_t AddLow);
Status __setChannel(E32_433T30D* this, const RF_FREQ channel);
Status __setTransmissionPower(E32_433T30D* this, const RF_TRANS_POWER power);
Status __setIODriver(E32_433T30D* this, const RF_IO_MODE driver);
Status __setFECSettings(E32_433T30D* this, const RF_FEC fecmode);
Status __setWirelesWakeup(E32_433T30D* this, const RF_WIRELESS time);
Status __setUARTParity(E32_433T30D* this, const RF_UART_PARITY paritybyte);
Status __setUARTBaudRate(E32_433T30D* this, const RF_UART_BAUD baudrate);
Status __setDebuggerUARTParity(E32_433T30D* this, const RF_UART_PARITY paritybyte);
Status __setDebuggerUARTBaudRate(E32_433T30D* this, const RF_UART_BAUD baudrate);
Status __setAirDataRate(E32_433T30D* this, const RF_AIR_DATA airdatarate);
Status __setAuxTimeoutTime(E32_433T30D* this, const size_t time);
Status __setNoAuxTimeoutTime(E32_433T30D* this, const size_t time);

void __setFunctionPointers(E32_433T30D* this);

void __viewSettings(E32_433T30D* this);
void __setInitialValues(E32_433T30D *this);
void __setRFUARTPort(E32_433T30D *this);
void __setDebuggerUARTPort(E32_433T30D* this);

struct E32_433T30D{
	UART_HandleTypeDef *SerialPort; // NULL
	UART_HandleTypeDef *DebugPort; // NULL
	ConfigRF *tempConfig;

	struct _paramConfs{
	        size_t auxTimeout; // 1000
	        size_t noAuxTimeOut; // 30
	        size_t serialTimeout; // 1000
	        size_t packetStartTimeStamp; // 0
	        size_t packetEndTimeStamp; // 0
	}_paramConfs;

	struct _debuggerConfs{
	        unsigned long baudRate; // 9600
	        RF_DEBUGGER_UART_PARITY parity; // DEBUGGER_UART_PARITY_8N1
	}_debuggerConfs;

	struct _devConfs{
	        struct Sped RFSped;
	        struct Option RFOption;
	        uint8_t channel;
	        uint8_t AddHigh;
	        uint8_t AddLow;
    }_devConfs;

    RF_GPIOPins _pinConfs;

    // Function Pointers
    //void (*RFInit)(E32_433T30D* this, const uint8_t *HighAddress, const uint8_t *LowAddress, const uint8_t *channel);

    uint8_t (*calculateCRC8)(E32_433T30D* this, const uint8_t *data, const size_t length);

    Status (*receiveSingleData)(E32_433T30D* this, uint8_t *data);
    Status (*receiveDataPacket)(E32_433T30D* this, uint8_t *data, const size_t length);

    Status (*sendFixedSingleData)(E32_433T30D* this, const uint8_t AddressHigh, const uint8_t AddressLow, const uint8_t Channel, const uint8_t* data);
    Status (*sendTransparentSingleData)(E32_433T30D* this, const uint8_t* data);

    Status (*sendFixedDataPacket)(E32_433T30D* this, const uint8_t AddressHigh, const uint8_t AddressLow, const uint8_t Channel, const uint8_t *data, const size_t size);
    Status (*sendBroadcastDataPacket)(E32_433T30D* this, const uint8_t Channel, const uint8_t *data, const size_t size);
    Status (*sendTransparentDataPacket)(E32_433T30D* this, uint8_t *data, const size_t size);

    Status (*setTransmissionMode)(E32_433T30D* this, const RF_TRANS_MODE Mode);
    Status (*setAddresses)(E32_433T30D* this, const uint8_t AddHigh, const uint8_t AddLow);
    Status (*setChannel)(E32_433T30D* this, const RF_FREQ channel);
    Status (*setTransmissionPower)(E32_433T30D* this, const RF_TRANS_POWER power);
    Status (*setIODriver)(E32_433T30D* this, const RF_IO_MODE driver);
    Status (*setFECSettings)(E32_433T30D* this, const RF_FEC fecmode);
    Status (*setWirelesWakeup)(E32_433T30D* this, const RF_WIRELESS time);
    Status (*setUARTParity)(E32_433T30D* this, const RF_UART_PARITY paritybyte);
    Status (*setUARTBaudRate)(E32_433T30D* this, const RF_UART_BAUD baudrate);
    Status (*setDebuggerUARTParity)(E32_433T30D* this, const RF_UART_PARITY paritybyte);
    Status (*setDebuggerUARTBaudRate)(E32_433T30D* this, const RF_UART_BAUD baudrate);
    Status (*setAirDataRate)(E32_433T30D* this, const RF_AIR_DATA airdatarate);
    Status (*setAuxTimeoutTime)(E32_433T30D* this, const size_t time);
    Status (*setNoAuxTimeoutTime)(E32_433T30D* this, const size_t time);

    void (*viewSettings)(E32_433T30D* this);
};


Status RFPinSet(RF_GPIOPinData* pinData, GPIO_TypeDef *handle, uint16_t port);
// Genel Fonksiyonlar
Status RFInit_NoPin(E32_433T30D* handler, UART_HandleTypeDef *RFPort);
Status RFInit(E32_433T30D* handler, UART_HandleTypeDef *RFPort, RF_GPIOPins pinData);
Status RFInit_Debugger(E32_433T30D* handler, UART_HandleTypeDef *RFPort, UART_HandleTypeDef *debuggerPort, RF_GPIOPins pinData);
Status RFClose(E32_433T30D *handler);

Status RFStart(E32_433T30D *handler);

#endif /* INC_RF_H_ */

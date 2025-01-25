/*
 * rf.c
 *
 *  Created on: Dec 18, 2024
 *      Author: batum
 */

#include "rf.h"

void __debugPrintf(E32_433T30D* this, const char *format, ...){
	if (this->DebugPort != NULL) {
		va_list args;

		va_start(args, format);
		int len = vsnprintf(NULL, 0, format, args);
		va_end(args);

		if (len <= 0) {
			return;
		}

		char *bufferTemp = (char*) malloc(len + 1);
		if (bufferTemp == NULL) {
			return;
		}

		va_start(args, format);
		vsnprintf(bufferTemp, len + 1, format, args);
		va_end(args);

		HAL_UART_Transmit(this->DebugPort, (uint8_t*) bufferTemp, len,
				HAL_MAX_DELAY);

		free(bufferTemp);
	}
}

void __clearSerialBuffer(E32_433T30D* this){
	if (__HAL_UART_GET_FLAG(this->SerialPort, UART_FLAG_ORE)) {
		__HAL_UART_CLEAR_OREFLAG(this->SerialPort);
	}

	__HAL_UART_FLUSH_DRREGISTER(this->SerialPort);
}

Status __waitAUX(E32_433T30D* this, uint32_t timeout){
	uint32_t startTime = HAL_GetTick();
	if (this->_pinConfs.AUXPin.set != 0) {
		while (HAL_GPIO_ReadPin(this->_pinConfs.AUXPin.handle, this->_pinConfs.AUXPin.pin) == GPIO_PIN_RESET) {
			if (HAL_GetTick() - startTime > timeout) {
				return E32_Timeout;
			}
			__managedDelay(this, 2);
		}

		__managedDelay(this, 10);
	} else {
		while (HAL_GetTick() - startTime < this->_paramConfs.noAuxTimeOut) {
			__managedDelay(this, 2);
		}
	}
	return E32_Success;
}

void __managedDelay(E32_433T30D* this, uint32_t timeout){
	uint32_t temp = HAL_GetTick();

	if((uint32_t) (temp) <= 0){
		temp = 0;
	}

	while((HAL_GetTick() - temp) < timeout){}
}

uint8_t __calculateCRC8(E32_433T30D* this, const uint8_t *data, const size_t length){
    if(length + 1 > MAX_TX_BUFFER_SIZE){
    	__debugPrintf(this, "CRC8 Fonksiyonu Maksimum Paketten Büyük/n");
        return E32_CrcBroken;
    }

    uint8_t crc = 0x00;

    for (size_t i = 0; i < length - 1; ++i) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x80){
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

void __changeRFUARTBaudRate(UART_HandleTypeDef *port, uint32_t newBaudRate, uint32_t newParity){
	HAL_UART_Abort(port);

	__HAL_UART_FLUSH_DRREGISTER(port);

	port->Init.BaudRate = newBaudRate;
	port->Init.Parity = newParity;

	if(HAL_UART_Init(port) != HAL_OK){
		//Error_Handler();
	}
}

Status __setSettings(E32_433T30D *this) {
	if (this->_pinConfs.M0Pin.set != 0) {
		uint8_t SpedByte = 0, OptionByte = 0;
		uint8_t MesArr[6];

		SpedByte = (this->_devConfs.RFSped.UARTParity << 6)
				| (this->_devConfs.RFSped.UARTBaud << 3)
				| (this->_devConfs.RFSped.AirDataRate);
		OptionByte = (this->_devConfs.RFOption.TransmissionMode << 7)
				| (this->_devConfs.RFOption.IODriver << 6)
				| (this->_devConfs.RFOption.WirelessWakeUp << 3)
				| (this->_devConfs.RFOption.FECset << 2)
				| (this->_devConfs.RFOption.TransmissionPower);

		RF_WaitAUX(this);

		uint32_t originalBaudRate = this->SerialPort->Init.BaudRate;
		uint32_t originalParity = this->SerialPort->Init.Parity;
		__changeRFUARTBaudRate(this->SerialPort, 9600, UART_PARITY_NONE);
		__managedDelay(this, 200);

		RF_Operating_Config();

		__managedDelay(this, 20);

		MesArr[0] = 0xC0;
		MesArr[1] = this->_devConfs.AddHigh;
		MesArr[2] = this->_devConfs.AddLow;
		MesArr[3] = SpedByte;
		MesArr[4] = this->_devConfs.channel;
		MesArr[5] = OptionByte;

		HAL_UART_Transmit(this->SerialPort, (uint8_t*) MesArr,
				sizeof(MesArr) / sizeof(MesArr[0]), HAL_MAX_DELAY);

		RF_WaitAUX(this);

		__managedDelay(this, 750);

		RF_Operating_Normal();

		__changeRFUARTBaudRate(this->SerialPort, originalBaudRate,
				originalParity);
		__managedDelay(this, 200);

		__managedDelay(this, 20);

		return E32_Success;
	}
	else{
		__debugPrintf(this, "M0 and M1 pins not set. Device's configs cannot be change./n");
		return E32_FailureMode;
	}
}

Status __getSettings(E32_433T30D* this){
	if (this->_pinConfs.M0Pin.set != 0) {
		this->tempConfig = (ConfigRF*) malloc(sizeof(ConfigRF));
		if (this->tempConfig == NULL) {
			return E32_BrokenGetSet;
		}

		__clearSerialBuffer(this);
		uint8_t SpedByte = 0, OptionByte = 0;
		uint8_t MesArr[6], sendpack[3];

		RF_WaitAUX(this);

		uint32_t originalBaudRate = this->SerialPort->Init.BaudRate;
		uint32_t originalParity = this->SerialPort->Init.Parity;
		__changeRFUARTBaudRate(this->SerialPort, 9600, UART_PARITY_NONE);

		__managedDelay(this, 200);
		RF_Operating_Config();

		__managedDelay(this, 100);

		sendpack[0] = 0xC1;
		sendpack[1] = 0xC1;
		sendpack[2] = 0xC1;

		HAL_UART_Transmit(this->SerialPort, (uint8_t*) sendpack,
				sizeof(sendpack) / sizeof(sendpack[0]), HAL_MAX_DELAY);

		long startTime = HAL_GetTick();
		uint8_t cnt = 0;
		while (cnt < sizeof(MesArr)) {
			if (HAL_GetTick() - startTime > this->_paramConfs.serialTimeout) {
				return E32_Timeout;
			}

			if (__HAL_UART_GET_FLAG(this->SerialPort, UART_FLAG_RXNE)) {
				HAL_UART_Receive(this->SerialPort, &MesArr[cnt], 1, 10);
				cnt++;
			}
		}

		/*
		 *  Eğer üstteki çalışmazsa bunu dene
		 if(__HAL_UART_GET_FLAG(this->SerialPort, UART_FLAG_RXNE)){
		 HAL_UART_Receive(this->SerialPort, MesArr, sizeof(MesArr), 10);
		 }
		 */

		this->tempConfig->AddressHigh = MesArr[1];
		this->tempConfig->AddressLow = MesArr[2];
		SpedByte = MesArr[3];
		this->tempConfig->Channel = MesArr[4];
		OptionByte = MesArr[5];

		this->tempConfig->RFSped.UARTParity = (RF_UART_PARITY) ((SpedByte >> 6)
				& 0b11);
		this->tempConfig->RFSped.UARTBaud = (RF_UART_BAUD) ((SpedByte >> 3)
				& 0b111);
		this->tempConfig->RFSped.AirDataRate =
				(RF_AIR_DATA) ((SpedByte) & 0b111);

		this->tempConfig->RFOption.TransmissionMode =
				(RF_TRANS_MODE) ((OptionByte >> 7) & 0b1);
		this->tempConfig->RFOption.IODriver = (RF_IO_MODE) ((OptionByte >> 6)
				& 0b1);
		this->tempConfig->RFOption.WirelessWakeUp = (RF_WIRELESS) ((OptionByte
				>> 3) & 0b111);
		this->tempConfig->RFOption.FECset = (RF_FEC) ((OptionByte >> 2) & 0b1);
		this->tempConfig->RFOption.TransmissionPower =
				(RF_TRANS_POWER) ((OptionByte) & 0b11);

		RF_Operating_Normal();

		__changeRFUARTBaudRate(this->SerialPort, originalBaudRate, originalParity);
		__managedDelay(this, 200);

		__managedDelay(this, 100);

		return E32_Success;
	}
	else{
		__debugPrintf(this, "M0 and M1 pins not set. Device's configs cannot be fetch./n");
		return E32_FailureMode;
	}
}

char* __getTranmissionPower(const E32_433T30D* this) {
	switch (this->tempConfig->RFOption.TransmissionPower) {
	case TRANSMISSIONPOWER_21:
		return "21 dBm";
		break;

	case TRANSMISSIONPOWER_24:
		return "24 dBm";
		break;

	case TRANSMISSIONPOWER_27:
		return "27 dBm";
		break;

	case TRANSMISSIONPOWER_30:
		return "30 dBm";
		break;
	}
	return "30 dBm";
}

char* __getFECFilter(const E32_433T30D* this) {
	switch (this->tempConfig->RFOption.FECset) {
	case FEC_ON:
		return "Aktif";
		break;

	case FEC_OFF:
		return "Devre Disi";
		break;
	}
	return "30 dBm";
}

char* __getWirelessWakeup(const E32_433T30D* this) {
	switch (this->tempConfig->RFOption.WirelessWakeUp) {
	case WIRELESSWAKEUP_250:
		return "250 ms";
		break;

	case WIRELESSWAKEUP_500:
		return "500 ms";
		break;

	case WIRELESSWAKEUP_750:
		return "750 ms";
		break;

	case WIRELESSWAKEUP_1000:
		return "1000 ms";
		break;

	case WIRELESSWAKEUP_1250:
		return "1250 ms";
		break;

	case WIRELESSWAKEUP_1500:
		return "1500 ms";
		break;

	case WIRELESSWAKEUP_1750:
		return "1750 ms";
		break;

	case WIRELESSWAKEUP_2000:
		return "2000 ms";
		break;
	}
	return "30 dBm";
}

char* __getIOMode(const E32_433T30D* this) {
	switch (this->tempConfig->RFOption.IODriver) {
	case IO_OPENDRAIN:
		return "IO Open Drain Modu";
		break;

	case IO_PUSHPULL:
		return "IO Push Pull Modu";
		break;
	}
	return "30 dBm";
}

char* __getTransmissionType(const E32_433T30D* this) {
	switch (this->tempConfig->RFOption.TransmissionMode) {
	case TRANSPARENTMODE:
		return "Seffaf Mod";
		break;

	case FIXEDMODE:
		return "Sabit Kanal Modu";
		break;

	default:
		return "Seffaf Mod";
		break;
	}
	return "30 dBm";
}

char* __getAirData(const E32_433T30D* this) {
	switch (this->tempConfig->RFSped.AirDataRate) {
	case AIRDATARATE_03k:
		return "0.3k bps";
		break;

	case AIRDATARATE_12k:
		return "1.2k bps";
		break;

	case AIRDATARATE_24k:
		return "2.4k bps (Varsayilan)";
		break;

	case AIRDATARATE_48k:
		return "4.8k bps";
		break;

	case AIRDATARATE_96k:
		return "9.6k bps";
		break;

	case AIRDATARATE_192k:
		return "19.2k bps";
		break;
	}
	return "30 dBm";
}

char* __getUARTParity(const E32_433T30D* this) {
	switch (this->tempConfig->RFSped.UARTParity) {
	case UARTPARITY_8N1:
		return "8 Bit, Parity Yok, 1 Durdurma Biti";
		break;

	case UARTPARITY_8E1:
		return "8 Bit, Çift Parity, 1 Durdurma Biti";
		break;

	case UARTPARITY_8O1:
		return "8 Bit, Tek Parity, 1 Durdurma Biti";
		break;
	}
	return "30 dBm";
}

char* __getUARTBaudRate(const E32_433T30D* this) {
	switch (this->tempConfig->RFSped.UARTBaud) {
	case UARTBAUDRATE_1200:
		return "1200 bps";
		break;

	case UARTBAUDRATE_2400:
		return "2400 bps";
		break;

	case UARTBAUDRATE_4800:
		return "4800 bps";
		break;

	case UARTBAUDRATE_9600:
		return "9600 bps (Varsayilan)";
		break;

	case UARTBAUDRATE_19200:
		return "19200 bps";
		break;

	case UARTBAUDRATE_38400:
		return "38400 bps";
		break;

	case UARTBAUDRATE_57600:
		return "57600 bps";
		break;

	case UARTBAUDRATE_115200:
		return "115200 bps";
		break;
	}
	return "30 dBm";
}

float __airDataRateEnum2Value(const E32_433T30D* this){
	switch (this->tempConfig->RFSped.AirDataRate) {
	case AIRDATARATE_03k:
		return 0.3;
	case AIRDATARATE_12k:
		return 1.2;
	case AIRDATARATE_24k:
		return 2.4;
	case AIRDATARATE_48k:
		return 4.8;
	case AIRDATARATE_96k:
		return 9.6;
	case AIRDATARATE_192k:
		return 19.2;
	default:
		return 4.8;
	}
}

uint32_t __UARTRateEnum2Value(const RF_UART_BAUD baud){
	switch (baud) {
	case UARTBAUDRATE_1200:
		return 1200;
	case UARTBAUDRATE_2400:
		return 2400;
	case UARTBAUDRATE_4800:
		return 4800;
	case UARTBAUDRATE_9600:
		return 9600;
	case UARTBAUDRATE_19200:
		return 19200;
	case UARTBAUDRATE_38400:
		return 38400;
	case UARTBAUDRATE_57600:
		return 57600;
	case UARTBAUDRATE_115200:
		return 115200;
	default:
		return 9600;
	}
}

uint32_t __UARTParityEnum2Value(const RF_UART_PARITY parity){
	switch (parity) {
	case UARTPARITY_8N1:
		return UART_PARITY_NONE;
	case UARTPARITY_8O1:
		return UART_PARITY_ODD;
	case UARTPARITY_8E1:
		return UART_PARITY_EVEN;
	default:
		return UART_PARITY_NONE;
	}
}

uint64_t  __calculatePacketSendTime(E32_433T30D* this, const size_t packetSize){
    uint16_t packetBitSize = packetSize * 8;
    uint64_t  packetTime = 0;

    // Air Data Rate Time
    packetTime += (1000 * packetBitSize) / (__airDataRateEnum2Value(this) * 1000);

    // UART Time
    packetTime += (1000 * packetBitSize) / __UARTRateEnum2Value(this->tempConfig->RFSped.UARTBaud);

    return packetTime;
}

Status __setPinConfig(E32_433T30D* this, const RF_GPIOPinData m0, const RF_GPIOPinData m1, const RF_GPIOPinData aux){
	this->_pinConfs.AUXPin = aux;
	this->_pinConfs.M0Pin = m0;
	this->_pinConfs.M1Pin = m1;

	if (this->_pinConfs.AUXPin.handle != NULL) {
		this->_pinConfs.AUXPin.set = 1;

		GPIO_InitTypeDef GPIO_Temp = { 0 };

		__HAL_RCC_GPIOH_CLK_ENABLE();
		__HAL_RCC_GPIOC_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();

		HAL_GPIO_WritePin(this->_pinConfs.AUXPin.handle, this->_pinConfs.AUXPin.pin, GPIO_PIN_RESET);

		GPIO_Temp.Pin = this->_pinConfs.AUXPin.pin;
		GPIO_Temp.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_Temp.Pull = GPIO_NOPULL;
		GPIO_Temp.Speed = GPIO_SPEED_FREQ_LOW;

		HAL_GPIO_Init(this->_pinConfs.AUXPin.handle, &GPIO_Temp);
	}

	if (this->_pinConfs.M0Pin.handle != NULL && this->_pinConfs.M1Pin.handle != NULL) {
		this->_pinConfs.M0Pin.set = 1;
		this->_pinConfs.M1Pin.set = 1;

		GPIO_InitTypeDef GPIO_Temp = {0};

		__HAL_RCC_GPIOH_CLK_ENABLE();
		__HAL_RCC_GPIOC_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();

		HAL_GPIO_WritePin(this->_pinConfs.M0Pin.handle, this->_pinConfs.M0Pin.pin|this->_pinConfs.M1Pin.pin, GPIO_PIN_RESET);

		GPIO_Temp.Pin = this->_pinConfs.M0Pin.pin|this->_pinConfs.M1Pin.pin;
		GPIO_Temp.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_Temp.Pull = GPIO_NOPULL;
		GPIO_Temp.Speed = GPIO_SPEED_FREQ_LOW;

		HAL_GPIO_Init(this->_pinConfs.M0Pin.handle, &GPIO_Temp);
	}


	return E32_Success;
}

Status __ChangeSettings(E32_433T30D* this, const uint8_t HighAddress, const uint8_t LowAddress, const uint8_t channel,
                 const RF_UART_PARITY parity, const RF_UART_BAUD baud, const RF_AIR_DATA airdata,
                 const RF_TRANS_MODE transmode, const RF_IO_MODE IOmode, const RF_WIRELESS wirelesswake,
                 const RF_FEC fecmode, const RF_TRANS_POWER transpower){

	this->_devConfs.AddHigh = HighAddress;
	this->_devConfs.AddLow = LowAddress;
	this->_devConfs.channel = channel;
	this->_devConfs.RFSped.UARTParity = parity;
	this->_devConfs.RFSped.UARTBaud = baud;
	this->_devConfs.RFSped.AirDataRate = airdata;
	this->_devConfs.RFOption.TransmissionMode = transmode;
	this->_devConfs.RFOption.IODriver = IOmode;
	this->_devConfs.RFOption.WirelessWakeUp = wirelesswake;
	this->_devConfs.RFOption.FECset = fecmode;
	this->_devConfs.RFOption.TransmissionPower = transpower;

	return E32_Success;
}

Status __packageTimerCheck(E32_433T30D* this){
	if (HAL_GetTick() <= this->_paramConfs.packetEndTimeStamp) {
		__debugPrintf(this,
				"Delay duration is not enough to send this data packet !!!/n");
		__debugPrintf(this, "Previous Packet Still Sending../n");
		__debugPrintf(this,
				"Try to increase UART or Air Data Rate. Or decrease data packet size./n");
		__debugPrintf(this,
				"Working in this config can cause unwanted delays and can cause a damage on device.../n");
		__debugPrintf(this,
				"Delay Time : %d   Previous Package's Start Time : %d/nPrevious Package's End Time : %d   Previous Package's Duration : %d/n",
				HAL_GetTick() - this->_paramConfs.packetStartTimeStamp,
				this->_paramConfs.packetStartTimeStamp,
				this->_paramConfs.packetEndTimeStamp,
				this->_paramConfs.packetEndTimeStamp
						- this->_paramConfs.packetStartTimeStamp);
		return E32_NoPackageTime;
	}
    return E32_Success;
}

Status __tempConftoDevice(E32_433T30D* this){
    this->_devConfs.RFOption = this->tempConfig->RFOption;
    this->_devConfs.RFSped = this->tempConfig->RFSped;
    this->_devConfs.AddHigh = this->tempConfig->AddressHigh;
    this->_devConfs.AddLow = this->tempConfig->AddressLow;
    this->_devConfs.channel = this->tempConfig->Channel;
    return E32_Success;
}

// Receive Fonksiyonlari tam iyi halinde degil bakilmasi lazim
Status __receiveSingleData(E32_433T30D* this, uint8_t *data){
    RF_WaitAUX(this);

    uint32_t t = HAL_GetTick();
    while(!(__HAL_UART_GET_FLAG(this->SerialPort, UART_FLAG_RXNE))){
        if(HAL_GetTick() - t > 1000){
        	__debugPrintf(this, "Veri Okuma Zaman Asimina Ugradi.../n");
            return E32_Timeout;
        }
    }

    RF_WaitAUX(this);

    HAL_UART_Receive(this->SerialPort, data, 1, HAL_MAX_DELAY);

    __clearSerialBuffer(this);
    __debugPrintf(this, "Veri Alindi.../n");
    return E32_Success;
}

Status __receiveDataPacket(E32_433T30D* this, uint8_t *data, const size_t length){
    uint32_t t = HAL_GetTick();

    uint8_t cnt = 0;
	while (cnt < length) {
		if (HAL_GetTick() - t > this->_paramConfs.serialTimeout) {
			return E32_Timeout;
		}

		if (__HAL_UART_GET_FLAG(this->SerialPort, UART_FLAG_RXNE)) {
			HAL_UART_Receive(this->SerialPort, &data[cnt], 1, 30);
			cnt++;
		}
	}

    RF_WaitAUX(this);

    uint8_t crc = 0x00;
    crc = __calculateCRC8(this, data, length);

    if(crc != data[length - 1]){
    	__debugPrintf(this, "Paket CRC Uyuşmuyor/n");
        return E32_CrcBroken;
    }

    __clearSerialBuffer(this);
    return E32_Success;
}

Status __sendFixedSingleData(E32_433T30D* this, const uint8_t AddressHigh, const uint8_t AddressLow, const uint8_t Channel, const uint8_t* data){
	RF_PackageTimerCheck(this);

    uint8_t packet[4];
    packet[0] = AddressHigh;
    packet[1] = AddressLow;
    packet[2] = Channel;
    packet[3] = *data;

    size_t packageTime = __calculatePacketSendTime(this, sizeof(packet));

    RF_WaitAUX(this);

    this->_paramConfs.packetStartTimeStamp = HAL_GetTick();
    this->_paramConfs.packetEndTimeStamp = this->_paramConfs.packetStartTimeStamp + packageTime;
    HAL_UART_Transmit(this->SerialPort, packet, sizeof(packet) / sizeof(packet[0]), 400);

    RF_WaitAUX(this);

    __clearSerialBuffer(this);

    return E32_Success;
}

Status __sendTransparentSingleData(E32_433T30D* this, const uint8_t* data){
	RF_PackageTimerCheck(this);

    RF_WaitAUX(this);

    size_t packageTime = __calculatePacketSendTime(this, 1);

    this->_paramConfs.packetStartTimeStamp = HAL_GetTick();
    this->_paramConfs.packetEndTimeStamp = this->_paramConfs.packetStartTimeStamp + packageTime;
    HAL_UART_Transmit(this->SerialPort, data, 1, 200);

    RF_WaitAUX(this);

    __clearSerialBuffer(this);

    return E32_Success;
}

Status __sendFixedDataPacket(E32_433T30D* this, const uint8_t AddressHigh, const uint8_t AddressLow, const uint8_t Channel, const uint8_t *data, const size_t size){
	RF_PackageTimerCheck(this);

    if(size > MAX_TX_BUFFER_SIZE_FIXED_CRC){
        return E32_BigPacket;
    }

    uint8_t packetSize = size + 4;

    uint8_t packet[packetSize];

    packet[0] = AddressHigh;
    packet[1] = AddressLow;
    packet[2] = Channel;
    memcpy(&packet[3], data, size);

    size_t packageTime = __calculatePacketSendTime(this, sizeof(packet));

    uint8_t crc;
    crc = __calculateCRC8(this, data, size);
    packet[packetSize - 1] = crc;

    RF_WaitAUX(this);

    this->_paramConfs.packetStartTimeStamp = HAL_GetTick();
    this->_paramConfs.packetEndTimeStamp = this->_paramConfs.packetStartTimeStamp + packageTime;
    HAL_UART_Transmit(this->SerialPort, packet, sizeof(packet), 400);

    RF_WaitAUX(this);

    __managedDelay(this, 5);

    __clearSerialBuffer(this);

    return E32_Success;
}

Status __sendBroadcastDataPacket(E32_433T30D* this, const uint8_t Channel, const uint8_t *data, const size_t size){
	return __sendFixedDataPacket(this, 0x00, 0x00, Channel, data, size);
}

Status __sendTransparentDataPacket(E32_433T30D* this, uint8_t *data, const size_t size){
	RF_PackageTimerCheck(this);

    if(size > MAX_TX_BUFFER_SIZE_CRC){
        return E32_BigPacket;
    }

    uint8_t packetSize = size + 1;

    uint8_t packet[packetSize];
    memcpy(packet, data, size);

    uint8_t crc = __calculateCRC8(this, packet, size);
    packet[size] = crc;

    size_t packageTime = __calculatePacketSendTime(this, sizeof(packet));

    RF_WaitAUX(this);

    this->_paramConfs.packetStartTimeStamp = HAL_GetTick();
    this->_paramConfs.packetEndTimeStamp = this->_paramConfs.packetStartTimeStamp + packageTime;
    HAL_UART_Transmit(this->SerialPort, packet, sizeof(packet), 400);

    RF_WaitAUX(this);

    __managedDelay(this, 5);

    __clearSerialBuffer(this);

    return E32_Success;
}

Status __setTransmissionMode(E32_433T30D* this, const RF_TRANS_MODE Mode){
    switch (Mode)
    {
    case TRANSPARENTMODE:
        this->_devConfs.RFOption.TransmissionMode = TRANSPARENTMODE;
        break;

    case FIXEDMODE:
        this->_devConfs.RFOption.TransmissionMode = FIXEDMODE;
        break;

    case BROADCASTMODE:
        this->_devConfs.RFOption.TransmissionMode = FIXEDMODE;
        break;

    default:
        this->_devConfs.RFOption.TransmissionMode = TRANSPARENTMODE;
        return E32_FailureMode;
    }

    return E32_Success;
}

Status __setAddresses(E32_433T30D* this, const uint8_t AddHigh, const uint8_t AddLow){
    if(AddHigh > MAX_FREQ_VAL || AddHigh < MIN_FREQ_VAL){
        return E32_OutOfLimit;
    }
    if(AddLow > MAX_FREQ_VAL || AddLow < MIN_FREQ_VAL){
        return E32_OutOfLimit;
    }

    this->_devConfs.AddHigh = AddHigh;
    this->_devConfs.AddLow = AddLow;
    return E32_Success;
}

Status __setChannel(E32_433T30D* this, const RF_FREQ channel){
    if(channel > FREQ_441 || channel < FREQ_410){
        return E32_OutOfLimit;
    }

    this->_devConfs.channel = channel;
    return E32_Success;
}

Status __setTransmissionPower(E32_433T30D* this, const RF_TRANS_POWER power){
    if(power > TRANSMISSIONPOWER_21 || power < TRANSMISSIONPOWER_30){
        return E32_FailureMode;
    }

    this->_devConfs.RFOption.TransmissionPower = power;
    return E32_Success;
}

Status __setIODriver(E32_433T30D* this, const RF_IO_MODE driver){
    if(driver > IO_OPENDRAIN || driver < IO_PUSHPULL){
        return E32_FailureMode;
    }

    this->_devConfs.RFOption.IODriver = driver;
    return E32_Success;
}

Status __setFECSettings(E32_433T30D* this, const RF_FEC fecmode){
    if(fecmode > FEC_ON || fecmode < FEC_OFF){
        return E32_FailureMode;
    }

    this->_devConfs.RFOption.FECset = fecmode;
    return E32_Success;
}

Status __setWirelesWakeup(E32_433T30D* this, const RF_WIRELESS time){
    if(!(time > WIRELESSWAKEUP_2000 || time < WIRELESSWAKEUP_250)){
        return E32_Success;
    }

    this->_devConfs.RFOption.WirelessWakeUp = time;
    return E32_Success;
}

Status __setUARTParity(E32_433T30D* this, const RF_UART_PARITY paritybyte){
    if(paritybyte > UARTPARITY_8E1 || paritybyte < UARTPARITY_8N1){
        return E32_FailureMode;
    }

    this->_devConfs.RFSped.UARTParity = paritybyte;
    return E32_Success;
}

Status __setUARTBaudRate(E32_433T30D* this, const RF_UART_BAUD baudrate){
    if(baudrate > UARTBAUDRATE_115200 || baudrate < UARTBAUDRATE_1200){
        return E32_FailureMode;
    }

    this->_devConfs.RFSped.UARTBaud = baudrate;
    return E32_Success;
}

Status __setDebuggerUARTParity(E32_433T30D* this, const RF_UART_PARITY paritybyte){
    if(paritybyte > UARTPARITY_8E1 || paritybyte < UARTPARITY_8N1){
        return E32_FailureMode;
    }

    this->_debuggerConfs.parity = paritybyte;
    return E32_Success;
}

Status __setDebuggerUARTBaudRate(E32_433T30D* this, const RF_UART_BAUD baudrate){
    if(baudrate > UARTBAUDRATE_115200 || baudrate < UARTBAUDRATE_1200){
        return E32_FailureMode;
    }

    this->_debuggerConfs.baudRate = baudrate;
    return E32_Success;
}

Status __setAirDataRate(E32_433T30D* this, const RF_AIR_DATA airdatarate){
    if(airdatarate > AIRDATARATE_192k || airdatarate < AIRDATARATE_03k){
        return E32_FailureMode;
    }

    this->_devConfs.RFSped.AirDataRate = airdatarate;
    return E32_Success;
}

Status __setAuxTimeoutTime(E32_433T30D* this, const size_t time){
    if(time < 30){
    	this->_paramConfs.auxTimeout = 30;
        return E32_FailureMode;
    }

    this->_paramConfs.auxTimeout = time;
    return E32_Success;
}

Status __setNoAuxTimeoutTime(E32_433T30D* this, const size_t time){
    if(time < 30){
    	this->_paramConfs.noAuxTimeOut = 30;
        return E32_FailureMode;
    }

    this->_paramConfs.noAuxTimeOut = time;
    return E32_Success;
}

void __viewSettings(E32_433T30D* this){
	__debugPrintf(this, "------------------------------------------------------/n");
	__debugPrintf(this, "Yuksek Adres: %d/n", this->_devConfs.AddHigh);
	__debugPrintf(this, "Dusuk Adres: %d/n", this->_devConfs.AddLow);

	__debugPrintf(this, "Kanal: %d - %d MHz/n/n", this->_devConfs.channel, this->_devConfs.channel + 410);

	__debugPrintf(this, "Sped Ayarlari/n");
	__debugPrintf(this, "  UART Baud Rate: %s/n", __getUARTBaudRate(this));
	__debugPrintf(this, "  UART Parity: %s/n", __getUARTParity(this));
	__debugPrintf(this, "  Air Data Rate: %s/n/n", __getAirData(this));

	__debugPrintf(this, "Option Ayarlari/n");
	__debugPrintf(this, "  Transfer Turu: %s/n", __getTransmissionType(this));
	__debugPrintf(this, "  IO Turu: %s/n", __getIOMode(this));
	__debugPrintf(this, "  Wireless Uyanma Suresi: %s/n", __getWirelessWakeup(this));
	__debugPrintf(this, "  FEC Filtresi: %s/n", __getFECFilter(this));
	__debugPrintf(this, "  Aktarim Gucu: %s/n", __getTranmissionPower(this));
	__debugPrintf(this, "------------------------------------------------------/n");
}

void __setFunctionPointers(E32_433T30D* this){
	this->calculateCRC8 = __calculateCRC8;
	this->receiveSingleData = __receiveSingleData;
	this->receiveDataPacket = __receiveDataPacket;
	this->sendFixedSingleData = __sendFixedSingleData;
	this->sendTransparentSingleData = __sendTransparentSingleData;
	this->sendFixedDataPacket = __sendFixedDataPacket;
	this->sendBroadcastDataPacket = __sendBroadcastDataPacket;
	this->sendTransparentDataPacket = __sendTransparentDataPacket;
	this->setTransmissionMode = __setTransmissionMode;
	this->setAddresses = __setAddresses;
	this->setChannel = __setChannel;
	this->setTransmissionPower = __setTransmissionPower;
	this->setIODriver = __setIODriver;
	this->setFECSettings = __setFECSettings;
	this->setWirelesWakeup = __setWirelesWakeup;
	this->setUARTParity = __setUARTParity;
	this->setUARTBaudRate = __setUARTBaudRate;
	this->setDebuggerUARTParity = __setDebuggerUARTParity;
	this->setDebuggerUARTBaudRate = __setDebuggerUARTBaudRate;
	this->setAirDataRate = __setAirDataRate;
	this->setAuxTimeoutTime = __setAuxTimeoutTime;
	this->setNoAuxTimeoutTime = __setNoAuxTimeoutTime;
	this->viewSettings = __viewSettings;
}

void __setInitialValues(E32_433T30D *this){
	this->SerialPort = NULL;
	this->DebugPort = NULL;
	this->_paramConfs.auxTimeout = 1000;
	this->_paramConfs.noAuxTimeOut = 30;
	this->_paramConfs.serialTimeout = 1000;
	this->_paramConfs.packetStartTimeStamp = 0;
	this->_paramConfs.packetEndTimeStamp = 0;
	this->_debuggerConfs.baudRate = 9600;
	this->_debuggerConfs.parity = DEBUGGER_UART_PARITY_8N1;
	this->_devConfs.RFSped.UARTParity = UARTPARITY_8N1;
	this->_devConfs.RFSped.UARTBaud = UARTBAUDRATE_9600;
	this->_devConfs.RFSped.AirDataRate = AIRDATARATE_24k;
	this->_devConfs.RFOption.TransmissionMode = TRANSPARENTMODE;
	this->_devConfs.RFOption.IODriver = IO_PUSHPULL;
	this->_devConfs.RFOption.WirelessWakeUp = WIRELESSWAKEUP_250;
	this->_devConfs.RFOption.FECset = FEC_ON;
	this->_devConfs.RFOption.TransmissionPower = TRANSMISSIONPOWER_30;
	this->_devConfs.AddHigh = 0;
	this->_devConfs.AddLow = 0;
	this->_devConfs.channel = FREQ_433;
	this->tempConfig = NULL;
}

void __setRFUARTPort(E32_433T30D *this){
	__changeRFUARTBaudRate(this->SerialPort, __UARTRateEnum2Value(this->_devConfs.RFSped.UARTBaud), __UARTParityEnum2Value(this->_devConfs.RFSped.UARTParity));
}

void __setDebuggerUARTPort(E32_433T30D* this) {
	__changeRFUARTBaudRate(this->DebugPort, this->_debuggerConfs.baudRate, this->_debuggerConfs.parity);
}

Status RFInit_NoPin(E32_433T30D* handler, UART_HandleTypeDef *RFPort){
	__setInitialValues(handler);
	__setFunctionPointers(handler);

	RF_GPIOPinData m0 = {NULL, 0, 0};
	RF_GPIOPinData m1 = {NULL, 0, 0};
	RF_GPIOPinData aux = {NULL, 0, 0};

	__setPinConfig(handler, m0, m1, aux);

	handler->SerialPort = RFPort;
	return E32_Success;
}

Status RFInit(E32_433T30D* handler, UART_HandleTypeDef *RFPort, RF_GPIOPins pinData){
	__setInitialValues(handler);
	__setFunctionPointers(handler);
	__setPinConfig(handler, pinData.M0Pin, pinData.M1Pin, pinData.AUXPin);
	handler->SerialPort = RFPort;
	return E32_Success;
}

Status RFInit_Debugger(E32_433T30D* handler, UART_HandleTypeDef *RFPort, UART_HandleTypeDef *debuggerPort, RF_GPIOPins pinData){
	__setInitialValues(handler);
	__setFunctionPointers(handler);
	__setPinConfig(handler, pinData.M0Pin, pinData.M1Pin, pinData.AUXPin);
	handler->SerialPort = RFPort;
	handler->DebugPort = debuggerPort;
	return E32_Success;
}

Status RFClose(E32_433T30D *handler){
	__debugPrintf(handler, "RF is Closing/n");
	if(handler->tempConfig != NULL){
		free(handler->tempConfig);
		handler->tempConfig = NULL;
	}
	return E32_Success;
}

Status RFStart(E32_433T30D *handler){
	__setRFUARTPort(handler);
	if (handler->DebugPort != NULL){
		__setDebuggerUARTPort(handler);
	}

	__managedDelay(handler, 50);

	__setSettings(handler);

	__managedDelay(handler, 100);

	__getSettings(handler);

	return E32_Success;
}

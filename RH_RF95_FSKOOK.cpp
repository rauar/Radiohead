// RH_RF95_FSKOOK.cpp
// Author: Mike McCauley (mikem@airspayce.com)
// Author: Alex Rau (alex@retroduction.org)
// Copyright (C) 2014 Mike McCauley
// Copyright (C) 2024 Alex Rau

#include <RH_RF95_FSKOOK.h>

// Interrupt vectors for the 3 Arduino interrupt pins
// Each interrupt can be handled by a different instance of RH_RF95, allowing you to have
// 2 or more RF95s per Arduino
RH_RF95_FSKOOK* RH_RF95_FSKOOK::_deviceForInterrupt[RH_RF95_NUM_INTERRUPTS] = {0, 0, 0};
uint8_t RH_RF95_FSKOOK::_interruptCount = 0; // Index into _deviceForInterrupt for next device

// These are indexed by the values of ModemConfigChoice
// Stored in flash (program) memory to save SRAM
// It is important to keep the modulation index for FSK between 0.5 and 10
// modulation index = 2 * Fdev / BR
// Note that I have not had much success with FSK with Fd > ~5
// You have to construct these by hand, using the data from the RF95 Datasheet :-(
// or use the SX1231 starter kit software (Ctl-Alt-N to use that without a connected radio)

#define PACKETCONFIG1 ( RH_RF95_PACKETCONFIG1_PACKETFORMAT_VARIABLE | RH_RF95_PACKETCONFIG1_DCFREE_MANCHESTER | RH_RF95_PACKETCONFIG1_CRC_ON | RH_RF95_PACKETCONFIG1_ADDRESSFILTERING_NONE )

PROGMEM static const RH_RF95_FSKOOK::ModemConfig MODEM_CONFIG_TABLE[] =
{
    // 01,                                02,         03,         04,      05,      0A,                                         12,   13,    30,            31
    // FSK, No Manchester, no shaping, whitening, CRC, no address filtering
    // AFC BW == RX BW == 2 x bit rate
    // Low modulation indexes of ~ 1 at slow speeds do not seem to work very well. Choose MI of 2.

    // MODULATIONTYPE,                    BITRATEMSB, BITRATELSB, FDEVMSB, FDEVLSB, PARAMP,                                     RXBW, AFCBW, PACKETCONFIG1, PACKETCONFIG2
    {  RH_RF95_OPMODE_MODULATIONTYPE_FSK, 0x3e,       0x80,       0x00,    0x52,    RH_RF95_PARAMP_MODULATIONSHAPING_FSK_NONE,  0xf4, 0xf4,  PACKETCONFIG1, RH_RF95_PACKETCONFIG2_DATAMODE_PACKET}, // FSK_Rb2Fd5      
    {  RH_RF95_OPMODE_MODULATIONTYPE_FSK, 0x34,       0x15,       0x00,    0x4f,    RH_RF95_PARAMP_MODULATIONSHAPING_FSK_NONE,  0xf4, 0xf4,  PACKETCONFIG1, RH_RF95_PACKETCONFIG2_DATAMODE_PACKET}, // FSK_Rb2_4Fd4_8
    {  RH_RF95_OPMODE_MODULATIONTYPE_FSK, 0x1a,       0x0b,       0x00,    0x9d,    RH_RF95_PARAMP_MODULATIONSHAPING_FSK_NONE,  0xf4, 0xf4,  PACKETCONFIG1, RH_RF95_PACKETCONFIG2_DATAMODE_PACKET}, // FSK_Rb4_8Fd9_6
    {  RH_RF95_OPMODE_MODULATIONTYPE_FSK, 0x0d,       0x05,       0x01,    0x3b,    RH_RF95_PARAMP_MODULATIONSHAPING_FSK_NONE,  0xf4, 0xf4,  PACKETCONFIG1, RH_RF95_PACKETCONFIG2_DATAMODE_PACKET}, // FSK_Rb9_6Fd19_2
    {  RH_RF95_OPMODE_MODULATIONTYPE_FSK, 0x06,       0x83,       0x02,    0x75,    RH_RF95_PARAMP_MODULATIONSHAPING_FSK_NONE,  0xf3, 0xf3,  PACKETCONFIG1, RH_RF95_PACKETCONFIG2_DATAMODE_PACKET}, // FSK_Rb19_2Fd38_4
    {  RH_RF95_OPMODE_MODULATIONTYPE_FSK, 0x03,       0x41,       0x04,    0xea,    RH_RF95_PARAMP_MODULATIONSHAPING_FSK_NONE,  0xf2, 0xf2,  PACKETCONFIG1, RH_RF95_PACKETCONFIG2_DATAMODE_PACKET}, // FSK_Rb38_4Fd76_8
    {  RH_RF95_OPMODE_MODULATIONTYPE_FSK, 0x02,       0x2c,       0x07,    0xae,    RH_RF95_PARAMP_MODULATIONSHAPING_FSK_NONE,  0xe2, 0xe2,  PACKETCONFIG1, RH_RF95_PACKETCONFIG2_DATAMODE_PACKET}, // FSK_Rb57_6Fd120
    {  RH_RF95_OPMODE_MODULATIONTYPE_FSK, 0x01,       0x00,       0x08,    0x00,    RH_RF95_PARAMP_MODULATIONSHAPING_FSK_NONE,  0xe1, 0xe1,  PACKETCONFIG1, RH_RF95_PACKETCONFIG2_DATAMODE_PACKET}, // FSK_Rb125Fd125
    {  RH_RF95_OPMODE_MODULATIONTYPE_FSK, 0x00,       0x80,       0x10,    0x00,    RH_RF95_PARAMP_MODULATIONSHAPING_FSK_NONE,  0xe0, 0xe0,  PACKETCONFIG1, RH_RF95_PACKETCONFIG2_DATAMODE_PACKET}, // FSK_Rb250Fd250
    {  RH_RF95_OPMODE_MODULATIONTYPE_FSK, 0x02,       0x40,       0x03,    0x33,    RH_RF95_PARAMP_MODULATIONSHAPING_FSK_NONE,  0x42, 0x42,  PACKETCONFIG1, RH_RF95_PACKETCONFIG2_DATAMODE_PACKET}, // FSK_Rb55555Fd50 

    //  02,        03,   04,   05,   06,   19,   1a,  37
    // GFSK (BT=1.0), No Manchester, whitening, CRC, no address filtering
    // AFC BW == RX BW == 2 x bit rate

    // MODULATIONTYPE,                    BITRATEMSB, BITRATELSB, FDEVMSB, FDEVLSB, PARAMP,                                     RXBW, AFCBW, PACKETCONFIG1,     PACKETCONFIG2
    {  RH_RF95_OPMODE_MODULATIONTYPE_FSK, 0x3e,       0x80,       0x00,    0x52,    RH_RF95_PARAMP_MODULATIONSHAPING_FSK_BT1_0, 0xf4, 0xf4,  PACKETCONFIG1, RH_RF95_PACKETCONFIG2_DATAMODE_PACKET}, // GFSK_Rb2Fd5
    {  RH_RF95_OPMODE_MODULATIONTYPE_FSK, 0x34,       0x15,       0x00,    0x4f,    RH_RF95_PARAMP_MODULATIONSHAPING_FSK_BT1_0, 0xf4, 0xf4,  PACKETCONFIG1, RH_RF95_PACKETCONFIG2_DATAMODE_PACKET}, // GFSK_Rb2_4Fd4_8
    {  RH_RF95_OPMODE_MODULATIONTYPE_FSK, 0x1a,       0x0b,       0x00,    0x9d,    RH_RF95_PARAMP_MODULATIONSHAPING_FSK_BT1_0, 0xf4, 0xf4,  PACKETCONFIG1, RH_RF95_PACKETCONFIG2_DATAMODE_PACKET}, // GFSK_Rb4_8Fd9_6
    {  RH_RF95_OPMODE_MODULATIONTYPE_FSK, 0x0d,       0x05,       0x01,    0x3b,    RH_RF95_PARAMP_MODULATIONSHAPING_FSK_BT1_0, 0xf4, 0xf4,  PACKETCONFIG1, RH_RF95_PACKETCONFIG2_DATAMODE_PACKET}, // GFSK_Rb9_6Fd19_2
    {  RH_RF95_OPMODE_MODULATIONTYPE_FSK, 0x06,       0x83,       0x02,    0x75,    RH_RF95_PARAMP_MODULATIONSHAPING_FSK_BT1_0, 0xf3, 0xf3,  PACKETCONFIG1, RH_RF95_PACKETCONFIG2_DATAMODE_PACKET}, // GFSK_Rb19_2Fd38_4
    {  RH_RF95_OPMODE_MODULATIONTYPE_FSK, 0x03,       0x41,       0x04,    0xea,    RH_RF95_PARAMP_MODULATIONSHAPING_FSK_BT1_0, 0xf2, 0xf2,  PACKETCONFIG1, RH_RF95_PACKETCONFIG2_DATAMODE_PACKET}, // GFSK_Rb38_4Fd76_8
    {  RH_RF95_OPMODE_MODULATIONTYPE_FSK, 0x02,       0x2c,       0x07,    0xae,    RH_RF95_PARAMP_MODULATIONSHAPING_FSK_BT1_0, 0xe2, 0xe2,  PACKETCONFIG1, RH_RF95_PACKETCONFIG2_DATAMODE_PACKET}, // GFSK_Rb57_6Fd120
    {  RH_RF95_OPMODE_MODULATIONTYPE_FSK, 0x01,       0x00,       0x08,    0x00,    RH_RF95_PARAMP_MODULATIONSHAPING_FSK_BT1_0, 0xe1, 0xe1,  PACKETCONFIG1, RH_RF95_PACKETCONFIG2_DATAMODE_PACKET}, // GFSK_Rb125Fd125
    {  RH_RF95_OPMODE_MODULATIONTYPE_FSK, 0x00,       0x80,       0x10,    0x00,    RH_RF95_PARAMP_MODULATIONSHAPING_FSK_BT1_0, 0xe0, 0xe0,  PACKETCONFIG1, RH_RF95_PACKETCONFIG2_DATAMODE_PACKET}, // GFSK_Rb250Fd250
    {  RH_RF95_OPMODE_MODULATIONTYPE_FSK, 0x02,       0x40,       0x03,    0x33,    RH_RF95_PARAMP_MODULATIONSHAPING_FSK_BT1_0, 0x42, 0x42,  PACKETCONFIG1, RH_RF95_PACKETCONFIG2_DATAMODE_PACKET}, // GFSK_Rb55555Fd50 

    //  02,        03,   04,   05,   06,   19,   1a,  37
    // OOK, No Manchester, no shaping, whitening, CRC, no address filtering
    // with the help of the SX1231 configuration program
    // AFC BW == RX BW
    // All OOK configs have the default:
    // Threshold Type: Peak
    // Peak Threshold Step: 0.5dB
    // Peak threshiold dec: ONce per chip
    // Fixed threshold: 6dB

    // MODULATIONTYPE,                    BITRATEMSB, BITRATELSB, FDEVMSB, FDEVLSB, PARAMP,                                   RXBW, AFCBW, PACKETCONFIG1,     PACKETCONFIG2
    { RH_RF95_OPMODE_MODULATIONTYPE_OOK,  0x7d,       0x00,       0x00,    0x10,    RH_RF95_PARAMP_MODULATIONSHAPING_OOK_2BR, 0x88, 0x88, PACKETCONFIG1, RH_RF95_PACKETCONFIG2_DATAMODE_PACKET}, // OOK_Rb1Bw1
    { RH_RF95_OPMODE_MODULATIONTYPE_OOK,  0x68,       0x2b,       0x00,    0x10,    RH_RF95_PARAMP_MODULATIONSHAPING_OOK_2BR, 0xf1, 0xf1, PACKETCONFIG1, RH_RF95_PACKETCONFIG2_DATAMODE_PACKET}, // OOK_Rb1_2Bw75
    { RH_RF95_OPMODE_MODULATIONTYPE_OOK,  0x34,       0x15,       0x00,    0x10,    RH_RF95_PARAMP_MODULATIONSHAPING_OOK_2BR, 0xf5, 0xf5, PACKETCONFIG1, RH_RF95_PACKETCONFIG2_DATAMODE_PACKET}, // OOK_Rb2_4Bw4_8
    { RH_RF95_OPMODE_MODULATIONTYPE_OOK,  0x1a,       0x0b,       0x00,    0x10,    RH_RF95_PARAMP_MODULATIONSHAPING_OOK_2BR, 0xf4, 0xf4, PACKETCONFIG1, RH_RF95_PACKETCONFIG2_DATAMODE_PACKET}, // OOK_Rb4_8Bw9_6
    { RH_RF95_OPMODE_MODULATIONTYPE_OOK,  0x0d,       0x05,       0x00,    0x10,    RH_RF95_PARAMP_MODULATIONSHAPING_OOK_2BR, 0xf3, 0xf3, PACKETCONFIG1, RH_RF95_PACKETCONFIG2_DATAMODE_PACKET}, // OOK_Rb9_6Bw19_2
    { RH_RF95_OPMODE_MODULATIONTYPE_OOK,  0x06,       0x83,       0x00,    0x10,    RH_RF95_PARAMP_MODULATIONSHAPING_OOK_2BR, 0xf2, 0xf2, PACKETCONFIG1, RH_RF95_PACKETCONFIG2_DATAMODE_PACKET}, // OOK_Rb19_2Bw38_4
    { RH_RF95_OPMODE_MODULATIONTYPE_OOK,  0x03,       0xe8,       0x00,    0x10,    RH_RF95_PARAMP_MODULATIONSHAPING_OOK_2BR, 0xe2, 0xe2, PACKETCONFIG1, RH_RF95_PACKETCONFIG2_DATAMODE_PACKET}, // OOK_Rb32Bw64

};
RH_RF95_FSKOOK::RH_RF95_FSKOOK(uint8_t slaveSelectPin, uint8_t interruptPin, RHGenericSPI& spi)
    :
    RHSPIDriver(slaveSelectPin, spi)
{
    _interruptPin = interruptPin;
    _idleMode = RH_RF95_MODE_STDBY;
    _myInterruptIndex = 0xff; // Not allocated yet
}

void RH_RF95_FSKOOK::setIdleMode(uint8_t idleMode)
{
    _idleMode = idleMode;
}

bool RH_RF95_FSKOOK::init()
{
    if (!RHSPIDriver::init())
	    return false;

    // Determine the interrupt number that corresponds to the interruptPin
    int interruptNumber = digitalPinToInterrupt(_interruptPin);
    if (interruptNumber == NOT_AN_INTERRUPT)
	    return false;

#ifdef RH_ATTACHINTERRUPT_TAKES_PIN_NUMBER
    interruptNumber = _interruptPin;
#endif

    // Tell the low level SPI interface we will use SPI within this interrupt
    spiUsingInterrupt(interruptNumber);

    // Get the device type and check it
    // This also tests whether we are really connected to a device
    // My test devices return 0x12
    _deviceType = spiRead(RH_RF95_REG_42_VERSION);

#ifdef RH_RF95_HAVE_SERIAL
    Serial.print("Chip version: ");
    Serial.println(_deviceType);
#endif

    if (_deviceType == 00 || _deviceType == 0xff)
	    return false;

    // Add by Adrien van den Bossche <vandenbo@univ-tlse2.fr> for Teensy
    // ARM M4 requires the below. else pin interrupt doesn't work properly.
    // On all other platforms, its innocuous, belt and braces
    pinMode(_interruptPin, INPUT); 

    // Set up interrupt handler
    // Since there are a limited number of interrupt glue functions isr*() available,
    // we can only support a limited number of devices simultaneously
    // ON some devices, notably most Arduinos, the interrupt pin passed in is actually the 
    // interrupt number. You have to figure out the interruptnumber-to-interruptpin mapping
    // yourself based on knowledge of what Arduino board you are running on.
    if (_myInterruptIndex == 0xff)
    {
	// First run, no interrupt allocated yet
	if (_interruptCount <= RH_RF95_NUM_INTERRUPTS)
	    _myInterruptIndex = _interruptCount++;
	else
	    return false; // Too many devices, not enough interrupt vectors
    }
    _deviceForInterrupt[_myInterruptIndex] = this;
    if (_myInterruptIndex == 0)
	    attachInterrupt(interruptNumber, isr0, RISING);
    else if (_myInterruptIndex == 1)
	    attachInterrupt(interruptNumber, isr1, RISING);
    else if (_myInterruptIndex == 2)
	    attachInterrupt(interruptNumber, isr2, RISING);
    else
	    return false; // Too many devices, not enough interrupt vectors

    setModeIdle();

    // Configure important RH_RF95 registers
    // Here we set up the standard packet format for use by the RH_RF95 library:
    // 4 bytes preamble
    // 2 SYNC words 2d, d4
    // 2 CRC CCITT octets computed on the header, length and data (this in the modem config data)
    // 0 to 60 bytes data
    // RSSI Threshold -114dBm
    // We dont use the RH_RF95s address filtering: instead we prepend our own headers to the beginning
    // of the RH_RF95 payload

    // The following can be changed later by the user if necessary.
    // Set up default configuration

    setModemConfig(GFSK_Rb9_6Fd19_2);
    setPreambleLength(4); // RF22 default
    uint8_t syncwords[] = { 0x2d, 0xd4 };
    setSyncWords(syncwords, sizeof(syncwords));// RF22 default
    setShouldSendHeaderInPayload(true);
    setVariableMessageLength();
    setFrequency(434.0);
    setTxPower(2); 

    spiWrite(RH_RF95_REG_35_FIFOTHRESH, RH_RF95_FIFOTHRESH_TXSTARTCONDITION_NOTEMPTY | 0x0F);

    return true;
}

void RH_RF95_FSKOOK::setFixedMessageLength(uint8_t msgLen)
{

    Serial.print("Switching to fixed length mode with length ");
    Serial.println(msgLen);

    _fixedLengthMode = true;

    uint8_t packetConfig1Reg = spiRead(RH_RF95_REG_30_PACKETCONFIG1);
    Serial.print("PacketConfig1 (before): ");
    Serial.println(packetConfig1Reg);
    Serial.print("PacketLength (before): ");
    Serial.println(spiRead(RH_RF95_REG_32_PAYLOADLENGTH));
    packetConfig1Reg &= ~RH_RF95_PACKETCONFIG1_PACKETFORMAT;
    packetConfig1Reg |= (RH_RF95_PACKETCONFIG1_PACKETFORMAT_FIXED);
    spiWrite(RH_RF95_REG_30_PACKETCONFIG1, packetConfig1Reg);
    spiWrite(RH_RF95_REG_32_PAYLOADLENGTH, msgLen);
    Serial.print("PacketConfig1 (verify): ");
    Serial.println(spiRead(RH_RF95_REG_30_PACKETCONFIG1));
    Serial.print("PacketConfig2 (verify): ");
    Serial.println(spiRead(RH_RF95_REG_31_PACKETCONFIG2));
    Serial.print("PacketLength (verify): ");
    Serial.println(spiRead(RH_RF95_REG_32_PAYLOADLENGTH));

}

void RH_RF95_FSKOOK::setVariableMessageLength()
{
    Serial.println("Switching to variable length mode...");
    
    _fixedLengthMode = false;
    
    uint8_t packetConfig1Reg = spiRead(RH_RF95_REG_30_PACKETCONFIG1);
    packetConfig1Reg &= ~RH_RF95_PACKETCONFIG1_PACKETFORMAT;
    packetConfig1Reg |= (RH_RF95_PACKETCONFIG1_PACKETFORMAT_VARIABLE);
    spiWrite(RH_RF95_REG_30_PACKETCONFIG1, packetConfig1Reg);
    spiWrite(RH_RF95_REG_32_PAYLOADLENGTH, 0);

}

void RH_RF95_FSKOOK::setShouldSendHeaderInPayload(bool value) {
    _shouldSendHeaderInPayload = value;
}

void RH_RF95_FSKOOK::setEncoding(WhiteningType value) {

    uint8_t packetConfig1Reg = spiRead(RH_RF95_REG_30_PACKETCONFIG1);

    packetConfig1Reg &= ~RH_RF95_PACKETCONFIG1_DCFREE;
    packetConfig1Reg &= ~RH_RF95_PACKETCONFIG1_CRCWHITENINGTYPE;

    if ( value == DC_NONE) {
        packetConfig1Reg |= (RH_RF95_PACKETCONFIG1_DCFREE_NONE);
    } else if (value == DC_MANCHESTER) {
        packetConfig1Reg |= (RH_RF95_PACKETCONFIG1_DCFREE_MANCHESTER);
    } else if (value == DC_WHITENING_CCITT) {
        packetConfig1Reg |= (RH_RF95_PACKETCONFIG1_DCFREE_WHITENING);
        packetConfig1Reg |= (RH_RF95_PACKETCONFIG1_CRCWHITENINGTYPE_CCITT);
    } else if (value == DC_WHITENING_IBM){
        packetConfig1Reg |= (RH_RF95_PACKETCONFIG1_DCFREE_WHITENING);
        packetConfig1Reg |= (RH_RF95_PACKETCONFIG1_CRCWHITENINGTYPE_IBM);
    }

    spiWrite(RH_RF95_REG_30_PACKETCONFIG1, packetConfig1Reg);
}

// C++ level interrupt handler for this instance
// RH_RF95 is unusual in that it has several interrupt lines, and not a single, combined one.
// On Moteino, only one of the several interrupt lines (DI0) from the RH_RF95 is connnected to the processor.
// We use the single interrupt line to get PACKETSENT and PAYLOADREADY interrupts.
void RH_RF95_FSKOOK::handleInterrupt()
{

    // Get the interrupt cause
    uint8_t irqflags2 = spiRead(RH_RF95_REG_3F_IRQFLAGS2);
    //Serial.print("Interrupt register: ");
    //Serial.println("Got interrupt...");
    //Serial.println(irqflags2);

    if (_mode == RHModeTx && (irqflags2 & RH_RF95_IRQFLAGS2_PACKETSENT))
    {
        // A transmitter message has been fully sent
        setModeIdle(); // Clears FIFO
        _txGood++;
    	Serial.println("PACKETSENT");
    }

    // Must look for PAYLOADREADY, not CRCOK, since only PAYLOADREADY occurs _after_ AES decryption
    // has been done
    if (_mode == RHModeRx && (irqflags2 & RH_RF95_IRQFLAGS2_PAYLOADREADY))
    {
        // A complete message has been received with good CRC
        _lastRssi = -((int8_t)(spiRead(RH_RF95_REG_11_RSSIVALUE) >> 1));
        _lastPreambleTime = millis();

        setModeIdle();
        // Save it in our buffer
        readFifo();
    	Serial.println("PAYLOADREADY");
    }
}

// Low level function reads the FIFO and checks the address
void RH_RF95_FSKOOK::readFifo()
{
    ATOMIC_BLOCK_START;
    _spi.beginTransaction();
    digitalWrite(_slaveSelectPin, LOW);
    _spi.transfer(RH_RF95_REG_00_FIFO); // Send the start address with the write mask off
    uint8_t payloadlen = _spi.transfer(0); // First byte is payload len (counting the headers)
    if (payloadlen <= RH_RF95_MAX_MESSAGE_LEN &&
	    (payloadlen >= RH_RF95_HEADER_LEN || !_shouldSendHeaderInPayload))
    {
        if (_shouldSendHeaderInPayload)
            _rxHeaderTo = _spi.transfer(0);

        // Check addressing
        if (_promiscuous ||
            _rxHeaderTo == _thisAddress ||
            _rxHeaderTo == RH_BROADCAST_ADDRESS)
        {
            if ( _shouldSendHeaderInPayload) {
                // Get the rest of the headers
                _rxHeaderFrom  = _spi.transfer(0);
                _rxHeaderId    = _spi.transfer(0);
                _rxHeaderFlags = _spi.transfer(0);
            }
            // And now the real payload
            for (_bufLen = 0; _bufLen < (payloadlen - RH_RF95_HEADER_LEN); _bufLen++)
            _buf[_bufLen] = _spi.transfer(0);
            _rxGood++;
            _rxBufValid = true;
        }
    }
    digitalWrite(_slaveSelectPin, HIGH);
    _spi.endTransaction();
    ATOMIC_BLOCK_END;
    // Any junk remaining in the FIFO will be cleared next time we go to receive mode.

}

// These are low level functions that call the interrupt handler for the correct
// instance of RH_RF95.
// 3 interrupts allows us to have 3 different devices
void RH_INTERRUPT_ATTR RH_RF95_FSKOOK::isr0()
{
    if (_deviceForInterrupt[0])
	_deviceForInterrupt[0]->handleInterrupt();
}
void RH_INTERRUPT_ATTR RH_RF95_FSKOOK::isr1()
{
    if (_deviceForInterrupt[1])
	_deviceForInterrupt[1]->handleInterrupt();
}
void RH_INTERRUPT_ATTR RH_RF95_FSKOOK::isr2()
{
    if (_deviceForInterrupt[2])
	_deviceForInterrupt[2]->handleInterrupt();
}

int8_t RH_RF95_FSKOOK::temperatureRead()
{
    // Caution: must be in standby.
//    setModeIdle();
    return spiRead(RH_RF95_REG_3C_TEMP);
}

bool RH_RF95_FSKOOK::setFrequency(float centre, float afcPullInRange)
{
    // Frf = FRF / FSTEP
    uint32_t frf = (uint32_t)((centre * 1000000.0) / RH_RF95_FSTEP);
    spiWrite(RH_RF95_REG_06_FRFMSB, (frf >> 16) & 0xff);
    spiWrite(RH_RF95_REG_07_FRFMID, (frf >> 8) & 0xff);
    spiWrite(RH_RF95_REG_08_FRFLSB, frf & 0xff);

    // afcPullInRange is not used
    (void)afcPullInRange;
    return true;
}

int8_t RH_RF95_FSKOOK::rssiRead()
{
    return -((int8_t)(spiRead(RH_RF95_REG_11_RSSIVALUE) >> 1));
}

void RH_RF95_FSKOOK::setOpMode(uint8_t mode)
{
    uint8_t opmode = spiRead(RH_RF95_REG_01_OPMODE);
    opmode &= ~RH_RF95_MODE;
    opmode |= (mode & RH_RF95_MODE);
    spiWrite(RH_RF95_REG_01_OPMODE, opmode);

    // Wait for mode to change.
    while (!(spiRead(RH_RF95_REG_3E_IRQFLAGS1) & RH_RF95_IRQFLAGS1_MODEREADY));
}

void RH_RF95_FSKOOK::setModulationType(uint8_t mode)
{
    uint8_t opmode = spiRead(RH_RF95_REG_01_OPMODE);
    opmode &= ~RH_RF95_OPMODE_MODULATIONTYPE;
    opmode |= (mode & RH_RF95_OPMODE_MODULATIONTYPE);
    spiWrite(RH_RF95_REG_01_OPMODE, opmode);

    // Wait for mode to change.
    while (!(spiRead(RH_RF95_REG_3E_IRQFLAGS1) & RH_RF95_IRQFLAGS1_MODEREADY));
}

void RH_RF95_FSKOOK::setModeIdle()
{
    //Serial.println("Setting IDLE mode...");
    if (_mode != RHModeIdle)
    {
        /*
        if (_power >= 18)
        {
            // If high power boost, return power amp to receive mode
            spiWrite(RH_RF69_REG_5A_TESTPA1, RH_RF69_TESTPA1_NORMAL);
            spiWrite(RH_RF69_REG_5C_TESTPA2, RH_RF69_TESTPA2_NORMAL);
        }
    */
        setOpMode(RH_RF95_MODE_STDBY);
        _mode = RHModeIdle;
    }
}

bool RH_RF95_FSKOOK::sleep()
{
    //Serial.println("Setting SLEEP mode...");
    if (_mode != RHModeSleep)
    {
        setOpMode(RH_RF95_MODE_SLEEP);
        _mode = RHModeSleep;
    }
    return true;
}

void RH_RF95_FSKOOK::setModeRx()
{
    //Serial.println("Setting RX mode...");
    if (_mode != RHModeRx)
    {
        /*
	if (_power >= 18)
	{
	    // If high power boost, return power amp to receive mode
	    spiWrite(RH_RF69_REG_5A_TESTPA1, RH_RF69_TESTPA1_NORMAL);
	    spiWrite(RH_RF69_REG_5C_TESTPA2, RH_RF69_TESTPA2_NORMAL);
	}
    */
	setOpMode(RH_RF95_MODE_RXCONTINUOUS); // Clears FIFO
	spiWrite(RH_RF95_REG_40_DIOMAPPING1, RH_RF95_DIOMAPPING1_DIO0MAPPING_00); // Set interrupt line 0 PayloadReady
	_mode = RHModeRx;
    }
}

void RH_RF95_FSKOOK::setModeTx()
{

    Serial.println("Setting TX mode...");
    if (_mode != RHModeTx)
    {
        setOpMode(RH_RF95_MODE_TX);
        spiWrite(RH_RF95_REG_40_DIOMAPPING1, RH_RF95_DIOMAPPING1_DIO0MAPPING_00); // Interrupt on PacketSent
        _mode = RHModeTx;
    }
}

void RH_RF95_FSKOOK::setTxPower(int8_t power, bool useRFO)
{
    _useRFO = useRFO;
    
    // Sigh, different behaviours depending on whether the module use PA_BOOST or the RFO pin
    // for the transmitter output
    if (useRFO)
    {
	if (power > 15)
	    power = 15;
	if (power < 0)
	    power = 0;
	// Set the MaxPower register to 0x7 => MaxPower = 10.8 + 0.6 * 7 = 15dBm
	// So Pout = Pmax - (15 - power) = 15 - 15 + power
	spiWrite(RH_RF95_REG_09_PA_CONFIG, RH_RF95_MAX_POWER | power);
	spiWrite(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_DISABLE);
    }
    else
    {
	if (power > 20)
	    power = 20;
	if (power < 2)
	    power = 2;

	// For RH_RF95_PA_DAC_ENABLE, manual says '+20dBm on PA_BOOST when OutputPower=0xf'
	// RH_RF95_PA_DAC_ENABLE actually adds about 3dBm to all power levels. We will use it
	// for 8, 19 and 20dBm
	if (power > 17)
	{
	    spiWrite(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_ENABLE);
	    power -= 3;
	}
	else
	{
	    spiWrite(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_DISABLE);
	}

	// RFM95/96/97/98 does not have RFO pins connected to anything. Only PA_BOOST
	// pin is connected, so must use PA_BOOST
	// Pout = 2 + OutputPower (+3dBm if DAC enabled)
	spiWrite(RH_RF95_REG_09_PA_CONFIG, RH_RF95_PA_SELECT | (power-2));
    }
}

// Sets registers from a canned modem configuration structure
void RH_RF95_FSKOOK::setModemRegisters(const ModemConfig* config)
{
    setModulationType(config->reg_01);
    spiBurstWrite(RH_RF95_REG_02_BITRATEMSB,    &config->reg_02, 4);
    spiWrite(RH_RF95_REG_0A_PA_RAMP,              config->reg_0a);
    spiBurstWrite(RH_RF95_REG_12_RXBW,          &config->reg_12, 2);
    spiBurstWrite(RH_RF95_REG_30_PACKETCONFIG1, &config->reg_30, 2);
}

// Set one of the canned FSK Modem configs
// Returns true if its a valid choice
bool RH_RF95_FSKOOK::setModemConfig(ModemConfigChoice index)
{
    if (index > (signed int)(sizeof(MODEM_CONFIG_TABLE) / sizeof(ModemConfig)))
        return false;

    ModemConfig cfg;
    memcpy_P(&cfg, &MODEM_CONFIG_TABLE[index], sizeof(RH_RF95_FSKOOK::ModemConfig));
    setModemRegisters(&cfg);

    return true;
}

void RH_RF95_FSKOOK::setPreambleLength(uint16_t bytes)
{
    spiWrite(RH_RF95_REG_25_PREAMBLEMSB, bytes >> 8);
    spiWrite(RH_RF95_REG_26_PREAMBLELSB, bytes & 0xff);
}

void RH_RF95_FSKOOK::setSyncWords(const uint8_t* syncWords, uint8_t len)
{
    uint8_t syncconfig = spiRead(RH_RF95_REG_27_SYNCONFIG);
    if (syncWords && len && len <= 8)
    {
	    spiBurstWrite(RH_RF95_REG_28_SYNCVALUE1, syncWords, len);
	    syncconfig |= RH_RF95_SYNCCONFIG_SYNCON;

        syncconfig &= ~RH_RF95_SYNCCONFIG_SYNCSIZE;
        syncconfig |= (len-1);
    }
    else {
	    syncconfig &= ~RH_RF95_SYNCCONFIG_SYNCON;
        syncconfig &= ~RH_RF95_SYNCCONFIG_SYNCSIZE;
    }

    spiWrite(RH_RF95_REG_27_SYNCONFIG, syncconfig);
}


bool RH_RF95_FSKOOK::available()
{
    if (_mode == RHModeTx) {
        Serial.println("RX not possible with chip in TX mode");
	    return false;
    }
    setModeRx(); // Make sure we are receiving

    return _rxBufValid;
}

bool RH_RF95_FSKOOK::recv(uint8_t* buf, uint8_t* len)
{
    if (!available())
	    return false;

    if (buf && len)
    {
	ATOMIC_BLOCK_START;
	if (*len > _bufLen)
	    *len = _bufLen;
	memcpy(buf, _buf, *len);
	ATOMIC_BLOCK_END;
    }
    _rxBufValid = false; // Got the most recent message
    //printBuffer("recv:", buf, *len);
    return true;
}

bool RH_RF95_FSKOOK::send(const uint8_t* data, uint8_t len)
{

    if (len > RH_RF95_MAX_MESSAGE_LEN)
	    return false;

    waitPacketSent(); // Make sure we dont interrupt an outgoing message
    setModeIdle(); // Prevent RX while filling the fifo

    if (!waitCAD()) {
        Serial.println("Channel activity detected...");
	    return false;  // Check channel activity
    }

    ATOMIC_BLOCK_START;

    digitalWrite(_slaveSelectPin, LOW);

    if (_shouldSendHeaderInPayload) {

        if ( !_fixedLengthMode) {
            spiWrite(RH_RF95_REG_00_FIFO, len + RH_RF95_HEADER_LEN);
        }
        spiWrite(RH_RF95_REG_00_FIFO, _txHeaderTo);
        spiWrite(RH_RF95_REG_00_FIFO, _txHeaderFrom);
        spiWrite(RH_RF95_REG_00_FIFO, _txHeaderId);
        spiWrite(RH_RF95_REG_00_FIFO, _txHeaderFlags);
    } else {
        if ( !_fixedLengthMode) {
            spiWrite(RH_RF95_REG_00_FIFO, len);
        }
    }

    spiBurstWrite(RH_RF95_REG_00_FIFO, data, len);

    digitalWrite(_slaveSelectPin, HIGH);

    ATOMIC_BLOCK_END;

    setModeTx();
    
    return true;
}

uint8_t RH_RF95_FSKOOK::maxMessageLength()
{
    return RH_RF95_MAX_MESSAGE_LEN;
}

bool RH_RF95_FSKOOK::printRegister(uint8_t reg)
{  
#ifdef RH_HAVE_SERIAL
    Serial.print("Reg: ");
    Serial.print(reg, HEX);
    Serial.print(" DEC value: ");
    Serial.println(spiRead(reg), DEC);
#endif
    return true;
}

bool RH_RF95_FSKOOK::printRegisters()
{  
    Serial.println("Registers:");
    uint8_t i;
    for (i = 0; i < 0x5d; i++)
	    printRegister(i);

    return true;
}

// RH_RF95_FSKOOK.h
// Author: Mike McCauley (mikem@airspayce.com)
// Author: Alex Rau (alex@retroduction.org)
// Copyright (C) 2014 Mike McCauley
// Copyright (C) 2024 Alex Rau


#ifndef RH_RF95_FSKOOK_h
#define RH_RF95_FSKOOK_h

#include <RHGenericSPI.h>
#include <RHSPIDriver.h>

// The crystal oscillator frequency of the RF95 module
#define RH_RF95_FXOSC 32000000.0

// The Frequency Synthesizer step = RH_RF95_FXOSC / 2^^19
#define RH_RF95_FSTEP  (RH_RF95_FXOSC / 524288)

// This is the maximum number of interrupts the driver can support
// Most Arduinos can handle 2, Megas can handle more
#define RH_RF95_NUM_INTERRUPTS 3

// This is the bit in the SPI address that marks it as a write
#define RH_RF95_SPI_WRITE_MASK 0x80

// Max number of octets the RH_RF95 Rx and Tx FIFOs can hold
#define RH_RF95_FIFO_SIZE 64

// The length of the headers we might add (depends on if header should be added at all).
#define RH_RF95_HEADER_LEN 4

// This is the maximum message length that can be supported by this driver. Limited by
// the size of the FIFO, since we are unable to support on-the-fly filling and emptying 
// of the FIFO.
// Can be pre-defined to a smaller size (to save SRAM) prior to including this header
// Here we allow for 4 bytes of address and header and payload to be included in the 64 byte encryption limit.
// the one byte payload length is not encrypted
// If header information is not part of the payload ( setShouldSendHeaderInPayload(false) ) this limit still correcty applies.
#ifndef RH_RF95_MAX_MESSAGE_LEN
#define RH_RF95_MAX_MESSAGE_LEN (RH_RF95_FIFO_SIZE - RH_RF95_HEADER_LEN)
#endif

// FSK/OOK Register names
#define RH_RF95_REG_00_FIFO                                 0x00
#define RH_RF95_REG_01_OPMODE                               0x01
#define RH_RF95_REG_02_BITRATEMSB                           0x02
#define RH_RF95_REG_03_BITRATELSB                           0x03
#define RH_RF95_REG_04_FDEVMSB                              0x04
#define RH_RF95_REG_05_FDEVLSB                              0x05
#define RH_RF95_REG_06_FRFMSB                               0x06
#define RH_RF95_REG_07_FRFMID                               0x07
#define RH_RF95_REG_08_FRFLSB                               0x08
#define RH_RF95_REG_09_PA_CONFIG                            0x09
#define RH_RF95_REG_0A_PA_RAMP                              0x0a
#define RH_RF95_REG_0b_REGOCP                               0x0b
#define RH_RF95_REG_0c_REGLNA                               0x0c
#define RH_RF95_REG_0d_RXCONFIG                             0x0d
#define RH_RF95_REG_0e_RSSICONFIG                           0x0e
#define RH_RF95_REG_0f_RSSICOLLISION                        0x0f
#define RH_RF95_REG_10_RSSITHRESH                           0x10
#define RH_RF95_REG_11_RSSIVALUE                            0x11
#define RH_RF95_REG_12_RXBW                                 0x12
#define RH_RF95_REG_13_AFCBW                                0x13
#define RH_RF95_REG_14_OOKPEAK                              0x14
#define RH_RF95_REG_15_OOKFIX                               0x15
#define RH_RF95_REG_16_OOKAVG                               0x16
#define RH_RF95_REG_17_RESERVED17                           0x17
#define RH_RF95_REG_18_RESERVED18                           0x18
#define RH_RF95_REG_19_RESERVED19                           0x19
#define RH_RF95_REG_1a_AFCFEI                               0x1a
#define RH_RF95_REG_1b_AFCMSB                               0x1b
#define RH_RF95_REG_1c_AFCLSB                               0x1c
#define RH_RF95_REG_1d_FEIMSB                               0x1d
#define RH_RF95_REG_1e_FEILSB                               0x1e
#define RH_RF95_REG_1f_PREAMLEDETECT                        0x1f
#define RH_RF95_REG_20_RXTIMEOUT1                           0x20
#define RH_RF95_REG_21_RXTIMEOUT2                           0x21
#define RH_RF95_REG_22_RXTIMEOUT3                           0x22
#define RH_RF95_REG_23_RXDELAY                              0x23
#define RH_RF95_REG_24_OSC                                  0x24
#define RH_RF95_REG_25_PREAMBLEMSB                          0x25
#define RH_RF95_REG_26_PREAMBLELSB                          0x26
#define RH_RF95_REG_27_SYNCONFIG                            0x27
#define RH_RF95_REG_28_SYNCVALUE1                           0x28
#define RH_RF95_REG_29_SYNCVALUE2                           0x29
#define RH_RF95_REG_2A_SYNCVALUE3                           0x2A
#define RH_RF95_REG_2B_SYNCVALUE4                           0x2B
#define RH_RF95_REG_2C_SYNCVALUE5                           0x2C
#define RH_RF95_REG_2D_SYNCVALUE6                           0x2D
#define RH_RF95_REG_2E_SYNCVALUE7                           0x2E
#define RH_RF95_REG_2F_SYNCVALUE8                           0x2F
#define RH_RF95_REG_30_PACKETCONFIG1                        0x30
#define RH_RF95_REG_31_PACKETCONFIG2                        0x31
#define RH_RF95_REG_32_PAYLOADLENGTH                        0x32
#define RH_RF95_REG_33_NODEADRS                             0x33
#define RH_RF95_REG_34_BROADCASTADRS                        0x34
#define RH_RF95_REG_35_FIFOTHRESH                           0x35
#define RH_RF95_REG_36_SEQCONFIG1                           0x36
#define RH_RF95_REG_37_SEQCONFIG1                           0x37
#define RH_RF95_REG_38_TIMERRESOL                           0x38
#define RH_RF95_REG_39_TIMER1COEF                           0x39
#define RH_RF95_REG_3A_TIMER2COEF                           0x3A
#define RH_RF95_REG_3B_IMAGECAL                             0x3B
#define RH_RF95_REG_3C_TEMP                                 0x3C
#define RH_RF95_REG_3D_LOWBATT                              0x3D
#define RH_RF95_REG_3E_IRQFLAGS1                            0x3E
#define RH_RF95_REG_3F_IRQFLAGS2                            0x3F
#define RH_RF95_REG_40_DIOMAPPING1                          0x40
#define RH_RF95_REG_41_DIOMAPPING2                          0x41
#define RH_RF95_REG_42_VERSION                              0x42
#define RH_RF95_REG_44_PLLHOP                               0x44
#define RH_RF95_REG_4B_TCXO                                 0x4B
#define RH_RF95_REG_4D_PA_DAC                               0x4D
#define RH_RF95_REG_5B_FORMERTEMP                           0x5B
#define RH_RF95_REG_5D_BITRATEFRAC                          0x5D
#define RH_RF95_REG_61_AGCREF                               0x61
#define RH_RF95_REG_62_AGCTHRESH1                           0x62
#define RH_RF95_REG_63_AGCTHRESH2                           0x63
#define RH_RF95_REG_64_AGCTHRESH3                           0x64


// These register masks etc are named wherever possible
// corresponding to the bit and field names in the RFM95 Manual

// RH_RF95_REG_01_OP_MODE                             0x01
#define RH_RF95_LONG_RANGE_MODE                       0x80
#define RH_RF95_MODULATION_TYPE                       0x60
#define RH_RF95_LOW_FREQUENCY_MODE                    0x08
#define RH_RF95_MODE                                  0x07
#define RH_RF95_MODE_SLEEP                            0x00
#define RH_RF95_MODE_STDBY                            0x01
#define RH_RF95_MODE_FSTX                             0x02
#define RH_RF95_MODE_TX                               0x03
#define RH_RF95_MODE_FSRX                             0x04
#define RH_RF95_MODE_RXCONTINUOUS                     0x05
#define RH_RF95_OPMODE_MODULATIONTYPE                 0x60
#define RH_RF95_OPMODE_MODULATIONTYPE_FSK             0x00
#define RH_RF95_OPMODE_MODULATIONTYPE_OOK             0x20

// RH_RF95_REG_09_PA_CONFIG                           0x09
#define RH_RF95_PA_SELECT                             0x80
#define RH_RF95_MAX_POWER                             0x70
#define RH_RF95_OUTPUT_POWER                          0x0f

#define RH_RF95_PARAMP_MODULATIONSHAPING                 0x60
#define RH_RF95_PARAMP_MODULATIONSHAPING_FSK_NONE        0x00
#define RH_RF95_PARAMP_MODULATIONSHAPING_FSK_BT1_0       0x20
#define RH_RF95_PARAMP_MODULATIONSHAPING_FSK_BT0_5       0x40
#define RH_RF95_PARAMP_MODULATIONSHAPING_FSK_BT0_3       0x60
#define RH_RF95_PARAMP_MODULATIONSHAPING_OOK_NONE        0x00
#define RH_RF95_PARAMP_MODULATIONSHAPING_OOK_BR          0x20
#define RH_RF95_PARAMP_MODULATIONSHAPING_OOK_2BR         0x4

// RH_RF95_REG_40_DIOMAPPING1
#define RH_RF95_DIOMAPPING1_DIO0MAPPING                       0xC0
#define RH_RF95_DIOMAPPING1_DIO0MAPPING_00                    0x00
#define RH_RF95_DIOMAPPING1_DIO0MAPPING_01                    0x40
#define RH_RF95_DIOMAPPING1_DIO0MAPPING_10                    0x80
#define RH_RF95_DIOMAPPING1_DIO0MAPPING_11                    0xC0
#define RH_RF95_DIOMAPPING1_DIO1MAPPING                       0x30
#define RH_RF95_DIOMAPPING1_DIO1MAPPING_00                    0x00
#define RH_RF95_DIOMAPPING1_DIO1MAPPING_01                    0x10
#define RH_RF95_DIOMAPPING1_DIO1MAPPING_10                    0x20
#define RH_RF95_DIOMAPPING1_DIO1MAPPING_11                    0x30
#define RH_RF95_DIOMAPPING1_DIO2MAPPING                       0x0c
#define RH_RF95_DIOMAPPING1_DIO2MAPPING_00                    0x00
#define RH_RF95_DIOMAPPING1_DIO2MAPPING_01                    0x04
#define RH_RF95_DIOMAPPING1_DIO2MAPPING_10                    0x08
#define RH_RF95_DIOMAPPING1_DIO2MAPPING_11                    0x0c
#define RH_RF95_DIOMAPPING1_DIO3MAPPING                       0x03
#define RH_RF95_DIOMAPPING1_DIO3MAPPING_00                    0x00
#define RH_RF95_DIOMAPPING1_DIO3MAPPING_01                    0x01
#define RH_RF95_DIOMAPPING1_DIO3MAPPING_10                    0x02
#define RH_RF95_DIOMAPPING1_DIO3MAPPING_11                    0x03

// RH_RF95_REG_3E_IRQFLAGS1
#define RH_RF95_IRQFLAGS1_MODEREADY                         0x80
#define RH_RF95_IRQFLAGS1_RXREADY                           0x40
#define RH_RF95_IRQFLAGS1_TXREADY                           0x20
#define RH_RF95_IRQFLAGS1_PLLLOCK                           0x10
#define RH_RF95_IRQFLAGS1_RSSI                              0x08
#define RH_RF95_IRQFLAGS1_TIMEOUT                           0x04
#define RH_RF95_IRQFLAGS1_PREAMBLEDETECT                    0x02
#define RH_RF95_IRQFLAGS1_SYNADDRESSMATCH                   0x01

// RH_RF95_REG_3F_IRQFLAGS2
#define RH_RF95_IRQFLAGS2_FIFOFULL                          0x80
#define RH_RF95_IRQFLAGS2_FIFOEMPTY                         0x40
#define RH_RF95_IRQFLAGS2_FIFOLEVEL                         0x20
#define RH_RF95_IRQFLAGS2_FIFOOVERRUN                       0x10
#define RH_RF95_IRQFLAGS2_PACKETSENT                        0x08
#define RH_RF95_IRQFLAGS2_PAYLOADREADY                      0x04
#define RH_RF95_IRQFLAGS2_CRCOK                             0x02
#define RH_RF95_IRQFLAGS2_LOWBAT                            0x01

// RH_RF95_REG_27_SYNCCONFIG
#define RH_RF95_SYNCCONFIG_AUTORESTARTRXMODE                0xC0
#define RH_RF95_SYNCCONFIG_PREAMBLEPOLARITY                 0x20
#define RH_RF95_SYNCCONFIG_SYNCON                           0x10
#define RH_RF95_SYNCCONFIG_SYNCSIZE                         0x07

// RH_RF95_REG_30_PACKETCONFIG1
#define RH_RF95_PACKETCONFIG1_PACKETFORMAT                  0x80
#define RH_RF95_PACKETCONFIG1_PACKETFORMAT_FIXED            0x00
#define RH_RF95_PACKETCONFIG1_PACKETFORMAT_VARIABLE         0x80
#define RH_RF95_PACKETCONFIG1_DCFREE                        0x60
#define RH_RF95_PACKETCONFIG1_DCFREE_NONE                   0x00
#define RH_RF95_PACKETCONFIG1_DCFREE_MANCHESTER             0x20
#define RH_RF95_PACKETCONFIG1_DCFREE_WHITENING              0x40
#define RH_RF95_PACKETCONFIG1_DCFREE_RESERVED               0x60
#define RH_RF95_PACKETCONFIG1_CRC_ON                        0x10
#define RH_RF95_PACKETCONFIG1_CRCAUTOCLEAROFF               0x08
#define RH_RF95_PACKETCONFIG1_ADDRESSFILTERING              0x06
#define RH_RF95_PACKETCONFIG1_ADDRESSFILTERING_NONE         0x00
#define RH_RF95_PACKETCONFIG1_ADDRESSFILTERING_NODE         0x02
#define RH_RF95_PACKETCONFIG1_ADDRESSFILTERING_NODE_BC      0x04
#define RH_RF95_PACKETCONFIG1_ADDRESSFILTERING_RESERVED     0x06
#define RH_RF95_PACKETCONFIG1_CRCWHITENINGTYPE              0x01
#define RH_RF95_PACKETCONFIG1_CRCWHITENINGTYPE_CCITT        0x00
#define RH_RF95_PACKETCONFIG1_CRCWHITENINGTYPE_IBM          0x01

// RH_RF95_REG_30_PACKETCONFIG2
#define RH_RF95_PACKETCONFIG2_DATAMODE                        0x40
#define RH_RF95_PACKETCONFIG2_DATAMODE_CONTINUOUS             0x0
#define RH_RF95_PACKETCONFIG2_DATAMODE_PACKET                 0x40

// RH_RF95_REG_35_FIFOTHRESH
#define RH_RF95_FIFOTHRESH_TXSTARTCONDITION_NOTEMPTY        0x80
#define RH_RF95_FIFOTHRESH_FIFOTHRESHOLD                    0x7f

// RH_RF95_REG_4D_PA_DAC                              0x4d
#define RH_RF95_PA_DAC_DISABLE                        0x04
#define RH_RF95_PA_DAC_ENABLE                         0x07

// Define this to include Serial printing in diagnostic routines
#define RH_RF95_HAVE_SERIAL


/////////////////////////////////////////////////////////////////////
/// \class RH_RF95_FSKOOK RH_RF95_FSKOOK.h <RH_RF95_FSKOOK.h>
/// \brief Driver to send and receive unaddressed, unreliable datagrams via an RF95 compatible radio transceiver using non-LoRa modulation types.
///
/// For a discussion on how to wire, typical usage etc. please have a look into the documentation of RH_RF69.h. Most of the documentation there
/// should be valid for this implementation as well. Exception are:
/// - RF95 does not support encryption on the hardware level like the RF69 does.
/// - Default tx power is set to 2 (be a good citizen and start with low power)
/// Everything related to encryption has been therefore removed for this implementation.
///
/// This driver has been initially based on Mike McCauley's RH_RF69 implementation due to the similaries between RFM69 
/// and RFM9x modules. This implementation does not cover LoRa modes but only FSK/GFSK/OOK modes. If you need LoRa
/// then use RFM95.h instead.

/// Additional notes: this driver implements *more* options which are not commonly available across other Radiohead modules.
/// For example this implementation supports:
/// - disabling the header information (from, to, etc.) inside the payload
/// - using encoding types other than whitening with a well-known byte sequence (basically no encoding or manchester encoding)
/// - using a well-known fixed length payload instead of variable length payloads which require an additional payload length byte in the transmission
/// - preamble length can be adjusted
/// - sync words can be adjusted

class RH_RF95_FSKOOK : public RHSPIDriver
{
public:

    /// \brief Defines register values for a set of modem configuration registers
    ///
    /// Defines register values for a set of modem configuration registers
    /// that can be passed to setModemRegisters() if none of the choices in
    /// ModemConfigChoice suit your need setModemRegisters() writes the
    /// register values from this structure to the appropriate RF95 registers
    /// to set the desired modulation type, data rate and deviation/bandwidth.
    typedef struct
    {
        uint8_t reg_01; // OPMODE_MODULATIONTYPE
	    uint8_t reg_02; ///< Value for register RH_RF95_REG_02_BITRATEMSB
	    uint8_t reg_03; ///< Value for register RH_RF95_REG_03_BITRATELSB
	    uint8_t reg_04; ///< Value for register RH_RF95_REG_04_FDEVMSB
	    uint8_t reg_05; ///< Value for register RH_RF95_REG_05_FDEVLSB
        uint8_t reg_0a; ///< Value for register RH_RF95_REG_0A_PARAMP
	    uint8_t reg_12; ///< Value for register RH_RF95_REG_12_RXBW
	    uint8_t reg_13; ///< Value for register RH_RF95_REG_13_AFCBW
	    uint8_t reg_30; ///< Value for register RH_RF95_REG_30_PACKETCONFIG1
	    uint8_t reg_31; ///< Value for register RH_RF95_REG_31_PACKETCONFIG2
    } ModemConfig;
  
    /// Choices for setModemConfig() for a selected subset of common
    /// modulation types, and data rates. If you need another configuration,
    /// use the register calculator.  and call setModemRegisters() with your
    /// desired settings.  
    /// These are indexes into MODEM_CONFIG_TABLE. We strongly recommend you use these symbolic
    /// definitions and not their integer equivalents: its possible that new values will be
    /// introduced in later versions (though we will try to avoid it).
    /// CAUTION: some of these configurations do not work corectly and are marked as such.
    typedef enum
    {
	FSK_Rb2Fd5 = 0,	   ///< FSK, Whitening, Rb = 2kbs,    Fd = 5kHz    // transmission visible in SDR
	FSK_Rb2_4Fd4_8,    ///< FSK, Whitening, Rb = 2.4kbs,  Fd = 4.8kHz  // transmission visible in SDR
	FSK_Rb4_8Fd9_6,    ///< FSK, Whitening, Rb = 4.8kbs,  Fd = 9.6kHz  // transmission visible in SDR
	FSK_Rb9_6Fd19_2,   ///< FSK, Whitening, Rb = 9.6kbs,  Fd = 19.2kHz // transmission visible in SDR
	FSK_Rb19_2Fd38_4,  ///< FSK, Whitening, Rb = 19.2kbs, Fd = 38.4kHz // transmission visible in SDR
	FSK_Rb38_4Fd76_8,  ///< FSK, Whitening, Rb = 38.4kbs, Fd = 76.8kHz // transmission visible in SDR
	FSK_Rb57_6Fd120,   ///< FSK, Whitening, Rb = 57.6kbs, Fd = 120kHz
	FSK_Rb125Fd125,    ///< FSK, Whitening, Rb = 125kbs,  Fd = 125kHz
	FSK_Rb250Fd250,    ///< FSK, Whitening, Rb = 250kbs,  Fd = 250kHz
	FSK_Rb55555Fd50,   ///< FSK, Whitening, Rb = 55555kbs,Fd = 50kHz for RFM69 lib compatibility

	GFSK_Rb2Fd5,	    ///< GFSK, Whitening, Rb = 2kbs,    Fd = 5kHz
	GFSK_Rb2_4Fd4_8,    ///< GFSK, Whitening, Rb = 2.4kbs,  Fd = 4.8kHz  // transmission visible in SDR
	GFSK_Rb4_8Fd9_6,    ///< GFSK, Whitening, Rb = 4.8kbs,  Fd = 9.6kHz  // transmission visible in SDR
	GFSK_Rb9_6Fd19_2,   ///< GFSK, Whitening, Rb = 9.6kbs,  Fd = 19.2kHz // transmission visible in SDR
	GFSK_Rb19_2Fd38_4,  ///< GFSK, Whitening, Rb = 19.2kbs, Fd = 38.4kHz // transmission visible in SDR
	GFSK_Rb38_4Fd76_8,  ///< GFSK, Whitening, Rb = 38.4kbs, Fd = 76.8kHz // transmission visible in SDR
	GFSK_Rb57_6Fd120,   ///< GFSK, Whitening, Rb = 57.6kbs, Fd = 120kHz
	GFSK_Rb125Fd125,    ///< GFSK, Whitening, Rb = 125kbs,  Fd = 125kHz
	GFSK_Rb250Fd250,    ///< GFSK, Whitening, Rb = 250kbs,  Fd = 250kHz // transmission visible in SDR
	GFSK_Rb55555Fd50,   ///< GFSK, Whitening, Rb = 55555kbs,Fd = 50kHz

	OOK_Rb1Bw1,         ///< OOK, Whitening, Rb = 1kbs,    Rx Bandwidth = 1kHz.   // transmission visible in SDR
	OOK_Rb1_2Bw75,      ///< OOK, Whitening, Rb = 1.2kbs,  Rx Bandwidth = 75kHz.   // transmission visible in SDR
	OOK_Rb2_4Bw4_8,     ///< OOK, Whitening, Rb = 2.4kbs,  Rx Bandwidth = 4.8kHz.  // transmission visible in SDR
	OOK_Rb4_8Bw9_6,     ///< OOK, Whitening, Rb = 4.8kbs,  Rx Bandwidth = 9.6kHz.  // transmission visible in SDR
	OOK_Rb9_6Bw19_2,    ///< OOK, Whitening, Rb = 9.6kbs,  Rx Bandwidth = 19.2kHz. // transmission visible in SDR
	OOK_Rb19_2Bw38_4,   ///< OOK, Whitening, Rb = 19.2kbs, Rx Bandwidth = 38.4kHz. // tx hangs, transmitter sends permanently carrier
	OOK_Rb32Bw64,       ///< OOK, Whitening, Rb = 32kbs,   Rx Bandwidth = 64kHz.   // tx hangs, transmitter sends permanently carrier

//	Test,
    } ModemConfigChoice;

    typedef enum
    { DC_NONE, DC_MANCHESTER, DC_WHITENING_CCITT, DC_WHITENING_IBM } WhiteningType;

    /// Constructor.
    /// \param[in] spi Pointer to the SPI interface object to use. 
    ///                Defaults to the standard Arduino hardware SPI interface
    RH_RF95_FSKOOK(uint8_t slaveSelectPin = SS, uint8_t interruptPin = 2, RHGenericSPI& spi = hardware_spi);
  
    /// Initialises this instance and the radio module connected to it.
    /// The following steps are taken:
    /// - Initialise the slave select pin and the SPI interface library
    /// - Checks the connected RF95 module can be communicated
    /// - Attaches an interrupt handler
    /// - Configures the RF95 module
    /// - Sets the frequency to 434.0 MHz
    /// - Sets the modem data rate to FSK_Rb2Fd5
    /// \return  true if everything was successful
    bool        init();

    /// Reads the on-chip temperature sensor.
    /// The RF95 must be in Idle mode (= RF95 Standby) to measure temperature.
    /// The measurement is uncalibrated and without calibration, you can expect it to be far from
    /// correct.
    /// \return The measured temperature, in degrees C from -40 to 85 (uncalibrated)
    int8_t        temperatureRead();   

    /// Sets the transmitter and receiver 
    /// centre frequency
    /// \param[in] centre Frequency in MHz. 240.0 to 960.0. Caution, RF95 comes in several
    /// different frequency ranges, and setting a frequency outside that range of your radio will probably not work
    /// \param[in] afcPullInRange Not used
    /// \return true if the selected frquency centre is within range
    bool        setFrequency(float centre, float afcPullInRange = 0.05);

    /// Reads and returns the current RSSI value. 
    /// Causes the current signal strength to be measured and returned
    /// If you want to find the RSSI
    /// of the last received message, use lastRssi() instead.
    /// \return The current RSSI value on units of 0.5dB.
    int8_t        rssiRead();

    /// Sets the parameters for the RF95 OPMODE.
    /// This is a low level device access function, and should not normally ned to be used by user code. 
    /// Instead can use stModeRx(), setModeTx(), setModeIdle()
    /// \param[in] mode RF95 OPMODE to set, one of RH_RF95_OPMODE_MODE_*.
    void           setOpMode(uint8_t mode);

    void           setModulationType(uint8_t mode);

    /// If current mode is Rx or Tx changes it to Idle. If the transmitter or receiver is running, 
    /// disables them.
    void           setModeIdle();

    /// If current mode is Tx or Idle, changes it to Rx. 
    /// Starts the receiver in the RF95.
    void           setModeRx();

    /// If current mode is Rx or Idle, changes it to Rx. F
    /// Starts the transmitter in the RF95.
    void           setModeTx();

    /// Sets the transmitter power output level, and configures the transmitter pin.
    /// Be a good neighbour and set the lowest power level you need.
    /// Some SX1276/77/78/79 and compatible modules (such as RFM95/96/97/98) 
    /// use the PA_BOOST transmitter pin for high power output (and optionally the PA_DAC)
    /// while some (such as the Modtronix inAir4 and inAir9) 
    /// use the RFO transmitter pin for lower power but higher efficiency.
    /// You must set the appropriate power level and useRFO argument for your module.
    /// Check with your module manufacturer which transmtter pin is used on your module
    /// to ensure you are setting useRFO correctly. 
    /// Failure to do so will result in very low 
    /// transmitter power output.
    /// Caution: legal power limits may apply in certain countries.
    /// After init(), the power will be set to 13dBm, with useRFO false (ie PA_BOOST enabled).
    /// \param[in] power Transmitter power level in dBm. For RFM95/96/97/98 LORA with useRFO false, 
    /// valid values are from +2 to +20. For 18, 19 and 20, PA_DAC is enabled, 
    /// For Modtronix inAir4 and inAir9 with useRFO true (ie RFO pins in use), 
    /// valid values are from 0 to 15.
    /// \param[in] useRFO If true, enables the use of the RFO transmitter pins instead of
    /// the PA_BOOST pin (false). Choose the correct setting for your module.
    void           setTxPower(int8_t power, bool useRFO = false);

    /// Sets all the registers required to configure the data modem in the RF95, including the data rate, 
    /// bandwidths etc. You can use this to configure the modem with custom configurations if none of the 
    /// canned configurations in ModemConfigChoice suit you.
    /// \param[in] config A ModemConfig structure containing values for the modem configuration registers.
    void           setModemRegisters(const ModemConfig* config);

    /// Select one of the predefined modem configurations. If you need a modem configuration not provided 
    /// here, use setModemRegisters() with your own ModemConfig. The default after init() is RH_RF95::GFSK_Rb250Fd250.
    /// \param[in] index The configuration choice.
    /// \return true if index is a valid choice.
    bool        setModemConfig(ModemConfigChoice index);

    /// Use a fixed length for tx and rx - this avoids sending a length byte over the air but requires common
    /// agreement on sender and reception side.
    /// In theory we are allowed to send around 2048
    void setFixedMessageLength(uint8_t msgLen);
    
    /// This activates sending a length byte as first byte of the payload (strangely the hardware does not do
    /// this on its own 🤔 so this implementation needs to do this itself 🤬)
    void setVariableMessageLength();

    /// This method allows turning off (or on) the header in the payload (which is sent by default in the Radiohead library).
    void setShouldSendHeaderInPayload(bool value);

    /// Allows specifing the payload encoding (none, manchester or whitening ccitt/ibm)
    void setEncoding(WhiteningType value);

    /// Starts the receiver and checks whether a received message is available.
    /// This can be called multiple times in a timeout loop
    /// \return true if a complete, valid message has been received and is able to be retrieved by
    /// recv()
    bool        available();

    /// Turns the receiver on if it not already on.
    /// If there is a valid message available, copy it to buf and return true
    /// else return false.
    /// If a message is copied, *len is set to the length (Caution, 0 length messages are permitted).
    /// You should be sure to call this function frequently enough to not miss any messages
    /// It is recommended that you call it in your main loop.
    /// \param[in] buf Location to copy the received message
    /// \param[in,out] len Pointer to the number of octets available in buf. The number be reset to the actual number of octets copied.
    /// \return true if a valid message was copied to buf
    bool        recv(uint8_t* buf, uint8_t* len);

    /// Waits until any previous transmit packet is finished being transmitted with waitPacketSent().
    /// Then loads a message into the transmitter and starts the transmitter. Note that a message length
    /// of 0 is NOT permitted. 
    /// \param[in] data Array of data to be sent
    /// \param[in] len Number of bytes of data to send (> 0)
    /// \return true if the message length was valid and it was correctly queued for transmit
    bool        send(const uint8_t* data, uint8_t len);


    /// Sets the length of the preamble
    /// in bytes. 
    /// Caution: this should be set to the same 
    /// value on all nodes in your network. Default is 4.
    /// Sets the message preamble length in REG_0?_PREAMBLE?SB
    /// \param[in] bytes Preamble length in bytes.  
    void           setPreambleLength(uint16_t bytes);

    /// Sets the sync words for transmit and receive 
    /// Caution: SyncWords should be set to the same 
    /// value on all nodes in your network. Nodes with different SyncWords set will never receive
    /// each others messages, so different SyncWords can be used to isolate different
    /// networks from each other. Default is { 0x2d, 0xd4 }.
    /// Caution: tests here show that with a single sync word (ie where len == 1), 
    /// RFM69 reception can be unreliable.
    /// To disable sync word generation and detection, call with the defaults: setSyncWords();
    /// \param[in] syncWords Array of sync words, 1 to 4 octets long. NULL if no sync words to be used.
    /// \param[in] len Number of sync words to set, 1 to 4. 0 if no sync words to be used.
    void           setSyncWords(const uint8_t* syncWords = NULL, uint8_t len = 0);

    /// Returns the time in millis since the most recent preamble was received, and when the most recent
    /// RSSI measurement was made.
    uint32_t getLastPreambleTime();

    /// The maximum message length supported by this driver
    /// \return The maximum message length supported by this driver
    uint8_t maxMessageLength();

    /// Prints the value of a single register
    /// to the Serial device if RH_HAVE_SERIAL is defined for the current platform
    /// For debugging/testing only
    /// \return true if successful
    bool printRegister(uint8_t reg);

    /// Prints the value of all the RF95 registers
    /// to the Serial device if RH_HAVE_SERIAL is defined for the current platform
    /// For debugging/testing only
    /// \return true if successful
    bool printRegisters();

    /// Sets the radio operating mode for the case when the driver is idle (ie not
    /// transmitting or receiving), allowing you to control the idle mode power requirements
    /// at the expense of slower transitions to transmit and receive modes.
    /// By default, the idle mode is RH_RF95_OPMODE_MODE_STDBY,
    /// but eg setIdleMode(RH_RF95_OPMODE_MODE_SLEEP) will provide a much lower
    /// idle current but slower transitions. Call this function after init().
    /// \param[in] idleMode The chip operating mode to use when the driver is idle. One of RH_RF95_OPMODE_*
    void setIdleMode(uint8_t idleMode);

    /// Sets the radio into low-power sleep mode.
    /// If successful, the transport will stay in sleep mode until woken by 
    /// changing mode it idle, transmit or receive (eg by calling send(), recv(), available() etc)
    /// Caution: there is a time penalty as the radio takes a finite time to wake from sleep mode.
    /// \return true if sleep mode was successfully entered.
    virtual bool    sleep();

    /// Return the integer value of the device type
    /// as read from the device in from RH_RF95_REG_42_VERSION.
    /// Expect 0x24, depending on the type of device actually
    /// connected.
    /// \return The integer device type
    uint16_t deviceType() {return _deviceType;};

protected:
    /// This is a low level function to handle the interrupts for one instance of RF95.
    /// Called automatically by isr*()
    /// Should not need to be called by user code.
    void           handleInterrupt();

    /// Low level function to read the FIFO and put the received data into the receive buffer
    /// Should not need to be called by user code.
    void           readFifo();

    /// False if the PA_BOOST transmitter output pin is to be used.
    /// True if the RFO transmitter output pin is to be used.
    bool                _useRFO;

    /// Low level interrupt service routine for RF95 connected to interrupt 0
    static void         isr0();

    /// Low level interrupt service routine for RF95 connected to interrupt 1
    static void         isr1();

    /// Low level interrupt service routine for RF95 connected to interrupt 1
    static void         isr2();

    /// Array of instances connected to interrupts 0 and 1
    static RH_RF95_FSKOOK*     _deviceForInterrupt[];

    /// Index of next interrupt number to use in _deviceForInterrupt
    static uint8_t      _interruptCount;

    /// The configured interrupt pin connected to this instance
    uint8_t             _interruptPin;

    /// The index into _deviceForInterrupt[] for this device (if an interrupt is already allocated)
    /// else 0xff
    uint8_t             _myInterruptIndex;

    /// The radio OP mode to use when mode is RHModeIdle
    uint8_t             _idleMode; 

    /// The reported device type
    uint8_t             _deviceType;

    /// The selected output power in dBm
    int8_t              _power;

    /// The message length in _buf
    volatile uint8_t    _bufLen;

    /// Array of octets of the last received message or the next to transmit message
    uint8_t             _buf[RH_RF95_MAX_MESSAGE_LEN];

    /// True when there is a valid message in the Rx buffer
    volatile bool    _rxBufValid;

    /// Time in millis since the last preamble was received (and the last time the RSSI was measured)
    uint32_t            _lastPreambleTime;

    bool                _shouldSendHeaderInPayload;
    bool                _fixedLengthMode;
};


#endif

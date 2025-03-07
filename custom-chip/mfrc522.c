///
/// MFRC522 wokwi simulator
///
#include "crc.h"
#include "fifo.h"
#include "picc.h"
#include "wokwi-api.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// #define RUST_DRIVER
// #define DEBUG_SPI

const uint8_t UID[4] = {0xde, 0xad, 0xbe, 0xef};

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

// Constants taken from the c++ arduino driver:
// https://github.com/miguelbalboa/rfid/blob/4b95b9010a35629d727c759b62de7e58175e7f44/src/MFRC522.h
// BEGIN <MFRC522.h>
const uint8_t MFRC522_firmware_referenceV1_0[] = {
    0x00, 0xC6, 0x37, 0xD5, 0x32, 0xB7, 0x57, 0x5C, 0xC2, 0xD8, 0x7C,
    0x4D, 0xD9, 0x70, 0xC7, 0x73, 0x10, 0xE6, 0xD2, 0xAA, 0x5E, 0xA1,
    0x3E, 0x5A, 0x14, 0xAF, 0x30, 0x61, 0xC9, 0x70, 0xDB, 0x2E, 0x64,
    0x22, 0x72, 0xB5, 0xBD, 0x65, 0xF4, 0xEC, 0x22, 0xBC, 0xD3, 0x72,
    0x35, 0xCD, 0xAA, 0x41, 0x1F, 0xA7, 0xF3, 0x53, 0x14, 0xDE, 0x7E,
    0x02, 0xD9, 0x0F, 0xB5, 0x5E, 0x25, 0x1D, 0x29, 0x79};

// MFRC522 registers. Described in chapter 9 of the datasheet.
// When using SPI all addresses are shifted one bit left in the "SPI address
// byte" (section 8.1.2.3)
enum PCD_Register {
    // Page 0: Command and status
    //						  0x00			//
    // reserved for future use
    CommandReg = 0x01, // starts and stops command execution
    ComIEnReg = 0x02,  // enable and disable interrupt request control bits
    DivIEnReg = 0x03,  // enable and disable interrupt request control bits
    ComIrqReg = 0x04,  // interrupt request bits
    DivIrqReg = 0x05,  // interrupt request bits
    ErrorReg = 0x06, // error bits showing the error status of the last command
                     // executed
    Status1Reg = 0x07,    // communication status bits
    Status2Reg = 0x08,    // receiver and transmitter status bits
    FIFODataReg = 0x09,   // input and output of 64 byte FIFO buffer
    FIFOLevelReg = 0x0A,  // number of bytes stored in the FIFO buffer
    WaterLevelReg = 0x0B, // level for FIFO underflow and overflow warning
    ControlReg = 0x0C,    // miscellaneous control registers
    BitFramingReg = 0x0D, // adjustments for bit-oriented frames
    CollReg = 0x0E, // bit position of the first bit-collision detected on the
                    // RF interface
                    //						  0x0F
                    //// reserved for future use

    // Page 1: Command
    // 						  0x10			//
    // reserved for future use
    ModeReg = 0x11,      // defines general modes for transmitting and receiving
    TxModeReg = 0x12,    // defines transmission data rate and framing
    RxModeReg = 0x13,    // defines reception data rate and framing
    TxControlReg = 0x14, // controls the logical behavior of the antenna driver
                         // pins TX1 and TX2
    TxASKReg = 0x15,     // controls the setting of the transmission modulation
    TxSelReg = 0x16,     // selects the internal sources for the antenna driver
    RxSelReg = 0x17,     // selects internal receiver settings
    RxThresholdReg = 0x18, // selects thresholds for the bit decoder
    DemodReg = 0x19,       // defines demodulator settings
                           // 						  0x1A
                           // // reserved for future use
                           // 0x1B			// reserved for future use
    MfTxReg = 0x1C, // controls some MIFARE communication transmit parameters
    MfRxReg = 0x1D, // controls some MIFARE communication receive parameters
                    // 						  0x1E
                    // // reserved for future use
    SerialSpeedReg = 0x1F, // selects the speed of the serial UART interface

    // Page 2: Configuration
    // 						  0x20			//
    // reserved for future use
    CRCResultRegH = 0x21, // shows the MSB and LSB values of the CRC calculation
    CRCResultRegL = 0x22,
    // 						  0x23			//
    // reserved for future use
    ModWidthReg = 0x24, // controls the ModWidth setting?
                        // 						  0x25
                        // // reserved for future use
    RFCfgReg = 0x26,    // configures the receiver gain
    GsNReg = 0x27, // selects the conductance of the antenna driver pins TX1 and
                   // TX2 for modulation
    CWGsPReg = 0x28,  // defines the conductance of the p-driver output during
                      // periods of no modulation
    ModGsPReg = 0x29, // defines the conductance of the p-driver output during
                      // periods of modulation
    TModeReg = 0x2A,  // defines settings for the internal timer
    TPrescalerReg = 0x2B, // the lower 8 bits of the TPrescaler value. The 4
                          // high bits are in TModeReg.
    TReloadRegH = 0x2C,   // defines the 16-bit timer reload value
    TReloadRegL = 0x2D,
    TCounterValueRegH = 0x2E, // shows the 16-bit timer value
    TCounterValueRegL = 0x2F,

    // Page 3: Test Registers
    // 						  0x30			//
    // reserved for future use
    TestSel1Reg = 0x31,  // general test signal configuration
    TestSel2Reg = 0x32,  // general test signal configuration
    TestPinEnReg = 0x33, // enables pin output driver on pins D1 to D7
    TestPinValueReg =
        0x34, // defines the values for D1 to D7 when it is used as an I/O bus
    TestBusReg = 0x35,    // shows the status of the internal test bus
    AutoTestReg = 0x36,   // controls the digital self-test
    VersionReg = 0x37,    // shows the software version
    AnalogTestReg = 0x38, // controls the pins AUX1 and AUX2
    TestDAC1Reg = 0x39,   // defines the test value for TestDAC1
    TestDAC2Reg = 0x3A,   // defines the test value for TestDAC2
    TestADCReg = 0x3B,    // shows the value of ADC I and Q channels
                          // 						  0x3C
                          // // reserved for production tests
                          // 0x3D
                          // // reserved for production tests
                          // 0x3E
                          // // reserved for production tests
                          // 0x3F
                          // // reserved for production tests
    MaxReg = 0x80,
};
const char *reg_name(uint8_t reg) {
    switch (reg) {
    case CommandReg:
        return "CommandReg";
    case ComIEnReg:
        return "ComIEnReg";
    case DivIEnReg:
        return "DivIEnReg";
    case ComIrqReg:
        return "ComIrqReg";
    case DivIrqReg:
        return "DivIrqReg";
    case ErrorReg:
        return "ErrorReg";
    case Status1Reg:
        return "Status1Reg";
    case Status2Reg:
        return "Status2Reg";
    case FIFODataReg:
        return "FIFODataReg";
    case FIFOLevelReg:
        return "FIFOLevelReg";
    case WaterLevelReg:
        return "WaterLevelReg";
    case ControlReg:
        return "ControlReg";
    case BitFramingReg:
        return "BitFramingReg";
    case CollReg:
        return "CollReg";
    case ModeReg:
        return "ModeReg";
    case TxModeReg:
        return "TxModeReg";
    case RxModeReg:
        return "RxModeReg";
    case TxControlReg:
        return "TxControlReg";
    case TxASKReg:
        return "TxASKReg";
    case TxSelReg:
        return "TxSelReg";
    case RxSelReg:
        return "RxSelReg";
    case RxThresholdReg:
        return "RxThresholdReg";
    case DemodReg:
        return "DemodReg";
    case MfTxReg:
        return "MfTxReg";
    case MfRxReg:
        return "MfRxReg";
    case SerialSpeedReg:
        return "SerialSpeedReg";
    case CRCResultRegH:
        return "CRCResultRegH";
    case CRCResultRegL:
        return "CRCResultRegL";
    case ModWidthReg:
        return "ModWidthReg";
    case RFCfgReg:
        return "RFCfgReg";
    case GsNReg:
        return "GsNReg";
    case CWGsPReg:
        return "CWGsPReg";
    case ModGsPReg:
        return "ModGsPReg";
    case TModeReg:
        return "TModeReg";
    case TPrescalerReg:
        return "TPrescalerReg";
    case TReloadRegH:
        return "TReloadRegH";
    case TReloadRegL:
        return "TReloadRegL";
    case TCounterValueRegH:
        return "TCounterValueRegH";
    case TCounterValueRegL:
        return "TCounterValueRegL";
    case TestSel1Reg:
        return "TestSel1Reg";
    case TestSel2Reg:
        return "TestSel2Reg";
    case TestPinEnReg:
        return "TestPinEnReg";
    case TestPinValueReg:
        return "TestPinValueReg";
    case TestBusReg:
        return "TestBusReg";
    case AutoTestReg:
        return "AutoTestReg";
    case VersionReg:
        return "VersionReg";
    case AnalogTestReg:
        return "AnalogTestReg";
    case TestDAC1Reg:
        return "TestDAC1Reg";
    case TestDAC2Reg:
        return "TestDAC2Reg";
    case TestADCReg:
        return "TestADCReg";
    default:
        return "Unknown";
    }
}

// MFRC522 commands. Described in chapter 10 of the datasheet.
enum PCD_Command {
    PCD_Idle = 0x00,             // no action, cancels current command execution
    PCD_Mem = 0x01,              // stores 25 bytes into the internal buffer
    PCD_GenerateRandomID = 0x02, // generates a 10-byte random ID number
    PCD_CalcCRC = 0x03, // activates the CRC coprocessor or performs a self-test
    PCD_Transmit = 0x04,    // transmits data from the FIFO buffer
    PCD_NoCmdChange = 0x07, // no command change, can be used to modify the
                            // CommandReg register bits without affecting the
                            // command, for example, the PowerDown bit
    PCD_Receive = 0x08,     // activates the receiver circuits
    PCD_Transceive =
        0x0C, // transmits data from FIFO buffer to antenna and automatically
              // activates the receiver after transmission
    PCD_MFAuthent =
        0x0E, // performs the MIFARE standard authentication as a reader
    PCD_SoftReset = 0x0F // resets the MFRC522
};

enum CommIRQBits {
    /// ComIrqReg: timer decrements the timer value in register TCounterValReg
    /// to zero
    TIMER_IRQ = 1 << 0,
    /// ComIrqReg: an error bit in ErrorReg is set
    ERR_IRQ = 1 << 1,
    /// ComIrqReg: TODO
    IDLE_IRQ = 1 << 4,
    /// ComIrqReg: receiver detected the end of a valid data stream
    RX_IRQ = 1 << 5,
    /// ComIrqReg: last bit of the transmitted data was sent out
    TX_IRQ = 1 << 6,
};
enum DivIRQBits {
    /// DivIrqReg: CalcCRC command is active and all data is processed
    CRC_IRQ = 1 << 2,
    /// DevIrqReg: MFIN is active
    MFIN_ACTIVE = 1 << 4,
};
// End <MFRC522.h>

enum State {
    ExpectAddress,
    ExpectForData,
    ExpectData,
    ExpectZero,
};
const char *state_name(uint8_t state) {
    switch (state) {
    case ExpectAddress:
        return "ExpectAddress";
    case ExpectForData:
        return "ExpectForData";
    case ExpectData:
        return "ExpectData";
    case ExpectZero:
        return "ExpectZero";
    default:
        return "";
    }
}

typedef struct {
    pin_t cs_pin;
    pin_t irq_pin;
    pin_t rst_pin;

    uint32_t spi;
    // uint8_t spi_buffer[128]; // FIXME: Haven't gotten this to work since
    uint8_t spi_buffer[1];
    uint8_t send_size;

    /* State Machine for MODE, REGISTER */
    uint8_t state;
    uint8_t address;
    FIFO fifo;
    uint8_t internal_mem[25];
    uint8_t registers[MaxReg];
    uint32_t card;
    bool card_informed;
} chip_state_t;

static void chip_pin_change(void *user_data, pin_t pin, uint32_t value);
static void chip_spi_done(void *user_data, uint8_t *buffer, uint32_t count);
void process_command(chip_state_t *chip, uint8_t *buffer, uint32_t buffer_size);
uint8_t readRegister(chip_state_t *chip, uint8_t address);
void writeRegister(chip_state_t *chip, uint8_t address, uint8_t data);
bool chip_spi_selected(chip_state_t *chip);
void handle_picc_cmd(chip_state_t *chip, uint8_t cmd);
void chip_timer_callback(chip_state_t *chip);
void _chip_timer_callback(void *user_data) {
    printf("Timer callback\n");
    chip_timer_callback((chip_state_t *)user_data);
}

void propagate_irq(chip_state_t *chip) {
    bool propagate_irq =
        (0x7f & chip->registers[ComIEnReg] & chip->registers[ComIrqReg]) != 0;
    propagate_irq =
        propagate_irq ||
        ((chip->registers[DivIEnReg] & 0x14 & chip->registers[DivIrqReg]) != 0);
    // bool irq_active_high = (chip->registers[ComIEnReg] & 0x80) != 0;
    if (propagate_irq) {
        printf("Propagating to IRQ pin (Set)\n");
        // pin_write(chip->irq_pin, irq_active_high ? HIGH : LOW);
        pin_write(chip->irq_pin, LOW);
    } else {
        printf("Propagating to IRQ pin (Clear)\n");
        // pin_write(chip->irq_pin, irq_active_high ? LOW : HIGH);
        pin_write(chip->irq_pin, HIGH);
    }
}
void clear_irq(chip_state_t *chip, uint8_t clear_bits) {
    uint8_t previous_irq = chip->registers[ComIrqReg];
    chip->registers[ComIrqReg] &= ~clear_bits;
    printf("Clearing IRQ bits to %02x (%02x -> %02x). Will inform on %02x\n",
           (uint8_t)~clear_bits, previous_irq, chip->registers[ComIrqReg],
           0x7f & chip->registers[ComIEnReg]);
    propagate_irq(chip);
}
void set_irq(chip_state_t *chip, uint8_t set_bits) {
    uint8_t previous_irq = chip->registers[ComIrqReg];
    chip->registers[ComIrqReg] |= set_bits;
    printf("Setting IRQ bits to %02x (%02x -> %02x). Will inform on %02x\n",
           set_bits, previous_irq, chip->registers[ComIrqReg],
           0x7f & chip->registers[ComIEnReg]);
    propagate_irq(chip);
}

// uint8_t fifo_low_watermark(chip_state_t *chip) {
//     chip->registers[Status1Reg]
// }
uint16_t timer_pre_scaler(chip_state_t *chip) {
    return (((uint16_t)chip->registers[TModeReg] & 0x7) << 8) |
           chip->registers[TPrescalerReg];
}
uint16_t timer_reload(chip_state_t *chip) {
    return ((uint16_t)chip->registers[TReloadRegH] << 8) |
           chip->registers[TReloadRegL];
}
uint64_t timer_in_usec(chip_state_t *chip) {
    return 100 * (uint64_t)timer_pre_scaler(chip) *
           (1 + (uint64_t)timer_reload(chip)) / 678;
}
uint8_t fifo_water_level(chip_state_t *chip) {
    return chip->registers[WaterLevelReg] & ~(1 << 6);
}
void chip_reset(chip_state_t *chip) {
    // Reset the registers
    bzero(chip->registers, sizeof(chip->registers));
    bzero(chip->spi_buffer, sizeof(chip->spi_buffer));

    // Reset the command state machine
    chip->state = ExpectAddress;
    chip->address = 0xff;

    chip->send_size = 0;

    // put version in place
    writeRegister(chip, VersionReg, 0x91);
}

void chip_init(void) {
    chip_state_t *chip = malloc(sizeof(chip_state_t));
    bzero(chip, sizeof(chip_state_t));

    chip->cs_pin = pin_init("CS", INPUT_PULLUP);
    chip->irq_pin = pin_init("IRQ", OUTPUT);
    pin_write(chip->irq_pin, HIGH);
    chip->rst_pin = pin_init("RST", INPUT_PULLUP);

    chip->card = attr_init("cardPresent", 0);

    pin_write(chip->irq_pin, HIGH);

    const pin_watch_config_t watch_config = {
        .edge = BOTH,
        .pin_change = chip_pin_change,
        .user_data = chip,
    };
    pin_watch(chip->cs_pin, &watch_config);

    const spi_config_t spi_config = {
        .sck = pin_init("SCK", INPUT),
        .miso = pin_init("MISO", OUTPUT),
        .mosi = pin_init("MOSI", INPUT),
        .done = chip_spi_done,
        .user_data = chip,
    };
    chip->spi = spi_init(&spi_config);

    chip_reset(chip);

    uint32_t interval = 10000;
    const timer_config_t config = {
        .callback = _chip_timer_callback,
        .user_data = chip,
    };
    timer_t timer_id = timer_init(&config);
    timer_start(timer_id, interval, true);
#ifdef RUST_DRIVER
    printf("Expecting extra CS toggle (Rust Mode)\n");
#endif

    printf("MFRC522 Simulator Initialized!\n");
}

void chip_pin_change(void *user_data, pin_t pin, uint32_t value) {
    chip_state_t *chip = (chip_state_t *)user_data;

    // Handle CS pin logic
    if (pin == chip->cs_pin) {
        if (value == LOW) {
#ifdef DEBUG_SPI
            printf("CS enabled\n");
#endif
            spi_start(chip->spi, chip->spi_buffer, sizeof(chip->spi_buffer));
        } else {
#ifdef DEBUG_SPI
            printf("CS disabled\n");
#endif

            spi_stop(chip->spi);
#ifdef RUST_DRIVER
            // FIXME: When using the rust driver CS is toggled an extra time,
            // need to investigate how to handle this better than simply an
            // extra wait for data state. With the cpp driver a single
            // transaction sends all the data for a write, so when it goes high
            // we can start waiting for address again.
            if (chip->state == ExpectForData) {
                chip->state = ExpectData;
            } else
#endif
            {
                chip->state = ExpectAddress;
            }
        }
    }

    // Handle RST pin
    if (pin == chip->rst_pin && value == LOW) {
        spi_stop(chip->spi); // Process remaining data in SPI buffer
        printf("Resetting MCP23S17\n");
        chip_reset(chip);
    }
}

void maybe_send(chip_state_t *chip) {
    // If CS is still low, it means there's more data to receive
    if (chip_spi_selected(chip) /*&& chip->send_size*/) {
#ifdef DEBUG_SPI
        printf("Bytes FOR wire : (%d) ", chip->send_size);
        for (int i = 0; i < chip->send_size; i++) {
            printf(" %02x", chip->spi_buffer[i]);
        }
        printf("\n");
#endif
        // spi_start(chip->spi, chip->spi_buffer, /*chip->send_size*/ 1);
        spi_start(chip->spi, chip->spi_buffer, sizeof(chip->spi_buffer));
        chip->send_size = 0;
    }
}

uint8_t generate_sak(uint8_t selcmd) {
    uint8_t sak = 0x00;

    switch (selcmd) {
    case PICC_CMD_SEL_CL1: // Single UID or first part of long UID
        sak = 0x08;        // MIFARE Classic 1K
        break;
    case PICC_CMD_SEL_CL2: // More UID bytes exist
        sak = 0x20;        // UID not complete
        break;
    case PICC_CMD_SEL_CL3: // Final UID part for long UIDs
        sak = 0x00;        // Final UID level
        break;
    default:
        sak = 0x00; // Default case, unknown behavior
        break;
    }

    // Bit 6 must always be 0 (RFU)
    sak &= ~(1 << 6);

    return sak;
}

void poll_command_loop(chip_state_t *chip) {
    uint8_t command = chip->registers[CommandReg] & 0x0f;
    switch (command) {
    case PCD_Idle:
        printf("Command: Idle\n");
        break;
    case PCD_Mem:
        printf("Command: Mem\n");
        if (chip->fifo.count == 0) {
            for (int i = 0; i < 25; i++) {
                fifo_write(&chip->fifo, chip->internal_mem[i]);
            }
        } else {
            uint8_t size = MIN(chip->fifo.count, 25);
            for (int i = 0; i < size; i++) {
                uint8_t x = 0;
                fifo_read(&chip->fifo, &x);
                chip->internal_mem[i] = x;
            }
        }
        chip->registers[CommandReg] = PCD_Idle;
        set_irq(chip, IDLE_IRQ);
        break;
    case PCD_GenerateRandomID:
        printf("Command: GenerateRandomID\n");
        for (int i = 0; i < 10; i++) {
            chip->internal_mem[i] = rand();
        }
        chip->registers[CommandReg] = PCD_Idle;
        set_irq(chip, IDLE_IRQ);
        break;
    case PCD_CalcCRC:
        // The content of the FIFO is transferred to the CRC co-processor and a
        // CRC calculation is started. The result of this calculation is stored
        // in the CRCResultReg register. The CRC calculation is not limited to a
        // dedicated number of bytes. The calculation is not stopped, when the
        // FIFO gets empty during the data stream. The next byte written to the
        // FIFO is added to the calculation

        // The pre-set value of the CRC is controlled by the CRCPreset bits of
        // the ModeReg register and the value is loaded to the CRC co-processor
        // when the command is started.

        // This command has to be terminated by writing any command to register
        // CommandReg e.g. the command Idle.

        // If the SelfTest bits in the register AutoTestReg are set correct, the
        // MFRC522 is in Self Test mode and starting the CalCCRC command
        // performs a digital selftest. The result of the selftest is written to
        // the FIFO.
        printf("Command: CalcCRC\n");
        if (chip->registers[AutoTestReg] !=
            0x00) // TODO: Check specifically for the SelfTest bits
        {
            printf("Copying data to the fifo register");
            fifo_flush(&chip->fifo);
            bool success = fifo_write_many(
                &chip->fifo, MFRC522_firmware_referenceV1_0, FIFO_SIZE);
            if (!success) {
                printf("Error writing firmware reference to fifo");
                return;
            }
            chip->registers[FIFOLevelReg] = FIFO_SIZE;
        } else {
            printf("Calculating CRC for %d bytes in fifo ", chip->fifo.count);
            fifo_print(&chip->fifo);
            printf("\n");
            if (chip->fifo.count > 0) {
                uint16_t crc = (chip->registers[CRCResultRegH] << 8) |
                               chip->registers[CRCResultRegL];
                uint8_t crc_mode = chip->registers[ModeReg] & 0x3;
                if (crc_mode != 1) {
                    printf("WARN: Unexpected crc settings in ModeReg\n");
                }
                // TODO: crc init depends on the bit[1:2] of the ModeReg,
                // 00->0x0000, 01->6363 (implemented), 10->A671, 11->ffff
                while (chip->fifo.count > 0) {
                    uint8_t x = 0;
                    if (!fifo_read(&chip->fifo, &x))
                        break;
                    crc = crc_update(crc, x);
                }
                chip->registers[CRCResultRegH] = crc >> 8;
                chip->registers[CRCResultRegL] = crc & 0xff;
                chip->registers[Status1Reg] =
                    (1 << 5) | (1 << 4);                // set CRCOk | CRCReady
                chip->registers[DivIrqReg] |= (1 << 2); // set CRC_IRQ bit

                // TODO: if we are in CRC mode and we get more data it continues
                // to be appended to the crc. Doesn't look like the c++ or rust
                // drivers are hitting this case though.
            }
        }
        break;
    case PCD_Transmit:
        printf("Command Transmit\n");
        break;
    case PCD_NoCmdChange:
        printf("Command: NoCmdChange\n");
        break;
    case PCD_Receive:
        printf("Command: Receive\n");
        break;
    case PCD_Transceive: {
        // This circular command repeats transmitting data from the FIFO and
        // receiving data from the RF field continuously. The first action is
        // transmitting and after a transmission the command is changed to
        // receive a data stream.

        // Each transmission process has to be started by setting bit StartSend
        // in the register BitFramingReg to logic 1. This command has to be
        // cleared by software by writing any command to register CommandReg
        // e.g. the command idle.

        // Remark: If the bit RxMultiple in register RxModeReg is set to logic
        // 1, this command will never leave the receiving state, because the
        // receiving will not be cancelled automatically

        // TODO: We need some tranceive state tx/rx, because otherwise we try to
        // process the outgoing FIFO as input. By clearing the bit frameing we
        // cover most cases though.
        if ((chip->registers[BitFramingReg] & (1 << 7)) == 0) {
            printf("Tranceive but StartSend, bailing\n");
            return;
        }
        if (chip->fifo.count == 0) {
            printf("No fifo input in transceive, bailing ?\n");
            return;
        }
        uint8_t cmd = 0;
        printf("Command: Transceive fifo: %d: ", chip->fifo.count);
        fifo_print(&chip->fifo);
        printf("\n");
        fifo_peek(&chip->fifo, 0, &cmd);
        handle_picc_cmd(chip, cmd);

        break;
    }
    case PCD_MFAuthent:
        // In total 12 bytes shall be written to the FIFO. (see docs for
        // details)

        // This command terminates automatically when the MIFARE ® card is
        // authenticated and the bit MFCrypto1On in the Status2Reg register is
        // set to logic 1.

        // This command does not terminate automatically when the card does not
        // answer, therefore the timer should be initialized to automatic mode.
        // In this case, beside the bit IdleIrq, the bit TimerIrq can be used as
        // termination criteria. During authentication processing the bit RxIrq
        // and bit TxIrq are blocked. The Crypto1On bit is only valid after
        // termination of the authent command (either after processing the
        // protocol or after writing IDLE to the command register).

        // In case there is an error during authentication, the bit ProtocolErr
        // in the ErrorReg register is set to logic 1 and the bit Crypto1On in
        // register Status2Reg is set to logic 0
        printf("Command: MFAuthent\n");
        chip->registers[Status2Reg] |= (1 << 3); // Crypto1On
        chip->registers[CommandReg] = PCD_Idle;
        set_irq(chip, IDLE_IRQ);
        break;
    case PCD_SoftReset:
        printf("Command: SoftReset\n");
        chip_reset(chip);
        break;
    default:
        printf("Command not recognized 0x%02x\n", command);
    }
}

void chip_timer_callback(chip_state_t *chip) { /*poll_command_loop(chip);*/ }

void handle_picc_cmd(chip_state_t *chip, uint8_t cmd) {
    // https://www.mouser.com/ds/2/302/MF1S503x-89574.pdf
    switch (cmd) {
    case PICC_CMD_REQA:
    case PICC_CMD_WUPA: {
        printf("PICC Cmd: %s\n", 0x26 == cmd ? "REQA" : "WUPA");
        // pop command
        fifo_read(&chip->fifo, &cmd);

        uint32_t card = attr_read(chip->card);
        printf("attr card = %d\n", card);
        if (card != 0 /*&& !chip->card_informed*/) {
            printf("Card detected\n");
            chip->card_informed = true;

            fifo_flush(&chip->fifo);
            // ATQA
            fifo_write(&chip->fifo, 0x44);
            fifo_write(&chip->fifo, 0x00);
            chip->registers[BitFramingReg] &=
                ~(1 << 7); // clear send bit host will reset
            chip->registers[ControlReg] =
                0x0; // clear at least the valid bits to mean last byte is fully
                     // valid
            chip->registers[FIFOLevelReg] = chip->fifo.count;
            set_irq(chip, TX_IRQ | RX_IRQ);
            // propagate_irq(chip); // TODO: always propagate from set_irq, but
            // need to debug host side first
        } else {
            printf("Setting timeout bit into IRQ reg\n");
            set_irq(chip, TIMER_IRQ); // Set timeout bit
        }
        break;
    }
    case PICC_CMD_SEL_CL1:
    case PICC_CMD_SEL_CL2:
    case PICC_CMD_SEL_CL3: {
        // pop cmd
        if (chip->fifo.count < 2) {
            // expect length
            printf("Expect length, waiting for more data in fifo\n");
            return;
        }
        printf("PICC Cmd: Select\n");
        uint8_t len = 0;
        fifo_peek(&chip->fifo, 1, &len);
        len = len >> 4;
        if (chip->fifo.count < len) {
            // expect message of len
            printf("Expect %d bytes per command, waiting for more data in fifo "
                   "(have=%d)\n",
                   len, chip->fifo.count);
            return;
        }
        // pop cmd and length
        fifo_read(&chip->fifo, &cmd);
        fifo_read(&chip->fifo, &len);
        len = len >> 4;

        uint8_t sak = generate_sak(cmd);
        uint16_t crc = crc_init();
        crc = crc_update(crc, sak);
        if (len == 2) {
            printf("responding with UID\n");
            fifo_flush(&chip->fifo);
            uint8_t bcc = 0;
            for (int i = 0; i < sizeof(UID); i++) {
                fifo_write(&chip->fifo, UID[i]);
                bcc ^= UID[i];
            }
            fifo_write(&chip->fifo, bcc);
        } else {
            printf("responding with SAK\n");
            // send SAK
            fifo_flush(&chip->fifo);
            fifo_write(&chip->fifo, sak);
            fifo_write(&chip->fifo, crc & 0xff);
            fifo_write(&chip->fifo, crc >> 8);
        }

        chip->registers[FIFOLevelReg] = chip->fifo.count;
        chip->registers[ControlReg] = 0x0; // clear at least the valid bits to
                                           // mean last byte is fully valid
        chip->registers[BitFramingReg] &=
            ~(1 << 7); // clear send bit host will reset
        set_irq(chip, TX_IRQ | RX_IRQ);
        break;
    }
    case PICC_CMD_MF_READ: {
        // [0x30 ADDR CRC CRC]
        if (chip->fifo.count < 4) {
            return;
        }
        printf("PICC Cmd: Read\n");
        uint8_t address = 0, crcl = 0, crch = 0;
        fifo_read(&chip->fifo, &cmd);
        fifo_read(&chip->fifo, &address);
        fifo_read(&chip->fifo, &crcl);
        fifo_read(&chip->fifo, &crch);
        uint16_t crc = crc_init();
        crc = crc_update(crc, cmd);
        crc = crc_update(crc, address);
        if ((crc & 0xff) != crcl || (crc >> 8) != crch) {
            printf("CRC mismatch, rejecting read command\n");
            // send nak?
            return;
        }
        crc = crc_init();
        fifo_flush(&chip->fifo);
        // TODO: Fix these so they look good to the host?
        if (address == 0) {
            // mfg block, send NUID first
            for (int i = 0; i < sizeof(UID); i++) {
                fifo_write(&chip->fifo, UID[i]);
                crc = crc_update(crc, UID[i]);
            }
            for (int i = sizeof(UID); i < 16; i++) {
                fifo_write(&chip->fifo, 0x55);
                crc = crc_update(crc, 0x55);
            }
        } else if ((address & 0x3) == 0x3) {
            // sector trailer
            // keya
            for (int i = 0; i < 5; i++) {
                fifo_write(&chip->fifo, 0x00);
                crc = crc_update(crc, 0x00);
            }
            // access bits
            fifo_write(&chip->fifo, 0xff);
            crc = crc_update(crc, 0xff);
            fifo_write(&chip->fifo, 0x0f);
            crc = crc_update(crc, 0x0f);
            fifo_write(&chip->fifo, 0x00);
            crc = crc_update(crc, 0x00);
            // key b
            for (int i = 0; i < 5; i++) {
                fifo_write(&chip->fifo, 0x00);
                crc = crc_update(crc, 0x00);
            }
        } else {
            // data block
            for (int i = 0; i < 12; i++) {
                fifo_write(&chip->fifo, 0x00);
                crc = crc_update(crc, 0x00);
            }
            fifo_write(&chip->fifo, address);
            crc = crc_update(crc, address);
            fifo_write(&chip->fifo, ~address);
            crc = crc_update(crc, ~address);
            fifo_write(&chip->fifo, address);
            crc = crc_update(crc, address);
            fifo_write(&chip->fifo, ~address);
            crc = crc_update(crc, ~address);
        }
        // write the crc
        fifo_write(&chip->fifo, crc & 0xff);
        fifo_write(&chip->fifo, crc >> 8);

        chip->registers[FIFOLevelReg] = chip->fifo.count;
        chip->registers[ControlReg] = 0x0; // clear at least the valid bits to
                                           // mean last byte is fully valid
        chip->registers[BitFramingReg] &=
            ~(1 << 7); // clear send bit host will reset
        set_irq(chip, TX_IRQ | RX_IRQ);
        break;
    }
    case PICC_CMD_HLTA: {
        if (chip->fifo.count < 4) {
            return;
        }
        printf("PICC Cmd: Halt\n");
        fifo_read(&chip->fifo, &cmd);
        uint8_t zero = 0;
        uint8_t crcl = 0, crch = 0;
        fifo_read(&chip->fifo, &zero);
        fifo_read(&chip->fifo, &crcl);
        fifo_read(&chip->fifo, &crch);
        if (0 != zero) {
            printf("Strange: Halt command not zero\n");
            return;
        }
        set_irq(chip, TX_IRQ);
        break;
    }
    case PICC_CMD_RATS:
    case PICC_CMD_MF_AUTH_KEY_A:
    case PICC_CMD_MF_AUTH_KEY_B:
    case PICC_CMD_MF_WRITE:
    case PICC_CMD_MF_DECREMENT:
    case PICC_CMD_MF_INCREMENT:
    case PICC_CMD_MF_RESTORE:
    case PICC_CMD_MF_TRANSFER:
    case PICC_CMD_UL_WRITE:
        printf("PICC Cmd: %02x (unsupported)\n", cmd);
        break;
    default:
        printf("Unrecognized PICC cmd: 0x%02x\n", cmd);
        // fifo_flush(&chip->fifo);
    }
}

void spi_send(chip_state_t *chip, uint8_t data) {
    // Should never happen, we always only send a single byte responce to read
    // and nothing in response to write; but for sanity...
    if (chip->send_size > sizeof(chip->spi_buffer)) {
        printf("SPI buffer overflow\n");
        chip->send_size = 0;
    }
    chip->spi_buffer[chip->send_size] = data;
    chip->send_size++;
}

void process_byte(chip_state_t *chip, uint8_t byte) {
    switch (chip->state) {
    case ExpectAddress: {
        chip->address = (byte & ~0x80) >> 1; // clear bit to get real address
        bool reading = 0 != (byte & 0x80);
        if (reading) {
            uint8_t data = readRegister(chip, chip->address);
            spi_send(chip, data);
            chip->state = ExpectZero;
        } else {
            // HACK: I don;t understand the protocol, but additional state to
            // keep data coming after next CS
            if (chip->address == FIFODataReg) {
                chip->state = ExpectForData;
            } else {
                chip->state = ExpectData;
            }
        }
        break;
    }
    case ExpectZero: {
        // for bulk read if the user sends us the same address again we can
        // proceed to read again until we get a 0x00.
        bool reading = 0 != (byte & 0x80);
        uint8_t address = (byte & ~0x80) >> 1;
        if (reading && address == chip->address) {
            printf("another byte please\n");
            uint8_t data = readRegister(chip, chip->address);
            spi_send(chip, data);
            return;
        }
        if (byte != 0) {
            printf("Warn: Expected to read 0\n");
        }
        chip->state = ExpectAddress;
        break;
    }

    case ExpectData:
    case ExpectForData: {
        // chip->state = ExpectAddress;
        writeRegister(chip, chip->address, byte);
        break;
    }
    }
}

void process_buffer(chip_state_t *chip, uint8_t *buffer, uint32_t buffer_size) {
#ifdef DEBUG_SPI
    printf("Bytes from wire : %s: 0x%x (%d)", state_name(chip->state),
           chip->address, buffer_size);
    for (int i = 0; i < buffer_size; i++) {
        printf(" %02x", buffer[i]);
    }
    printf("\n");
#endif

    for (int i = 0; i < buffer_size; i++) {
        process_byte(chip, buffer[i]);
    }
    poll_command_loop(chip); // FIXME: can we call this from timer instead?
}

void chip_spi_done(void *user_data, uint8_t *buffer, uint32_t count) {
    chip_state_t *chip = (chip_state_t *)user_data;
    if (!count) {
        // This means that we got here from spi_stop, and no data was received
        return;
    }

    process_buffer(chip, buffer, count);

    maybe_send(chip);
}

uint8_t readRegister(chip_state_t *chip, uint8_t address) {
    if (address >= MaxReg) {
        printf("Warn: Register read out of range: 0x%x\n", address);
        return 0;
    }

    uint8_t data = chip->registers[address];

    if (address == FIFODataReg) {
        data = 0;
        bool success = fifo_read(&chip->fifo, &data);
        printf("~~Read from fifo (%d) 0x%02x, %d\n", chip->fifo.count, data,
               success);
        if (chip->fifo.count < fifo_water_level(chip)) {
            printf("WARN: waterlevel low alert");
            // TODO: Set underflow flag
        }
    }

    if (address == FIFOLevelReg) {
        data = chip->fifo.count;
    }

    if (address == ControlReg) {
        // always return 0 for the timer start/ stop bits
        data = data & 0x3;
    }

    printf("Reading register: %s (0x%x) data=0x%x\n", reg_name(address),
           address, data);

    return data;
}

void writeRegister(chip_state_t *chip, uint8_t address, uint8_t data) {
    if (address >= MaxReg) {
        printf("Warn: Register write out of range: 0x%x\n", address);
        return;
    }
    printf("Writing register: %s (0x%x) data=0x%x\n", reg_name(address),
           address, data);
    if (address == VersionReg) {
        data = 0x91;
    }

    if (address == CommandReg) {
        data = data & ~(1 << 4); // always clear PowerDown bit

        if (data == PCD_CalcCRC) {
            uint16_t crc = crc_init();
            chip->registers[CRCResultRegH] = crc >> 8;
            chip->registers[CRCResultRegL] = crc & 0xff;
        }
    }

    if (address == ComIrqReg) {
        bool clear_req = (data & (1 << 7)) == 0;
        // never save the set/clear bit
        data &= ~(1 << 7);
        if (clear_req) {
            clear_irq(chip, data);
        } else {
            // Is this even allowed?
            set_irq(chip, data);
        }
        return; // handled
    }

    if (address == FIFOLevelReg) {
        // printf("setting fifo level to: %d\n", data);
        if (0 != (data & 0x80)) {
            fifo_flush(&chip->fifo);
        }
        return; // handled
    }

    if (address == FIFODataReg) {
        bool success = fifo_write(&chip->fifo, data);
        printf("~~Write to fifo(%d) 0x%02x %d\n", chip->fifo.count, data,
               success);

        if ((64 - chip->fifo.count) < fifo_water_level(chip)) {
            printf("WARN: waterlevel high alert");
            // TODO: Set overflow flag if not success
        }

        return;
    }
    if (address == ControlReg) {
        if ((data & (1 << 6)) != 0) {
            // TStartNow
            printf("~~Start timer every %llu us", timer_in_usec(chip));
        }
        if ((data & (1 << 7)) != 0) {
            // TStopNow
            printf("~~Stop timer");
        }
    }

    chip->registers[address] = data;
}

bool chip_spi_selected(chip_state_t *chip) {
    return (pin_read(chip->cs_pin) == LOW);
}

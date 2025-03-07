
#include <string>
#include <MFRC522.h>

#define MFRC522_SS 8   // GPIO8 for MFRC522 Chip Select
#define MFRC522_RST 15 // GPIO15 for MFRC522 Reset
#define MFRC522_INT 14 // GPIO14 for MFRC522 Interrupt (if used)
#define MFRC522_SCK 36
#define MFRC522_MISO 37
#define MFRC522_MOSI 35

#define CARD_ID_MAX_LEN 6
#define CARD_ID_MIN_LEN 4


struct SystemInfo {
  uint16_t battery_percent;
  bool is_charging;

  bool new_card;
  byte card_uuid[CARD_ID_MAX_LEN];

  bool new_save_result;

  void reset() {
    new_card = false;
    memset(&card_uuid, 0, sizeof(card_uuid));

  }
};
SystemInfo si{};

// Init array that will store new NUID
byte nuidPICC[CARD_ID_MAX_LEN];

MFRC522 rfid(MFRC522_SS, MFRC522_RST); // Instance of the class

volatile bool new_card = false;

void scan_irq() { 
  new_card = true;
  // Serial.println("Resetting cached rfid id"); // don't print in isr ya dummy
}

void setup_rfid() {
  Serial.print("MOSI: ");
  Serial.println(MFRC522_MOSI);
  Serial.print("MISO: ");
  Serial.println(MFRC522_MISO);
  Serial.print("SCK: ");
  Serial.println(MFRC522_SCK);
  Serial.print("SS: ");
  Serial.println(MFRC522_SS);

  SPI.begin(MFRC522_SCK, MFRC522_MISO, MFRC522_MOSI,
            MFRC522_SS); // Init SPI bus
  rfid.PCD_Init();       // Init MFRC522
  // 	pinMode(MFRC522_SS, OUTPUT);
	// digitalWrite(MFRC522_SS, HIGH);
	
	// // If a valid pin number has been set, pull device out of power down / reset state.
	// if (true) {
	// 	// First set the resetPowerDownPin as digital input, to check the MFRC522 power down mode.
	// 	pinMode(MFRC522_RST, INPUT);
	
	// 	if (digitalRead(MFRC522_RST) == LOW) {	// The MFRC522 chip is in power down mode.
	// 		pinMode(MFRC522_RST, OUTPUT);		// Now set the resetPowerDownPin as digital output.
	// 		digitalWrite(MFRC522_RST, LOW);		// Make sure we have a clean LOW state.
	// 		delayMicroseconds(2);				// 8.8.1 Reset timing requirements says about 100ns. Let us be generous: 2μsl
	// 		digitalWrite(MFRC522_RST, HIGH);		// Exit power down mode. This triggers a hard reset.
	// 		// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74μs. Let us be generous: 50ms.
	// 		delay(50);
	// 	}
	// }

  /* read and printout the MFRC522 version (valid values 0x91 & 0x92)*/
  byte readReg = rfid.PCD_ReadRegister(rfid.VersionReg);
  printf("RFID Ver: %02x\n", readReg);
   readReg = rfid.PCD_ReadRegister(rfid.VersionReg);
  printf("RFID Ver: %02x\n", readReg);
     readReg = rfid.PCD_ReadRegister(rfid.VersionReg);
  printf("RFID Ver: %02x\n", readReg);
     readReg = rfid.PCD_ReadRegister(rfid.VersionReg);
  printf("RFID Ver: %02x\n", readReg);
  if (rfid.PCD_PerformSelfTest()) {
    Serial.println("Self test passed\n");
  } else {
    Serial.println("Self test failed\n");
  }

  // Setup the interrupt
  pinMode(MFRC522_INT, INPUT_PULLUP);
  // From:
  // https://github.com/miguelbalboa/rfid/blob/0ff12a1c0afd414a5340930b14a2960bb543c28f/examples/MinimalInterrupt/MinimalInterrupt.ino#L70
  /*
   * Allow the ... irq to be propagated to the IRQ pin
   * For test purposes propagate the IdleIrq and loAlert
   */
  byte regVal = 0xA0; // rx irq
  rfid.PCD_WriteRegister(rfid.ComIEnReg, regVal);
  attachInterrupt(digitalPinToInterrupt(MFRC522_INT), scan_irq, FALLING);
}

/*
 * The function sending to the MFRC522 the needed commands to activate the
 * reception
 */
void activateRec(MFRC522 mfrc522) {
  mfrc522.PCD_WriteRegister(mfrc522.FIFODataReg, mfrc522.PICC_CMD_REQA);
  mfrc522.PCD_WriteRegister(mfrc522.CommandReg, mfrc522.PCD_Transceive);
  mfrc522.PCD_WriteRegister(mfrc522.BitFramingReg, 0x87);
}

/*
 * The function to clear the pending interrupt bits after interrupt serving
 * routine
 */
void clearInt(MFRC522 mfrc522) {
  mfrc522.PCD_WriteRegister(mfrc522.ComIrqReg, 0x7F);
}

unsigned long reset_cached_card = 0;
bool read_card() {
  // Ignore the IsNewCardPresent as we handle this with the ISR now
  // if (!rfid.PICC_IsNewCardPresent()) {
  //   logger->log_message("no new card present\n");
  //   return;
  // }
  // Verify if the NUID has been read
  if (!rfid.PICC_ReadCardSerial()) {
    Serial.println("PICC_ReadCardSerial\n");
    return false;
  }

  MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
  printf("PICC type: %s\n", rfid.PICC_GetTypeName(piccType));
  int rfid_uid_len = rfid.uid.size;
  if (rfid_uid_len < CARD_ID_MIN_LEN) {
    printf("Id too short (%s), bailing\n", rfid.uid.size);
    return false;
  }

  if (memcmp(&rfid.uid.uidByte[0], &nuidPICC[0],
             std::min(rfid_uid_len, CARD_ID_MAX_LEN))) {
    reset_cached_card = millis() + 300000;

    Serial.println("A new card has been detected.\n");
    rfid.PICC_DumpToSerial(&rfid.uid);

    // Store NUID into nuidPICC array
    for (byte i = 0; i < 4; i++) {
      nuidPICC[i] = rfid.uid.uidByte[i];
    }

    printf("The NUID tag is: %s\n",
                        toHex(rfid.uid.uidByte, rfid.uid.size).c_str());

    return true;
  } else {
    Serial.println("Card read previously.\n");
    return false;
  }
}


void card_reader_loop(SystemInfo &system_info) {
  if (reset_cached_card != 0 && millis() > reset_cached_card) {
    Serial.println("Resetting cached rfid id");
    memset(&nuidPICC[0], 0, sizeof(nuidPICC));
    reset_cached_card = 0;
  }
  // Check if we received an interrupt since last loop
  if (!new_card) {
    // Serial.println("no new card present");
  } else {
    if (read_card()) {
      system_info.new_card = true;
      // FIXME: move this outof this location. We should nmotify based on chagne
      // to state machine (else we will never see the pending state on display)
      memset(&system_info.card_uuid, 0, sizeof(system_info.card_uuid));
      memcpy(&system_info.card_uuid, nuidPICC, sizeof(system_info.card_uuid));
    }
    clearInt(rfid);

    // Halt PICC
    rfid.PICC_HaltA();

    new_card = false;

    // Stop encryption on PCD
    rfid.PCD_StopCrypto1();
  }

  activateRec(rfid);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Hello, ESP32-S3!");
  setup_rfid();
  
}

void loop() {
  Serial.println("Loop ...");
  // put your main code here, to run repeatedly:
  delay(100); // this speeds up the simulation
  
  card_reader_loop(si);
}

std::string toHex(byte *buffer, byte bufferSize) {
  char s[bufferSize * 3 + 2] = {};
  for (byte i = 0; i < bufferSize; i++) {
    snprintf(s + i * 3, 4, "%02x ", buffer[i]);
  }
  std::string ret = s;
  return ret;
}

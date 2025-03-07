## MFRC522 custom-chip for wokwi

This is not a faithful implementation of the chip. This was written to toy around with wokwi, the initial goal was a version of the mfrc522 that would get through the init sequences of the arduino driver and the rust driver. See the projects under `tests/` for the examples.

Some useful references:
- Datasheet: https://www.nxp.com/docs/en/data-sheet/MFRC522.pdf
- Application Note: https://www.nxp.com/docs/en/application-note/AN10833.pdf
- MIFARE Classic 1K docs: https://www.mouser.com/ds/2/302/MF1S503x-89574.pdf

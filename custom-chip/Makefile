# SPDX-FileCopyrightText: © 2022 Uri Shaked <uri@wokwi.com>
# SPDX-License-Identifier: MIT

WASI_SDK_PATH=${HOME}/Downloads/wasi-sdk-24.0-arm64-macos

SOURCES = mfrc522.c
TARGET  = dist/mfrc522.wasm

.PHONY: all
all: $(TARGET) dist/mfrc522.json

.PHONY: clean
clean:
		rm -rf dist

dist:
		mkdir -p dist

$(TARGET): dist $(SOURCES) wokwi-api.h
	  $(WASI_SDK_PATH)/bin/clang --target=wasm32-unknown-wasi --sysroot $(WASI_SDK_PATH)/share/wasi-sysroot -nostartfiles -Wl,--import-memory -Wl,--export-table -Wl,--no-entry -Werror -o $(TARGET) $(SOURCES)

dist/mfrc522.json: dist mfrc522.json
	  cp mfrc522.json dist

# .PHONY: test
# test:
# 	  cd test && arduino-cli compile -e -b arduino:avr:uno blink
# 

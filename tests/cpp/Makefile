
# brew install arduino-cli
# arduino-cli config init
# Add 'https://dl.espressif.com/dl/package_esp32_index.json' into that config file
# arduino-cli core update-index
# arduino-cli core install esp32:esp32
#

firmware: build/esp32.esp32.node32s/cpp.ino.bin

build/esp32.esp32.node32s/cpp.ino.bin: cpp.ino
	arduino-cli compile --fqbn esp32:esp32:esp32s3 -e --clean 

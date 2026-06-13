# --- ESP8266 (NodeMCU v2) ---
fqbn := "esp8266:esp8266:nodemcuv2"
port := "/dev/ttyUSB0"

compile:
    arduino-cli compile --fqbn {{fqbn}} lightsensor/lightsensor.ino

upload:
    arduino-cli upload -p {{port}} --fqbn {{fqbn}} lightsensor/lightsensor.ino

flash: compile upload

# --- ESP32-C3 SuperMini ---
# Native USB CDC must be enabled for Serial (CDCOnBoot=cdc).
fqbn32 := "esp32:esp32:esp32c3:CDCOnBoot=cdc"
port32 := "/dev/ttyACM0"

compile32:
    arduino-cli compile --fqbn {{fqbn32}} lightsensor_esp32/lightsensor_esp32.ino

upload32:
    arduino-cli upload -p {{port32}} --fqbn {{fqbn32}} lightsensor_esp32/lightsensor_esp32.ino

flash32: compile32 upload32

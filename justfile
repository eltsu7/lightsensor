# ESP32-C3 SuperMini. Native USB CDC must be enabled for Serial (CDCOnBoot=cdc).
fqbn := "esp32:esp32:esp32c3:CDCOnBoot=cdc"

compile:
    arduino-cli compile --fqbn {{fqbn}} firmware/lightsensor/lightsensor.ino

upload:
    arduino-cli upload -p "$(uv run python lightmeter/port_detect.py)" --fqbn {{fqbn}} firmware/lightsensor/lightsensor.ino

flash: compile upload

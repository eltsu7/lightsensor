fqbn := "esp8266:esp8266:nodemcuv2"
port := "/dev/ttyUSB0"

compile:
    arduino-cli compile --fqbn {{fqbn}} lightsensor/lightsensor.ino

upload:
    arduino-cli upload -p {{port}} --fqbn {{fqbn}} lightsensor/lightsensor.ino

flash: compile upload

# Ordered v3 board: bare RP2040, W25Q32 4 MiB flash, native USB CDC.
fqbn := "rp2040:rp2040:generic:flash=4194304_2097152,freq=133,boot2=boot2_generic_03h_2_padded_checksum,usbstack=picosdk,uploadmethod=default"
sketch := "firmware/lightsensor/lightsensor.ino"
# Override with SENSOR_PORT="UF2 Board" for a board already in ROM BOOTSEL mode.
port := env_var_or_default("SENSOR_PORT", "/dev/ttyACM0")

compile:
    arduino-cli compile --fqbn {{fqbn}} {{sketch}}

upload:
    arduino-cli upload --port "{{port}}" --fqbn {{fqbn}} {{sketch}}

flash: compile upload

setup:
    arduino-cli config add board_manager.additional_urls https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
    arduino-cli core update-index
    arduino-cli core install rp2040:rp2040


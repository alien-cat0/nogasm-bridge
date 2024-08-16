# nogasm-bridge
Home assistant auto discovery bridge for nogasm. Paired with nogasm-i2c
Also includes updated casing

## Prerequisite
- you soldered the teensy LC in the i2c repo
- you have a mqtt broker and added it to your home assistant
  - see docker-compose.yaml for example

## Usage
- clone this repo
- change your wifi SSID, password, and mqtt broker info
- start your nogasm
- nogasm should be added in your home assistant automaticlly. 

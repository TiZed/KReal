#!/bin/sh

tool_path=~/.arduino15/packages/chipKIT/tools/pic32prog/v2.1.24

$tool_path/pic32prog -d /dev/ttyACM0 dist/default/production/Firmware.production.hex


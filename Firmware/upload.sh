#!/bin/sh

tool_path=/tools/mpide-0150-linux64-20150820

$tool_path/hardware/tools/avrdude64 -p 32MX320F128H -P /dev/ttyUSB0 -v -C $tool_path/hardware/tools/avrdude.conf -c stk500v2 -b 115200 -D -U flash:w:dist/default/production/firmware.production.hex 2>&1 | tee upload.log

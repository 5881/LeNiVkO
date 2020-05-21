#!/bin/zsh
if [ -z $2 ]; then NAME="ascii"; else NAME=$2; fi;
echo "static const uint8_t ${NAME}[]=" |tee "${NAME}.h"
cat $1|gzip -9|base64|sed -e 's/^/\"/g;s/$/\"/g'|tee -a "${NAME}.h"
echo ';' |tee -a "${NAME}.h"

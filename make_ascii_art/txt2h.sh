#!/bin/zsh
if [ -z $2 ]; then NAME="ascii"; else NAME=$2; fi;
N_LINE=$(wc -l $1|awk '{ print $1 }')
echo "static const uint8_t ${NAME}[]=" >out
for i in {1..$N_LINE}
do
	#echo $i
	STR=$(sed -n 's/\\/\\\\/g;s/\"/\\"/g;'"${i}p" $1)
	echo $STR
	echo \"$STR'\\n'\" >> out
done
echo ';' >> out

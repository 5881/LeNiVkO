#!/bin/bash
echo "*****************SYSTEM INFO*****************" > report.txt
echo "*****************RELEASE*****************" >> report.txt
cat /etc/*-release* >>report.txt
echo "*****************UNAME*****************" >> report.txt 
uname -a >>report.txt
echo "*****************USER*****************" >> report.txt
who >>report.txt
whoami >>report.txt
echo "*****************IP*****************" >> report.txt
ip addr show >>report.txt
#cat report.txt
python -m http.server 8080 &

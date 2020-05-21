#Эмулятор клавиатуры/мыши с управлением NRF24L01+
Устройство состоит из эмлятора (аналог Rubber Ducky) и пульта управления.
Пульт работает в диапазоне 2.4GHz.
В устройствах применены микроконтроллеры stm32f103c8t6, в качестве радио 
модуля использован NRF24L01+.
Код проекта написан на C с использованием библиотеки LibopenCM3

В директориях ./make_ascii_art и ./payloads находятся примеры текстовой 
графики и скрипты для их автоматической конвертации в заголовочные файлы С.

# Instructions
 
 1. $sudo pacman -S openocd arm-none-eabi-binutils arm-none-eabi-gcc arm-none-eabi-newlib arm-none-eabi-gdb
 2. $git clone https://github.com/5881/LENIVKO
 3. $cd LENIVKO
 4. $git submodule update --init # (Only needed once)
 5. $TARGETS=stm32/f1 make -C libopencm3 # (Only needed once)
 Пульт
 6. cd lenivko_pult
 7. $make 
 8. $make flash
 Эмулятор
 9. cd../lenivko_usb
 10. $make 
 11. $make flash


Александр Белый 2020

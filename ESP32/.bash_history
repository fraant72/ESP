cd ..
ls
mv ESP8266_3 ESP82663
mv ESP8266 ESP82663_3
mv ESP82663_3 ESP8266_3
cd get-started/
make menuconfig
ls
make
cd blink/
make menuconfig
ls
make
set |grep -i esp
cd /root
vim .bashrc
cd Scrivania/bash --
bash --help
cd Scrivania/
vim esp32.sh 
vim esp8266.sh 
vim esp8266_3.sh 

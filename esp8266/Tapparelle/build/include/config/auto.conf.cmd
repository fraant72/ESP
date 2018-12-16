deps_config := \
	/root/esp/ESP8266_RTOS_SDK/components/app_update/Kconfig \
	/root/esp/ESP8266_RTOS_SDK/components/aws_iot/Kconfig \
	/root/esp/ESP8266_RTOS_SDK/components/esp8266/Kconfig \
	/root/esp/ESP8266_RTOS_SDK/components/freertos/Kconfig \
	/root/esp/ESP8266_RTOS_SDK/components/libsodium/Kconfig \
	/root/esp/ESP8266_RTOS_SDK/components/log/Kconfig \
	/root/esp/ESP8266_RTOS_SDK/components/lwip/Kconfig \
	/root/esp/ESP8266_RTOS_SDK/components/mdns/Kconfig \
	/root/esp/ESP8266_RTOS_SDK/components/newlib/Kconfig \
	/root/esp/ESP8266_RTOS_SDK/components/pthread/Kconfig \
	/root/esp/ESP8266_RTOS_SDK/components/ssl/Kconfig \
	/root/esp/ESP8266_RTOS_SDK/components/tcpip_adapter/Kconfig \
	/root/esp/ESP8266_RTOS_SDK/components/wpa_supplicant/Kconfig \
	/root/esp/ESP8266_RTOS_SDK/components/bootloader/Kconfig.projbuild \
	/root/esp/ESP8266_RTOS_SDK/components/esptool_py/Kconfig.projbuild \
	/root/esp/ESP8266_RTOS_SDK/components/partition_table/Kconfig.projbuild \
	/root/esp/ESP8266_RTOS_SDK/Kconfig

include/config/auto.conf: \
	$(deps_config)


$(deps_config): ;

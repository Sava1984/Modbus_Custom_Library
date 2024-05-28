# Modbus_STM32_HAL
Тестовый проект для работы по протоколу Modbus RTU на микроконтроллере STM32F103c8T6 в Cube IDE.
Использована плата типа BluePill. Задействованы UART1 и UART2, работающие на прерываниях и DMA.

Настройки связи - 115200,8,N,1.
UART1 Slave ID 2, UART2 Slave ID 3.

Реализованы функции 03 - чтение holding регистров и функция 06 - запись holding регистра.

Обработка ошибок llegal adress и illegal function.



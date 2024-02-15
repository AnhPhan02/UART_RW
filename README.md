Initialize values for the UART set
---Baudrate 
---Frame data
---Enable Interrupts
---Enable UART
*****The purpose of using interrupts in this program is to return data proactively and no affect other programs. while data is being read
from the UART, the MCU can do many things without being affected. motor control, sesor parameter reading. etc.

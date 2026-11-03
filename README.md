# DCC turntable decoder using digital ST servo
Simple DCC turntable decoder utilizing digital ST servo, cheap CH32V003 processor and Arduino code

I created this project for my fun and as experiment. It have surprisingly good result, then I decided to share it for public.

Some highlights:
* Utilize program flash for store configuration (because need more data than EEPROM offer)
* utilize DMA for serial data transfer as servo use 1M BPS speed, it must be fast.

![some photo](/fulda1/DCC_turntable_with_digital_ST_servo/hardware/Servo1.jpg "San Juan Mountains")

# Z-Tek_Cubesat_ADCS
Ad astra, per aspera.


Cubesat ADCS code written by Zarvan Movdawalla and Team, from MPSTME Mumbai.

Primary Processor: RP2040, on Raspberry Pi Pico

Sensors:
MPU6050/LSM6x
XMC5883

PRIMARY SYSTEM ARCHITECTURE:
PRIMCOMP -> ADCS
         -> EPS
         -> RF-TRANSCVR

DATALINK VIA: SX1278 (custom PA/LNA)

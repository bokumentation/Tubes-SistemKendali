# PIN YANG DIPAKAI

## DRV8833 #1

STBY ---> GPIO21
AIN1 ---> GPIO35
AIN2 ---> GPIO36
BIN1 ---> GPIO37
BIN2 ---> GPIO38

### Motor:

AO1 AO2 ---> Motor FR
BO1 BO2 ---> Motor FL

## DRV8833 #2

STBY ---> GPIO47
AIN1 ---> GPIO39
AIN2 ---> GPIO40
BIN1 ---> GPIO41
BIN2 ---> GPIO42

### Motor:

AO1 AO2 ---> Motor BR
BO1 BO2 ---> Motor BL


## Header I2C

GND ---> GND
VCC ---> 3V3
SDA ---> GPIO8
SCL ---> GPIO9

## Sensor RCWL-9616 or RCWL- 1601

### Sensor Kanan

VCC  ---> 3V3
TRIG ---> GPIO17
ECHO ---> GPIO18
GND  ---> GND

### Sensor Kiri

VCC  ---> 3V3
TRIG ---> GPIO15
ECHO ---> GPIO16
GND  ---> GND

### Sensor Belakang

VCC  ---> 3V3
TRIG ---> GPIO6
ECHO ---> GPIO7
GND  ---> GND

### Sensor Depan

VCC  ---> ESP32 3V3
TRIG ---> GPIO4
ECHO ---> GPIO5
GND  ---> ESP32 GND


## POWER DISTRIBUTION: MP1584

Buck OUT+ ---> ESP32 3V3
Buck OUT- ---> ESP32 GND


### Cabang JST ke DRV8833

JST +  ---> DRV8833 #1 VM
       ---> DRV8833 #2 VM

JST -  ---> DRV8833 #1 GND
       ---> DRV8833 #2 GND

### Sambung JST ke Buck

JST +  ---> Buck IN+
JST -  ---> Buck IN-
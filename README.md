# mpu6050
Library for MPU-6050 Gyro sensor

前準備
```
raspi-config # i2c enable
apt update && \
apt-get install python3-pip python3-dev python3-smbus rpi.gpio -y && \
 pip3 install pyserial
```
USBへの書き込み権限が無いと言われたときはsudo で実行

# -*- coding: utf-8 -*-
#!/usr/bin/python3

# import module
import smbus            # use I2C
import math             # mathmatics
from time import sleep  # time module
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(4, GPIO.OUT)

#
# define
#
# slave address
DEV_ADDR = 0x68         # device address
# register address
ACCEL_XOUT = 0x3b
ACCEL_YOUT = 0x3d
ACCEL_ZOUT = 0x3f
TEMP_OUT = 0x41
GYRO_XOUT = 0x43
GYRO_YOUT = 0x45
GYRO_ZOUT = 0x47
PWR_MGMT_1 = 0x6b       # PWR_MGMT_1
PWR_MGMT_2 = 0x6c       # PWR_MGMT_2

bus = smbus.SMBus(1)
                        # Sleep disabled
bus.write_byte_data(DEV_ADDR, PWR_MGMT_1, 0)

#
# Sub function
#
# 1byte read
def read_byte(adr):
    return bus.read_byte_data(DEV_ADDR, adr)
# 2byte read
def read_word(adr):
    high = bus.read_byte_data(DEV_ADDR, adr)
    low = bus.read_byte_data(DEV_ADDR, adr+1)
    val = (high << 8) + low
    return val
# Sensor data read
def read_word_sensor(adr):
    val = read_word(adr)
    if (val >= 0x8000):         # minus
        return -((65535 - val) + 1)
    else:                       # plus
        return val
#
# 温度
#
def get_temp():
    temp = read_word_sensor(TEMP_OUT)
    x = temp / 340 + 36.53      # data sheet(register map)記載の計算式.
    return x

#
# 角速度(full scale range ±250 deg/s
#        LSB sensitivity 131 LSB/deg/s
#        -> ±250 x 131 = ±32750 LSB[16bitで表現])
#   Gyroscope Configuration GYRO_CONFIG (reg=0x1B)
#   FS_SEL(Bit4-Bit3)でfull scale range/LSB sensitivityの変更可.
#
# get gyro data
def get_gyro_data_lsb():
    x = read_word_sensor(GYRO_XOUT)
    y = read_word_sensor(GYRO_YOUT)
    z = read_word_sensor(GYRO_ZOUT)
    return [x, y, z]
def get_gyro_data_deg():
    x,y,z = get_gyro_data_lsb()
    x = x / 131.0
    y = y / 131.0
    z = z / 131.0
    return [x, y, z]

#
# 加速度(full scale range ±2g
#        LSB sensitivity 16384 LSB/g)
#        -> ±2 x 16384 = ±32768 LSB[16bitで表現])
#   Accelerometer Configuration ACCEL_CONFIG (reg=0x1C)
#   AFS_SEL(Bit4-Bit3)でfull scale range/LSB sensitivityの変更可.
#
# get accel data
def get_accel_data_lsb():
    x = read_word_sensor(ACCEL_XOUT)
    y = read_word_sensor(ACCEL_YOUT)
    z = read_word_sensor(ACCEL_ZOUT)
    return [x, y, z]
# get accel data
def get_accel_data_g():
    x,y,z = get_accel_data_lsb()
    x = x / 16384.0
    y = y / 16384.0
    z = z / 16384.0
    return [x, y, z]
# 傾き計算(1軸の傾斜の計算) for accel data
# 1軸だけ傾く場合はこの関数で計算できる.
def calc_slope_for_accel_1axis(x, y, z): # radian
    #
    # θ = asin(出力加速度[g]/1g)
    #
    # Y, Z軸固定. X軸だけ傾いた場合.
    if x > 1:    x = 1
    elif x < -1: x = -1
    slope_x = math.asin( x / 1 )
    # X, Z軸固定. Y軸だけ傾いた場合.
    if y > 1: y = 1
    elif y < -1: y = -1
    slope_y = math.asin( y / 1 )
    # X, Y軸固定. Z軸だけ傾いた場合.
    if z > 1: z = 1
    elif z < -1: z = -1
    slope_z = math.asin( z / 1 )
    return [slope_x, slope_y, slope_z]
# 傾き計算(2軸の傾斜の計算) for accel data
# 2軸を使用することで360°測定できる.
def calc_slope_for_accel_2axis_deg(x, y, z): # degree
    #
    # θ = atan(X軸出力加速度[g]/Y軸出力加速度[g])
    #
    slope_xy = math.atan( x / y )
    deg_xy = math.degrees( slope_xy )
    if x > 0 and y > 0:    # 第1象限(0°〜+90°).
        deg_xy = deg_xy
    if x > 0 and y < 0:    # 第2象限(+90°〜±180°).
        deg_xy += 180.0
    if x < 0 and y < 0:    # 第3象限(±180°〜-90°).
        deg_xy -= 180.0
    if x < 0 and y > 0:    # 第4象限(-90°〜0°).
        deg_xy = deg_xy
#    slope_xy = math.atan2( x, y )
#    deg_xy = math.degrees( slope_xy )
    return deg_xy
# 傾き計算(3軸の傾斜の計算) for accel data
# 3軸を使用することで完全な球体(θΨΦ)を測定できる.
# θ = 水平線とX軸との角度
# Ψ = 水平線とy軸との角度
# Φ = 重力ベクトルとz軸との角度
def calc_slope_for_accel_3axis_deg(x, y, z): # degree
    # θ（シータ）
    theta = math.atan( x / math.sqrt( y*y + z*z ) )
    # Ψ（プサイ）
    psi = math.atan( y / math.sqrt( x*x + z*z ) )
    # Φ（ファイ）
    phi = math.atan( math.sqrt( x*x + y*y ) / z )

    deg_theta = math.degrees( theta )
    deg_psi   = math.degrees( psi )
    deg_phi   = math.degrees( phi )
    return [deg_theta, deg_psi, deg_phi]

def disp_values():
    # slope from accel
    accel_x1,accel_y1,accel_z1 = get_accel_data_g()
    slope_x1,slope_y1,slope_z1 = calc_slope_for_accel_1axis(accel_x1,accel_y1,accel_z1)
    slope_x1 = math.degrees( slope_x1 )
    slope_y1 = math.degrees( slope_y1 )
    slope_z1 = math.degrees( slope_z1 )
    print('x: %06.3f' % slope_x1)
    print('y: %06.3f' % slope_y1)
    print('z: %06.3f' % slope_z1)
    accel_x2,accel_y2,accel_z2 = get_accel_data_g()
    slope_xy = calc_slope_for_accel_2axis_deg(accel_x2,accel_y2,accel_z2)
    print('slope x,y')
    print('xy: %06.3f' % slope_xy)
    accel_x3,accel_y3,accel_z3 = get_accel_data_g()
    theta,psi,phi = calc_slope_for_accel_3axis_deg(accel_x3,accel_y3,accel_z3)
    print('theta=%06.3f' %theta)
    print('psi=%06.3f' % psi)
    print('phi=%06.3f' % phi)

    sleep(0.1)

#
# Main function
#

try:
    while True:
        GPIO.output(4, GPIO.LOW)
        print("====== ch 0.======")
        disp_values()
        GPIO.output(4, GPIO.HIGH)
        print("")
        print("====== ch 1 ====== ")
        disp_values()

except KeyboardInterrupt:
    GPIO.output(4, GPIO.LOW)
    GPIO.cleanup()
    print("Keyboard intterrupted")

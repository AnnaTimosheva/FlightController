#Initializing libraries for working with RaCEBoard
import math
import os
import time
import board
import pwmio
import busio
import digitalio
import pwmio
import analogio
import sdcardio
import storage


from lib import LSM6DSL
from lib import BMP280
from lib import GNSS
from lib import PCF8563
from lib import QMC5883L
from lib import Ra01S

#Recording time from the moment the program was launched
start_ = time.monotonic()   

apogee = 0   #maximum height variable
counter = 0    #number of height drops variable
critical = 7     #critical number of height drops - change if necessary
frequency_buzzer = 500 # In Hz
my_file1 = "raw.txt"       #name of file with raw telemetry data
my_file2 = "FlightResult.txt"    #name of file with the results of flight


time.sleep(1)     #Initialization delay

#Initializing UART interface on pins IO17, IO18 for GNSS module
####ATTENTION: Initializing GNSS module first in order to prevent the core crash#### 
gnss_uart = busio.UART(board.IO17, board.IO18, baudrate=9600, timeout=10)  
gnss = GNSS.GPS(gnss_uart, debug=False)
last_print = time.monotonic()  #Trying to get signal variable


#Initializing buzzer on pin IO4
buzzer_pin = board.IO4
buzzer = pwmio.PWMOut(pin=buzzer_pin, duty_cycle=0, frequency=frequency_buzzer)

def BuzzerOn():
    buzzer.duty_cycle = 32768

def BuzzerOff():
    buzzer.duty_cycle = 0


#Initializing LED on pin IO8 - remove if it is not necessary
led = digitalio.DigitalInOut(board.IO8)
led.direction = digitalio.Direction.OUTPUT


#Initializing SPI interface on pins IO12, IO11, IO13, IO15 for SD card module
spi0_module     = busio.SPI(clock=board.IO12, MOSI=board.IO11, MISO=board.IO13)
sd_card_cs_pin  = board.IO15
spi0_speed = 2000000
sd_card = sdcardio.SDCard(spi0_module, sd_card_cs_pin, spi0_speed)
vfs     = storage.VfsFat(sd_card)
storage.mount(vfs, '/sd')


#Write down your first string to the file of results
file = open("/sd/"+my_file2, "a")
file.write("Результаты запуска от команды 'Исконный стиль'\n")
file.close()


#Initializing button on pin IO9
button           = digitalio.DigitalInOut(board.IO9)
button.direction = digitalio.Direction.INPUT
button.pull      = digitalio.Pull.UP


#Press the button to stop trying to get GNSS signal if it is not available for a long time
while button.value != 0:  
    gnss.update()

    current = time.monotonic()
    if current - last_print >= 1.0:
        last_print = current
        
        if gnss.has_fix:      #If GNSS is ready - write down start coordinates to the file of results
            time.sleep(1)     
            BuzzerOn()
            time.sleep(1)
            BuzzerOff()
            file = open("/sd/"+my_file2, "a")
            file.write(f"Координаты старта: Широта: {gnss.latitude_degrees} градусов {gnss.latitude_minutes} минут, долгота: {gnss.longitude_degrees} градусов {gnss.longitude_minutes} минут\n")
            file.close()
            break
    
#You will hear buzzer sound once if GNSS is not available, twice - if it is available 
time.sleep(1)  
BuzzerOn()
time.sleep(1)
BuzzerOff()


#Initializing servo on pin IO48
####ATTENTION: Servo will set its zero position after initialization#####
while True:
    if button.value == 0:
    
        min_angle = 0
        max_angle = 180
        min_width = 1640
        max_width = 7872

        PWM1_pin = board.IO48

        Servo1 = pwmio.PWMOut(pin=PWM1_pin, duty_cycle=min_width, frequency=50)
        break

def ServoSetAngle(servo, angle):
        
    if(angle < min_angle):
        angle = min_angle
    if(angle > max_angle):
        angle = max_angle
            
    duty = (angle - min_angle) * (max_width - min_width) / (max_angle - min_angle) + min_width
    servo.duty_cycle = int(duty)


#Initializating I2C interface on pins IO2, IO3 for working with BMP280, PCF8563, lsm6dsl
i2c1_module = busio.I2C(scl=board.IO2, sda=board.IO3)

bmp280      = BMP280.Adafruit_BMP280_I2C(i2c1_module, BMP280._BMP280_ADDRESS)   #Initializing barometer BMP280
bmp280.sea_level_pressure = 101.325 #kPa
ZeroHeight = 54.046  ####ATTENTION: Set zero height of your area in this variable or use bmp280.altitude()####
file = open("/sd/"+my_file2, "a")
file.write(f"Высота старта: {ZeroHeight} м над уровнем моря\n")
file.close()

PCF8563     = PCF8563.PCF8563_I2C(i2c1_module, PCF8563.PCF8563_DEFAULT_ADDRESS)   #Initializing clock - remove if it is not necessary
PCF8563.setDate(2023,07,28)
PCF8563.setTime(0,0,0)

lsm6dsl     = LSM6DSL.LSM6DSL_I2C(i2c1_module, LSM6DSL.LSM6DSL_DEFAULT_ADDRESS)  #Initializing accelerometer and gyroscope


#Initializing SPI interface on pins IO7, IO6, IO5 for radio module Ra01S
Ra01S_cs_pin    = board.IO7
Ra01S_nRst_pin  = board.IO6
Ra01S_nInt_pin  = board.IO5

Ra01S_cs = digitalio.DigitalInOut(Ra01S_cs_pin)
Ra01S_cs.direction = digitalio.Direction.OUTPUT
Ra01S_cs.value = True

Ra01S_nRst = digitalio.DigitalInOut(Ra01S_nRst_pin)
Ra01S_nRst.direction = digitalio.Direction.OUTPUT
Ra01S_nRst.value = True

Ra01S_nInt = digitalio.DigitalInOut(Ra01S_nInt_pin)
Ra01S_nInt.direction = digitalio.Direction.INPUT

Ra01S     = Ra01S.Ra01S_SPI(spi0_module, Ra01S_cs, Ra01S_nRst, Ra01S_nInt, spi0_speed)

Ra01S.on()   #Initializing radio module
Ra01S.SetLowPower()
Ra01S.SetChannel(0)  ####ATTENTION: Choose the number of your work channel to transieve data####


#You will hear buzzer sound again after the initialization finish
time.sleep(1)
BuzzerOn()
time.sleep(1)
BuzzerOff()


#Getting the value of start accel
for j in range(0,10):
    acc = lsm6dsl.acceleration()
acc_m = math.sqrt(acc[0]*acc[0]+ acc[1]*acc[1]+ acc[2]*acc[2]) #Wait for star until accel is lower than 1,5g - change if it is necessary

#Wait for star until accel is lower than 1,5g - change if it is necessary
while acc_m < 15.0:
    for j in range(0,10):
        acc = lsm6dsl.acceleration()
    acc_m = math.sqrt(acc[0]*acc[0]+ acc[1]*acc[1]+ acc[2]*acc[2])

#You will hear the buzzer sound when rocket start - remove it is not necessary
time.sleep(1)
BuzzerOn()
time.sleep(1)
BuzzerOff()


ready_ = time.monotonic()     #Moment of rocket start
init_time = ready_ - start_   #Initialization time variable
file = open("/sd/"+my_file2, "a")
file.write(f"Подготовка к старту заняла {init_time} секунд\n")
file.close()

time.sleep(2)   #ATTENTION: this is a delay for engine work. Telemetry data may be incorrect. If you want to get telemetry anyway, 
                #try this instead of time.sleep:

#engine_ = time.monotonic()
#engine_time = engine_ - ready_
#while engine_time < 2.0:
    #file = open("/sd/"+my_file1, "a")
    #file.write(f"{upflight_time} {bmp280.altitude() - ZeroHeight} ")
    #file.close()
    #for j in range(0,10):
        #acc = lsm6dsl.acceleration()
        #gyro = lsm6dsl.gyro()
        #temp = lsm6dsl.temperature()
    #acc_m = math.sqrt(acc[0]*acc[0]+ acc[1]*acc[1]+ acc[2]*acc[2])
    #gyro_m = math.sqrt(gyro[0]*gyro[0]+ gyro[1]*gyro[1]+ gyro[2]*gyro[2])    
    #file = open("/sd/"+my_file1, "a")
    #file.write(f"{temp} {acc[0]} {acc[1]} {acc[2]} {acc_m} {velocity} {gyro[0]} {gyro[1]} {gyro[2]} {gyro_m}\n ") 
    #file.close()
    #if gnss.has_fix:
            # gnss.update()
            # current = time.monotonic()
            # if current - last_print >= 1.0:
            #     last_print = current
            #     file = open("/sd/"+my_file1, "a")
            #     file.write(f"{gnss.timestamp_utc.tm_mday}.{gnss.timestamp_utc.tm_mon}.{gnss.timestamp_utc.tm_year} {gnss.timestamp_utc.tm_hour}:{gnss.timestamp_utc.tm_min}:{gnss.timestamp_utc.tm_sec} ")
            #     file.close()
            #     file = open("/sd/"+my_file1, "a")
            #     file.write(f"{gnss.latitude_degrees} {gnss.latitude_minutes} {gnss.longitude_degrees} {gnss.longitude_minutes} ")
            #     file.close()
            #     if gnss.satellites is not None:
            #         file = open("/sd/"+my_file1, "a")
            #         file.write(f"{gnss.satellites} ")
            #         file.close()
            #     if gnss.altitude_m is not None:
            #         file = open("/sd/"+my_file1, "a")
            #         file.write(f"{gnss.altitude_m} ")
            #         file.close()
            #     if gnss.speed_knots is not None:
            #         file = open("/sd/"+my_file1, "a")
            #         file.write(f"{gnss.speed_knots} ")
            #         file.close()
            #     if gnss.track_angle_deg is not None:
            #         file = open("/sd/"+my_file1, "a")
            #         file.write(f"{gnss.track_angle_deg} ")
            #         file.close()
            #     if gnss.horizontal_dilution is not None:
            #         file = open("/sd/"+my_file1, "a")
            #         file.write(f"{gnss.horizontal_dilution} ")
            #         file.close()
            #     if gnss.height_geoid is not None:
            #         file = open("/sd/"+my_file1, "a")
            #         file.write(f"{gnss.height_geoid}\n")
            #         file.close()
    #engine_ = time.monotonic()
    #engine_time = engine_ - ready_    


#Getting telemetry until reaching the apogee
while True:
    Hour, Minute, Second = PCF8563.getTime()    #Use if it is necessary
    upflight_ = time.monotonic()
    upflight_time = upflight_ - ready_
    if bmp280.altitude() > apogee and counter < critical and upflight_time < 9.0:    #Apogee is reached if height drop is detected or flight time is out of 9 seconds
        file = open("/sd/"+my_file1, "a")
        file.write(f"{upflight_time} {bmp280.altitude() - ZeroHeight} ")
        file.close()
        apogee = bmp280.altitude()
        for j in range(0,10):
            acc = lsm6dsl.acceleration()
            gyro = lsm6dsl.gyro()
            temp = lsm6dsl.temperature()
        acc_m = math.sqrt(acc[0]*acc[0]+ acc[1]*acc[1]+ acc[2]*acc[2])
        gyro_m = math.sqrt(gyro[0]*gyro[0]+ gyro[1]*gyro[1]+ gyro[2]*gyro[2])
        file = open("/sd/"+my_file1, "a")
        file.write(f"{temp} {acc[0]} {acc[1]} {acc[2]} {acc_m} {gyro[0]} {gyro[1]} {gyro[2]} {gyro_m}\n ") 
        file.close()
        if gnss.has_fix:
            gnss.update()
            current = time.monotonic()
            if current - last_print >= 1.0:
                last_print = current
                file = open("/sd/"+my_file1, "a")
                file.write(f"{gnss.timestamp_utc.tm_mday}.{gnss.timestamp_utc.tm_mon}.{gnss.timestamp_utc.tm_year} {gnss.timestamp_utc.tm_hour}:{gnss.timestamp_utc.tm_min}:{gnss.timestamp_utc.tm_sec} ")
                file.close()
                file = open("/sd/"+my_file1, "a")
                file.write(f"{gnss.latitude_degrees} {gnss.latitude_minutes} {gnss.longitude_degrees} {gnss.longitude_minutes} ")
                file.close()
                if gnss.satellites is not None:
                    file = open("/sd/"+my_file1, "a")
                    file.write(f"{gnss.satellites} ")
                    file.close()
                if gnss.altitude_m is not None:
                    file = open("/sd/"+my_file1, "a")
                    file.write(f"{gnss.altitude_m} ")
                    file.close()
                if gnss.speed_knots is not None:
                    file = open("/sd/"+my_file1, "a")
                    file.write(f"{gnss.speed_knots} ")
                    file.close()
                if gnss.track_angle_deg is not None:
                    file = open("/sd/"+my_file1, "a")
                    file.write(f"{gnss.track_angle_deg} ")
                    file.close()
                if gnss.horizontal_dilution is not None:
                    file = open("/sd/"+my_file1, "a")
                    file.write(f"{gnss.horizontal_dilution} ")
                    file.close()
                if gnss.height_geoid is not None:
                    file = open("/sd/"+my_file1, "a")
                    file.write(f"{gnss.height_geoid}\n")
                    file.close()
            
    else:
        counter = counter
    if bmp280.altitude() < apogee:
        counter += 1
    if counter == critical:
        try:                  #ATTENTION: Use try - except to be sure that there won't be any errors while sending message
            Ra01S.SendS(f"Team 'Iskonniy stil' reached apogee - {apogee - ZeroHeight} m\n")  #This message will be send once in apogee only.Place this in while cycle if it is necessary
        except:
            break
        break
    time.sleep(0.1) 

apogee_ = time.monotonic()
apogee_time = apogee_ - ready_
file = open("/sd/"+my_file2, "a")
file.write(f"Ракета достигла апогея {apogee - ZeroHeight} метров спустя {apogee_time} секунд после старта\n")
file.close()


#Activate rescue system
ServoSetAngle(Servo1, 0)
time.sleep(0.5)
ServoSetAngle(Servo1, 90)
time.sleep(0.5)

save_ = time.monotonic()
save_time = save_ - ready_

file = open("/sd/"+my_file2, "a")
file.write(f"Система спасения сработала спустя {save_time} секунд после старта\n")
file.close()


#After rescue system activation we get telemetry until the button is not pressed
while button.value != 0:
    Hour, Minute, Second = PCF8563.getTime()  #Use if it is necessary
    downflight_ = time.monotonic()
    downflight_time = downflight_ - ready_
    file = open("/sd/"+my_file1, "a")
    file.write(f"{downflight_time} {bmp280.altitude() - ZeroHeight} ")
    file.close()
    for j in range(0,10):
        acc = lsm6dsl.acceleration()
        gyro = lsm6dsl.gyro()
        temp = lsm6dsl.temperature()
    acc_m = math.sqrt(acc[0]*acc[0]+ acc[1]*acc[1]+ acc[2]*acc[2])
    gyro_m = math.sqrt(gyro[0]*gyro[0]+ gyro[1]*gyro[1]+ gyro[2]*gyro[2])
    file = open("/sd/"+my_file1, "a")
    file.write(f"{temp} {acc[0]} {acc[1]} {acc[2]} {acc_m} {gyro[0]} {gyro[1]} {gyro[2]} {gyro_m}\n ")  
    file.close()
    if gnss.has_fix:
        gnss.update()
        current = time.monotonic()
        if current - last_print >= 1.0:
            last_print = current
            file = open("/sd/"+my_file1, "a")
            file.write(f"{gnss.timestamp_utc.tm_mday}.{gnss.timestamp_utc.tm_mon}.{gnss.timestamp_utc.tm_year} {gnss.timestamp_utc.tm_hour}:{gnss.timestamp_utc.tm_min}:{gnss.timestamp_utc.tm_sec} ")
            file.close()
            file = open("/sd/"+my_file1, "a")
            file.write(f"{gnss.latitude_degrees} {gnss.latitude_minutes} {gnss.longitude_degrees} {gnss.longitude_minutes} ")
            file.close()
            if gnss.satellites is not None:
                file = open("/sd/"+my_file1, "a")
                file.write(f"{gnss.satellites} ")
                file.close()
            if gnss.altitude_m is not None:
                file = open("/sd/"+my_file1, "a")
                file.write(f"{gnss.altitude_m} ")
                file.close()
            if gnss.speed_knots is not None:
                file = open("/sd/"+my_file1, "a")
                file.write(f"{gnss.speed_knots} ")
                file.close()
            if gnss.track_angle_deg is not None:
                file = open("/sd/"+my_file1, "a")
                file.write(f"{gnss.track_angle_deg} ")
                file.close()
            if gnss.horizontal_dilution is not None:
                file = open("/sd/"+my_file1, "a")
                file.write(f"{gnss.horizontal_dilution} ")
                file.close()
            if gnss.height_geoid is not None:
                file = open("/sd/"+my_file1, "a")
                file.write(f"{gnss.height_geoid}\n")
                file.close()

#Do this to detect approximate time of finish
    if abs(bmp280.altitude() - ZeroHeight) < 10.0:
        end_ = time.monotonic()
        end_time = (end_ - ready_) + 0.5
        ZeroHeight = 1000000000000000.0


#Return servo in zero position
ServoSetAngle(Servo1, 90)
time.sleep(0.5)
ServoSetAngle(Servo1, 70)
time.sleep(0.5)

file = open("/sd/"+my_file2, "a")
file.write(f"Время полёта составило {end_time} секунд\n")
file.close()

find_ = time.monotonic()
find_time = find_ - ready_
file = open("/sd/"+my_file2, "a")
file.write(f"Ракета обнаружена спустя {find_time} секунд после старта\n")
file.close()


#LED will just blink until the RaCEBoard is turned on
while True:
    led.value = True
    time.sleep(0.1)
    led.value = False
    time.sleep(0.1)
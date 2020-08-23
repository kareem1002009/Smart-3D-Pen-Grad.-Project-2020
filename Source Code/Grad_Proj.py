from __future__ import print_function
from mbientlab.metawear import MetaWear, libmetawear, parse_value
from ctypes import c_void_p, cast, POINTER
from mbientlab.metawear.cbindings import *
from time import sleep
from threading import Event
import platform
import time
import sys
from colorama import init
from colorama import Fore, Back, Style
from keyboard import is_pressed
from pythonosc import udp_client

init()


class MyDevice:

    def __init__(self, device):
        self.device = device
        self.callback = FnVoid_VoidP_DataP(self.data_handler)
        self.gpioCallback = FnVoid_VoidP_DataP(self.gpio_handler)
        self.resetCallback = FnVoid_VoidP_DataP(self.reset_handler)
        self.samples = 0
        self.position = {'x': 0, 'y': 0, 'z': 0}
        self.velocity = {'x': 0, 'y': 0, 'z': 0}
        self.maxdvX = []
        self.maxdvY = []
        self.maxdvZ = []
        self.pre_X = 0
        self.pre_Y = 0
        self.pre_Z = 0
        self.dum_X = 0
        self.dum_Y = 0
        self.dum_Z = 0
        self.pre_velocityX = 0
        self.pre_velocityY = 0
        self.pre_velocityZ = 0
        self.preSentX = -10000
        self.preSentY = -10000
        self.preSentZ = -10000
        self.data_listX = []
        self.data_listY = []
        self.data_listZ = []
        self.lastEpoch =0
        self.FilterCount = 0
        self.StopCountX = 0
        self.StopCountY = 0
        self.StopCountZ = 0
        self.flagX = 0
        self.flagY = 0 
        self.flagZ = 0
        self.moveEnd = 5
        self.IsPressed = 0
        self.window_size = 0.8*100
        self.sumVelocityX = [0,0,0,0,0]
        self.sumVelocityY = [0,0,0,0,0]
        self.sumVelocityZ = [0,0,0,0,0]
        self.flag_suppressX = 0
        self.flag_suppressY = 0
        self.flag_suppressZ = 0
        self.neglect_counter = 0
        self.flag_move_end = 0
        self.dt1 = 0.02

        self.client = udp_client.SimpleUDPClient("localhost", 12000)

    def Send_data(self,type,x,y,z):
        self.client.send_message(type,"L 0 "+str(self.IsPressed)+" 0 "+str(x)+" "+str(y)+" "+str(z)+" 0 ")

    def trap_integrate(self, first_arg, second_arg, t):
        return ((first_arg + second_arg) / 2) * t


    def velocity_sign_change(self,axis,newData) :
        if   axis == 0 :
            averagedX_value = round((sum(self.sumVelocityX) / len(self.sumVelocityX)), 3)
            if  (averagedX_value > 4 and newData < 0) and (not(abs(averagedX_value) < abs(newData) )) :
                self.flag_Sign_change = 1
            elif(averagedX_value < -4 and newData > 0) and (not(abs(averagedX_value) < abs(newData) )) : 
                self.flag_Sign_change = 1

            if newData != 0 :
                self.sumVelocityX = self.sumVelocityX[1:10]
                self.sumVelocityX.append(newData)
        #elif axis == 1 :

        #elif axis == 2 :
        return

    def which_to_be_suppressed(self,newX,newY,newZ) :
        limit = 7
        averagedX = round((sum(self.sumVelocityX) / len(self.sumVelocityX)), 3)
        averagedY = round((sum(self.sumVelocityY) / len(self.sumVelocityY)), 3)
        averagedZ = round((sum(self.sumVelocityZ) / len(self.sumVelocityZ)), 3)
        
        if averagedX != 0:
            if  abs(averagedZ)/abs(averagedX) > limit :
                self.flag_suppressX = 1
            if abs(averagedY)/abs(averagedX) > limit :
                self.flag_suppressX = 1


        if averagedY != 0 :
            if abs(averagedZ)/abs(averagedY) > limit :
                self.flag_suppressY = 1
            if abs(averagedX)/abs(averagedY) > limit :
                self.flag_suppressY = 1

        if averagedZ != 0 :
            if  abs(averagedY)/abs(averagedZ) > limit :
                self.flag_suppressZ = 1
            if abs(averagedX)/abs(averagedZ) > limit :
                self.flag_suppressZ = 1

        self.sumVelocityX = self.sumVelocityX[1:5]
        self.sumVelocityX.append(newX)
        self.sumVelocityY = self.sumVelocityY[1:5]
        self.sumVelocityY.append(newY)
        self.sumVelocityZ = self.sumVelocityZ[1:5]
        self.sumVelocityZ.append(newZ)
        

    def integrate(self, X,Y,Z):
        if self.samples >= 50 :
            self.which_to_be_suppressed(self.velocity['x'],self.velocity['y'],self.velocity['z'])
            
            if self.flag_suppressX == 1 :
                self.velocity['x'] = 0 
                X = 0
                self.pre_velocityX = 0
                self.pre_X = 0
                

            if self.flag_suppressY == 1 :
                self.velocity['y'] = 0
                Y = 0
                self.pre_velocityY = 0
                self.pre_X = 0
                

            if self.flag_suppressZ == 1 :
                self.velocity['z'] = 0
                Z = 0
                self.pre_velocityZ = 0
                self.pre_Z =0
                

            print("flagX = ",self.flag_suppressX,"flagY = ",self.flag_suppressY,"flagZ = ",self.flag_suppressZ)

            # ------- Handling sudden sign change --------
            if X*self.pre_X >= 0 or self.flagX == 1:
                self.flagX = 0
                self.velocity['x'] = round(self.velocity['x'] + self.trap_integrate(self.pre_X,X,self.dt1),  6 )
                self.position['x'] = round(self.position['x'] + self.trap_integrate(self.pre_velocityX,self.velocity['x'],self.dt1),3)
                self.pre_velocityX = self.velocity['x']
                self.pre_X = X
            else : 
                self.flagX = 1


            if Y*self.pre_Y >= 0 or self.flagY == 1 :
                self.flagY = 0
                self.velocity['y'] = round(self.velocity['y'] + self.trap_integrate(self.pre_Y,Y,self.dt1),  6 )
                self.position['y'] = round(self.position['y'] + self.trap_integrate(self.pre_velocityY,self.velocity['y'],self.dt1),3)
                self.pre_velocityY = self.velocity['y']
                self.pre_Y = Y

            else :
                self.flagY = 1 


            if Z*self.pre_Z >= 0 or self.flagZ == 1:
                self.flagZ = 0
                self.velocity['z'] = round(self.velocity['z'] + self.trap_integrate(self.pre_Z,Z,self.dt1),  6 )
                self.position['z'] = round(self.position['z'] + self.trap_integrate(self.pre_velocityZ,self.velocity['z'],self.dt1),3)
                self.pre_velocityZ = self.velocity['z']
                self.pre_Z = Z
            else :
                self.flagZ = 1
            
            
            # ---------- Movement End Check ------------
            if X == 0 :
                self.StopCountX += 1
            else:
                self.StopCountX = 0

            if self.StopCountX >= self.moveEnd :
                self.pre_velocityX = 0
                self.velocity['x'] = 0
            if Y == 0 :
                self.StopCountY += 1
            else:
                self.StopCountY = 0
            if self.StopCountY >= self.moveEnd :
                self.pre_velocityY = 0
                self.velocity['y'] = 0
            if Z == 0 :
                self.StopCountZ += 1
            else:
                self.StopCountZ = 0

            if self.StopCountZ >= self.moveEnd :
                self.pre_velocityZ = 0
                self.velocity['z'] = 0

            
            if self.StopCountX >= self.moveEnd and self.StopCountY >= self.moveEnd and self.StopCountZ >= self.moveEnd :
                self.flag_move_end = 1
                self.flag_suppressX = 0
                self.flag_suppressY = 0
                self.flag_suppressZ = 0
                self.sumVelocityX = [0,0,0,0,0]
                self.sumVelocityY = [0,0,0,0,0]
                self.sumVelocityZ = [0,0,0,0,0]
                
            
            print("Samples No. : ", self.samples)
            print(Fore.RED +'acc: X= '+str(X)+' Y= '+str(Y)+' Z= '+str(Z))
            print(Fore.GREEN +'vel: X= '+str(self.pre_velocityX) +' Y= '+str(self.pre_velocityY)+' Z= '+str(self.pre_velocityZ))
            print(Fore.YELLOW + 'pos: X= '+str(self.position['x'])+' Y= '+str(self.position['y'])+' Z= '+str(self.position['z']))
            print(Fore.CYAN,"flagX = ",self.flag_suppressX,"flagY = ",self.flag_suppressY,"flagZ = ",self.flag_suppressZ)
            print(Style.RESET_ALL)

            sendX = round(self.position['x'],1)
            sendY = round(self.position['y'],1)
            sendZ = round(self.position['z'],1)
            if abs(sendX-self.preSentX) >= 1 or  abs(sendY-self.preSentY) >= 1 or abs(sendZ-self.preSentZ) >= 1 :
                self.Send_data("/data",sendX,sendY,sendZ)
                self.preSentX =  sendX
                self.preSentY =  sendY
                self.preSentZ =  sendZ

        return

    def data_handler(self, ctx, data):
        self.samples += 1
        if self.samples == self.neglect_counter+1 :
            self.neglect_counter += 2
            acc = parse_value(data)
            X = round(acc.x * 9.79334 * 100 , 2 )
            Y = round(acc.y * 9.79334 * 100 , 2 )
            Z = round(acc.z * 9.79334 * 100 , 2 )
            if X > -self.window_size and X < self.window_size :
                X = 0
            if Y > -self.window_size and Y < self.window_size :
                Y = 0
            if Z > -self.window_size and Z < self.window_size :
                Z = 0
            if X!=0 or Y!=0 or Z!=0 or self.flag_move_end == 0:
                self.flag_move_end = 0
                self.integrate(X,Y,Z)
        

    def gpio_handler(self,ctx,data):
        values = parse_value(data)
        if values == 0 and self.IsPressed == 0 :
            self.IsPressed = 1
        if values == 1 and self.IsPressed == 1 :
            self.IsPressed = 0
            sendX = round(self.position['x'],1)
            sendY = round(self.position['y'],1)
            sendZ = round(self.position['z'],1)
            self.Send_data("/data",sendX,sendY,sendZ)
        print("gpio = ",values)
        print("IsPressed = ",self.IsPressed)

    def reset_handler(self,ctx,data):
        values = parse_value(data)
        if values == 0 :
            self.Send_data("/reset",0,0,0)
            sleep(0.5)
            self.velocity['x'] = 0
            self.velocity['y'] = 0
            self.velocity['z'] = 0
            self.position['x'] = 0
            self.position['y'] = 0
            self.position['z'] = 0
            self.samples = 0 
            self.neglect_counter = 0
            self.pre_X = 0
            self.pre_Y = 0
            self.pre_Z = 0
            self.pre_velocityX = 0
            self.pre_velocityY = 0
            self.pre_velocityZ = 0
            self.preSentX = -10000
            self.preSentY = -10000
            self.preSentZ = -10000
            self.StopCountX = 0
            self.StopCountY = 0
            self.StopCountZ = 0
            self.flagX = 0
            self.flagY = 0 
            self.flagZ = 0
            self.sumVelocityX = [0,0,0,0,0]
            self.sumVelocityY = [0,0,0,0,0]
            self.sumVelocityZ = [0,0,0,0,0]
            self.flag_suppressX = 0
            self.flag_suppressY = 0
            self.flag_suppressZ = 0
            self.flag_move_end = 0



    def calibrate(self):
        e = Event()

        def calibration_data_handler(ctx, board, pointer):
            print("calibration data: %s" % (pointer.contents))

            libmetawear.mbl_mw_sensor_fusion_write_calibration_data(
                board, pointer)
            libmetawear.mbl_mw_memory_free(pointer)
            e.set()

        def calibration_handler(ctx, pointer):
            value = parse_value(pointer)
            print("state: %s" % (value))

            if (value.accelrometer >= Const.SENSOR_FUSION_CALIBRATION_ACCURACY_HIGH  and
                    value.gyroscope >= Const.SENSOR_FUSION_CALIBRATION_ACCURACY_HIGH and
                    value.magnetometer >= Const.SENSOR_FUSION_CALIBRATION_ACCURACY_HIGH ):

                libmetawear.mbl_mw_sensor_fusion_read_calibration_data(
                    self.device.board, None, fn_wrapper_01)
            else:
                sleep(1.0)
                libmetawear.mbl_mw_datasignal_read(signal)

        fn_wrapper_01 = FnVoid_VoidP_VoidP_CalibrationDataP(
            calibration_data_handler)
        fn_wrapper_02 = FnVoid_VoidP_DataP(calibration_handler)
        signal = libmetawear.mbl_mw_sensor_fusion_calibration_state_data_signal(
            self.device.board)

        libmetawear.mbl_mw_sensor_fusion_set_mode(self.device.board, SensorFusionMode.NDOF  )
        libmetawear.mbl_mw_sensor_fusion_write_config(self.device.board)
        libmetawear.mbl_mw_datasignal_subscribe(signal, None, fn_wrapper_02)
        libmetawear.mbl_mw_sensor_fusion_start(self.device.board)
        libmetawear.mbl_mw_datasignal_read(signal)
        e.wait()
        sleep(1.0)
        libmetawear.mbl_mw_sensor_fusion_stop(self.device.board)
        libmetawear.mbl_mw_datasignal_unsubscribe(signal)
        e.clear()


    def setup(self):
        

        libmetawear.mbl_mw_sensor_fusion_set_mode(self.device.board, SensorFusionMode.IMU_PLUS  )
        libmetawear.mbl_mw_sensor_fusion_set_acc_range(self.device.board, SensorFusionAccRange._4G)
        libmetawear.mbl_mw_sensor_fusion_set_gyro_range(self.device.board, SensorFusionGyroRange._1000DPS  )
        libmetawear.mbl_mw_sensor_fusion_write_config(self.device.board)
        AccSignal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(self.device.board, SensorFusionData.LINEAR_ACC )
        gpioSignal = libmetawear.mbl_mw_gpio_get_pin_monitor_data_signal(self.device.board,0)
        resetSignal = libmetawear.mbl_mw_gpio_get_pin_monitor_data_signal(self.device.board,1)
         
        

        libmetawear.mbl_mw_datasignal_subscribe(AccSignal, None, self.callback)
        libmetawear.mbl_mw_datasignal_subscribe(gpioSignal, None, self.gpioCallback)
        libmetawear.mbl_mw_datasignal_subscribe(resetSignal, None, self.resetCallback)
        libmetawear.mbl_mw_gpio_set_pull_mode(self.device.board,0,GpioPullMode.UP)
        libmetawear.mbl_mw_gpio_set_pull_mode(self.device.board,1,GpioPullMode.UP)
        libmetawear.mbl_mw_gpio_set_pin_change_type(self.device.board,0,GpioPinChangeType.ANY)
        libmetawear.mbl_mw_gpio_set_pin_change_type(self.device.board,1,GpioPinChangeType.FALLING)

    def start(self):
        libmetawear.mbl_mw_sensor_fusion_enable_data(self.device.board, SensorFusionData.LINEAR_ACC )
        libmetawear.mbl_mw_sensor_fusion_start(self.device.board)
        libmetawear.mbl_mw_gpio_start_pin_monitoring(self.device.board,0)
        libmetawear.mbl_mw_gpio_start_pin_monitoring(self.device.board,1)
        print("Start Drawing")
    def stop(self):
        libmetawear.mbl_mw_sensor_fusion_stop(self.device.board)
        AccSignal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(self.device.board, SensorFusionData.LINEAR_ACC)
        libmetawear.mbl_mw_datasignal_unsubscribe(AccSignal)
        gpioSignal = libmetawear.mbl_mw_gpio_get_pin_monitor_data_signal(self.device.board,0)
        libmetawear.mbl_mw_datasignal_unsubscribe(gpioSignal)
        libmetawear.mbl_mw_gpio_stop_pin_monitoring(self.device.board,0)
        resetSignal = libmetawear.mbl_mw_gpio_get_pin_monitor_data_signal(self.device.board,1)
        libmetawear.mbl_mw_datasignal_unsubscribe(resetSignal)
        libmetawear.mbl_mw_gpio_stop_pin_monitoring(self.device.board,1)
        libmetawear.mbl_mw_debug_disconnect(self.device.board)
        sleep(5.0)
        self.device.disconnect()
        sleep(1.0)
        print("DISCONNECTED!")

Mac = 'EC:7A:04:95:D6:B3'
device = MetaWear(Mac)
device.connect()
print("Connected")
d = MyDevice(device)
d.calibrate()
d.setup()
d.start()
while True :
    sleep(2)
    if is_pressed('e'):
        d.stop()
        break


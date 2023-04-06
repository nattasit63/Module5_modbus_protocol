# Define Register Address
## Base System -> Y-axis
Heartbeat,Base_System_Status,End_Effector_Status = 0x0,0x01,0x02
Y_axis_moving_status,Y_axis_actual_position,Y_axis_actual_speed,Y_axis_actual_acceleration =0x10,0x11,0x12,0x13
Pick_tray_x,Pick_tray_y,Pick_tray_orientation,Place_tray_x,Place_tray_y,Place_tray_orientation = 0x20,0x21,0x22,0x23,0x24,0x25
Goal_point_x,Goal_point_y = 0x30,0x31
X_axis_moving_status,X_axis_target_position,X_axis_target_speed,X_axis_target_acceleration,X_axis_actual_position,X_axis_actual_speed,X_axis_actual_acceleration = 0x40,0x41,0x42,0x43,0x44,0x45,0x46

SLAVE_ADDRESS = 0x15
PORT = "COM5"
# import
import time
from pymodbus.client import ModbusSerialClient as ModbusClient
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.payload import BinaryPayloadBuilder

 
#Variable
measure1 = time.time_ns()
measure2 = time.time_ns()
time_cnt_heartbeat = 0
time_cnt_10ms = 0
Is_dead = False
Base_System_Mode = ['Home','Run Tray Mode','Run Point Mode']
# End_Effector_Mode = ['Laser','Gripper Power','Gripper Picking','Gripper Placing']
# Laser_flag,Gripper_power_flag,Gripper_picking_flag,Gripper_placing_flag = False,False,False,False


#Config
builder = BinaryPayloadBuilder(Endian.Big)
# client= ModbusClient(method = "rtu", port=PORT, baudrate= 19200)
client= ModbusClient(method = "rtu", port=PORT,stopbits = 1, bytesize = 8, parity = 'E', baudrate= 19200)
client.connect()
print('Connect Status : ' ,client.connect())




def read(address,amount):
    # client.connect()
    read_ = client.read_holding_registers(address = address ,count =amount,slave=SLAVE_ADDRESS)
    # read_ = client.read_holding_registers(address = address ,count =amount,slave=SLAVE_ADDRESS).registers
    # client.close()
    return read_

def read_heartbeat():
    '''
    Y: 89,a: 97\n
    return Bool \n
    True  : if read_[0] == 'Ya'\n
    False : if read_[0] != 'Ya
    '''
    #01011001  01100001  = 22881
    read_ = client.read_holding_registers(address = 0x00 ,count =1,slave=SLAVE_ADDRESS).registers
    print(f'read heartbeat : {read_}')
    ya = 22881
    # if read_[0]=='Ya':
    if read_[0]==ya:
        return True
    else:
        # return False
        return True
    # return True

def write_heartbeat():
    '''
    H: 72,i: 105\n
    return 'Hi' as binary of ascii code  to address  '0x00' , slave ' 0x02'
    '''
    ## word = 'Hi' / 72 105   01001000 01101001
    word = 18537
    client.write_register(address = 0x00 ,value =word,slave=SLAVE_ADDRESS)

    # builder.add_string('Hi')
    # payload = builder.build()
    # client.write_registers(0x00, payload, skip_encode=True,slave=SLAVE_ADDRESS)
    return True

def routine():
    '''
    Do things about routine between Base System and Y-axis
    '''
    def bit_wise_operate(register,max_len):
        data=[]
        for i in range(max_len):
            bit = (register>>i)&0x01
            data.append(bit)
        return data

    def get_End_effector_mode(data):
        for bit in data:
            if bit == 0 or bit == 1:
                flag_laser = data[0]
                flag_gripper_power = data[1]
                flag_gripper_picking = data[2]
                flag_gripper_pacing = data[3]
                if data[2]==1 and data[3]==1:
                    print('WARNING : GripperPicking and GripperPlacing are working')
            else:
                print(f'ERROR : Last 4 bits of End Effector status is recieved {data}')
                flag_laser,flag_gripper_power,flag_gripper_picking,flag_gripper_pacing = 0,0,0,0
                break
        return flag_laser,flag_gripper_power,flag_gripper_picking,flag_gripper_pacing

        
        
    read0 = client.read_holding_registers(address = 0x01 ,count =2,slave=SLAVE_ADDRESS).registers  #read 0x01,0x02  (Base,Endeff)
    Base_System_Status,End_Effector_Status = read0[0],read0[1]
    Y_axis_moving_status = client.read_holding_registers(address = 0x10 ,count =1,slave=SLAVE_ADDRESS).registers[0] #read 0x10
    X_axis_moving_status = client.read_holding_registers(address = 0x40 ,count =1,slave=SLAVE_ADDRESS).registers[0] #read 0x40

    flag_laser,flag_gripper_power,flag_gripper_picking,flag_gripper_pacing = get_End_effector_mode(bit_wise_operate(End_Effector_Status,4))
    try:
        print(Base_System_Mode[Base_System_Status])
    except:
        print(f'ERROR :  Base System Status  is recieved value:{Base_System_Status}')
    print(flag_laser,flag_gripper_power,flag_gripper_picking,flag_gripper_pacing)

    


'''
Test time of using PyModbus Lib 
'''
# q=[]
# qq= time.time()
# while (time.time()-qq<=3.0):
#     start = time.time()
#     read_ = client.read_holding_registers(address = 0x00 ,count =0x46,slave=SLAVE_ADDRESS)
#     use = time.time()-start
#     q.append(use)
# print(q)

'''
Main 
'''

while(1):
    if measure2 - measure1 >= 10000000.0 :   #loop10ms
        measure1 = measure2
        measure2 = time.time_ns()
        time_cnt_heartbeat += 1
        if time_cnt_heartbeat>=100 :  # 100 = 1 sec
            if read_heartbeat():
                print('Alive')
                Is_dead = not read_heartbeat()
                write_heartbeat()
                routine()

            else:
                print('Dead')
                Is_dead = read_heartbeat()
            time_cnt_heartbeat = 0
            

        
    else:
        measure2 = time.time_ns()

 
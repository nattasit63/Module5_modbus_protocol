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
Y_moving_Mode = ['Jog','Go pick','Go place']
# End_Effector_Mode = ['Laser','Gripper Power','Gripper Picking','Gripper Placing']
# Laser_flag,Gripper_power_flag,Gripper_picking_flag,Gripper_placing_flag = False,False,False,False


#Config
builder = BinaryPayloadBuilder(Endian.Big)
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
    # print(f'read heartbeat : {read_}')
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

    return True

def command_end_effector(read_register,laser_bit,gripper_power_bit,gripper_pick_bit,gripper_place_bit):
    
    # End_Effector_Status=client.read_holding_registers(address = 0x02 ,count =1,slave=SLAVE_ADDRESS).registers[0]  #read 0x02  (Endeff)
    End_Effector_Status=read_register[2]
    if laser_bit==1:
        End_Effector_Status = (End_Effector_Status)|0x01
    if laser_bit==0:
        End_Effector_Status &= ~0x0001
    if gripper_power_bit==1:
        End_Effector_Status = (End_Effector_Status)|0x02
    if gripper_power_bit==0:
        End_Effector_Status &= ~0x0002
    if gripper_pick_bit==1:
        End_Effector_Status = (End_Effector_Status)|0b100
    if gripper_pick_bit==0:
        End_Effector_Status &= ~(0b100)
    if gripper_place_bit==1:
        End_Effector_Status = End_Effector_Status| 0b1000
    if gripper_place_bit==0:
        End_Effector_Status &= ~(0b1000)
    # client.write_register(address = 0x02 ,value =End_Effector_Status,slave=SLAVE_ADDRESS)

    return End_Effector_Status


def command_operation(read_register,run_mode,set_tray_and_point,goal_point=[0.0,0.0]):
    '''
    parameter : run_mode
    \tmode 0\t:\tHome
    \tmode 1\t:\tRun Tray Mode
    \tmode 2\t:\tRun Point Mode
    ------------------------------
    parameter : set_tray_and_point
    \t1\t:\tSet Pick Tray
    \t2\t:\tSet Place Tray
    \t3\t:\tSet Goal Point
    ------------------------------
    parameter : goal_point
    List of goal position [x,y]

    '''
    pre_pick,pre_place = 0,0

    def read_y_actual(read_register):
        y_axis_actual_position = read_register[0x11]
        y_axis_actual_speed = read_register[0x12]
        y_axis_actual_acceleration = read_register[0x13]
        return [y_axis_actual_position,y_axis_actual_speed,y_axis_actual_acceleration]
    def pick_tray(read_register):
        pick_tray_x = read_register[0x20]
        pick_tray_y = read_register[0x21]
        pick_tray_orientation = read_register[0x22]
        return [pick_tray_x,pick_tray_y,pick_tray_orientation]
    def place_tray(read_register):
        place_tray_x = read_register[0x23]
        place_tray_y = read_register[0x24]
        place_tray_orientation = read_register[0x25]
        return [place_tray_x,place_tray_y,place_tray_orientation]

    y_axis_moving_status = read_register[0x10]
    is_jog = y_axis_moving_status & 0x0001

    print('-'*50)
    print('Command Operation\n')
    if set_tray_and_point == 1 :
        mode = 'Set Pick Tray'
        if is_jog == 1:
            print(f'{mode} : Jogging')
            y_axis_actual = read_y_actual(read_register)
            pre_pick = 1
            print(y_axis_actual)
        if is_jog == 0 and pre_pick ==1:  # After Finish Jogging
            print(f'{mode} : Finish Jogging')
            pick_tray_acutual=pick_tray(read_register)
            print(f'Pick Tray Data : {pick_tray_acutual}')
            pre_pick = 0
    
    if set_tray_and_point == 2 :
        mode = 'Set Place Tray'
        if is_jog == 1:
            print(f'{mode} : Jogging')
            y_axis_actual = read_y_actual(read_register)
            pre_place = 1
            print(y_axis_actual)
        if is_jog == 0 and pre_place ==1:  # After Finish Jogging
            print(f'{mode} : Finish Jogging')
            place_tray_acutual=place_tray(read_register)
            print(f'Place Tray Data : {place_tray_acutual}')
            pre_place = 0
        
    if set_tray_and_point == 3 :
        mode = 'Set Goal Point'
        goal_x,goal_y = goal_point[0]*10,goal_point[1]*10
        y_axis_actual = read_y_actual(read_register)
        # client.write_registers(address=0x30,values=[goal_x,goal_y],slave=SLAVE_ADDRESS)
        client.write_register(address=0x30,value=goal_x,slave=SLAVE_ADDRESS)
        client.write_register(address=0x31,value=goal_y,slave=SLAVE_ADDRESS)
        print(f'goal x,y : [{goal_x},{goal_y}]')


    Base_System_Status= read_register[1]
    if run_mode==0 or run_mode==1 or run_mode==2:
        Base_System_Status =run_mode
        y_axis_actual = read_y_actual(read_register)
    else:
        print('Mode Input Error')
    
    return Base_System_Status,y_axis_actual

def routine(read1):
    '''
    Do things about routine between Base System and Y-axis
    '''
    print('-'*50)
    print('Routine\n')
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
                    print('WARNING : GripperPicking and GripperPlacing are both working.')
            else:
                print(f'ERROR : Last 4 bits of End Effector status is recieved :{data}')
                flag_laser,flag_gripper_power,flag_gripper_picking,flag_gripper_pacing = 0,0,0,0
                break
        return flag_laser,flag_gripper_power,flag_gripper_picking,flag_gripper_pacing

    def get_y_moving_mode(data):
        for bit in data:
            if bit == 0 or bit == 1:
                flag_y_jog = data[0]
                flag_y_gopick = data[1]
                flag_y_goplace = data[2]
                if flag_y_gopick+flag_y_jog+flag_y_goplace>1:                 
                    print('WARNING : The commands on the Y-axis moving status are overlapping.')
                    print(f'\t>> Jog:{flag_y_jog} | GoPick:{flag_y_gopick} | GoPlace:{flag_y_goplace}')
            else:
                print(f'ERROR : Last 3 bits of Y-axis moving status is recieved :{data}')
                break
        return flag_y_jog,flag_y_gopick,flag_y_goplace

    def get_x_moving_mode(data):
        for bit in data:
            if bit == 0 or bit == 1:
                flag_x_home = data[0]
                flag_x_run = data[1]
            else:
                print(f'ERROR : Last 2 bits of X-axis moving status is recieved :{data}')
                break
        return flag_x_home,flag_x_run

    # read1 = client.read_holding_registers(address = 0x01 ,count =70,slave=SLAVE_ADDRESS).registers  
    Base_System_Status,End_Effector_Status,read_y_moving_status,read_x_moving_status= read1[0x01],read1[0x02],read1[0x10],read1[0x40]

    #Bitwise and get flag in End_Effector_Status
    flag_laser,flag_gripper_power,flag_gripper_picking,flag_gripper_pacing = get_End_effector_mode(bit_wise_operate(End_Effector_Status,4))
    flag_y_jog,flag_y_gopick,flag_y_goplace = get_y_moving_mode(bit_wise_operate(read_y_moving_status,3))
    flag_x_home,flag_x_run = get_x_moving_mode(bit_wise_operate(read_x_moving_status,2))

    # Read Base_System_Status
    try:
        print('Base System Mode :',Base_System_Mode[Base_System_Status])
    except:
        print(f'ERROR :  Base System Status  is recieved :{Base_System_Status}')
    
    # Read End_Effector_Status
    print('End_Effector_Status : ',flag_laser,flag_gripper_power,flag_gripper_picking,flag_gripper_pacing)
    print(f'\t>> Laser Bit : {flag_laser}')
    # Read Y_axis_moving_status
    print('Y_axis_moving_status : ',flag_y_jog,flag_y_gopick,flag_y_goplace)
    # Read X_axis_moving_status
    print('X_axis_moving_status : ',flag_x_home,flag_x_run)


    print('-'*50)


    



'''
Main 
'''


while(1):
    
    if measure2 - measure1 >= 10000000.0 :   #loop10ms
        measure1 = measure2
        measure2 = time.time_ns()
        time_cnt_heartbeat += 1
        if time_cnt_heartbeat>=100 :  # 100 = 1 sec
            is_alive =  read_heartbeat()
            if is_alive:
                a =time.time()
                # print('Alive')
                Is_dead = not is_alive

                read_register = client.read_holding_registers(address = 0x00 ,count =70,slave=SLAVE_ADDRESS).registers
                read_register[0x10] = 1 # Jogging
                # read_register[0x10] = 0  #Finish Jogging

                write_heartbeat()
                read_register[0x02] = command_end_effector(read_register,1,1,1,0)
                read_register[0x01],y_axis_actual = command_operation(read_register,run_mode=1,set_tray_and_point=3,goal_point=[105,253])
                read_register[0x11],read_register[0x12],read_register[0x13] = y_axis_actual[0],y_axis_actual[1],y_axis_actual[2]  #postion ,speed ,acceleration


                routine(read_register)
                
                print(time.time()-a)
                

            else:
                print('Dead')
                # Is_dead = read_heartbeat()
                

            time_cnt_heartbeat = 0
    else:
        measure2 = time.time_ns()
    

 
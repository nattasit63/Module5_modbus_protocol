SLAVE_ADDRESS = 0x15
PORT = "COM5"
# import
import time
from pymodbus.client import ModbusSerialClient as ModbusClient
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.payload import BinaryPayloadBuilder


client= ModbusClient(method = "rtu", port=PORT,stopbits = 1, bytesize = 8, parity = 'E', baudrate= 19200)
client.connect()
print('Connect Status : ' ,client.connect())


##Write Upload
client.write_register(address = 0x00 ,value =22881,slave=SLAVE_ADDRESS)

client.write_register(address = 0x01 ,value =0x01,slave=SLAVE_ADDRESS)
client.write_register(address = 0x02 ,value =0x00,slave=SLAVE_ADDRESS)
client.write_register(address = 0x10 ,value =0x02,slave=SLAVE_ADDRESS)
client.write_register(address = 0x40 ,value =0x01,slave=SLAVE_ADDRESS)
read0 = client.read_holding_registers(address = 0x01 ,count =2,slave=SLAVE_ADDRESS).registers  #read 0x01,0x02  (Base,Endeff)
Base_System_Status,End_Effector_Status = read0[0],read0[1]

client.close()



# print(End_Effector_Status,type(End_Effector_Status))


# bit0 = End_Effector_Status&0x01
# bit1 = (End_Effector_Status>>1)&0x01
# bit2 = (End_Effector_Status>>2)&0x01
# bit3 = (End_Effector_Status>>3)&0x01
# print(bit0,bit1,bit2,bit3)

# bit=[]
# for i in range(4):
#     bw = (End_Effector_Status>>i)&0x01
#     bit.append(bw)
# print(bit)
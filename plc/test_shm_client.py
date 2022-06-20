import sysv_ipc
import struct

key_raw_pv = sysv_ipc.ftok("/home/smartmpc/shm/raw_pv.dat", 11)
key_raw_mv = sysv_ipc.ftok("/home/smartmpc/shm/raw_mv.dat", 12)
key_filtered_pv = sysv_ipc.ftok("/home/smartmpc/shm/filtered_pv.dat", 13)
shm_raw_pv = sysv_ipc.SharedMemory(key_raw_pv)
shm_raw_mv = sysv_ipc.SharedMemory(key_raw_mv)
shm_filtered_pv = sysv_ipc.SharedMemory(key_filtered_pv)

# read
temp = shm_raw_pv.read()
int_raw_pv1 = []
int_raw_pv2 = []
for i in range(len(temp)//4):
    int_raw_pv1.append(int.from_bytes(temp[i*4:(i+1)*4], "little"))
    int_raw_pv2.append(struct.unpack('<i',temp[i*4:(i+1)*4])[0])
print("int_raw_pv1 =",int_raw_pv1)
print("int_raw_pv2 =",int_raw_pv2)

temp = shm_raw_mv.read()
int_raw_mv = []
for i in range(len(temp)//4):
    int_raw_mv.append(struct.unpack('<i',temp[i*4:(i+1)*4])[0])
print("int_raw_mv =",int_raw_mv)


temp = shm_filtered_pv.read()
double_filtered_pv = []
for i in range(len(temp)//8):
    double_filtered_pv.append(struct.unpack('<d',temp[i*8:(i+1)*8])[0])
print("double_filtered_pv =",double_filtered_pv)


# write
temp = bytearray(shm_raw_pv.read())
for i in range(len(temp)//4):
    temp[4*i:4*(i+1)] = struct.pack('<i',i**2)
shm_raw_pv.write(temp)

temp = bytearray(shm_filtered_pv.read())
for i in range(len(temp)//8):
    temp[8*i:8*(i+1)] = struct.pack('<d',i*0.1)
shm_filtered_pv.write(temp)


# read
temp = shm_raw_pv.read()
int_raw_pv2 = []
for i in range(len(temp)//4):
    int_raw_pv2.append(struct.unpack('<i',temp[i*4:(i+1)*4])[0])
print("int_raw_pv =",int_raw_pv2)

temp = shm_raw_mv.read()
int_raw_mv = []
for i in range(len(temp)//4):
    int_raw_mv.append(struct.unpack('<i',temp[i*4:(i+1)*4])[0])
print("int_raw_mv =",int_raw_mv)


temp = shm_filtered_pv.read()
double_filtered_pv = []
for i in range(len(temp)//8):
    double_filtered_pv.append(struct.unpack('<d',temp[i*8:(i+1)*8])[0])
print("double_filtered_pv =",double_filtered_pv)

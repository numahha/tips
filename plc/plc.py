import pymcprotocol
import socket

import time

def test0():
    pymc3e = pymcprotocol.Type3E()
    pymc3e.connect("192.168.3.36", 1026)
    res = []
    start = time.time()
    for i in range(100):
        res.append(pymc3e.batchread_wordunits(headdevice="W001010", readsize=7))
    print("time",time.time() - start)
    #input()
    for i in range(100):    
        print(res[i])

def test1():

    data_index=11
    wordsize=2

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(('192.168.3.36', 1026))
    sock.settimeout(5)
    """
    print("[1]", b'P\x00\x00\xff\xff\x03\x00\x0c\x00\x04\x00\x01\x04\x00\x00n\x00\x00\xa8\x01\x00') # 500000ffff03000c000400010400006e0000a80100
    print("[2]", b'P\x00\x00\xff\xff\x03\x00\x0c\x00\x04\x00\x01\x04\x00\x00n\x00\x00\xa8\x01\x00'.hex()) # 500000ffff03000c000400010400006e0000a80100
    print("[3]", "500000ffff03000c000400010400006e0000a80100".encode() )
    print("[4]", bytes.fromhex("500000ffff03000c000400010400006e0000a80100"), type(bytes.fromhex("500000ffff03000c000400010400006e0000a80100")) )

    #sock.sendall(b'P\x00\x00\xff\xff\x03\x00\x0c\x00\x04\x00\x01\x04\x00\x00n\x00\x00\xa8\x01\x00') # OK
    #sock.sendall(bytes.fromhex("500000ffff03000c000400010400006e0000a80100")) # ok
    """

    print(b'P\x00\x00\xff\xff\x03\x00\x0c\x00\x04\x00\x01\x04\x00\x00n\x00\x00\xa8\x01\x00')
    print(b"P\x00\x00\xff\xff\x03\x00\x0c\x00\x04\x00\x01\x04\x00\x00n\x00\x00\xa8\x01\x00".hex()) # 500000ffff03000c00010001040000640000a80200
    sock.sendall(b'P\x00\x00\xff\xff\x03\x00\x0c\x00\x04\x00\x01\x04\x00\x00n\x00\x00\xa8\x01\x00')

    response = sock.recv(1024)
    print("response",response)
    print("response.hex()",response.hex(),type(response.hex()))
    print("response[data_index:data_index+wordsize].hex()",response[data_index:data_index+wordsize].hex())
    print("response[data_index:data_index+wordsize]",response[data_index:data_index+wordsize])

    value =int.from_bytes(response[data_index:data_index+wordsize], "little", signed = True)

    print("value",value)
    """
    print("z",int.from_bytes(b'%\x00', "little", signed = True))

    z = 37
    print(z.to_bytes(4, 'big'),z.to_bytes(4, 'little'))
    print(int.from_bytes(z.to_bytes(4, 'little'), "little", signed = True) )
    print(int.from_bytes(z.to_bytes(4, 'little'), "little", signed = True).to_bytes(4, 'little') )
    """
import time
def test2(val=1000):
    pymc3e = pymcprotocol.Type3E()
    pymc3e.connect("192.168.3.36", 3003)
    for i in range(10):
        pymc3e.batchwrite_wordunits(headdevice="W001010", values=[i])
        time.sleep(1)


#test0()
#test1()
test2()

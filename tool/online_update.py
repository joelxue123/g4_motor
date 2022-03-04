#!/usr/bin/env python
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
from pymodbus.file_message import FileRecord,WriteFileRecordRequest
import time

# import logging
# import sys
# logging.basicConfig(filename='1.txt', level=logging.DEBUG)

BLOCK_SIZE = 128    # 128 bytes

def online_update():
    client = ModbusClient(method='rtu', port="COM7",baudrate=115200, stopbits=1, bytesize=8, parity='N')
    client.connect()
    station_id = 0xff

    # 通过modbus写文件操作，将更新文件写入从站内存中去

    time.sleep(3)
    rq = client.write_register(0x9104, 2, unit=station_id)
    while rq.isError():
        time.sleep(0.02)
        rq = client.write_register(0x9104, 2, unit=station_id)
    print("进入更新模式")

    client.close()


if __name__ == "__main__":
    # import logging
    # import sys
    # logging.basicConfig(stream=sys.stdout, level=logging.DEBUG)
    online_update()

#!/usr/bin/env python
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
import time

error_time = 0
# import logging
# import sys
# logging.basicConfig(filename='1.txt', level=logging.DEBUG)

if __name__ == "__main__":
    com_port = "COM14"
    station_id = 0xff

    client = ModbusClient(method='rtu', port=com_port,baudrate=115200, stopbits=1, bytesize=8, parity='N')
    client.connect()

    rr = client.read_holding_registers(0x7D1, 1, unit=station_id)
    while rr.isError():
        time.sleep(0.02)
        rr = client.read_holding_registers(0x7D1, 1, unit=station_id)

    stas = rr.registers[0] >> 5
    print(stas)

    rq = client.write_registers(0x03E9, [0x0000], unit=station_id)
    rq = client.write_registers(0x03E9, [0x0001], unit=station_id)

    time.sleep(2)  

    print("init success!")
    rr = client.read_holding_registers(0x7D1, 1, unit=station_id)
    while rr.isError():
        time.sleep(0.02)
        rr = client.read_holding_registers(0x7D1, 1, unit=station_id)

    stas = rr.registers[0] >> 5
    print(stas)

    while True:
        time.sleep(3)

        _time = 0
        rq = client.write_registers(0x03E9, [0x0309], unit=station_id)
        _time = _time + 1
        print("rotator test stop")
        #rq = client.write_registers(0x03E9, [0x0409], unit=station_id)
        time.sleep(3)
        rr = client.read_holding_registers(0x7D1, 1, unit=station_id)
        while rr.isError():
            time.sleep(0.02)
            rr = client.read_holding_registers(0x7D1, 1, unit=station_id)

        stas = rr.registers[0] >> 5
        print(stas)

        _time = 0
        rq = client.write_registers(0x03E9, [0x0409], unit=station_id)
        _time = _time + 1
        print("rotator test stop")
        #rq = client.write_registers(0x03E9, [0x0409], unit=station_id)
        time.sleep(3)
        rr = client.read_holding_registers(0x7D1, 1, unit=station_id)
        while rr.isError():
            time.sleep(0.02)
            rr = client.read_holding_registers(0x7D1, 1, unit=station_id)

        stas = rr.registers[0] >> 5
        print(stas)

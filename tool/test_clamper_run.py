#!/usr/bin/env python
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
import time

error_time = 0

if __name__ == "__main__":
    com_port = "/dev/ttyUSB0"
    station_id = 0x09
    client = ModbusClient(method='rtu', port=com_port,baudrate=115200, stopbits=1, bytesize=8, parity='N')
    client.connect()

    while True:
        rr = client.read_holding_registers(0x7d0, 1, unit=station_id)
        #while rr.isError():
        #   time.sleep(0.02)
        #    rr = client.read_holding_registers(0x7d0, 1, unit=station_id)

        #stas = rr.registers[0] >> 5
        #print(stas)
        time.sleep(0.1)

    rq = client.write_registers(0x03E8, [0x0000], unit=station_id)
    rq = client.write_registers(0x03E8, [0x0001], unit=station_id)

    while True:
        time.sleep(2)  
        print("init success!")
        rr = client.read_holding_registers(0x7d0, 1, unit=station_id)
        while rr.isError():
            time.sleep(0.02)
            rr = client.read_holding_registers(0x7d0, 1, unit=station_id)

        stas = rr.registers[0] >> 5 
        print(stas)
        
        _time = 0
        rq = client.write_registers(0x03EA, [0xFF10, 0xFF01], unit=station_id)
        _time = _time + 1
        print("lamer test stop")
        time.sleep(1)  
        rr = client.read_holding_registers(0x7d0, 1, unit=station_id)
        while rr.isError():
            time.sleep(0.02)
            rr = client.read_holding_registers(0x7d0, 1, unit=station_id)

        stas = rr.registers[0] >> 5 
        print(stas)

        _time = 0
        rq = client.write_registers(0x03EA, [0xFFF0, 0xFF01], unit=station_id)
        _time = _time + 1
        print("lamer test stop")
        time.sleep(1)  

        rr = client.read_holding_registers(0x7d0, 1, unit=station_id)
        while rr.isError():
            time.sleep(0.02)
            rr = client.read_holding_registers(0x7d0, 1, unit=station_id)

        stas = rr.registers[0] >> 5
        print(stas)

        _time = 0
        rq = client.write_registers(0x03EA, [0xFF00, 0xFF01], unit=station_id)
        _time = _time + 1
        print("lamer test stop")
        time.sleep(2)  
        rr = client.read_holding_registers(0x7d0, 1, unit=station_id)
        while rr.isError():
            time.sleep(0.02)
            rr = client.read_holding_registers(0x7d0, 1, unit=station_id)

        stas = rr.registers[0] >> 5
        print(stas)
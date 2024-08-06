#!/usr/bin/env python3
# File:     bside.py
# Brief:    Power supply controller

from serial.tools.list_ports import comports
from serial import Serial, PARITY_NONE, STOPBITS_ONE, EIGHTBITS
from pymodbus.client import ModbusSerialClient
from enum import Enum
from time import sleep
from os import path
import click

class PSU:
    """
    Basic PSU com port controller
    """

    class Registers(Enum):
        OUTPUT = 0x01
        OVP_OCP_ACTIVE = 0x02
        PSU_MODEL = 0x03   # reads: 3010
        UNKNOWN_04 = 0x04   # reads: 19792
        UNKNOWN_05 = 0x05   # reads: 563
        VOLTAGE_MONITOR = 0x10
        CURRENT_MONITOR = 0x11
        OVP = 0x20
        OCP = 0x21
        VOLTAGE_SET = 0x30
        CURRENT_SET = 0x31
        UNKNOWN_40 = 0x40 # reads: 3200
        UNKNOWN_41 = 0x41 # reads: 10100

    def __init__(self, com_port=None, slaveId=0x1, debug=False):
        self.debug = debug
        self.com_port = self.find_PSU_com_port( com_port )
        #breakpoint()
        self.pymc = ModbusSerialClient(port=self.com_port, baudrate=9600, timeout=5)
        self.slaveId = slaveId

    def find_PSU_com_port(self, com_port=None):
        """Searches for PSU USB COM port adapter"""
        found_com_port = None

        adapter_ids = {
            "CH340": ("1A86", "7523")
            # add other serial devices here if we find them
            }

        com_ports_list = list(comports())

        if com_port is None:
            # detect PSU in list of ports
            for port in com_ports_list:
                # some adapters don't report VID and PID - these are not supported so don't try and test against them
                if port.vid and port.pid:
                    for adapter in adapter_ids:
                        if ('{:04X}'.format(port.vid), '{:04X}'.format(port.pid)) == adapter_ids[adapter]:
                            if self.debug:
                                print(f'Found supported {port.manufacturer} adapter {adapter} on {port.device}')
                            # assume last com port found, multiple PSUs not supported (yet!)
                            if found_com_port is None or port.device > found_com_port:
                                found_com_port = port.device
        else:
            # check specified port exists on host
            for port in com_ports_list:
                if path.realpath(com_port) == port.device:
                    found_com_port = path.realpath(com_port)
                if com_port == port.device:
                    found_com_port = port.device

        if found_com_port is None:
            raise OSError('PSU com port adapter not found')

        if self.debug:
            print(f'Assuming PSU on {found_com_port}')

        return found_com_port

    def write(self, address, value):
        rr = self.pymc.write_register(address.value, value, slave=self.slaveId)
        if rr.isError() and self.debug:
            print(address.name, rr.message)

    def read(self, address, len=1):
        value = None
        rr = self.pymc.read_holding_registers(address.value, len, slave=self.slaveId)

        # sometimes rr is returned as an error byte string rather than a
        # pdu.register_read_message.ReadHoldingRegistersResponse - however checking the
        # type can causes problems because older versions of pymodbus don't have
        # pdu.register_read_message.ReadHoldingRegistersResponse to compare against
        try:
            if not rr.isError():
                value = rr.registers[0]
            else:
                if self.debug:
                    print(address.name, rr.message)
        except:
            if self.debug:
                print(address.name, rr)
          
        return value

    def _scale_read( self, value, factor=100 ):
        if value is None:
            return None
        else:
            return value/factor
        

    @property
    def i_set(self):
        return self._scale_read(self.read(PSU.Registers.CURRENT_SET), factor=1000)

    @i_set.setter
    def i_set(self, amps):
        self.write(PSU.Registers.CURRENT_SET, int(round(amps*1000)))

    @property
    def v_set(self):
        return self._scale_read(self.read(PSU.Registers.VOLTAGE_SET))

    @v_set.setter
    def v_set(self, volts):
        self.write(PSU.Registers.VOLTAGE_SET, int(round(volts*100)))

    @property
    def ovp(self):
        return self._scale_read(self.read(PSU.Registers.OVP))

    @ovp.setter
    def ovp(self, volts):
        self.write(PSU.Registers.OVP, int(round(volts*100)))

    @property
    def ocp(self):
        return self._scale_read(self.read(PSU.Registers.OCP), factor=1000)

    @ocp.setter
    def ocp(self, amps):
        self.write(PSU.Registers.OCP, int(round(amps*1000)))

    @property
    def ovp_active(self):
        return True if self.read(PSU.Registers.OVP_OCP_ACTIVE) == 1 else False

    @property
    def ocp_active(self):
        return True if self.read(PSU.Registers.OVP_OCP_ACTIVE) == 2 else False

    @property
    def i_mon(self):
        return self._scale_read(self.read(PSU.Registers.CURRENT_MONITOR), factor=1000)

    @property
    def v_mon(self):
        return self._scale_read(self.read(PSU.Registers.VOLTAGE_MONITOR))

    @property
    def output(self):
        return True if self.read(PSU.Registers.OUTPUT) == 1 else False

    @output.setter
    def output(self, on):
        self.write(PSU.Registers.OUTPUT, int(on))

@click.command()
@click.option('--status', '-s', is_flag=True, default=False, help='Show PSU status after commands applied')
@click.option('--on', is_flag=True, default=False, help='Turn output on')
@click.option('--off', is_flag=True, default=False, help='Turn output off')
@click.option('--v-set', '-v', type=click.FloatRange(0, 30), help='Set voltage in Volts')
@click.option('--i-set', '-i', type=click.FloatRange(0, 10), help='Set current in Amps')
@click.option('--ovp', type=click.FloatRange(0, 33), help='Set OVP voltage in Volts')
@click.option('--ocp', type=click.FloatRange(0, 10.50), help='Set OCP current in Amps')
@click.option('--delay-on', '-d', type=int, help='Delay in seconds applied before enabling output')
@click.option('--debug', is_flag=True, default=False, help='Show internal debug messages')
@click.option('--com-port', type=str, default=None, help='Select com port to use. If omitted will self-detect PSU')
@click.option('--slave-id', type=int, default=1, help='For dual-bank PSUs, select slave to control EG 1 (default) or 2')
@click.option('--scan', is_flag=True, default=False, help='Scan registers for non-zero values')
def psu_cmd(status, on, off, v_set, i_set, ovp, ocp, delay_on, debug, com_port, slave_id, scan):
    """
    PSU controller
    
    If both --on and --off are specified, the output will be switched off first, other changes applied and then the output switched on.
    """
    psu = PSU(com_port=com_port, slaveId=slave_id, debug=debug)

    if scan:
        for address in range(0x0,0x50):
            # note also found some values read back from 0x9999 and 0xc110-0xc22f      
            class test(Enum):
                REGISTER = address
            try:
                result = psu.read(test.REGISTER)
                if result > 0:
                    print('Address', hex(address),':',result)
            except:
                print('Failed to read address', hex(address))
        exit(0)


    if off:
        psu.output = False
        print('Output disabled')
    
    if v_set:
        psu.v_set = v_set
        print(f"Voltage set to {v_set}V")

    if i_set:
        psu.i_set = i_set
        print(f"Current set to {i_set}A")

    if ovp:
        psu.ovp = ovp
        print(f"OVP set to {ovp}V")

    if ocp:
        psu.ocp = ocp
        print(f"OCP set to {ocp}A")

    if on:
        if delay_on:
            sleep(delay_on)
        psu.output = True
        print('Output enabled')
        
    if status:
        sleep(1) # allow PSU to settle for 1 second
        print(f'Output={psu.output}')
        print(f'Set voltage={psu.v_set}V')
        print(f'Set current={psu.i_set}A')
        print(f'Output voltage={psu.v_mon}V')
        print(f'Output current={psu.i_mon}A')
        print(f'OVP={psu.ovp}V')
        print(f'OCP={psu.ocp}A')
        print(f'OVP active={psu.ovp_active}')
        print(f'OCP active={psu.ocp_active}')

if __name__ == '__main__':
    psu_cmd()

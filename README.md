# BSIDE i8 Bench Power supply control module
Python module for controlling USB connected BSIDE bench power supplies under Windows and Linux.

It can change the voltage and current set-points as well as measure the momentary output voltage and current.
The output can also be turned on and off.

This module can either be imported into other python code or used directly from the command line, EG:

```
./bside_psu.py --v-set 12 --i-set 0.8 --on
Voltage set to 12.0V
Current set to 0.8A
Output enabled
```

# Caveats

The software that came with my PSU was on a CD/DVD that I could not read, so this is entirely reverse engineered by experimenting with modbus addresses.  There may be additional functionality that I have not uncovered; for example I have not found a way to enable/disable the OVP/OCP limiter via modbus.  It is possible there is a writable address to control this.

Note that changes made using modbus are sometimes not reflected on the PSU's LCD display, even though the values have been set. Entering and exiting the menu or editting the values via the PSU navigation will typically update the LCD's view of the settings.

I have not found a way to correct the spelling of "Programmalbe DC Power Supply".

# Dependencies

`python -m pip install click pyserial pymodbus`

Tested against Python 3.11.9 and pymodbus 3.7.0

# Example usage as a module

```
from bside_psu import PSU
from time import sleep

psu = PSU()

psu.output = False
psu.v_set = 3.3
psu.i_set = 0.7
psu.output = True

sleep(4)

print(f'Output={psu.output}')
print(f'Voltage={psu.v_mon}V')
print(f'Current={psu.i_mon}A')
```

Expected output:
```
Output=True
Voltage=3.3V
Current=0.0A
```

Note that when the class is created with `psu = PSU()` if the argument `com_port=` is omitted, it will search for the PSU by device VID/PID.
These power supplies integrate the CH340 USB/serial device, if you have more than one CH340 device you will need to specify the port to use - either when creating the PSU object, or from the command line using `--com-port=/dev/ttyUSBx`

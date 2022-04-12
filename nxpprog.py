#!/usr/bin/python3
#
# Copyright (c) 2009 Brian Murphy <brian@murphy.dk>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
# A simple programmer which works with the ISP protocol on NXP LPC arm
# processors.

import binascii
import os
import sys
import struct
import logging
import socket
import time
import ihex

import serial
import serial.tools.list_ports as port_list

CMD_SUCCESS = 0
INVALID_COMMAND = 1
SRC_ADDR_ERROR = 2
DST_ADDR_ERROR = 3
SRC_ADDR_NOT_MAPPED = 4
DST_ADDR_NOT_MAPPED = 5
COUNT_ERROR = 6
INVALID_SECTOR = 7
SECTOR_NOT_BLANK = 8
SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION = 9
COMPARE_ERROR = 10
BUSY = 11
PARAM_ERROR = 12
ADDR_ERROR = 13
ADDR_NOT_MAPPED = 14
CMD_LOCKED = 15
INVALID_CODE = 16
INVALID_BAUD_RATE = 17
INVALID_STOP_BIT = 18
CODE_READ_PROTECTION_ENABLED = 19

status = {
        "0":(0,"CMD_SUCCESS","Command is executed successfully. Sent by ISP handler only when command given by the host has been completely and successfully executed."),
        "1":(1,"INVALID_COMMAND", "Invalid command."),
        "2":(2,"SRC_ADDR_ERROR", "Source address is not on word boundary."),
        "3":(3,"DST_ADDR_ERROR", "Destination address is not on a correct boundary."),
        "4":(4,"SRC_ADDR_NOT_MAPPED", "Source address is not mapped in the memory map. Count value is taken in to consideration where applicable."),
        "5":(5,"DST_ADDR_NOT_MAPPED", "Destination address is not mapped in the memory map. Count value is taken in to consideration where applicable."),
        "6":(6,"COUNT_ERROR", "Byte count is not multiple of 4 or is not a permitted value."),
        "7":(7,"INVALID_SECTOR", "Sector number is invalid or end sector number is greater than start sector number."),
        "8":(8,"SECTOR_NOT_BLANK", "Sector is not blank."),
        "9":(9,"SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION", "Command to prepare sector for write operation was not executed."),
        "10":(10,"COMPARE_ERROR", "Source and destination data not equal."),
        "11":(11,"BUSY", "Flash programming hardware interface is busy."),
        "12":(12,"PARAM_ERROR", "Insufficient number of parameters or invalid parameter."),
        "13":(13,"ADDR_ERROR", "Address is not on word boundary."),
        "14":(14,"ADDR_NOT_MAPPED", "Address is not mapped in the memory map. Count value is taken in to consideration where applicable."),
        "15":(15,"CMD_LOCKED", "Command is locked."),
        "16":(16,"INVALID_CODE", "Unlock code is invalid."),
        "17":(17,"INVALID_BAUD_RATE", "Invalid baud rate setting."),
        "18":(18,"INVALID_STOP_BIT", "Invalid stop bit setting."),
        "19":(19,"CODE_READ_PROTECTION_ENABLED", "Code read protection enabled."),
    }

# flash sector sizes for lpc23xx/lpc24xx/lpc214x processors
flash_sector_lpc23xx = (4, 4, 4, 4, 4, 4, 4, 4, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 4, 4, 4, 4, 4, 4)

# flash sector sizes for 64k lpc21xx processors (without bootsector)
flash_sector_lpc21xx_64 = (8, 8, 8, 8, 8, 8, 8, 8)

# flash sector sizes for 128k lpc21xx processors (without bootsector)
flash_sector_lpc21xx_128 = (8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8)

# flash sector sizes for 256k lpc21xx processors (without bootsector)
flash_sector_lpc21xx_256 = (8, 8, 8, 8, 8, 8, 8, 8, 64, 64, 8, 8, 8, 8, 8, 8, 8, )

# flash sector sizes for lpc17xx processors
flash_sector_lpc17xx = (4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4 ,32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32,)

# flash sector sizes for lpc40xx processors
flash_sector_lpc40xx = (4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32,)

# flash sector sizes for lpc11xx processors
flash_sector_lpc11xx = (4, 4, 4, 4, 4, 4, 4, 4,)

# flash sector sizes for lpc18xx processors
flash_sector_lpc18xx = (8, 8, 8, 8, 8, 8, 8, 8, 64, 64, 64, 64, 64, 64, 64,)


flash_prog_buffer_base_default = 0x40001000
flash_prog_buffer_size_default = 4096

# cpu parameter table
cpu_parms = {
    
        "lpc11C1" : {
            "flash_sector" : flash_sector_lpc11xx,
            "flash_prog_buffer_base" : 0x10000300,
            "devid": 339742763,
            "flash_prog_buffer_size" : 1024
        },
        # 128k flash
        "lpc2364" : {
            "flash_sector" : flash_sector_lpc23xx,
            "flash_sector_count": 11,
            "devid": 369162498
        },
        # 256k flash
        "lpc2365" : {
            "flash_sector" : flash_sector_lpc23xx,
            "flash_sector_count": 15,
            "devid": 369158179
        },
        "lpc2366" : {
            "flash_sector" : flash_sector_lpc23xx,
            "flash_sector_count": 15,
            "devid": 369162531
        },
        # 512k flash
        "lpc2367" : {
            "flash_sector" : flash_sector_lpc23xx,
            "devid": 369158181
        },
        "lpc2368" : {
            "flash_sector" : flash_sector_lpc23xx,
            "devid": 369162533
        },
        "lpc2377" : {
            "flash_sector" : flash_sector_lpc23xx,
            "devid": 385935397
        },
        "lpc2378" : {
            "flash_sector" : flash_sector_lpc23xx,
            "devid": 385940773
        },
        "lpc2387" : {
            "flash_sector" : flash_sector_lpc23xx,
            "devid": 402716981

        },
        "lpc2388" : {
            "flash_sector" : flash_sector_lpc23xx,
            "devid": 402718517
        },
        # lpc21xx
        # some incomplete info here need at least sector count
        "lpc2141": {
            "devid": 196353,
            "flash_sector": flash_sector_lpc23xx,
            "flash_sector_count": 8,
        },
        "lpc2142": {
            "flash_sector": flash_sector_lpc23xx,
            "flash_sector_count": 9,
            "devid": 196369,
        },
        "lpc2144": {
            "flash_sector": flash_sector_lpc23xx,
            "flash_sector_count": 11,
            "devid": 196370,
        },
        "lpc2146": {
            "flash_sector": flash_sector_lpc23xx,
            "flash_sector_count": 15,
            "devid": 196387,
        },
        "lpc2148": {
            "flash_sector": flash_sector_lpc23xx,
            "flash_sector_count": 27,
            "devid": 196389,
        },
        "lpc2109" : {
            "flash_sector": flash_sector_lpc21xx_64,
            "devid": 33685249
        },
        "lpc2119" : {
            "flash_sector": flash_sector_lpc21xx_128,
            "devid": 33685266
        },
        "lpc2129" : {
            "flash_sector": flash_sector_lpc21xx_256,
            "devid": 33685267
        },
        "lpc2114" : {
            "flash_sector" : flash_sector_lpc21xx_128,
            "devid": 16908050
        },
        "lpc2124" : {
            "flash_sector" : flash_sector_lpc21xx_256,
            "devid": 16908051
        },
        "lpc2194" : {
            "flash_sector" : flash_sector_lpc21xx_256,
            "devid": 50462483
        },
        "lpc2292" : {
            "flash_sector" : flash_sector_lpc21xx_256,
            "devid": 67239699
        },
        "lpc2294" : {
            "flash_sector" : flash_sector_lpc21xx_256,
            "devid": 84016915
        },
        # lpc22xx
        "lpc2212" : {
            "flash_sector" : flash_sector_lpc21xx_128
        },
        "lpc2214" : {
            "flash_sector" : flash_sector_lpc21xx_256
        },
        # lpc24xx
        "lpc2458" : {
            "flash_sector" : flash_sector_lpc23xx,
            "devid": 352386869,
        },
        "lpc2468" : {
            "flash_sector" : flash_sector_lpc23xx,
            "devid": 369164085,
        },
        "lpc2478" : {
            "flash_sector" : flash_sector_lpc23xx,
            "devid": 386006837,
        },
        # lpc17xx
        "lpc1769" : {
            "flash_sector" : flash_sector_lpc17xx,
            "flash_prog_buffer_base" : 0x10000200,
            "csum_vec": 7,
            "devid": 0x26113f37,
            "cpu_type": "thumb",
        },
        "lpc1768" : {
            "flash_sector" : flash_sector_lpc17xx,
            "flash_prog_buffer_base" : 0x10001000,
            "csum_vec": 7,
            "devid": 0x26013f37,
            "cpu_type": "thumb",
        },
        "lpc1767" : {
            "flash_sector" : flash_sector_lpc17xx,
            "flash_prog_buffer_base" : 0x10001000,
            "csum_vec": 7,
            "devid": 0x26012837,
            "cpu_type": "thumb",
        },
        "lpc1766" : {
            "flash_sector" : flash_sector_lpc17xx,
            "flash_prog_buffer_base" : 0x10001000,
            "csum_vec": 7,
            "devid": 0x26013f33,
            "cpu_type": "thumb",
        },
        "lpc1765" : {
            "flash_sector" : flash_sector_lpc17xx,
            "flash_prog_buffer_base" : 0x10001000,
            "csum_vec": 7,
            "devid": 0x26013733,
            "cpu_type": "thumb",
        },
        "lpc1764" : {
            "flash_sector" : flash_sector_lpc17xx,
            "flash_prog_buffer_base" : 0x10001000,
            "csum_vec": 7,
            "devid": 0x26011922,
            "cpu_type": "thumb",
        },
        "lpc1763" : {
            "flash_sector" : flash_sector_lpc17xx,
            "flash_prog_buffer_base" : 0x10001000,
            "csum_vec": 7,
            "devid": 0x26012033,
            "cpu_type": "thumb",
        },
        "lpc1759" : {
            "flash_sector" : flash_sector_lpc17xx,
            "flash_prog_buffer_base" : 0x10001000,
            "csum_vec": 7,
            "devid": 0x25113737,
            "cpu_type": "thumb",
        },
        "lpc1758" : {
            "flash_sector" : flash_sector_lpc17xx,
            "flash_prog_buffer_base" : 0x10001000,
            "csum_vec": 7,
            "devid": 0x25013f37,
            "cpu_type": "thumb",
        },
        "lpc1756" : {
            "flash_sector" : flash_sector_lpc17xx,
            "flash_prog_buffer_base" : 0x10001000,
            "csum_vec": 7,
            "devid": 0x25011723,
            "cpu_type": "thumb",
        },
        "lpc1754" : {
            "flash_sector" : flash_sector_lpc17xx,
            "flash_prog_buffer_base" : 0x10001000,
            "csum_vec": 7,
            "devid": 0x25011722,
            "cpu_type": "thumb",
        },
        "lpc1752" : {
            "flash_sector" : flash_sector_lpc17xx,
            "flash_prog_buffer_base" : 0x10001000,
            "csum_vec": 7,
            "devid": 0x25001121,
            "cpu_type": "thumb",
        },
        "lpc1751" : {
            "flash_sector" : flash_sector_lpc17xx,
            "flash_prog_buffer_base" : 0x10001000,
            "csum_vec": 7,
            "devid": 0x25001110,
            "cpu_type": "thumb",
        },
        "lpc4078" : {
            "flash_sector" : flash_sector_lpc40xx,
            "flash_prog_buffer_base" : 0x10001000,
            "csum_vec": 7,
            "devid": 0x47193F47,
            "cpu_type": "thumb",
        },
        "lpc1114" : {
            "flash_sector" : flash_sector_lpc11xx,
            "flash_prog_buffer_base" : 0x10000400,
            "devid": 0x0444102B,
            "flash_prog_buffer_size" : 1024
        },
        # lpc18xx
        "lpc1817" : {
            "flash_sector" : flash_sector_lpc18xx,
            "flash_bank_addr": (0x1a000000, 0x1b000000),
            "flash_prog_buffer_base" : 0x10081000,
            "devid": (0xF001DB3F, 0),
            "devid_word1_mask": 0xff,
            "csum_vec": 7,
            "cpu_type": "thumb",
        },
        "lpc1832" : {
            "flash_sector" : flash_sector_lpc18xx,
            "flash_bank_addr": (0x1a000000),
            "flash_prog_buffer_base" : 0x10081000,
            "csum_vec": 7,
            "cpu_type": "thumb",
        },
        "lpc1833" : {
            "flash_sector" : flash_sector_lpc18xx,
            "flash_sector_count": 11,
            "flash_bank_addr": (0x1a000000, 0x1b000000),
            "flash_prog_buffer_base" : 0x10081000,
            "devid": (0xf001da30, 0x44),
            "csum_vec": 7,
            "cpu_type": "thumb",
        },
        "lpc1837" : {
            "flash_sector" : flash_sector_lpc18xx,
            "flash_bank_addr": (0x1a000000, 0x1b000000),
            "flash_prog_buffer_base" : 0x10081000,
            "devid": (0xf001da30, 0),
            "csum_vec": 7,
            "cpu_type": "thumb",
        },
        "lpc1853" : {
            "flash_sector" : flash_sector_lpc18xx,
            "flash_sector_count": 11,
            "flash_bank_addr": (0x1a000000, 0x1b000000),
            "flash_prog_buffer_base" : 0x10081000,
            "devid": (0xf001d830, 0),
            "csum_vec": 7,
            "cpu_type": "thumb",
        },
        "lpc1857" : {
            "flash_sector" : flash_sector_lpc18xx,
            "flash_bank_addr": (0x1a000000, 0x1b000000),
            "flash_prog_buffer_base" : 0x10081000,
            "devid": (0xf001d830, 0x44),
            "csum_vec": 7,
            "cpu_type": "thumb",
        },
}

prog = None


LPC_CHAR = {'Question': b'?',
            'Synchronized': b'Synchronized\r\n',
            'SynchronizedLeadingZeros': b'\x00\x00Synchronized\r\n',
            'OK': b'OK\r\n'}

## Animation stuff
ANIMATIONS = {
    "circles": [0x25D0, 0x25D3, 0x25D1, 0x25D2],
    "quadrants": [0x259F, 0x2599, 0x259B, 0x259C],
    "trigrams": [0x2630, 0x2631, 0x2632, 0x2634],
    "squarefills": [0x25E7, 0x25E9, 0x25E8, 0x25EA],
    "spaces": [0x2008, 0x2008, 0x2008, 0x2008],
    "braille":
    [0x2840, 0x2844, 0x2846, 0x2847, 0x28c7, 0x28e7, 0x28f7, 0x28fF],
    "dots12": [
        "⢀⠀","⡀⠀","⠄⠀","⢂⠀","⡂⠀","⠅⠀","⢃⠀","⡃⠀","⠍⠀","⢋⠀","⡋⠀","⠍⠁","⢋⠁",
        "⡋⠁","⠍⠉","⠋⠉","⠋⠉","⠉⠙","⠉⠙","⠉⠩","⠈⢙","⠈⡙","⢈⠩","⡀⢙","⠄⡙","⢂⠩",
        "⡂⢘","⠅⡘","⢃⠨","⡃⢐","⠍⡐","⢋⠠","⡋⢀","⠍⡁","⢋⠁","⡋⠁","⠍⠉","⠋⠉","⠋⠉",
        "⠉⠙","⠉⠙","⠉⠩","⠈⢙","⠈⡙","⠈⠩","⠀⢙","⠀⡙","⠀⠩","⠀⢘","⠀⡘","⠀⠨","⠀⢐",
        "⠀⡐","⠀⠠","⠀⢀","⠀⡀"
	]
}
selected_animation = ANIMATIONS["dots12"]
def unichar(i):
  try:
    return chr(i)
  except ValueError:
    return struct.pack('i', i).decode('utf-32')

spinner_starting_point = 0

def progress_bar(bar_length, current_block, total_blocks):
    #   global spinner_starting_point
    #   bar_len = bar_length
    #   filled_len = int(round(bar_len * (current_block + 1) / float(total_blocks)))
    
    #   percents = round(100.0 * (current_block + 1) / float(total_blocks), 1)
    
    #   bar = "█" * (filled_len - 1)
    #   bar = bar + "░" * (bar_len - filled_len)
    
    #   suffix = "Block # 0x{:X} of 0x{:X}".format(
    #       current_block + 1, int(total_blocks))
    
    #   sys.stdout.write(
    #       '\r▕%s▏ %s%% %s %s ' %
    #       (bar, percents,
    #       selected_animation[spinner_starting_point % len(selected_animation)],
    #       suffix))
    
    #   spinner_starting_point += 1
    
    #   sys.stdout.flush()
    pass

def panic(str):
    logging.error(str)
    if prog:
        prog.device.close()
    # sys.exit(1)

class SerialDevice(object):
    def __init__(self, device, baud, xonxoff=False, control=False):
        # Create the Serial object without port to avoid automatic opening
        self._serial = serial.Serial(port=None, baudrate=baud)

        # Disable RTS and DRT to avoid automatic reset to ISP mode (use --control for explicit reset)
        self._serial.rts = 0
        self._serial.dtr = 0

        # Select and open the port after RTS and DTR are set to zero
        self._serial.port = device
        self._serial.open()

        # set a two second timeout just in case there is nothing connected
        # or the device is in the wrong mode.
        # This timeout is too short for slow baud rates but who wants to
        # use them?
        self._serial.timeout = 5
        # device wants Xon Xoff flow control
        if xonxoff:
            self._serial.xonxoff = 1

        # reset pin is controlled by DTR implying int0 is controlled by RTS
        self.reset_pin = "dtr"

        if control:
            self.isp_mode()

        self._serial.flushInput()

    # put the chip in isp mode by resetting it using RTS and DTR signals
    # this is of course only possible if the signals are connected in
    # this way
    def isp_mode(self):
        self.reset(0)
        time.sleep(.1)
        self.reset(1)
        self.int0(1)
        time.sleep(.1)
        self.reset(0)
        time.sleep(.1)
        self.int0(0)

    def reset(self, level):
        if self.reset_pin == "rts":
            self._serial.rts = level
        else:
            self._serial.dtr = level

    def int0(self, level):
        # if reset pin is rts int0 pin is dtr
        if self.reset_pin == "rts":
            self._serial.dtr = level
        else:
            self._serial.rts = level

    def write(self, data):
        self._serial.write(data)

    def readline(self, timeout=None):
        if timeout:
            ot = self._serial.timeout
            self._serial.timeout = timeout

        line = b''
        while True:
            c = self._serial.read(1)
            if not c:
                break
            if c == b'\r':
                if not line:
                    continue
                else:
                    break
            if c == b'\n':
                if not line:
                    continue
                else:
                    break
            line += c

        if timeout:
            self._serial.timeout = ot

        return line.decode("UTF-8", "ignore")

    def close(self):
        if self._serial.is_open:
          self._serial.flush()
          self._serial.reset_input_buffer()
          self._serial.reset_output_buffer()
          # This is a workaround for a problem with serial terminals after the
          # usage of pySerial where the vmin (or min) serial flag is set to 0.
          # This causes chrome.serial and other serial port readers to throw an
          # exception and lose control over the serial device.
          self._serial.inter_byte_timeout = 1
          self._serial._reconfigure_port(force_update=True)
          self._serial.close()

class UdpDevice(object):
    def __init__(self, address):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.timeout = 5
        self._inet_addr = address[0]
        self._udp_port = address[1]
        self._eth_addr = address[2]

        if self._eth_addr:
            import subprocess
            # Try add host to ARP table
            obj = subprocess.Popen(['arp', '-s', self._inet_addr, self._eth_addr], stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
            res = obj.communicate()
            stdout_text = res[0].decode('ascii', 'ignore') if res[0] else ""
            stderr_text = res[1].decode('ascii', 'ignore') if res[1] else ""
            if obj.returncode or stderr_text:
                panic("Failed to register IP address " + "(Administrative privileges may be required)\r\n" + stderr_text.replace('\r', '').replace('\n', ''))

        self._sock.bind(('', self._udp_port))

    def write(self, data):
        self._sock.sendto(data, (self._inet_addr, self._udp_port))

    def readline(self, timeout=None):
        if timeout:
            ot = self._sock.timeout
            self._sock.timeout = timeout

        try:
            line, addr = self._sock.recvfrom(1024)
        except Exception as e:
            line = b""

        if timeout:
            self._sock.timeout = ot

        return line.decode("UTF-8", "ignore").replace('\r','').replace('\n','')

class nxpprog:
    def __init__(self, cpu, device, baud, osc_freq, xonxoff=False, control=False, address=None, verify=False):
        self.logger = logging.getLogger(f"NXP ISP {device}")
        self.logger.debug("Starting INIT.")
        self.echo_on = True
        self.verify = verify
        self.OK = 'OK'
        self.RESEND = 'RESEND'
        self.sync_str = 'Synchronized'

        # for calculations in 32 bit modulo arithmetic
        self.U32_MOD = (2 ** 32)

        # uuencoded line length
        self.uu_line_size = 45
        # uuencoded block length
        self.uu_block_size = self.uu_line_size * 20
        self.baud = baud
        if address:
            self.device = UdpDevice(address)
        else:
            self.device = SerialDevice(device, baud, xonxoff, control)

        self.cpu = cpu

        self.connection_init(osc_freq)

        self.banks = self.get_cpu_parm("flash_bank_addr", 0)

        if self.banks == 0:
            self.sector_commands_need_bank = False
        else:
            self.sector_commands_need_bank = True
        self.current_address = 0
        self.total_length = 0
        self.logger.info("INIT complete.")
        pass

    def connection_init(self, osc_freq):
        self.sync(osc_freq)

        if self.cpu == "autodetect":
            self.autodetect_chip()
        
        self.unlock_write()
        pass
            
    def autodetect_chip(self):
        self.logger.debug("Autodetecting chip")
        devid = self.get_devid()
        for dcpu in cpu_parms.keys():
            cpu_devid = cpu_parms[dcpu].get("devid")
            if not cpu_devid:
                continue

            # mask devid word1
            devid_word1_mask = cpu_parms[dcpu].get("devid_word1_mask")
            if devid_word1_mask and isinstance(devid, tuple) and devid[0] == cpu_devid[0] and (devid[1] & devid_word1_mask) == (cpu_devid[1] & devid_word1_mask):
                self.logger.debug("Detected %s" % dcpu)
                self.cpu = dcpu
                break

            if devid == cpu_devid:
                self.logger.debug("Detected %s" % dcpu)
                self.cpu = dcpu
                break
            
        if self.cpu == "autodetect":
            raise RuntimeError("Cannot autodetect from device id %d(0x%x), set cpu name manually" % (devid, devid))
        
        return True
        
    def unlock_write(self):
        self.logger.debug("Unlocking chip write protect.")
        # unlock write commands
        self.isp_command("U 23130")
        self.logger.info("Unlocked chip write protect.")
        return True

    def dev_write(self, data):
        self.device.write(data)

    def dev_writeln(self, data):
        data = data.encode('UTF-8') + b'\r\n'
        # print('> ' + data)
        self.device.write(data)

    def dev_readline(self, timeout=None):
        data = self.device.readline(timeout)
        # print('< ' + data)
        return data

    def errexit(self, str, status):
        if not status:
            panic("%s: timeout" % str)
        err = int(status)
        if err != 0:
            error_desc = [
                'CMD_SUCCESS: Command is executed successfully',
                'INVALID_COMMAND: Invalid command',
                'SRC_ADDR_ERROR: Source address is not on word boundary',
                'DST_ADDR_ERROR: Destination address is not on word or 256 byte boundary',
                'SRC_ADDR_NOT_MAPPED:  Source address is not mapped in the memory map',
                'DST_ADDR_NOT_MAPPED: Destination address is not mapped in the memory map',
                'COUNT_ERROR: Byte count is not multiple of 4 or is not a permitted value',
                'INVALID_SECTOR: Sector number is invalid or end sector number is greater than start sector number',
                'SECTOR_NOT_BLANK: Sector is not blank',
                'SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION: Command to prepare sector for write operation was not executed',
                'COMPARE_ERROR: Source and destination data not equal',
                'BUSY: Flash programming hardware interface is busy',
                'PARAM_ERROR: Insufficient number of parameters or invalid parameter',
                'ADDR_ERROR: Address not on word boundary',
                'ADDR_NOT_MAPPED: Address is not mapped in the memory map',
                'CMD_LOCKED: Command is locked',
                'INVALID_CODE: Unlock code is invalid',
                'INVALID_BAUD_RATE: Invalid baud rate setting',
                'INVALID_STOP_BIT: Invalid stop bit setting',
                'CODE_READ_PROTECTION_ENABLED: Code read protection enabled'
                ]
            str = str.replace('\r','').replace('\n','')
            errstr = error_desc[err] if err < len(error_desc) else ""
            panic("%s: %d - %s" % (str, err, errstr))

    def isp_command(self, cmd, retry=3):
        
        while retry > 0:
            retry -= 1
            self.dev_writeln(cmd)

            # throw away echo data
            if self.echo_on:
                echo = self.dev_readline()
                if self.verify and echo != cmd:
                    self.logger.debug('ISP command Invalid echo')

            status = self.dev_readline()
            if status:
                break
        return status

    def sync(self, osc):
        self.logger.debug("Start autobaud sync")
        #start sync by sending chip question mark to set autobaud, should receive Syncronized as response.
        self.dev_write(b'?')
        s = self.dev_readline()
        if not s:
            raise RuntimeError("No sync string not received.")
        
        if self.sync_str not in s:
            raise RuntimeError("Correct sync string not received.")
        
        #echo sync string to chip, should get OK as response
        self.dev_writeln(self.sync_str)
        s = self.dev_readline()

        # detect echo state
        if s == self.sync_str:
            self.echo_on = True
            s = self.dev_readline()
            
        elif s == self.OK:
            self.echo_on = False
        
        elif s != self.OK:
            logging.debug("echo sync = %s" % s)
            raise RuntimeError("Chip did not accept sync request.")

        self.logger.info(f"Autobaud sync at {self.baud}")
        
        self.logger.debug("Start OSC setting")
        # set the oscillator frequency
        self.dev_writeln('%d' % osc)
        if self.echo_on:
            #read echo from buffer
            s = self.dev_readline()
            if s != ('%d' % osc):
                raise RuntimeError("Chip did not echo OCS command.")
                
        # read response from OSC command 
        s = self.dev_readline()
        if s != self.OK:
            if s == str(INVALID_COMMAND):
                pass
            else:
                raise RuntimeError(f"Chip did not accept OCS command.\n\r{status[s]}")
        
        self.logger.info(f"OSC set at {osc}")
        
        self.set_echo(False)
        return True
    
    def set_echo(self, echo):
        self.logger.debug(f"Setting echo {echo}")
        # disable echo
        self.dev_writeln('A 0')
        if self.echo_on:
            s = self.dev_readline()
            if s != 'A 0':
                raise RuntimeError('Invalid echo')

        s = self.dev_readline()
        if s == str(CMD_SUCCESS):
            self.echo_on = False
        elif s == str(INVALID_COMMAND):
            pass
        else:
            raise RuntimeError("Echo disable failed")
        
        self.logger.info(f"Echo set {echo}")
        return True
    
    def sum(self, data):
        s = 0
        if isinstance(data, str):
            for ch in data:
                s += ord(ch)
        else:
            for i in data:
                s += i
        return s

    def write_ram_block(self, addr, data):
        data_len = len(data)

        for i in range(0, data_len, self.uu_line_size):
            c_line_size = data_len - i
            if c_line_size > self.uu_line_size:
                c_line_size = self.uu_line_size
            block = data[i:i+c_line_size]
            bstr = binascii.b2a_uu(block)
            self.dev_write(bstr)

        retry = 3
        while retry > 0:
            retry -= 1
            self.dev_writeln('%s' % self.sum(data))
            status = self.dev_readline()
            if status:
                break
        if not status:
            return "timeout"
        if status == self.RESEND:
            return "resend"
        if status == self.OK:
            return ""

        # unknown status result
        panic(status)

    def uudecode(self, line):
        # uu encoded data has an encoded length first
        linelen = ord(line[0]) - 32

        uu_linelen = (linelen + 3 - 1) // 3 * 4

        if uu_linelen + 1 != len(line):
            panic("Error in line length")

        # pure python implementation - if this was C we would
        # use bitshift operations here
        decoded = ""
        for i in range(1, len(line), 4):
            c = 0
            for j in line[i: i + 4]:
                ch = ord(j) - 32
                ch %= 64
                c = c * 64 + ch
            s = []
            for j in range(0, 3):
                s.append(c % 256)
                c = c // 256
            for j in reversed(s):
                decoded = decoded + chr(j)

        # only return real data
        return decoded[0:linelen]

    def read_block(self, addr, data_len, fd=None):
        self.isp_command("R %d %d" % ( addr, data_len ))

        expected_lines = (data_len + self.uu_line_size - 1) // self.uu_line_size

        data = ""
        for i in range(0, expected_lines, 20):
            lines = expected_lines - i
            if lines > 20:
                lines = 20
            cdata = ""
            for i in range(0, lines):
                line = self.dev_readline()

                decoded = self.uudecode(line)

                cdata += decoded

            s = self.dev_readline()

            if int(s) != self.sum(cdata):
                panic("Checksum mismatch on read got 0x%x expected 0x%x" % (int(s), self.sum(data)))
            else:
                self.dev_writeln(self.OK)

            if fd:
                fd.write(cdata)
            else:
                data += cdata

        if fd:
            return None
        else:
            return data

    def write_ram_data(self, addr, data, total_length):
        image_len = len(data)
        for i in range(0, image_len, self.uu_block_size):
            progress_bar(30, self.current_address, self.total_length)

            a_block_size = image_len - i
            if a_block_size > self.uu_block_size:
                a_block_size = self.uu_block_size

            self.isp_command("W %d %d" % ( addr, a_block_size ))

            retry = 3
            while retry > 0:
                retry -= 1
                err = self.write_ram_block(addr, data[i : i + a_block_size])
                if not err:
                    break
                elif err != "resend":
                    panic("Write error: %s" % err)
                else:
                    logging.debug("Resending")

            addr += a_block_size

    def find_flash_sector(self, addr):
        table = self.get_cpu_parm("flash_sector")
        flash_base_addr = self.get_cpu_parm("flash_bank_addr", 0)
        if flash_base_addr == 0:
            faddr = 0
        else:
            faddr = flash_base_addr[0] # fix to have a current flash bank
        for i in range(0, len(table)):
            n_faddr = faddr + table[i] * 1024
            if addr >= faddr and addr < n_faddr:
                return i
            faddr = n_faddr
        return -1

    def bytestr(self, ch, count):
        data = b''
        for i in range(0, count):
            data += bytes([ch])
        return data

    def insert_csum(self, orig_image):
        self.logger.debug("Inserting checksum into vector table.")
        
        # make this a valid image by inserting a checksum in the correct place
        intvecs = struct.unpack("<8I", orig_image[0:32])

        # default vector is 5: 0x14, new cortex cpus use 7: 0x1c
        valid_image_csum_vec = self.get_cpu_parm("csum_vec", 5)
        
        # calculate the checksum over the interrupt vectors
        csum = 0
        intvecs_list = []
        for vec in range(0, len(intvecs)):
            intvecs_list.append(intvecs[vec])
            if valid_image_csum_vec == 5 or vec <= valid_image_csum_vec:
                csum = csum + intvecs[vec]
        
        # remove the value at the checksum location
        csum -= intvecs[valid_image_csum_vec]

        csum %= self.U32_MOD
        csum = self.U32_MOD - csum

        intvecs_list[valid_image_csum_vec] = csum

        image = b''
        for vecval in intvecs_list:
            image += struct.pack("<I", vecval)

        image += orig_image[32:]
        
        self.logger.info("Inserted intvec checksum 0x%08x in image at offset %d" % (csum, valid_image_csum_vec))
        return image

    def prepare_flash_sectors(self, start_sector, end_sector):
        self.logger.debug(f"Preparing sectors {start_sector} - {end_sector}")
        if self.sector_commands_need_bank:
            self.isp_command("P %d %d 0" % (start_sector, end_sector))
        else:
            self.isp_command("P %d %d" % (start_sector, end_sector))

    def erase_sectors(self, start_sector, end_sector, verify=False):
        self.prepare_flash_sectors(start_sector, end_sector)

        self.logger.info("Erasing flash sectors %d-%d" % (start_sector, end_sector))

        if self.sector_commands_need_bank:
            self.isp_command("E %d %d 0" % (start_sector, end_sector))
        else:
            self.isp_command("E %d %d" % (start_sector, end_sector))

        if verify:
            self.blank_check_sectors(start_sector, end_sector)

    def blank_check_sectors(self, start_sector, end_sector):
        self.logger.debug("Blank checking sectors %d-%d" % (start_sector, end_sector))
        
        for i in range(start_sector, end_sector+1):
            offset = 0
            if self.sector_commands_need_bank:
                cmd = ("I %d %d 0" % (i, i))
            else:
                cmd = ("I %d %d" % (i, i))
            
            result = self.isp_command(cmd)
            if result == str(CMD_SUCCESS):
                pass
            elif result == str(SECTOR_NOT_BLANK):
                offset = self.dev_readline() # offset
                self.dev_readline() # content
            else:
                raise RuntimeError(f"Blank check failed at offset {offset}")
        
        self.logger.info("Sectors %d-%d are blank" % (start_sector, end_sector))

    def erase_flash_range(self, start_addr, end_addr, verify=False):
        start_sector = self.find_flash_sector(start_addr)
        end_sector = self.find_flash_sector(end_addr)

        self.erase_sectors(start_sector, end_sector, verify)

    def get_cpu_parm(self, key, default=None):
        ccpu_parms = cpu_parms.get(self.cpu)
        if not ccpu_parms:
            panic("No parameters defined for cpu %s" % self.cpu)
        parm = ccpu_parms.get(key)
        if parm:
            return parm
        if default is not None:
            return default
        else:
            panic("No value for required cpu parameter %s" % key)

    def erase_all(self, verify=False):
        end_sector = self.get_cpu_parm("flash_sector_count", len(self.get_cpu_parm("flash_sector"))) - 1

        self.erase_sectors(0, end_sector, verify)

    def blank_check_all(self):
        end_sector = self.get_cpu_parm("flash_sector_count", len(self.get_cpu_parm("flash_sector"))) - 1

        self.blank_check_sectors(0, end_sector)

    def prog_image(self, image, flash_addr_base=0, erase_all=False, verify=False):
        global panic
        success = True
        
        self.logger.debug("Programing chip.")
        # the base address of the ram block to be written to flash
        ram_addr = self.get_cpu_parm("flash_prog_buffer_base", flash_prog_buffer_base_default)
        
        # the size of the ram block to be written to flash
        # 256 | 512 | 1024 | 4096
        ram_block = self.get_cpu_parm("flash_prog_buffer_size", flash_prog_buffer_size_default)

        # if the image starts at the start of a flash bank then make it bootable
        # by inserting a checksum at the right place in the vector table
        if self.banks == 0:
            if flash_addr_base == 0:
                image = self.insert_csum(image)
        elif flash_addr_base in self.banks:
            image = self.insert_csum(image)
        
        image_len = len(image)
        self.total_length = image_len
        
        # pad to a multiple of ram_block size with 0xff
        pad_count_rem = image_len % ram_block
        pad_count = 0
        if pad_count_rem != 0:
            pad_count = ram_block - pad_count_rem
            image += self.bytestr(0xff, pad_count)
            image_len += pad_count
            self.logger.debug("Padding image with %d bytes" % pad_count)
        
        #erase chip if needed
        if erase_all:
            self.erase_all(verify)
        else:
            self.erase_flash_range(flash_addr_base, flash_addr_base + image_len - 1, verify)
        
        #send image to chip.
        for image_index in range(0, image_len, ram_block):
            a_ram_block = image_len - image_index
            if a_ram_block > ram_block:
                a_ram_block = ram_block

            flash_addr_start = image_index + flash_addr_base
            flash_addr_end = flash_addr_start + a_ram_block - 1

            self.logger.debug("Writing %d bytes to ram" % (a_ram_block))

            self.current_address = flash_addr_start

            self.write_ram_data(ram_addr, image[image_index: image_index + a_ram_block], image_len)

            s_flash_sector = self.find_flash_sector(flash_addr_start)

            e_flash_sector = self.find_flash_sector(flash_addr_end)

            self.prepare_flash_sectors(s_flash_sector, e_flash_sector)

            # copy ram to flash
            self.logger.debug(f"Copy address {ram_addr} to 0x{flash_addr_start:02x}")
            self.isp_command("C %d %d %d" % (flash_addr_start, ram_addr, a_ram_block))
            self.logger.info("Wrote %d bytes to 0x%x" % (a_ram_block, flash_addr_start))
            
            # optionally compare ram and flash
            if verify:
                result = self.isp_command("M %d %d %d" % (flash_addr_start, ram_addr, a_ram_block))
                if result == str(CMD_SUCCESS):
                    self.logger.info(f"Verified block at 0x{flash_addr_start:02x}")
                    pass
                elif result == str(COMPARE_ERROR):
                    self.dev_readline() # offset
                    success = False
                else:
                    raise RuntimeError("Flash address {flash_addr_start} did not verify to ram.")

            self.progress(image_len - 1, image_len)
        if success:
            self.logger.info("Programing was successful.")
        else:
            self.logger.error("Programming Failed.")
        return success
    
    def progress(self, total, current):
        pass
    
    def verify_image(self, flash_addr_base, image):
        success = True

        image_length = len(image)
        start_addr = flash_addr_base
        end_addr = flash_addr_base + image_length

        start_sector = self.find_flash_sector(start_addr)
        end_sector = self.find_flash_sector(end_addr)

        table = self.get_cpu_parm("flash_sector")
        flash_base_addr = self.get_cpu_parm("flash_bank_addr", 0)
        if flash_base_addr == 0:
            faddr = 0
        else:
            faddr = flash_base_addr[0] # fix to have a current flash bank

        index = 0
        sector = start_sector
        while sector <= end_sector:
            start_of_sector = faddr + 1024 * sum(table[:sector])
            end_of_sector = faddr + 1024 * sum(table[:sector+1])

            start = start_addr if start_of_sector < start_addr else start_of_sector
            end = end_addr if end_of_sector > end_addr else end_of_sector
            length = 4 * ((end - start) // 4)

            logging.info("Verify sector %i: Reading %d bytes from 0x%x" % (sector, length, start))
            data = self.read_block(start, length)
            if isinstance(image[0], int):
                data = [ord(x) for x in data]

            if len(data) != length:
                panic("Verify failed! lengths differ")

            for (i, (x, y)) in enumerate(zip(data, image[index:index+(end-start)])):
                if x != y:
                    logging.error("Verify failed! content differ at location 0x%x" % (faddr + i))
                    success = False
                    break

            index = index + length
            sector = sector + 1

        return success

    def start(self, addr=0):
        mode = self.get_cpu_parm("cpu_type", "arm")
        # start image at address 0
        if mode == "arm":
            m = "A"
        elif mode == "thumb":
            m = "T"
        else:
            panic("Invalid mode to start")

        self.isp_command("G %d %s" % (addr, m))

    def select_bank(self, bank):
        status = self.isp_command("S %d" % bank)

        if status == self.OK:
            return 1

        return 0

    def get_devid(self):
        self.isp_command("J")
        id1 = self.dev_readline()

        # FIXME find a way of doing this without a timeout
        id2 = self.dev_readline(.2)
        if id2:
            ret = (int(id1), int(id2))
        else:
            ret = int(id1)
        return ret

    def get_serial_number(self):
        self.isp_command("N")
        id1 = self.dev_readline()
        id2 = self.dev_readline(.2)
        id3 = self.dev_readline(.2)
        id4 = self.dev_readline(.2)
        return ' '.join([id1, id2, id3, id4])


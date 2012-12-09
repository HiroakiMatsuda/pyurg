#!/usr/bin/env python
# -*- coding: utf-8 -*-
# This module provides a class that controls a Laser Scanner
# manufactured by HOKUYO AUTOMATIC CO,LTD
# HOKUYO AUTOMATIC CO,LTD(http://www.hokuyo-aut.co.jp/index.html)
#
# This module has been tested on python ver.2.6.6
# It need pySerial(http://pyserial.sourceforge.net/)
#
# ver1.21210
# (C) 2012 Matsuda Hiroaki 

import serial
import math

class Urg(object):

        def __init__(self):
                self.myserial = serial.Serial()
                print('Generated the serial object')
                self.dmin = 0           # Maximum measurement distance
                self.dmax = 0           # Minimum measurement distance
                self.amin = 0           # The start of the scan step
                self.amax = 0           # The end of the scan step
                self.unit = 0           # One unit of the scan
                self.interval = 0       # Scanning interval
                self.byte = 0           # The size of one scan data
                self.head_size = 0      # Head size of the data
                self.timestamp = 0      # Timestamp
                self.ares = 0           # Angular resolution capability
                self.angl_min = 0       # Minimum value of scan angle
                self.angl_max = 0       # Maximum value of scan angle
                self.request_tag = ''   # Tag of a request mode

        def open_port(self, port = 'COM1', baudrate = 19200, timeout = 1):
                self.myserial.port = port
                self.myserial.baudrate = baudrate
                self.myserial.timeout = timeout
                self.myserial.parity = serial.PARITY_NONE
                try:
                        self.myserial.open()
                except IOError:
                        print('Failed to open port, check the device and port number')
                else:
                        print('Succeede to open port: ' + port)

        def close_port(self):
                self.myserial.close()

        def flush_outport(self):
                self.myserial.flushOutput()

        def flush_inport(self):
                self.myserial.flushInput()

        def set_port(self, baudrate = 19200, timeout = 1):
                self.myserial.baudrate = baudrate
                self.myserial.timeout = timeout
                self.myserial._reconfigurePort()
                print('Succeede to set baudrate:%d, timeout:%d'
                      %(baudrate, timeout))

        def set_urg(self, port = 'COM1'):
                self.open_port(port)
                self.set_scip2()
                parame = self.get_parame()
                self.ares = int(parame['ARES'])
                self.dmin = int(parame['DMIN'])
                self.dmax = int(parame['DMAX'])

                angle_range = 360.0 / self.ares * int(parame['AMAX'])
                self.angle_min = -angle_range / 2
                self.angle_max = angle_range / 2

                self.port = port

        def reset_urg(self, port):
                self.flush_inport()
                self.flush_outport()
                self.reset_status()
                self.close_port()
                self.open_port(port)
                self.flush_inport()
                self.flush_outport()
                
# Setting the operating mode
        # Change the protocol SCIP 2.x
        def set_scip2(self):
                self._write_command('SCIP2.0\n')
                read = self.myserial.read(8)[:-1]

                return read
        
        # Time stamp mode
        def set_timestamp(self, mode = 'TM1', string = ''):
                self._check_command(mode, 'TM0', 'TM1', 'TM2')

                send = ['TM']
                
                if mode is 'TM0':
                        send.appned('0')
                elif mode is 'TM1':
                        send.appned('1')
                elif mode is 'TM2':
                        send.appned('2')

                send += [string, '\n']        

                self._write_command(send)

                if mode is 'TM1':
                        raw = self.myserial.read(15 + len(string))
                        calc_sum = self._calc_sum(raw[-5:-3])
                        self._check_sum(raw[-3], calc_sum)
                        return self._character_encoding(raw[-7:-3])

                else:
                        raw = self.myserial.read(9 + len(string))
                        calc_sum = self._calc_sum(raw[-5:-3])
                        self._check_sum(raw[-3], calc_sum)
                        return raw[-5:-3]
                           
        # Turn on the laser
        def turn_on_laser(self, string = ''):
                send = ['BM', string, '\n']
                self._write_command(send)
                
                raw = self.myserial.read(8 + len(string))
                return raw[-5:-2]

        # Turn off the laser
        def turn_off_laser(self, string = ''):
                send = ['QT', string, '\n']
                self._write_command(send)
                
                raw = self.myserial.read(8 + len(string))
                return raw[-5:-2]

        # Reset the all status information
        def reset_all_status(self, string = ''):
                send = ['RS', string, '\n']
                self._write_command(send)
                
                raw = self.myserial.read(8 + len(string))
                return raw[-5:-2]
                

        # Reset the status information
        def reset_status(self, string = ''):
                send = ['RT', string, '\n']
                self._write_command(send)
                
                raw = self.myserial.read(8 + len(string))
                return raw[-5:-2]

# Getting Status Information
        # Version information
        def get_version(self, string = ''):
                send = ['VV', string, '\n']
                self._write_command(''.join(send))

                return self._read_information(string, 5)
        
        # Measurement parameters
        def get_parame(self, string = ''):
                send = ['PP', string, '\n']
                self._write_command(send)
                    
                return self._read_information(string, 9)

        # Status information
        def get_status(self, string = ''):
                send = ['II', string, '\n']
                self._write_command(send)
                    
                return self._read_information(string, 7)
                
# Getting Distance Data
        def request_md(self, amin, amax, unit = 1, interval = 0, num = 1, string = ''):
                send = ['MD', str(amin).zfill(4), str(amax).zfill(4), str(unit).zfill(2),
                        str(interval), str(num).zfill(2), string, '\n']
                self._write_command(send)
                
                head = self.myserial.read(21 + len(string))
                status = head[-5:-3]
                calc_sum = self._calc_sum(status)
                self._check_sum(head[-3], calc_sum)

                self.amin = amin
                self.amax = amax
                self.unit = unit
                self.interval = interval
                self.byte = 3
                self.head_size = 26
                self.request_tag = 'M'

                return status

        def request_ms(self, amin, amax, unit = 1, interval = 0, num = 1, string = ''):
                send = ['MS', str(amin).zfill(4), str(amax).zfill(4), str(unit).zfill(2),
                        str(interval), str(num).zfill(2), string, '\n']
                self._write_command(send)
                
                head = self.myserial.read(21 + len(string))
                status = head[-5:-3]
                calc_sum = self._calc_sum(status)
                self._check_sum(head[-3], calc_sum)

                self.amin = amin
                self.amax = amax
                self.unit = unit
                self.interval = interval
                self.byte = 2
                self.head_size = 26
                self.request_tag = 'M'

                return status

        def request_gd(self, amin, amax, unit = 1, string = ''):
                self.turn_on_laser()
                
                send = ['GD', str(amin).zfill(4), str(amax).zfill(4),
                        str(unit).zfill(2), string, '\n']
                self._write_command(send)
                
                head = self.myserial.read(18 + len(string))
                status = head[-5:-3]
                calc_sum = self._calc_sum(status)
                self._check_sum(head[-3], calc_sum)       

                self.amin = amin
                self.amax = amax
                self.unit = unit
                self.interval = 0
                self.byte = 3
                self.head_size = 23 + len(string)
                self.request_tag = 'G'

                return status

        def request_gs(self, amin, amax, unit = 1, string = ''):
                self.turn_on_laser()

                send = ['GS', str(amin).zfill(4), str(amax).zfill(4),
                        str(unit).zfill(2), string, '\n']
                self._write_command(send)
                
                head = self.myserial.read(18 + len(string))
                status = head[-5:-3]
                
                self.amin = amin
                self.amax = amax
                self.unit = unit
                self.interval = 0
                self.byte = 2
                self.head_size = 23 + len(string)
                self.request_tag = 'G'

                return status

        def request_me(self, amin, amax, unit = 1, interval = 0, num = 1, string = ''):
                send = ['ME', str(amin).zfill(4), str(amax).zfill(4), str(unit).zfill(2),
                        str(interval), str(num).zfill(2), string, '\n']
                self._write_command(send)
                
                head = self.myserial.read(21 + len(string))
                status = head[-5:-3]

                calc_sum = self._calc_sum(status)
                self._check_sum(head[-3], calc_sum)

                self.amin = amin
                self.amax = amax
                self.unit = unit
                self.interval = interval
                self.head_size = 26 + len(string)
                self.request_tag = 'M'

                return status

        def get_distance(self):
                if self.request_tag == 'M':
                        echo_length = 16
                elif self.request_tag == 'G':
                        echo_length = self.head_size - 10
                        
                # Read Echo
                echo = self.myserial.read(echo_length)
                # Read and Check status
                status = self.myserial.read(4)[:-2]
                self.check_status(status)
                # Read, Check and Encode Timestamp
                raw_timestamp = self.myserial.read(4)
                calc_sum = self._calc_sum(raw_timestamp)
                check_sum = self.myserial.read(2)
                self._check_sum(check_sum[0], calc_sum)
                
                timestamp = self._encode_char(raw_timestamp[:4])

                # Read Distance Data
                scan_point = self.amax - self.amin + 1
                line_num = scan_point * self.byte / 64

                data = []
                for i in range(line_num):
                        raw = self.myserial.read(66)
                        calc_sum = self._calc_sum(raw[:-2])
                        self._check_sum(raw[-2], calc_sum)
                        data.append(raw[:-2])

                odd_byte = scan_point * self.byte % 64
                if odd_byte != 0:
                        raw = self.myserial.read(odd_byte + 2)
                        calc_sum = self._calc_sum(raw[:-2])
                        self._check_sum(raw[-2], calc_sum)
                        data.append(raw[:-2])
                # Read last LF
                self.myserial.read(1)
                   
                raw_data = ''.join(data)
                
                distance = []
                dist = ''   
                for temp in raw_data:
                        dist += temp
                        if len(dist) == self.byte:
                                distance.append(self._encode_char(dist))
                                dist = ''
                                
                return distance, timestamp


        def get_distance_and_intensity(self):
                echo_length = self.head_size - 10
                # Read Echo
                echo = self.myserial.read(echo_length)
                # Read and Check status
                status = self.myserial.read(4)[:-2]
                self.check_status(status)
                # Read, Check and Encode Timestamp
                raw_timestamp = self.myserial.read(6)
                calc_sum = self._calc_sum(raw_timestamp[:4]) 
                self._check_sum(raw_timestamp[4], calc_sum)
                
                timestamp = self._encode_char(raw_timestamp[:4])

                # Read Distance and Intensity Data
                scan_point = self.amax - self.amin + 1
                line_num = scan_point * 6 / 64

                data = []                
                for i in range(line_num):
                        raw = self.myserial.read(66)
                        calc_sum = self._calc_sum(raw[:-2])
                        self._check_sum(raw[-2], calc_sum)
                        data.append(raw[:-2])

                odd_byte = scan_point * 6 % 64
                if odd_byte != 0:
                        raw = self.myserial.read(odd_byte + 2)
                        calc_sum = self._calc_sum(raw[:-2])
                        self._check_sum(raw[-2], calc_sum)
                        data.append(raw[:-2])
                # Read last LF
                self.myserial.read(1)

                raw_data = ''.join(data)

                distance = []
                intensity = []
                measure = ''
                count = 1

                for temp in raw_data:
                        measure += temp
                        if len(measure) == 3:
                                if count % 2 != 0:
                                        distance.append(self._encode_char(measure))
                                elif count % 2 == 0:
                                        intensity.append(self._encode_char(measure))
                                measure= ''
                                count += 1

                return distance, intensity, timestamp
                 
        def check_status(self, status):
                
                if status == '00':
                        print('Successful reception of command')
                elif status == '01':
                        raise ValueError('Non-numeric: Start step')
                elif status == '02':
                        raise ValueError('Non-numeric: End step')
                elif status == '03':
                        raise ValueError('Non-numeric: Uunit step number')
                elif status == '04':
                        raise ValueError('Maximum value')
                elif status == '05':
                        raise ValueError('Minimum value')
                elif status == '06':
                        raise ValueError('Non-numeric: Number of interval scan')
                elif status == '07':
                        raise ValueError('Non-numeric: Nnumber of transmissions')
                elif '21' <= status <= '49':
                        raise ValueError('Anomaly Detection, Communication interrupted')
                elif '50' <= status <= '97':
                        raise ValueError('Hardware fault')
                elif status == '98':
                        print('Communication recovery')

        def convert_to_xy(self, data):
                xy = []
                step = self._calc_one_step()
                                 
                rad = math.pi / 180.0
                
                for i, dist in enumerate(data):
                        if dist < 20:
                                dist = 0    
                        theta = self.angle_min + i * step
                        
                        xy.append([dist * math.cos(theta * rad), dist * math.sin(theta * rad)])
                        
                return xy

        def convert_to_x_y(self, data):
                x = []
                y = []
                
                step = self._calc_one_step()
                rad = math.pi / 180.0
                
                for i, dist in enumerate(data):
                        if dist < 20:
                                dist = 0
                        theta = self.angle_min +  i * step

                        x.append(dist * math.cos(theta * rad))
                        y.append(dist * math.sin(theta * rad))
                        
                                
                return x, y

# The following functions are provided for use in Urg class
        def _write_command(self, command):
                self.myserial.flushOutput()
                self.myserial.flushInput()
                self.myserial.write(''.join(command))

        def _calc_sum(self, value):
                check_sum = 0
                for i in value:
                        check_sum += ord(i)

                check_sum &= 0x3F
                check_sum += 0x30

                return check_sum

        def _check_sum(self, check_sum, calc_sum):
                if ord(check_sum) != calc_sum:
                        raise ValueError('Check Sum does not match')

        def _encode_char(self, char):   
                data = 0
                for temp in char:
                        data <<= 6
                        data &= (0x3F ^ 0xFFFF)
                        data |= ord(temp) - 0x30

                return data
                
        def _read_information(self, string, info_num):
                head = self.myserial.read(7 + len(string))
                info = {}
                
                for i in range(info_num):
                        raw = ''
                        read = ''
                        while read != '\n':
                                read = self.myserial.read(1)
                                raw += read

                        if raw[0] != '\n':
                                calc_sum = self._calc_sum(raw[:-3])
                                self._check_sum(raw[-2], calc_sum)
                                split_1 = raw.split(';')
                                split_2 = split_1[0].split(':')
                                info[split_2[0]] = split_2[1]

                return info

        def _calc_one_step(self):
                temp_step = 360.0 / self.ares * self.unit
                return temp_step * (self.interval + 1)
                

        def _check_command(self, command, *command_set):
                if command not in command_set:
                        raise ValueError('command must be choose in'
                                         + repr(command_set))

if __name__ == '__main__':
        import pyurg

        urg = pyurg.Urg()
        urg.set_urg('COM3')
        urg.check_status(urg.request_me(0, 1080, num = 99))

        for i in range(99):
                d, i, t = urg.get_distance_and_intensity()
                print t
        
        urg.close_port()

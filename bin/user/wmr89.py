#!/usr/bin/env python
#
# Copyright 2018 Matthew Wall
# See the file LICENSE.txt for your rights.
#
# Credits:
#   Marek for discovering the baud rate that enabled everyone to decode the
#   serial communications, and for providing a working implementation that
#   made development of this driver much easier!

"""Driver for Oregon Scientific WMR89 weather stations

The WMR89 uses a CP210x usb-to-serial adapter.  This driver communicates
directly using python serial, not using pyusb or other USB mechanisms.  This
means the system must map the device to a serial port, such as /dev/ttyUSB0.
This should happen automatically if the system has loaded the cp210x kernel
module.  To do so manually:

  sudo modprobe cp210x
  sudo sh -c 'echo 0fde ca0a > /sys/bus/usb-serial/drivers/cp210x/new_id

then the device should show up as /dev/ttyUSB0.
"""

from __future__ import with_statement
import serial
import syslog
import time

import weewx.drivers

DRIVER_NAME = 'WMR89'
DRIVER_VERSION = '0.1'


def loader(config_dict, _):
    return WMR89Driver(**config_dict[DRIVER_NAME])

def confeditor_loader():
    return WMR89ConfEditor()


def logmsg(level, msg):
    syslog.syslog(level, 'wmr89: %s' % msg)

def logdbg(msg):
    logmsg(syslog.LOG_DEBUG, msg)

def loginf(msg):
    logmsg(syslog.LOG_INFO, msg)

def logerr(msg):
    logmsg(syslog.LOG_ERR, msg)


def _fmt(x):
    return ' '.join(["%0.2X" % ord(c) for c in x])


class WMR89Driver(weewx.drivers.AbstractDevice):
    """weeWX driver that communicates with a WMR89 station.

    port - serial port
    [Required. Default is /dev/ttyUSB0]
    """
    def __init__(self, **stn_dict):
        loginf('driver version is %s' % DRIVER_VERSION)
        self.port = stn_dict.get('port', Station.DEFAULT_PORT)
        loginf('using serial port %s' % self.port)
        self.last_rain = None
        self.station = Station(self.port)
        self.station.open()

    def closePort(self):
        if self.station is not None:
            self.station.close()
            self.station = None

    @property
    def hardware_name(self):
        return DRIVER_NAME

    def genLoopPackets(self):
        for pkt in self.station.get_data():
            packet = {'dateTime': int(time.time() + 0.5),
                      'usUnits': weewx.US}
            packet.update(pkt)
            self._augment_packet(packet)
            yield packet

    def _augment_packet(self, packet):
        packet['rain'] = weewx.wxformulas.calculate_rain(
            packet['rain_total'], self.last_rain)
        self.last_rain = packet['rain_total']


class Station(object):

    DEFAULT_PORT = '/dev/ttyUSB0'
    MARKER = 'f2f2'.decode('hex')
    HEARTBEAT = 'd100'.decode('hex')

    def __init__(self, port):
        self.port = port
        self.serial_port = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, _, value, traceback):
        self.close()

    def open(self):
        logdbg("open serial port %s" % self.port)
        self.serial_port = serial.Serial(
            port=self.port, baudrate=128000, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS,
            xonxoff=False, timeout=2)

    def close(self):
        if self.serial_port is not None:
            logdbg("close serial port %s" % self.port)
            self.serial_port.close()
            self.serial_port = None

    def waiting(self):
        return self.serial_port.inWaiting()

    def send_heartbeat(self):
        self.serial_port.write(Station.HEARTBEAT)

    def read(self):
        return self.serial_port.read()

    def get_data(self):
        # generator that returns data as dict
        prev_datetime = None
        datetime = None
        pkt = dict()
        while True:
            line = ''
            if self.waiting() == 0:
                self.send_heartbeat()
                time.sleep(0.5)
            while self.waiting():
                c = self.read()
                line = line + c
            if len(line):
                a = line.split(Station.MARKER)
                a = filter(None, a)
                for i in range(len(a)):
                    x = a[i][0].encode('hex')
                    y = a[i][2].encode('hex')
                    if x == 'b0':
                        datetime = Station.decode_datetime(a[i])
                        if prev_datetime <> datetime:
                            yield pkt
                            pkt = dict()
                    if x == 'b5' and y == '00':
                        t, h = Station.decode_inside_th(a[i])
                        pkt['temperature_in'] = t
                        pkt['humidity_in'] = h
                    if x == 'b5' and y == '01':
                        t, h = Station.decode_outside_th(a[i])
                        pkt['temperature_out'] = t
                        pkt['humidity_out'] = h
                    if x == 'b2':
                        avg, gust, wdir, chill = Station.decode_wind(a[i])
                        pkt['wind_speed'] = avg
                        pkt['wind_gust'] = gust
                        pkt['wind_dir'] = wdir
                        pkt['wind_chill'] = chill
                    if x == 'b4':
                        pkt['pressure'] = Station.decode_pressure(a[i])
                    prev_datetime = datetime
            time.sleep(0.5)

    @staticmethod
    def decode_datetime(x):
        y = ord(x[5]) + 2000
        m = ord(x[6])
        d = ord(x[8])
        t = ord(x[9])
        return "%s.%s.%s:%s" % (y, m, d, t)

    @staticmethod
    def decode_inside_th(x):
        t = 0.1 * (256 * ord(x[3]) + ord(x[4]))
        h = ord(x[6])
        return t, h

    @staticmethod
    def decode_outside_th(x):
        t = 256 * ord(x[3]) + ord(x[4])
        if t >= 32768:
            t = t - 65536
        t = 0.1 * t
        h = ord(x[6])
        return t, h

    @staticmethod
    def decode_wind(x):
        avg = 0.36 * ord(x[3])
        gust = 0.36 * ord(x[5])
        wdir = 22.5 * ord(x[7])
        chill = ord(x[8])
        if chill < 125:
            chill = (ord(x[8]) - 32) * 5 / 9
        elif chill > 125:
            chill = ((chill - 255) - 32) * 5 / 9
        elif chill == 125:
            chill = None
        return avg, gust, wdir, chill

    @staticmethod
    def decode_pressure(x):
        # pressure is units of ?
        return 0.1 * (256 * ord(x[2]) + ord(x[3]))

    @staticmethod
    def set_baud_rate():
        import array
        import sys
        import fcntl
        fd = 0
        baudrate = 128000
        TCGETS2 = 0x802C542A
        TCSETS2 = 0x402C542B
        BOTHER = 0o010000
        CBAUD = 0o010017
        buf = array.array('i', [0] * 64) # is 44 really
        fcntl.ioctl(fd, TCGETS2, buf)
        buf[2] &= ~CBAUD
        buf[2] |= BOTHER
        buf[9] = buf[10] = baudrate
        assert(fcntl.ioctl(fd, TCSETS2, buf)==0)
        fcntl.ioctl(fd, TCGETS2, buf)
        if buf[9] != baudrate or buf[10] != baudrate:
            print("failed. speed is %d %d" % (buf[9],buf[10]))
            sys.exit(1)

class WMR89ConfEditor(weewx.drivers.AbstractConfEditor):
    @property
    def default_stanza(self):
        return """
[WMR89]
    # This section is for the Oregon Scientific WMR89 weather stations.

    # Serial port such as /dev/ttyS0, /dev/ttyUSB0, or /dev/cua0
    port = %s

    # The driver to use:
    driver = user.wmr89
""" % Station.DEFAULT_PORT

    def prompt_for_settings(self):
        print "Specify the serial port on which the station is connected, for"
        print "example: /dev/ttyUSB0 or /dev/ttyS0 or /dev/cua0."
        port = self._prompt('port', Station.DEFAULT_PORT)
        return {'port': port}


# define a main entry point for basic testing.  invoke this as follows from
# the weewx root dir:
#
# PYTHONPATH=bin python bin/weewx/drivers/wmr89.py

if __name__ == '__main__':
    import optparse

    usage = """%prog [options] [--help]"""

    syslog.openlog('wmr89', syslog.LOG_PID | syslog.LOG_CONS)
    syslog.setlogmask(syslog.LOG_UPTO(syslog.LOG_DEBUG))
    parser = optparse.OptionParser(usage=usage)
    parser.add_option('--version', dest='version', action='store_true',
                      help='display driver version')
    parser.add_option('--port', dest='port', metavar='PORT',
                      help='serial port to which the station is connected',
                      default=Station.DEFAULT_PORT)
    (options, args) = parser.parse_args()

    if options.version:
        print "%s driver version %s" % (DRIVER_NAME, DRIVER_VERSION)
        exit(0)

    with Station(options.port) as station:
        for pkt in station.get_data():
            print time.time(), pkt

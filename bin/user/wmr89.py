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
DRIVER_VERSION = '0.3'


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

    # map sensor values to the database schema fields
    # the default map is for the wview schema
    DEFAULT_MAP = {
        'pressure': 'pressure',
        'windSpeed': 'wind_speed',
        'windDir': 'wind_dir',
        'windGust': 'wind_gust',
        'windchill': 'wind_chill',
        'inTemp': 'temperature_in',
        'outTemp': 'temperature_out',
        'inHumidity': 'humidity_in',
        'outHumidity': 'humidity_out',
        'dewpoint': 'dewpoint_in',
        'dewpoint': 'dewpoint_out',
        'rain_total': 'rain_total',
        'rainRate': 'rain_rate'}

    def __init__(self, **stn_dict):
        loginf('driver version is %s' % DRIVER_VERSION)
        self.port = stn_dict.get('port', Station.DEFAULT_PORT)
        loginf('using serial port %s' % self.port)
        self.sensor_map = dict(self.DEFAULT_MAP)
        if 'sensor_map' in stn_dict:
            self.sensor_map.update(stn_dict['sensor_map'])
        loginf('sensor map is %s' % self.sensor_map)
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
            logdbg("sensors: $s" % pkt)
            packet = self._map_packet(pkt)
            logdbg("mapped: %s" % packet)
            if packet:
                packet['dateTime'] = int(time.time() + 0.5)
                packet['usUnits'] = weewx.METRIC
                self._calculate_rain_delta(packet)
                yield packet

    def _map_packet(self, pkt):
        # map sensor names to database fields
        # map hardware names to the requested database schema names
        p = dict()
        for label in self.sensor_map:
            if self.sensor_map[label] in pkt:
                p[label] = pkt[self.sensor_map[label]]
        return p

    def _calculate_rain_delta(self, packet):
        # calculate a rain delta given accumulated rain
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
        # generator that returns data as dict.  data come to us as multiple
        # lines of bytes from the station.  decode each line and return the
        # corresponding data as soon as we receive it.
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
                    if weewx.debug > 1:
                        logdbg("raw: %s" % _fmt(a[i]))
                    x = a[i][0].encode('hex')
                    y = a[i][2].encode('hex')
                    if x == 'b0':
                        # ignore the station's date and time
                        if weewx.debug > 1:
                            datetime = Station.decode_datetime(a[i])
                            now = time.time()
                            weeutil.timestamp_to_string(now)
                            logdbg("datetime: %s (%s)" % (datetime, now))
                    elif x == 'b1':
                        rr, rh, r24, rt = Station.decode_rain(a[i])
                        pkt['rain_total'] = rt
                        pkt['rain_rate'] = rr
                    elif x == 'b2':
                        avg, gust, wdir, chill = Station.decode_wind(a[i])
                        pkt['wind_speed'] = avg
                        pkt['wind_gust'] = gust
                        pkt['wind_dir'] = wdir
                        pkt['wind_chill'] = chill
                    elif x == 'b4':
                        pkt['pressure'] = Station.decode_pressure(a[i])
                    elif x == 'b5':
                        if y == '00':
                            t, h = Station.decode_inside_th(a[i])
                            pkt['temperature_in'] = t
                            pkt['humidity_in'] = h
                        elif y == '01':
                            t, h = Station.decode_outside_th(a[i])
                            pkt['temperature_out'] = t
                            pkt['humidity_out'] = h
                    else:
                        loginf("unknown packet type %0.2X: %s" %
                               (ord(x), _fmt(a[i])))
                    if pkt:
                        yield pkt
                        pkt = dict()
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
        # temperature in degree C, humidity in percent
        t = 0.1 * (256 * ord(x[3]) + ord(x[4]))
        h = ord(x[6])
        return t, h

    @staticmethod
    def decode_outside_th(x):
        # temperature in degree C, humidity in percent
        t = 256 * ord(x[3]) + ord(x[4])
        if t >= 32768:
            t = t - 65536
        t = 0.1 * t
        h = ord(x[6])
        return t, h

    @staticmethod
    def decode_wind(x):
        # speed in km/h
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
        # pressure is units of mbar
        return 0.1 * (256 * ord(x[2]) + ord(x[3]))

    @staticmethod
    def decode_rain(x):
        # hardare reports inches and inch/hour, convert to cm and cm/hour
        # rain in past hour in inches
        rh = 0.01 * (256 * ord(x[2]) + ord(x[3])) * 2.54
        if x[2:4].encode('hex') == 'fffe':
            rh = None
        # rain rate in inch/hour
        rr = 0.01 * (256 * ord(x[4]) + ord(x[5])) * 2.54
        # last 24 hours in inches
        r24 = 0.01 * (256 * ord(x[6]) + ord(x[7])) * 2.54
        # rain total in inches
        rt = 0.01 * (256 * ord(x[8]) + ord(x[9])) * 2.54
        return rr, rh, r24, rt

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

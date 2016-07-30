import serial
from time import *
from struct import *

class venus838flpx(object):
    """SkyTraq Venus 8 GNSS Receiver
    
    Send and receive binary message from the Venus838FLPx.
    Based on, https://github.com/smokingresistor/TeensyGPS
    """

    __DEBUG = True
    __device = None

    BAUDRATE_4800 = 0
    BAUDRATE_9600 = 1
    BAUDRATE_19200 = 2
    BAUDRATE_38400 = 3
    BAUDRATE_57600 = 4
    BAUDRATE_115200 = 5
    BAUDRATE_230400 = 6
    BAUDRATE_460800 = 7
    BAUDRATE_921600 = 8
    
    def __checksum(self, payload):
        """Return checksum value for the payload given.
        """
        cs = 0
        for b in payload:
            cs ^= b
        return cs
        
    def SetMessageType(self, outype=1):
        """Change GPS output type to NMEA or binary.

        MessageID, 09
        
        Type,
            00, No output
            01, NMEA
            02, Binary
            
        Attributes,
            00 Update to SRAM
            01 Update SRAM and Flash
            
        Response ACK or NACK
        """
        payload = [0x09, outype, 0x00]
        resp = self.__send(payload, 0)
        
        if resp != 1:
            return -1

        return 1
        
    def SetNMEAMessageInterval(self, GGA=1, GSA=0, GSV=0, GLL=0, RMC=1, VTG=0, ZDA=0):
        """Configure NMEA Message.

        MessageID, 08
        
        GGA, 0 - 255 # 0 Disabled
        GSA, 0 - 255 # 0 Disabled
        GSV, 0 - 255 # 0 Disabled
        GLL, 0 - 255 # 0 Disabled
        RMC, 0 - 255 # 0 Disabled
        VTG, 0 - 255 # 0 Disabled
        ZDA, 0 - 255 # 0 Disabled
        
        Attributes,
            00 Update to SRAM
            01 Update SRAM and Flash
                
        Response ACK or NACK
        """
        payload = [0x08, GGA, GSA, GSV, GLL, RMC, VTG, ZDA, 0x00]
        resp = self.__send(payload, 0)
        
        if resp != 1:
            return -1
        return 1
        
    def SetPositionUpdateRate(self, rate=10):
        """Configure position update rate.

        MessageID, 0E
        
        Rate,
            01, 1hz
            02, 2hz
            04, 4hz
            05, 5hz
            08, 8hz
            0A, 10hz
            14, 20hz
            19, 25hz
            28, 40hz
            32, 50hz
              
        Attributes,
            00 Update to SRAM
            01 Update SRAM and Flash
                
        Response ACK or NACK
        """

        payload = [0x0E, rate, 0x00]
        resp = self.__send(payload, 0)

        if resp != 1:
            return -1
        return 1
     
    def SetDeviceBaudrate(self, baudrate=1):
        """Auto-detect serial port baudrate and set it to given rate.
        """
        rates = [4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]

        for n in rates:

            # Set Baudrate and test Get Software Version command
            # If it return ACK then you know the rate selected is correct
            if self.__DEBUG:
                print('Testing Baudrate: '),
                print(n)
            self.__device.baudrate = n
            response = self.__send([0x02, 0x00], 0)

            # If we receive ACK then set the new baudrate
            if (response == 1):
                self.__send([0x05, 0x00, baudrate, 0], 0)
                self.__device.baudrate = rates[baudrate]
                return 1
                
        return 0
                
    def SetGNSSNavigationMode(self, mode=0):
        """Set GPS Navigation mode

        MessageID, 64, 17
        
        Modes,
            0: auto 
            1: pedestrian 
            2: car 
            3: marine 
            4: balloon 
            5: airborne
            
        Attributes,
            00 Update to SRAM
            01 Update SRAM and Flash
            
        Response ACK or NACK

        """
        payload = [0x64, 0x17, mode, 0x00]
        resp = self.__send(payload, 0)
        
        if resp != 1:
            return -1

        return 1

    def GetGNSSNavigationMode(self):
        """Get GPS Navigation mode
        MessageID, 0x64, 0x18
        Response 0x8B
        """
        response = self.__send([0x64,0x18], 0x8B)

        print(response)

        if (response < 4 or response[4] != 0x8B):
            return -1

        return 1

    def GetVersion(self):
        """Get software version of the GPS module
        MessageID, 0x02, 0x00
        Response 0x80
        """
        response = self.__send([0x02,0x00], 0x80)

        if (response < 19 or response[4] != 0x80):
            return
        
        print('Type: {0}'.format(response[4]))
        Kversion = (response[5] << 24) | (response[6]<<16) | (response[7]<<8) | response[8];
        print('Kversion: {0}'.format(Kversion))
        ODMversion = (response[9]<<24)|(response[10]<<16)|(response[11]<<8)|response[12];
        print('ODMversion: {0}'.format(ODMversion))
        Revision = (response[13]<<24)|(response[14]<<16)|(response[15]<<8)|response[16];
        print('Revision: {0}'.format(Revision))
        
        return True   
    
    def __receive(self, timeout=0):
        """Receive message from the GPS module. Return only binary answer messages. 
        """      
        t = time() + (1 + timeout)
        
        c = 0
        e = 0

        buffer = [0x00] * 128
        buffer_all = [0x00] * 2048

        header = [160, 161]
        payloadlen = 0

        try:
            while time() < t:
                while self.__device.inWaiting() != 0:
                    char = ord(self.__device.read())
                    if self.__DEBUG:
                        buffer_all[e] = char
                        e += 1
                    if (c < 2):
                        if (char == header[c]):
                            buffer[c] = char
                            c += 1
                            pass
                    elif c > 1 and c<4:
                        buffer[c] = char
                        c += 1
                        if c == 4:
                            payloadlen = (buffer[2] << 8) | buffer[3]
                        pass
                    elif c == 4:
                        buffer[c] = char
                        c += 1
                        pass
                    elif c > 4:
                        while c < payloadlen + 6:
                            char = ord(self.__device.read())
                            buffer[c] = char
                            c += 1
                        return buffer[:c]
        except:
            if self.__DEBUG:
                print('<<<'),
                print(buffer_all)
                print('[Venus838FLPx]: Critical error while receiving data!')
            return -1

        return -1
    
    def __send(self, message, msgid):
        """Send binary message to GPS module then wait for answer. Header, lenght, checksum and end are automaticly added.
        """
        c = len(message) + 7
        data = [0x00] * (c)

        message_len = pack('h',1)

        # Start
        data[0] = 0xA0
        data[1] = 0xA1

        # Len
        data[2] = 0x00
        data[3] = len(message)

        # Payload

        for i in xrange(0, len(message)):
            data[i + 4] = message[i]

        # Checksum
        data[c - 3] = self.__checksum(message)

        # End
        data[c - 2] = 0x0D
        data[c - 1] = 0x0A

        self.__device.flushInput()
        self.__device.flushOutput()

        if self.__DEBUG:
            print('>>>'),
            print(data)
        self.__device.write(data)
        
        buf = self.__receive()
        if buf == -1:
            if self.__DEBUG:
                print('Error: Receive command timeout.')
            return -1
        
        l = len(buf)
        
        if buf[0] != 0xA0 or buf[1] != 0xA1:
            if self.__DEBUG:
                print('Error: Message unrecognized.')
            return -1
        
        if buf[l-2] != 0x0D or buf[l-1] != 0x0A:
            if self.__DEBUG:
                print('Error: Message imcomplete.')
            return -1       
        
        if buf[4] == 0x83:
            if self.__DEBUG:
                print('ACK')
            if msgid != 0:
                buf = self.__receive()
                return buf
            return 1
                
        if buf[4] == 0x84:
            if self.__DEBUG:
                print('NACK')
            return 0
                
        return buf[4]
    
    def GetSerialDevice(self):
        """Return serial device.
        """
        return self.__device
        
    def __init__(self, PORT, UPDATE_RATE = 1):
        """Init GPS Module and set default configuration.
        """
        self.__device = serial.Serial(PORT, baudrate=9600, timeout=1)

        self.SetDeviceBaudrate(baudrate=self.BAUDRATE_115200) # 115200
        print('Change message type...')
        self.SetMessageType() # default NMEA
        print('Change navigation mode...')
        self.SetGNSSNavigationMode() # default auto
        print('Configure NMEA message interval...')
        self.SetNMEAMessageInterval() # default GGA and RMC
        print('Change update rate...')
        self.SetPositionUpdateRate(rate=UPDATE_RATE) # default 1hz


# Test
import pynmea2

gps = venus838flpx('/dev/ttyO1', 10)

device = None

while device == None:
    device = gps.GetSerialDevice()

print('Navigation mode: ')
gps.GetGNSSNavigationMode()
gps.GetVersion()
c = 0

lat = None
lon = None
speed = None
time = None
sats = None
qual = None
heading = None
alt = None

'''
while c < 100:
        c += 1
        streamreader = pynmea2.NMEAStreamReader(device)
        sample_data_count = 0

        try:
            for msg in streamreader.next():
                if msg.sentence_type == 'GGA':


                    lat = msg.latitude
                    lon = msg.longitude
                    time = msg.timestamp
                    sats = msg.num_sats
                    qual = msg.gps_qual
                    alt = msg.altitude

                if msg.sentence_type == 'RMC':

                    speed = msg.spd_over_grnd * 1.852
                    heading = msg.true_course

            print('Lat: {0}, Lon: {1}, Speed: {2}, Time: {3}, Sats: {4}, Quality: {5}, Heading: {6}'.format(lat, lon, speed, time, sats, qual, heading))

        except:
            raise
            pass
'''

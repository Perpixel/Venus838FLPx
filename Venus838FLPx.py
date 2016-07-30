import serial
from time import *
from struct import *

class Venus838FLPx(object):

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
    
    def __checksum(self, data):
        cs = 0
        n = len(data)
        for b in data:
            cs ^= b
        return cs
        
    def SetMessageType(self, outype=1):
        '''
        MessageID, 09
        
        Type,
            00, No output
            01, NMEA
            02, Binary
            
        Attributes,
            00 Update to SRAM
            01 Update SRAM and Flash
            
        Response ACK or NACK
        '''      
        payload = [0x09, outype, 0x00]
        resp = self.__send(payload, 0)
        
        if resp != 1:
            return -1
        return 1
        
    def SetNMEAMessageInterval(self, GGA=1, GSA=0, GSV=0, GLL=0, RMC=1, VTG=0, ZDA=0):
        '''
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
        '''
        payload = [0x08, GGA, GSA, GSV, GLL, RMC, VTG, ZDA, 0x00]
        resp = self.__send(payload, 0)
        
        if resp != 1:
            return -1
        return 1
        
    def SetPositionUpdateRate(self, rate=10):
        '''
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
        '''

        payload = [0x0E, rate, 0x00]
        resp = self.__send(payload, 0)

        if resp != 1:
            return -1
        return 1
     
    def SetDeviceBaudrate(self, baudrate=1):
        
        rates = [4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]

        for n in rates:

            # Set Baudrate and test Get Software Version command
            # If it return ACK then you know the rate selected is correct
            self.__device.baudrate = n
            response = self.__send([0x02, 0x00], 0)

            # If we receive ACK then set the new baudrate
            if (response == 1):
                self.__send([0x05, 0x00, baudrate, 0], 0)
                self.__device.baudrate = rates[baudrate]
                return 1
                
        return 0
                
    def GetVersion(self):
    
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
        
        chksum = self__checksum(response[4:17])
        
        if (response[17] != chksum):
            print('Checksum error')
            return False

        return True

    def GetGNSSNavigationMode():
        return True
        '''
        GETNAVMODE = [0x64,0x18]
        send(GETNAVMODE, s)
        
        c = 0
        data = [0x00] * 128
        
        while s.inWaiting() != 0:
            data[c] = ord(s.read())
            c += 1

        s.flushInput()
        s.flushOutput()
        
        if (data[4] != 0x83):
            return
        
        print(data[16])
        
        chksum = checksum(data[14:17])
        
        if (data[17] != chksum):
            print('Checksum error')
            return False

        return True
        '''
    
    def __receive(self, timeout=0):
                
        t = time() + (1 + timeout)
        
        c = 0

        buffer = [0x00] * 128
        header = [160, 161]
        payloadlen = 0

        try:
            while time() < t:
                while self.__device.inWaiting() != 0:
                    char = ord(self.__device.read())
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
            print ('[Venus838FLPx]: Critical error while receiving data!')
            return -1

        return -1
    
    def __send(self, message, msgid):
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

        self.__device.write(data)
        
        buf = self.__receive()
        if buf == -1:
            #print('Error: Receive command timeout.')
            return -1
        
        l = len(buf)
        
        if buf[0] != 0xA0 or buf[1] != 0xA1:
            #print('Error: Message unrecognized.')
            return -1
        
        if buf[l-2] != 0x0D or buf[l-1] != 0x0A:
            #print('Error: Message imcomplete.')
            return -1       
        
        if buf[4] == 0x83:
            #print('ACK')
            if msgid != 0:
                buf = self.__receive()
                return buf
            return 1
                
        if buf[4] == 0x84:
            #print('NACK')
            return 0
                
        return buf[4]
    
    def GetSerialDevice(self):
        return self.__device
        
    def __init__(self, PORT):
            
        self.__device = serial.Serial(PORT, baudrate=9600, bytesize=8, parity='N', stopbits=1, timeout=1)

        self.SetDeviceBaudrate(baudrate=self.BAUDRATE_115200) # 115200
        print('Change message type...')
        self.SetMessageType() # NMEA
        print('Configure NMEA message interval...')
        self.SetNMEAMessageInterval() # GGA and RMC
        print('Change update rate...')
        self.SetPositionUpdateRate(rate=20) # 10hz

# Test
'''
gps = Venus838FLPx('/dev/ttyO1')

device = None

while device == None:
    device = gps.GetSerialDevice()

print device

while 1:
        while device.inWaiting() == 0:
                pass
        l = device.readline() 
        print l
'''
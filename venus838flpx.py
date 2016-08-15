import serial
from time import *
from struct import *

class venus838flpx(object):
    """SkyTraq Venus 8 GNSS Receiver
    
    Send and receive binary message from the Venus838FLPx.
    Based on, https://github.com/smokingresistor/TeensyGPS
    """

    __DEBUG = False
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

    def Test(self):      
        #self.SetFactoryDefault()
        return
        
    def SetFactoryDefault(self):
       
        payload = [4, 1]
        resp = self.__send(payload)
        
        if resp != 1:
            return -1

        return 1 
        
    def ResetGNNS(self):
       
        payload = [1, 1, 16>>8, 16, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0]
        resp = self.__send(payload)
        
        if resp != 1:
            return -1

        return 1 

    def SetMessagesOutputRate(self):
       
        payload = [18, 10, 0, 0, 0, 0, 0, 0]
        resp = self.__send(payload)
        
        if resp != 1:
            return -1

        return 1 
        
    def ParseNavigationDataMessage(self):
        # in binary message mode only
        buf = (self.__receive());
                
        if buf == -1:
            return
        elif buf[4] == 168:
            
            fix = buf[5]
            num_sv = buf[6]
            gnss_week = buf[7] << 8 | buf[8];
            tow = buf[9] << 24 | buf[10] << 16 | buf[11] << 8 | buf[12];
            latitude = buf[13] << 24 | buf[14] << 16 | buf[15] << 8 | buf[16];
            longitude = buf[17] << 24 | buf[18] << 16 | buf[19] << 8 | buf[20];
            ell_altitude = buf[21] << 24 | buf[22] << 16 | buf[23] << 8 | buf[24];
            sea_altitude = buf[25] << 24 | buf[26] << 16 | buf[27] << 8 | buf[28];
            
            print('Fix: {0}, Num SV: {1}, GNSS Week: {2} , GNSS TOW: {3} , Latitude: {4} , Longitude: {5} , Ellipsoid Altitude: {6} , Sea level Altitude: {7}'.format(fix, num_sv, gnss_week, tow, latitude, longitude, ell_altitude, sea_altitude))
        
    def SetNavigationInterval(self, interval = 7):
    
        payload = [17, interval, 0]
        resp = self.__send(payload)
        
        if resp != 1:
            return -1

        return 1    
    
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
        resp = self.__send(payload)
        
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
        resp = self.__send(payload)
        
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
        resp = self.__send(payload)

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
                print n
            self.__device.baudrate = n
            self.SetFactoryDefault()
            response = self.__send([0x02, 0x00])
            # If we receive ACK then set the new baudrate
            if (response == 1):
                self.__send([0x05, 0x00, baudrate, 0])
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
        resp = self.__send(payload)
        
        if resp != 1:
            return -1

        return 1     

    def GetGNSSNavigationMode(self):
        """Get GPS Navigation mode
        MessageID, 0x64, 0x18
        Response 0x64

        The documentation is wrong about the response to get navigation mode. SUBID should be 0x8B according to it but the actual resposne sontain only the message id 0x64 followed byt the nav mode value.
        """
        if (self.__send([0x64,0x18]) == 1):
            # ACK received then read answer
            response = self.__receive()            
            if (len(response) == 1):
                # message corrupted
                return -1

            if (response[4] == 0x64):
                return response[5]

        return -1

    def GetVersion(self):
        """Get software version of the GPS module
        MessageID, 0x02, 0x00
        Response 0x80
        """
        if (self.__send([0x02,0x00]) == 1):
            # ACK received then read answer
            response = self.__receive()    
        
            if (len(response) == 1):
                # message corrupted
                return 0
            elif (response < 19 or response[4] != 0x80):
                return 0
        
            print('Type: {0}'.format(response[4]))
            Kversion = (response[5] << 24) | (response[6]<<16) | (response[7]<<8) | response[8];
            print('Kversion: {0}'.format(Kversion))
            ODMversion = (response[9]<<24)|(response[10]<<16)|(response[11]<<8)|response[12];
            print('ODMversion: {0}'.format(ODMversion))
            Revision = (response[13]<<24)|(response[14]<<16)|(response[15]<<8)|response[16];
            print('Revision: {0}'.format(Revision))
        
        return 1
  
    def GetSerialDevice(self):
        """Return serial device.
        """
        return self.__device

    def ResetIOBuffer(self):
        self.__device.reset_input_buffer();
        self.__device.reset_output_buffer();  

    def __checksum(self, payload):
        """Return checksum value for the payload given.
        """
        cs = 0
        for b in payload:
            cs ^= b
        return cs

    def __ValidateBinaryMessage(self, buf):
        """Validate that the message is complete and valid
        """
        l = len(buf)

        start = [0xA0, 0xA1]
        end = [0x0D, 0x0A]
        
        if self.__DEBUG:
            #print(buf)
            #print(buf[0:2])
            #print(buf[-2:])     
            pass

        if buf[0:2] != start:
            if self.__DEBUG:
                print('Error: Message unrecognized.')
            return -1
        elif buf[-2:] != end:
            if self.__DEBUG:
                print('Error: Message imcomplete.')
            return -1
        # check len
        # check checksum
        return 1
    
    def __receive(self, timeout=1):
        """Receive message from the GPS module. Return only binary answer messages. 
        """      
        t = time() + timeout
        
        c = 0

        buffer = [0x00] * 256
        header = [160, 161]
        payloadlen = 0

        try:
            while time() < t:
                while self.__device.inWaiting() != 0 and time() < t:

                    char = ord(self.__device.read())    

                    if (c == 0) and char != 160:
                        pass
                    elif (c == 1) and char != 161:
                        pass

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
                        # ok we got the full message, now validate it
                        if (self.__ValidateBinaryMessage(buffer[:c]) != 1):
                            # message corrupted
                            return -2
                        return buffer[:c]
        except:
            if self.__DEBUG:
                print('[Venus838FLPx]: Critical error while receiving data!')
            self.ResetIOBuffer()
            return -2

        self.ResetIOBuffer()
        return -1
    
    def __send(self, message):
        """Send binary message to GPS module then wait for ACK or NACK answer. Header, lenght, checksum and end are automaticly added.

        returns,
            error -1
            nack 0
            ack 1
        """
        c = len(message) + 7
        data = [0x00] * (c)

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

        self.__device.reset_input_buffer();
        self.__device.reset_output_buffer();

        if self.__DEBUG:
            print('>>>'),
            print(data)

        self.__device.write(data)
        self.__device.flush()

        buf = self.__receive()

        if buf == -1:
            if self.__DEBUG:
                print('Error: Receive command timeout.')
            sleep(0.1)
            return -1

        elif buf == -2:
            if self.__DEBUG:
                print('Error: response invalid.')
            sleep(0.1)
            return -1
        
        result = None        
       
        if buf[4] == 0x83:
            if self.__DEBUG:
                print('ACK')
            result = 1
                
        elif buf[4] == 0x84:
            if self.__DEBUG:
                print('NACK')
            result = 0
        else:
            result = -1

        sleep(0.01)
        return result
            
    def __init__(self, PORT, UPDATE_RATE = 1, verbose = False):
        """Init GPS Module and set default configuration.
        """
        self.__DEBUG = verbose
        print('Configure serial port...')
        self.__device = serial.Serial(PORT, baudrate=9600, timeout=1)
        self.SetDeviceBaudrate(baudrate=self.BAUDRATE_9600)
        self.Test()
        #print('Reset GNNS...')
        #self.ResetGNNS()
        print('Change message type...')
        self.SetMessageType(2) # default NMEA
        #print('Configure NMEA message interval...')
        #self.SetNMEAMessageInterval() # default GGA and RMC
        print('Change update rate...')
        self.SetPositionUpdateRate(rate=UPDATE_RATE) # default 1hz
        #print('Change navigation mode...')
        #self.SetGNSSNavigationMode() # default auto

import pynmea2
from venus838flpx import *

# Test
print('Testing venus838flpx class...')

GPS = venus838flpx('/dev/ttyO1', 20, True)

print('Binary mode: ')
GPS.SetMessageType()
print('Set nav mode to 3: ')
GPS.SetGNSSNavigationMode(3)
print('\tNavigation mode: {0}'.format(GPS.GetGNSSNavigationMode()))
print('Set nav mode to 0: ')
GPS.SetGNSSNavigationMode(1)
print('\tNavigation mode: {0}'.format(GPS.GetGNSSNavigationMode()))
print('Get firmware version: ')
GPS.GetVersion()

c = 0

lat = None
lon = None
speed = None
time = None
sats = None
qual = None
heading = None
alt = None

GPS.ResetIOBuffer()

while c < 100:
        c += 1
        streamreader = pynmea2.NMEAStreamReader(GPS.GetSerialDevice())
        sample_data_count = 0

        try:
            for msg in streamreader.next():
                #print(msg)
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
import serial
from math import ceil, floor

# -----------------------------------------------------------------------------------------------
class gps:
    def __init__(self):
        self.port = "/dev/ttyAMA0"
        self.lat = "*********"
        self.lon = "*********"
        self.beacon_lat = "00 00 00 00 0"
        self.beacon_lon = "0 00 00 00 00"

# -----------------------------------------------------------------------------------------------
    def parseGPS(self,data):
        if data[0:6] == "$GNRMC":
            sdata = data.split(",")
            if sdata[2] == "V":
                print ("longitude : %s, latitude : %s" % (self.lat,self.lon))
                return
            print ("---Parsing GNRMC---")
            lat_trans = str(floor(int(sdata[3][2:4]+sdata[3][5:9]) * 100 / 60))
            lon_trans = str(floor(int(sdata[5][3:5]+sdata[5][6:10]) * 100 / 60))
            
            if len(lat_trans)== 5 :
                lat_trans = "0" + lat_trans
            if len(lon_trans)== 5 :
                lon_trans = "0" + lon_trans
            
            self.beacon_lat = self.decode_str_lat(sdata[3][0:2] + lat_trans) 
            self.beacon_lon = self.decode_str_lon(sdata[5][0:3] + lon_trans)
#            print ("longitude : %s, latitude : %s" % (sdata[3] , sdata[5]))
#            print ("longitude : %s, latitude : %s" % (sdata[3][0:2] + "." + lat_trans, sdata[5][0:3] + "." + lon_trans))
#            print ("longitude : %s, latitude : %s" % (self.beacon_lat, self.beacon_lon))


# -----------------------------------------------------------------------------------------------
 
    def decode_str_lat(self,data):
        lat_str1 = "0" + data[0:1] + " "
        lat_str2 = data[1:3] + " "
        lat_str3 = data[3:5] + " "
        lat_str4 = data[5:7] + " "
        lat_str5 = data[7:8]
        return lat_str1 + lat_str2 + lat_str3 + lat_str4 + lat_str5

# -----------------------------------------------------------------------------------------------
    def decode_str_lon(self,data):
        lon_str1 = data[0:1] + " "
        lon_str2 = data[1:3] + " "
        lon_str3 = data[3:5] + " "
        lon_str4 = data[5:7] + " "
        lon_str5 = data[7:9]
        return lon_str1 + lon_str2 + lon_str3 + lon_str4 + lon_str5

# -----------------------------------------------------------------------------------------------
if __name__ == "__main__" :
    test_gps = gps()
    print("Receiving GPS data")
    ser=serial.Serial(test_gps.port, baudrate = 9600)
    while True:
        try:
            data = ser.readline()
            data = data.decode('utf-8')
            test_gps.parseGPS(data)
        except:
            ser = serial.Serial(test_gps.port, baudrate = 9600)
            pass

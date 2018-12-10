import subprocess
import socket
import sys
import blescan
import bluetooth._bluetooth as bluez
import gps_module
from i2clibraries import i2c_hmc5883l as compass

from time import sleep


# ibeacon advertising class
class Beacon_tx:
    
    def __init__(self):
# ibeacon setting
        self.bct_BLUETOOTHDEVICE = "hci0"
        self.bct_OGF = "0x08"
        self.bct_OCF_format = "0x0008" 
        self.bct_OCF_setting = "0x0006"
        self.bct_OCF_operate = "0x000A"
        self.bct_start = "01"
        self.bct_stop = "00"
        self.bct_IBEACONPROFIX = "1E 02 01 1A 1A FF 4C 00 02 15"
        self.bct_UUID = "FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF"
        self.bct_MAJOR = "00 00"
        self.bct_MINOR = "00 00"
        self.bct_POWER = "C8 00"
        self.formation = "0A"
        self.gps_lat = "12 34 56 78 9"
        self.gps_long = "1 23 45 67 89"

# -----------------------------------------------------------------------------------------------
# raspi bluetooth setting
    def beacon_TX_config(self,_up,_leadv3,_noscan):
        self.result1 = subprocess.check_output("sudo hciconfig " + self.bct_BLUETOOTHDEVICE + " "+_up, shell=True)
        self.result2 = subprocess.check_output("sudo hciconfig " + self.bct_BLUETOOTHDEVICE + " "+_leadv3, shell=True)
        self.result3 = subprocess.check_output("sudo hciconfig " + self.bct_BLUETOOTHDEVICE + " "+_noscan, shell=True)
        
# -----------------------------------------------------------------------------------------------
# raspi bluetooth finish
    def beacon_TX_config_down(self,_param):
        result = subprocess.check_output("sudo hciconfig " + self.bct_BLUETOOTHDEVICE + " "+_param, shell=True)
                                         
# -----------------------------------------------------------------------------------------------
# ibeacon data format
    def beacon_TX_cmd_format(self,_ocf, _ibeaconprofix, _uuid, _major, _minor, _power):
        _bct_ogf = self.bct_OGF + " "
        _ocf = _ocf + " "
        _ibeaconprofix = _ibeaconprofix + " "
        _uuid = _uuid + " "
        _major = _major + " "
        _minor = _minor + " "
        result = subprocess.check_output("sudo hcitool -i " + self.bct_BLUETOOTHDEVICE + " cmd " + _bct_ogf + _ocf + _ibeaconprofix + _uuid + _major + _minor + _power, shell=True)
        print("finish advertising")

# -----------------------------------------------------------------------------------------------

# ibeacon interval setting
    def beacon_TX_cmd_setting(self,_ocf, _interval):
        _bct_ogf = self.bct_OGF + " "
        _ocf = _ocf + " "
        _intervalHEX = '{:04X}'.format(int(_interval/0.625))
        _minInterval = _intervalHEX[2:] + " " + _intervalHEX[:2] + " "
        _maxInterval = _intervalHEX[2:] + " " + _intervalHEX[:2] + " "
        result = subprocess.check_output("sudo hcitool -i " + self.bct_BLUETOOTHDEVICE + " cmd " + _bct_ogf + _ocf + _maxInterval + _maxInterval + "03 00 00 00 00 00 00 00 00 07 00", shell=True)


# -----------------------------------------------------------------------------------------------

# ibeacon start flag
    def beacon_TX_cmd_operate(self,_ocf,_param):
        _bct_ogf = self.bct_OGF + " "
        _ocf = _ocf + " "
        result = subprocess.check_output("sudo hcitool -i" + self.bct_BLUETOOTHDEVICE + " cmd " + _bct_ogf + _ocf + _param, shell=True)


# -----------------------------------------------------------------------------------------------

# ibeacon data advertising function
    def beacon_TX_DevTrigger(self,_formation,gps_lat,gps_long,compass_head):
        _bct_uuid = _formation + " "+ gps_lat +  gps_long + " " + compass_head +"0 00 00 00 00"
        self.beacon_TX_cmd_format(self.bct_OCF_format, self.bct_IBEACONPROFIX, _bct_uuid, self.bct_MAJOR, self.bct_MINOR, self.bct_POWER)
        sleep(1)


# -----------------------------------------------------------------------------------------------

# test_main
if __name__ == "__main__" :
    test = Beacon_tx()
    test.beacon_TX_config("up","leadv 3","noscan")
    test.beacon_TX_cmd_format(test.bct_OCF_format, test.bct_IBEACONPROFIX, test.bct_UUID, test.bct_MAJOR, test.bct_MINOR, test.bct_POWER)
    test.beacon_TX_cmd_setting(test.bct_OCF_setting, 100)
    test.beacon_TX_cmd_operate(test.bct_OCF_operate, test.bct_start)
    test_compass = compass.i2c_hmc5883l(1)
    test_compass.setContinuousMode()
    test_compass.setDeclination(0,6)
    test_gps = gps_module.gps()
    
    print("Receiving GPS data")
    print ("BLE EMIT START")
    
    ser=gps_module.serial.Serial(test_gps.port, baudrate = 9600)
    while True:
        try:
            test.beacon_TX_DevTrigger(test.formation,test_gps.beacon_lat,test_gps.beacon_lon,test_compass.getHeadingString())
            print(test_compass)
            for i in range(1,17):
                data = ser.readline()
                data = data.decode('utf-8')
                test_gps.parseGPS(data)
        except:
            ser = gps_module.serial.Serial(test_gps.port, baudrate = 9600)
            pass
    
        finally:
# test finish
            print("BLE EMIT STOP")
            test.beacon_TX_cmd_operate(test.bct_OCF_operate, test.bct_stop)
            test.beacon_TX_config_down("down")
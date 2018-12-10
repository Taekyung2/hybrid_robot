import tcpServer
import executer
import time
import queue
import tx_Beacon
import gps_module
import spi_config as compass

# -----------------------------------------------------------------------------------------------
# make public queue
commandQueue = queue.Queue()

# -----------------------------------------------------------------------------------------------


# init module
andRaspTCP = tcpServer.TCPServer(commandQueue, "", 500)
andRaspTCP.start()

# -----------------------------------------------------------------------------------------------


# set module to executer
commandExecuter = executer.Executer(andRaspTCP)


# -----------------------------------------------------------------------------------------------

# set module to tx_Beacon
commandBeacon = tx_Beacon.Beacon_tx()
commandBeacon.beacon_TX_config("up","leadv 3","noscan")
commandBeacon.beacon_TX_cmd_format(commandBeacon.bct_OCF_format, commandBeacon.bct_IBEACONPROFIX, commandBeacon.bct_UUID, commandBeacon.bct_MAJOR, commandBeacon.bct_MINOR, commandBeacon.bct_POWER)
commandBeacon.beacon_TX_cmd_setting(commandBeacon.bct_OCF_setting, 100)
commandBeacon.beacon_TX_cmd_operate(commandBeacon.bct_OCF_operate, commandBeacon.bct_start)

# -----------------------------------------------------------------------------------------------

#set module to compass
commandCompass = compass.MPU9250()
commandCompass.initialize()


# -----------------------------------------------------------------------------------------------

#set module to GPS
commandGps = gps_module.gps()
ser=gps_module.serial.Serial(commandGps.port, baudrate = 9600)


# -----------------------------------------------------------------------------------------------

try:
    while True:
        try:
            print ("BLE EMIT START")
    #            commandBeacon.beacon_TX_DevTrigger(commandExecuter.formation,commandGps.beacon_lat,commandGps.beacon_lon,commandCompass.getHeadingString())
            commandBeacon.beacon_TX_DevTrigger(commandExecuter.tx_Beacon.formation,commandGps.beacon_lat,commandGps.beacon_lon,commandCompass.getHeadingString())
            print(commandCompass)
            print("Receiving GPS data")
            for i in range(1,10):
                data = ser.readline()
                data = data.decode('utf-8')
                commandGps.parseGPS(data)
            command = commandQueue.get_nowait()
            commandExecuter.startCommand(command)

        except queue.Empty:
            pass
        


    #        except KeyboardInterrupt:
    #            andRaspTCP.serverSocket.close()


        except:
            ser=gps_module.serial.Serial(commandGps.port, baudrate = 9600)
            pass
finally :
    print("BLE EMIT STOP")
    commandBeacon.beacon_TX_cmd_operate(commandBeacon.bct_OCF_operate, commandBeacon.bct_stop)
    commandBeacon.beacon_TX_config_down("down")
    andRaspTCP.serverSocket.close()

#while True:
#    time.sleep(3)
#    andRaspTCP.sendAll("321\n")
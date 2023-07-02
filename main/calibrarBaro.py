# Ref: https://www.youtube.com/watch?v=XXjFtYZEQNw
# TODO: TEST PROGRAM

import serial.tools.list_ports

ports = list(serial.tools.list_ports.comports())
serialInst = serial.Serial()

portsList = []

for onePort in ports:
    portsList.append(str(onePort))
    print(str(onePort))

val = input("Select port: COM")

for x in range(0, len(portsList)):
    if portsList[x].startswith("COM" + str(val)):
        portVal = "COM" + str(val)
        print(portVal)

serialInst.baudrate = 9600
serialInst.port = portVal
serialInst.open()

while True:
    command = "Initial _Altitude:" + input("Type the value of the initial altitude in meters (example: 123.23 or 123.0): ")
    serialInst.write(command.encode())

    if command == 'exit':
        exit()
        
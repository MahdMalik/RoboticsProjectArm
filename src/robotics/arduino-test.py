import serial
from enum import IntEnum
import time

BAUD_RATE = 115200
PORT = "/dev/ttyUSB0"

class SentPacketType(IntEnum):
    HANDSHAKE = 0x00
    MOVE_FORWARD = 0x01
    MOVE_BACKWARD = 0x02
    TURN_STATIONARY = 0x03
    STOP = 0x04


class ReceivedPacketType(IntEnum):
    HANDSHAKE_RESPONSE = 0x00
    ACTION_SUCCESS = 0x01
    ACTION_FAILED = 0x02

class ArduinoComm:
    def __init__(self, baud_rate, port):
        self.baud_rate = baud_rate
        self.port = port
        self.arduino = serial.Serial(port=self.port, baudrate=self.baud_rate, timeout=None, write_timeout=1)
        self.RECEIVED_PACKET_SIZE = 2

        time.sleep(2)

    def SendPacket(self, messageType, otherData):
        packet = [messageType, *otherData]
        self.arduino.write(packet)

    def ReceivePacket(self):
        response = self.arduino.read(self.RECEIVED_PACKET_SIZE)

        messageType = ReceivedPacketType(response[0])
        messageParam = response[1]

        return messageType, messageParam

    def Cleanup(self):
        self.arduino.close()

serialComm = ArduinoComm(BAUD_RATE, PORT)


print("Setup done!")
serialComm.SendPacket(SentPacketType.HANDSHAKE, [0x67])

packetType, parameter = serialComm.ReceivePacket()

print(f"Packet type was {packetType}, parameter was {parameter}")

serialComm.Cleanup()

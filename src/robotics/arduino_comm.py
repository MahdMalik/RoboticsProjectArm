# usage: python arduino_comm.py [port] [drive|plan]
import sys
import time
from enum import IntEnum

import serial

BAUD_RATE = 115200
PORT = "/dev/ttyUSB0"
DEFAULT_WAIT = 1
BOUNDED_WAIT = 6
MAX_DISTANCE_CM = 255
MAX_TURN_DEGREES = 180
MAX_PLAN_STEPS = 8
MIN_REPLAN_SECONDS = 0.5


class SentPacketType(IntEnum):
    HANDSHAKE = 0x00
    MOVE_FORWARD = 0x01
    MOVE_BACKWARD = 0x02
    TURN_LEFT = 0x03
    STOP = 0x04
    TURN_RIGHT = 0x05
    BATCH_BEGIN = 0x06


class ReceivedPacketType(IntEnum):
    HANDSHAKE_RESPONSE = 0x00
    ACTION_SUCCESS = 0x01
    ACTION_FAILED = 0x02
    BATCH_ACCEPTED = 0x03
    BATCH_DONE = 0x04
    BATCH_CANCELLED = 0x05
    TURN_MEASURED = 0x06


# parameter of ACTION_FAILED
class FailReason(IntEnum):
    BAD_BATCH = 1
    LOAD_TIMEOUT = 2
    SAFETY_STOP = 3


STEP_TYPES = {
    "forward": SentPacketType.MOVE_FORWARD,
    "backward": SentPacketType.MOVE_BACKWARD,
    "left": SentPacketType.TURN_LEFT,
    "right": SentPacketType.TURN_RIGHT,
}


# one movement: kind is forward, backward, left or right
# amount is cm for forward and backward, degrees for left and right
class Step:
    def __init__(self, kind, amount):
        self.kind = kind
        self.amount = amount

    def Validate(self):
        if self.kind not in STEP_TYPES:
            raise ValueError("kind must be forward, backward, left or right")
        limit = MAX_DISTANCE_CM
        if self.kind == "left" or self.kind == "right":
            limit = MAX_TURN_DEGREES
        # amount 0 means run until stop, which is not allowed inside a plan
        if self.amount < 1 or self.amount > limit:
            raise ValueError("amount must be 1 to " + str(limit) + " for " + self.kind)


class ArduinoComm:
    def __init__(self, baud_rate, port, min_replan_seconds=MIN_REPLAN_SECONDS):
        self.baud_rate = baud_rate
        self.port = port
        self.arduino = serial.Serial(port=port, baudrate=baud_rate, timeout=DEFAULT_WAIT, write_timeout=1)
        self.RECEIVED_PACKET_SIZE = 2
        self.SENT_PACKET_SIZE = 2
        self.min_replan_seconds = min_replan_seconds
        self.last_plan_time = None
        self.last_failure = None
        self.last_turn_degrees = None
        self.last_cancelled_steps = None
        self.events = []

        # board resets when the port opens
        time.sleep(2)

    def SendPacket(self, messageType, otherData=None):
        if otherData is None:
            otherData = []

        packet = bytearray([messageType])
        i = 0
        while i < len(otherData):
            packet.append(otherData[i])
            i += 1

        if len(packet) != self.SENT_PACKET_SIZE:
            raise ValueError("packet must be exactly 2 bytes")
        self.arduino.write(packet)

    def ReceivePacket(self, wait_seconds=DEFAULT_WAIT):
        self.arduino.timeout = wait_seconds
        response = self.arduino.read(self.RECEIVED_PACKET_SIZE)
        self.arduino.timeout = DEFAULT_WAIT

        if len(response) < self.RECEIVED_PACKET_SIZE:
            return None, None

        try:
            packet_type = ReceivedPacketType(response[0])
        except ValueError:
            packet_type = None
        return packet_type, response[1]

    # keeps the latest measured turn from the gyro
    def Record(self, packet):
        if packet[0] == ReceivedPacketType.TURN_MEASURED:
            self.last_turn_degrees = packet[1]

    # reads until ACTION_SUCCESS or ACTION_FAILED, other packets go to the event list
    def WaitResult(self, wait_seconds):
        deadline = time.time() + wait_seconds
        result = (None, None)
        done = False
        while not done:
            remaining = deadline - time.time()
            if remaining <= 0:
                done = True
            else:
                packet = self.ReceivePacket(remaining)
                if packet[0] is None:
                    if packet[1] is None:
                        done = True
                elif packet[0] == ReceivedPacketType.ACTION_SUCCESS or packet[0] == ReceivedPacketType.ACTION_FAILED:
                    result = packet
                    done = True
                elif packet[0] == ReceivedPacketType.TURN_MEASURED:
                    self.Record(packet)
                else:
                    self.events.append(packet)
        return result

    def SingleMove(self, packet_type, amount, limit, wait_seconds):
        if amount < 0 or amount > limit:
            raise ValueError("amount must be 0 to " + str(limit))
        self.SendPacket(packet_type, [amount])
        if wait_seconds is None:
            # amount 0 replies at once, a bounded move replies when it finishes
            wait_seconds = DEFAULT_WAIT
            if amount > 0:
                wait_seconds = BOUNDED_WAIT
        return self.WaitResult(wait_seconds)

    # amount 0 runs until Stop, anything else is a bounded move
    def Forward(self, distance_cm=0, wait_seconds=None):
        return self.SingleMove(SentPacketType.MOVE_FORWARD, distance_cm, MAX_DISTANCE_CM, wait_seconds)

    def Backward(self, distance_cm=0, wait_seconds=None):
        return self.SingleMove(SentPacketType.MOVE_BACKWARD, distance_cm, MAX_DISTANCE_CM, wait_seconds)

    def Left(self, degrees=0, wait_seconds=None):
        return self.SingleMove(SentPacketType.TURN_LEFT, degrees, MAX_TURN_DEGREES, wait_seconds)

    def Right(self, degrees=0, wait_seconds=None):
        return self.SingleMove(SentPacketType.TURN_RIGHT, degrees, MAX_TURN_DEGREES, wait_seconds)

    def Stop(self):
        self.SendPacket(SentPacketType.STOP, [0])
        return self.WaitResult(DEFAULT_WAIT)

    # steps is a list of Step objects or (kind, amount) tuples, 1 to 8 steps
    # a new plan replaces the one running: the car stops and the old plan is dropped
    # returns True when the Arduino accepts the plan, False otherwise (see last_failure)
    def SendPlan(self, steps, wait_seconds=DEFAULT_WAIT):
        self.last_failure = None
        self.last_cancelled_steps = None

        if len(steps) < 1 or len(steps) > MAX_PLAN_STEPS:
            raise ValueError("a plan needs 1 to " + str(MAX_PLAN_STEPS) + " steps")

        now = time.time()
        if self.last_plan_time is not None and now - self.last_plan_time < self.min_replan_seconds:
            self.last_failure = "replan too fast"
            return False

        # build the whole batch first, then send it in one write
        packet = bytearray([SentPacketType.BATCH_BEGIN, len(steps)])
        i = 0
        while i < len(steps):
            step = steps[i]
            if isinstance(step, tuple):
                step = Step(step[0], step[1])
            step.Validate()
            packet.append(STEP_TYPES[step.kind])
            packet.append(step.amount)
            i += 1
        # unread events belong to the plan being replaced
        self.events = []
        self.arduino.write(packet)
        self.last_plan_time = now

        deadline = time.time() + wait_seconds
        accepted = False
        done = False
        while not done:
            remaining = deadline - time.time()
            if remaining <= 0:
                self.last_failure = "no reply"
                done = True
            else:
                reply = self.ReceivePacket(remaining)
                if reply[0] == ReceivedPacketType.BATCH_ACCEPTED:
                    accepted = True
                    done = True
                elif reply[0] == ReceivedPacketType.ACTION_FAILED:
                    self.last_failure = "code " + str(reply[1])
                    if reply[1] in (1, 2, 3):
                        self.last_failure = FailReason(reply[1]).name
                    done = True
                elif reply[0] == ReceivedPacketType.BATCH_CANCELLED:
                    # the old plan stopped, reply holds how many of its steps finished
                    self.last_cancelled_steps = reply[1]
                elif reply[0] is not None:
                    # other leftovers from the old plan are dropped
                    self.Record(reply)
        return accepted

    # returns the events that arrived since the last call, oldest first
    # each event is (ReceivedPacketType, parameter)
    def ReadEvents(self, wait_seconds=0):
        if wait_seconds > 0 and self.arduino.in_waiting < 2 and len(self.events) == 0:
            packet = self.ReceivePacket(wait_seconds)
            if packet[0] is not None:
                self.Record(packet)
                self.events.append(packet)

        while self.arduino.in_waiting >= 2:
            packet = self.ReceivePacket(DEFAULT_WAIT)
            if packet[0] is not None:
                self.Record(packet)
                self.events.append(packet)

        events = self.events
        self.events = []
        return events

    # blocks until BATCH_DONE or BATCH_CANCELLED, or the timeout passes
    # returns (finished, events) where events includes the terminal one
    def WaitForPlan(self, timeout_seconds):
        deadline = time.time() + timeout_seconds
        collected = []
        finished = False
        while not finished and time.time() < deadline:
            events = self.ReadEvents(0.2)
            i = 0
            while i < len(events):
                collected.append(events[i])
                if events[i][0] == ReceivedPacketType.BATCH_DONE or events[i][0] == ReceivedPacketType.BATCH_CANCELLED:
                    finished = True
                i += 1
        return finished, collected

    def Cleanup(self):
        self.arduino.close()


if __name__ == "__main__":
    port = PORT
    if len(sys.argv) > 1:
        port = sys.argv[1]

    comm = ArduinoComm(BAUD_RATE, port)
    try:
        comm.SendPacket(SentPacketType.HANDSHAKE, [0x67])
        print(comm.ReceivePacket())

        if len(sys.argv) > 2 and sys.argv[2] == "drive":
            moves = [comm.Forward, comm.Backward, comm.Left, comm.Right]
            i = 0
            while i < len(moves):
                print(moves[i]())
                time.sleep(1)
                print(comm.Stop())
                i += 1

        if len(sys.argv) > 2 and sys.argv[2] == "plan":
            plan = [Step("forward", 20), Step("right", 30), Step("backward", 20)]
            print("accepted", comm.SendPlan(plan), comm.last_failure)
            print(comm.WaitForPlan(10))
            print("measured turn", comm.last_turn_degrees)
    finally:
        comm.Stop()
        comm.Cleanup()
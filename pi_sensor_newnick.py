#!/usr/bin/env python3
"""
Studio 13: Sensor Mini-Project
Raspberry Pi Sensor Interface — pi_sensor.py

Extends the communication framework from Studio 12 with:
  - Magic number + checksum framing for reliable packet delivery
  - A command-line interface that reads user commands while displaying
    live Arduino data

Packet framing format (103 bytes total):
  MAGIC (2 B) | TPacket (100 B) | CHECKSUM (1 B)

Usage:
  source env/bin/activate
  python3 pi_sensor.py
"""

import struct
import serial
import time
import sys
import select

from alex_camera import (
    cameraOpen,
    cameraClose,
    captureGreyscaleFrame,
    renderGreyscaleFrame,
)

from lidar import alex_lidar
from lidar_example_cli_plot import plot_single_scan


# ----------------------------------------------------------------
# SERIAL PORT SETUP
# ----------------------------------------------------------------

PORT     = "/dev/ttyACM0"
BAUDRATE = 9600

_ser = None


def openSerial():
    """Open the serial port and wait for the Arduino to boot."""
    global _ser
    _ser = serial.Serial(PORT, BAUDRATE, timeout=5)
    print(f"Opened {PORT} at {BAUDRATE} baud. Waiting for Arduino...")
    time.sleep(2)
    print("Ready.\n")


def closeSerial():
    """Close the serial port."""
    global _ser
    if _ser and _ser.is_open:
        _ser.close()


# ----------------------------------------------------------------
# TPACKET CONSTANTS
# (must match sensor_miniproject_template.ino / packets.h)
# ----------------------------------------------------------------

PACKET_TYPE_COMMAND  = 0
PACKET_TYPE_RESPONSE = 1
PACKET_TYPE_MESSAGE  = 2

COMMAND_ESTOP  = 0
COMMAND_COLOR  = 1

RESP_OK     = 0
RESP_STATUS = 1
RESP_COLOR  = 2

STATE_RUNNING = 0
STATE_STOPPED = 1

MAX_STR_LEN  = 32
PARAMS_COUNT = 16

TPACKET_SIZE = 1 + 1 + 2 + MAX_STR_LEN + (PARAMS_COUNT * 4)  # = 100
TPACKET_FMT  = f'<BB2x{MAX_STR_LEN}s{PARAMS_COUNT}I'


# ----------------------------------------------------------------
# RELIABLE FRAMING: magic number + XOR checksum
# ----------------------------------------------------------------

MAGIC      = b'\xDE\xAD'
FRAME_SIZE = len(MAGIC) + TPACKET_SIZE + 1   # 103 bytes


def computeChecksum(data: bytes) -> int:
    """Return the XOR of all bytes in data."""
    result = 0
    for byte in data:
        result ^= byte
    return result


def packFrame(packetType, command, data=b'', params=None):
    """Build a framed packet: MAGIC | TPacket bytes | checksum."""
    if params is None:
        params = [0] * PARAMS_COUNT
    data_padded = (data + b'\x00' * MAX_STR_LEN)[:MAX_STR_LEN]
    packet_bytes = struct.pack(TPACKET_FMT, packetType, command,
                               data_padded, *params)
    checksum = computeChecksum(packet_bytes)
    return MAGIC + packet_bytes + bytes([checksum])


def unpackTPacket(raw):
    """Deserialise a 100-byte TPacket into a dict."""
    fields = struct.unpack(TPACKET_FMT, raw)
    return {
        'packetType': fields[0],
        'command':    fields[1],
        'data':       fields[2],
        'params':     list(fields[3:]),
    }


def receiveFrame():
    """
    Read bytes from the serial port until a valid framed packet is found.
    Synchronises on the magic number, reads the TPacket body, then
    validates the checksum.  Returns a packet dict or None on timeout.
    """
    MAGIC_HI = MAGIC[0]
    MAGIC_LO = MAGIC[1]

    while True:
        b = _ser.read(1)
        if not b:
            return None
        if b[0] != MAGIC_HI:
            continue

        b = _ser.read(1)
        if not b:
            return None
        if b[0] != MAGIC_LO:
            continue

        raw = b''
        while len(raw) < TPACKET_SIZE:
            chunk = _ser.read(TPACKET_SIZE - len(raw))
            if not chunk:
                return None
            raw += chunk

        cs_byte = _ser.read(1)
        if not cs_byte:
            return None
        if cs_byte[0] != computeChecksum(raw):
            continue   # bad checksum: resync

        return unpackTPacket(raw)


def sendCommand(commandType, data=b'', params=None):
    """Send a framed COMMAND packet to the Arduino."""
    frame = packFrame(PACKET_TYPE_COMMAND, commandType,
                      data=data, params=params)
    _ser.write(frame)


# ----------------------------------------------------------------
# E-STOP STATE
# ----------------------------------------------------------------

_estop_state = STATE_RUNNING


def isEstopActive():
    """Return True if the E-Stop is currently active (system stopped)."""
    return _estop_state == STATE_STOPPED


# ----------------------------------------------------------------
# PACKET DISPLAY
# ----------------------------------------------------------------

def printPacket(pkt):
    """
    Print a received TPacket in human-readable form.

    RESP_STATUS   -> updates local E-Stop state and prints RUNNING/STOPPED.
    RESP_COLOR    -> prints R/G/B channel frequencies in Hz.
    PACKET_TYPE_MESSAGE -> prints the Arduino debug string.
    Any non-empty data field is printed as an Arduino debug message.
    """
    global _estop_state
    ptype = pkt['packetType']
    cmd   = pkt['command']

    if ptype == PACKET_TYPE_RESPONSE:
        if cmd == RESP_OK:
            print("Response: OK")

        elif cmd == RESP_STATUS:
            state = pkt['params'][0]
            _estop_state = state
            if state == STATE_RUNNING:
                print("Status: RUNNING")
            else:
                print("Status: STOPPED")

        elif cmd == RESP_COLOR:
            r = pkt['params'][0]
            g = pkt['params'][1]
            b = pkt['params'][2]
            print(f"Color: R={r} Hz, G={g} Hz, B={b} Hz")

        else:
            print(f"Response: unknown command {cmd}")

        # Print optional Arduino debug string embedded in any response packet.
        debug = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        if debug:
            print(f"Arduino debug: {debug}")

    elif ptype == PACKET_TYPE_MESSAGE:
        msg = pkt['data'].rstrip(b'\x00').decode('ascii', errors='replace')
        print(f"Arduino: {msg}")

    else:
        print(f"Packet: type={ptype}, cmd={cmd}")


# ----------------------------------------------------------------
# ACTIVITY 2: COLOR SENSOR
# ----------------------------------------------------------------

def handleColorCommand():
    """
    Check E-Stop, then send a COMMAND_COLOR packet to the Arduino.
    The Arduino will reply with a RESP_COLOR packet containing
    R, G, B channel frequencies in Hz in params[0..2].
    printPacket() handles displaying the response when it arrives.
    """
    if isEstopActive():
        print("Refused: E-Stop is active")
        return

    print("Sending color command...")
    sendCommand(COMMAND_COLOR)


# ----------------------------------------------------------------
# ACTIVITY 3: CAMERA
# ----------------------------------------------------------------

_camera          = None
_frames_remaining = 5


def openCamera():
    """Open the Raspberry Pi camera at startup."""
    global _camera
    _camera = cameraOpen()


def closeCamera():
    """Close the Raspberry Pi camera on exit."""
    global _camera
    if _camera is not None:
        cameraClose(_camera)
        _camera = None


def handleCameraCommand():
    """
    Capture and display one greyscale frame from the Pi camera.

    Order of checks (handout requirement):
      1. E-Stop gate  — refuse if system is stopped.
      2. Frames gate  — refuse if no frames remain.
      3. Capture and render the frame using alex_camera.py (unmodified).
      4. Decrement the counter and display frames remaining.
    """
    global _frames_remaining

    if isEstopActive():
        print("Refused: E-Stop is active")
        return

    if _frames_remaining <= 0:
        print("Refused: no camera frames remaining")
        return

    frame = captureGreyscaleFrame(_camera)
    renderGreyscaleFrame(frame)
    _frames_remaining -= 1
    print(f"Frames remaining: {_frames_remaining}")


# ----------------------------------------------------------------
# ACTIVITY 4: LIDAR
# ----------------------------------------------------------------

def handleLidarCommand():
    """
    Perform a single LIDAR scan and render it in the terminal.

    Uses alex_lidar.py (connect → scan → disconnect) and
    lidar_example_cli_plot.plot_single_scan() for rendering.
    E-Stop gate comes first.

    Note: complete convert_to_cartesian() in lidar_example_cli_plot.py
    before calling this — the plot will not render correctly without it.
    """
    if isEstopActive():
        print("Refused: E-Stop is active")
        return

    print("Starting single LIDAR scan...")

    lidar = alex_lidar.connectLidar()
    try:
        scan = alex_lidar.getSingleScan(lidar)
    finally:
        alex_lidar.disconnectLidar(lidar)

    plot_single_scan(scan)


# ----------------------------------------------------------------
# COMMAND-LINE INTERFACE
# ----------------------------------------------------------------

def handleUserInput(line):
    """
    Dispatch a single line of user input to the correct handler.

      e  -> software E-Stop (pre-wired)
      c  -> color sensor reading        (Activity 2)
      p  -> camera capture              (Activity 3)
      l  -> LIDAR scan                  (Activity 4)
    """
    if line == 'e':
        print("Sending E-Stop command...")
        sendCommand(COMMAND_ESTOP, data=b'This is a debug message')
    elif line == 'c':
        handleColorCommand()
    elif line == 'p':
        handleCameraCommand()
    elif line == 'l':
        handleLidarCommand()
    else:
        print(f"Unknown input: '{line}'. Valid: e, c, p, l")


def runCommandInterface():
    """
    Main command loop.

    Uses select.select() to simultaneously:
      - poll the serial port for incoming Arduino packets, and
      - read typed user commands from stdin,
    without either blocking the other.
    """
    print("Sensor interface ready. Type e / c / p / l and press Enter.")
    print("Press Ctrl+C to exit.\n")

    while True:
        if _ser.in_waiting >= FRAME_SIZE:
            pkt = receiveFrame()
            if pkt:
                printPacket(pkt)

        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        if rlist:
            line = sys.stdin.readline().strip().lower()
            if not line:
                time.sleep(0.05)
                continue
            handleUserInput(line)

        time.sleep(0.05)


# ----------------------------------------------------------------
# MAIN
# ----------------------------------------------------------------

if __name__ == '__main__':
    openSerial()
    openCamera()
    try:
        runCommandInterface()
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        closeCamera()
        closeSerial()

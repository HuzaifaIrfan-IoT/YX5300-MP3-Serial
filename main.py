#!/usr/bin/env python3
"""
YX5300 (CATALEX / Catalex Serial MP3) controller using pyserial.

Install:   pip install pyserial
Run REPL:  python yx5300.py COM3            (Windows)
           python yx5300.py /dev/ttyUSB0    (Linux/macOS)
One-shot:  python yx5300.py COM3 --play 1 --volume 20

Wiring (USB-TTL adapter <-> YX5300):
    Adapter TX -> Module RX
    Adapter RX -> Module TX
    GND -> GND
    VCC -> 5V (module is happiest at 5V; logic is 3.3V/5V tolerant on most boards)

Protocol: 9600 baud, 8N1.
Frame:  7E FF 06 CMD FB PARAM_H PARAM_L EF     (no checksum on YX5300)
"""

import argparse
import sys
import threading
import time

import serial

# ---- Command bytes ---------------------------------------------------------
CMD_NEXT = 0x01
CMD_PREV = 0x02
CMD_PLAY_INDEX = 0x03        # play track by index (1..255)
CMD_VOL_UP = 0x04
CMD_VOL_DOWN = 0x05
CMD_SET_VOL = 0x06           # 0..30
CMD_SET_EQ = 0x07            # 0 normal,1 pop,2 rock,3 jazz,4 classic,5 bass
CMD_LOOP_TRACK = 0x08        # loop a single track by index
CMD_SELECT_DEV = 0x09        # 1 = USB, 2 = TF/SD
CMD_SLEEP = 0x0A
CMD_WAKE = 0x0B
CMD_RESET = 0x0C
CMD_PLAY = 0x0D
CMD_PAUSE = 0x0E
CMD_PLAY_FOLDER = 0x0F       # high byte = folder (01..99), low byte = file (001..255)
CMD_LOOP_ALL = 0x11          # 1 = start repeat-all, 0 = stop
CMD_STOP = 0x16
CMD_LOOP_FOLDER = 0x17       # param low = folder number
CMD_RANDOM = 0x18
CMD_LOOP_CURRENT = 0x19      # 0 = enable, 1 = disable

# ---- Query commands --------------------------------------------------------
Q_STATUS = 0x42
Q_VOLUME = 0x43
Q_EQ = 0x44
Q_TOTAL_USB = 0x47
Q_TOTAL_TF = 0x48
Q_CURRENT_TF = 0x4C
Q_FOLDER_FILES = 0x4E
Q_FOLDER_COUNT = 0x4F

# ---- Names for messages coming FROM the module -----------------------------
RESPONSE_NAMES = {
    0x3A: "Storage inserted",
    0x3B: "Storage removed",
    0x3C: "USB track finished",
    0x3D: "SD/TF track finished",
    0x3E: "Flash track finished",
    0x3F: "Init complete (online devices bitmask)",
    0x40: "Error",
    0x41: "ACK",
    0x42: "Status",
    0x43: "Volume",
    0x44: "EQ",
    0x47: "Total files on USB",
    0x48: "Total files on SD/TF",
    0x4B: "Current track (USB)",
    0x4C: "Current track (SD/TF)",
    0x4E: "Files in folder",
    0x4F: "Folder count",
}

ERROR_CODES = {
    1: "Module busy / not initialised",
    2: "Module asleep",
    3: "Serial receive error",
    4: "Checksum error",
    5: "File index out of range",
    6: "Track not found",
    7: "Insertion error",
    8: "SD card error",
    9: "Entered sleep",
}

START, VER, LEN, END = 0x7E, 0xFF, 0x06, 0xEF


class YX5300:
    def __init__(self, port, baud=9600, on_message=None, feedback=True):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.feedback = 1 if feedback else 0
        self.on_message = on_message or self._default_handler
        self._stop_evt = threading.Event()
        self._lock = threading.Lock()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    # ---- low level ---------------------------------------------------------
    def send(self, cmd, param=0):
        """Send one 8-byte frame. `param` is a 16-bit value."""
        hi, lo = (param >> 8) & 0xFF, param & 0xFF
        frame = bytes([START, VER, LEN, cmd, self.feedback, hi, lo, END])
        with self._lock:
            self.ser.write(frame)
            self.ser.flush()
        time.sleep(0.05)  # module needs a little gap between commands
        return frame

    def close(self):
        self._stop_evt.set()
        self._reader.join(timeout=1)
        self.ser.close()

    def _read_loop(self):
        buf = bytearray()
        while not self._stop_evt.is_set():
            try:
                chunk = self.ser.read(32)
            except serial.SerialException:
                break
            if not chunk:
                continue
            buf.extend(chunk)
            while True:
                try:
                    i = buf.index(START)
                except ValueError:
                    buf.clear()
                    break
                del buf[:i]
                if len(buf) < 8:
                    break
                if buf[7] == END:
                    self.on_message(buf[3], buf[4], (buf[5] << 8) | buf[6])
                    del buf[:8]
                else:
                    del buf[0]  # bad frame, resync

    @staticmethod
    def _default_handler(cmd, fb, param):
        name = RESPONSE_NAMES.get(cmd, f"0x{cmd:02X}")
        extra = ""
        if cmd == 0x40:
            extra = f" ({ERROR_CODES.get(param, 'unknown')})"
        print(f"\n<- {name}: {param}{extra}")

    # ---- high level --------------------------------------------------------
    def reset(self):
        self.send(CMD_RESET)
        time.sleep(1.5)  # allow init + SD card scan

    def select_sd(self):
        self.send(CMD_SELECT_DEV, 2)
        time.sleep(0.3)

    def select_usb(self):
        self.send(CMD_SELECT_DEV, 1)
        time.sleep(0.3)

    def play(self):
        self.send(CMD_PLAY)

    def pause(self):
        self.send(CMD_PAUSE)

    def stop(self):
        self.send(CMD_STOP)

    def next(self):
        self.send(CMD_NEXT)

    def prev(self):
        self.send(CMD_PREV)

    def play_track(self, n):
        self.send(CMD_PLAY_INDEX, max(1, min(255, n)))

    def play_folder(self, folder, file):
        self.send(CMD_PLAY_FOLDER, (folder << 8) | file)

    def loop_track(self, n):
        self.send(CMD_LOOP_TRACK, n)

    def loop_all(self, on=True):
        self.send(CMD_LOOP_ALL, 1 if on else 0)

    def loop_folder(self, folder):
        self.send(CMD_LOOP_FOLDER, folder)

    def random(self):
        self.send(CMD_RANDOM)

    def volume(self, v):
        self.send(CMD_SET_VOL, max(0, min(30, v)))

    def volume_up(self):
        self.send(CMD_VOL_UP)

    def volume_down(self):
        self.send(CMD_VOL_DOWN)

    def eq(self, mode):
        self.send(CMD_SET_EQ, max(0, min(5, mode)))

    def sleep(self):
        self.send(CMD_SLEEP)

    def wake(self):
        self.send(CMD_WAKE)

    def query(self, q, param=0):
        self.send(q, param)


# ---- Interactive shell -----------------------------------------------------
HELP = """
Commands:
  play | pause | stop | next | prev
  track N            play track N (1-255)
  folder F N         play file N from folder F
  loop N             loop track N
  loopall on|off     repeat all tracks
  loopfolder F       loop folder F
  random             random play
  vol N              set volume 0-30
  vol+ | vol-        volume up/down
  eq N               0 normal,1 pop,2 rock,3 jazz,4 classic,5 bass
  sd | usb           select storage device
  sleep | wake | reset
  status | getvol | geteq | total | current | folders
  help | quit
"""


def repl(dev):
    print(HELP)
    while True:
        try:
            line = input("yx5300> ").strip().lower().split()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        c, a = line[0], line[1:]
        try:
            if c in ("quit", "exit", "q"):
                break
            elif c == "help":
                print(HELP)
            elif c == "play":
                dev.play()
            elif c == "pause":
                dev.pause()
            elif c == "stop":
                dev.stop()
            elif c == "next":
                dev.next()
            elif c == "prev":
                dev.prev()
            elif c == "track":
                dev.play_track(int(a[0]))
            elif c == "folder":
                dev.play_folder(int(a[0]), int(a[1]))
            elif c == "loop":
                dev.loop_track(int(a[0]))
            elif c == "loopall":
                dev.loop_all(a[0] == "on")
            elif c == "loopfolder":
                dev.loop_folder(int(a[0]))
            elif c == "random":
                dev.random()
            elif c == "vol":
                dev.volume(int(a[0]))
            elif c == "vol+":
                dev.volume_up()
            elif c == "vol-":
                dev.volume_down()
            elif c == "eq":
                dev.eq(int(a[0]))
            elif c == "sd":
                dev.select_sd()
            elif c == "usb":
                dev.select_usb()
            elif c == "sleep":
                dev.sleep()
            elif c == "wake":
                dev.wake()
            elif c == "reset":
                dev.reset()
            elif c == "status":
                dev.query(Q_STATUS)
            elif c == "getvol":
                dev.query(Q_VOLUME)
            elif c == "geteq":
                dev.query(Q_EQ)
            elif c == "total":
                dev.query(Q_TOTAL_TF)
            elif c == "current":
                dev.query(Q_CURRENT_TF)
            elif c == "folders":
                dev.query(Q_FOLDER_COUNT)
            else:
                print("Unknown command. Type 'help'.")
        except (IndexError, ValueError):
            print("Missing or invalid argument. Type 'help'.")
        time.sleep(0.2)  # let any reply print before the next prompt


def main():
    p = argparse.ArgumentParser(description="YX5300 MP3 module controller")
    p.add_argument("port", help="Serial port, e.g. COM3 or /dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=9600)
    p.add_argument("--no-reset", action="store_true", help="skip reset on connect")
    p.add_argument("--usb", action="store_true", help="use USB storage instead of SD")
    p.add_argument("--volume", type=int, help="set volume 0-30")
    p.add_argument("--play", type=int, metavar="N", help="play track N then exit")
    args = p.parse_args()

    try:
        dev = YX5300(args.port, args.baud)
    except serial.SerialException as e:
        sys.exit(f"Cannot open {args.port}: {e}")

    try:
        if not args.no_reset:
            dev.reset()
        dev.select_usb() if args.usb else dev.select_sd()
        if args.volume is not None:
            dev.volume(args.volume)
        if args.play is not None:
            dev.play_track(args.play)
            time.sleep(0.5)
        else:
            repl(dev)
    finally:
        dev.close()


if __name__ == "__main__":
    main()
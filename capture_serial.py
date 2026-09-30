#!/usr/bin/env python3
"""
Liest die serielle Ausgabe des ESP32-Frequenzmessers (src/main.cpp) mit
und schreibt jede Zeile in eine Textdatei - zusammen mit einem lokalen
PC-Zeitstempel (unabhaengig vom Timecode, den der ESP32 selbst pro Zeile
mitschickt).

Verwendung:
    python3 scripts/capture_serial.py
    python3 scripts/capture_serial.py --port /dev/cu.usbserial-110 --baud 115200
    python3 scripts/capture_serial.py --out messung.txt

Beenden mit CTRL+C. Die Datei wird laufend geschrieben (nicht erst am Ende).
"""

import argparse
import datetime
import sys

import serial

DEFAULT_PORT = "/dev/cu.usbserial-110"
DEFAULT_BAUD = 115200


def main() -> int:
    parser = argparse.ArgumentParser(description="ESP32-Serial-Ausgabe in eine Textdatei mitschneiden.")
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"Serieller Port (Standard: {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"Baudrate (Standard: {DEFAULT_BAUD})")
    parser.add_argument(
        "--out",
        default=None,
        help="Ausgabedatei (Standard: capture_<Datum>_<Uhrzeit>.txt im gleichen Ordner)",
    )
    args = parser.parse_args()

    out_path = args.out
    if out_path is None:
        stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        out_path = f"capture_{stamp}.txt"

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as exc:
        print(f"Fehler: Serieller Port {args.port} konnte nicht geoeffnet werden: {exc}")
        return 1

    print(f"Verbunden mit {args.port} @ {args.baud} Baud")
    print(f"Schreibe nach: {out_path}")
    print("Beenden mit CTRL+C")
    print()

    line_count = 0
    try:
        with open(out_path, "a", encoding="utf-8") as out_file:
            while True:
                raw_line = ser.readline()
                if not raw_line:
                    continue

                text = raw_line.decode(errors="replace").rstrip("\r\n")
                if not text:
                    continue

                pc_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                out_file.write(f"{pc_time}  {text}\n")
                out_file.flush()

                line_count += 1
                print(text)
    except KeyboardInterrupt:
        print()
        print(f"Beendet. {line_count} Zeilen nach {out_path} geschrieben.")
    finally:
        ser.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())

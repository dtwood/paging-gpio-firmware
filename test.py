#!/usr/bin/env python3

import datetime
import socket


HOST = "10.5.2.1"  # Standard loopback interface address (localhost)
PORT = 49280  # Port to listen on (non-privileged ports are > 1023)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    print("bound")
    s.listen()
    print("listening")
    while True:
        conn, addr = s.accept()
        with conn:
            print("Connected by", addr)
            buf = b""
            old_rx_time = None
            while True:
                try:
                    data = conn.recv(1024)
                except OSError:
                    break
                if not data:
                    break

                rx_time = datetime.datetime.now()
                if old_rx_time is not None:
                    delta = rx_time - old_rx_time
                else:
                    delta = None
                old_rx_time = rx_time
                buf += data
                (*frames, buf) = buf.split(b"\r")
                for frame in frames:
                    conn.send(b"ACK " + frame + b"\r")
                    print(f"[{rx_time.isoformat(timespec='milliseconds')} ({delta})]: {frame}")
        print("disconnected")

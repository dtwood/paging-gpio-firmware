#!/usr/bin/env python3

import datetime
import socket


HOST = "localhost"
PORT = 49281

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    print("bound")
    s.listen()
    print(f"listening on {HOST}:{PORT}")
    while True:
        conn, addr = s.accept()
        with conn:
            print("Connected by", addr)
            buf = b""
            old_rx_time = None
            old_time = 0
            times = []
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
                    if (
                        not frame
                        and delta is not None
                        and delta > datetime.timedelta(seconds=0.9)
                    ):
                        times.append(delta.total_seconds())
                        print(sum(times) / len(times))
                    # conn.send(b"ACK " + frame + b"\r")
                    print("[{} ({})]: {}".format(rx_time.isoformat(), delta, frame))
        print("disconnected")

#!/usr/bin/env python3

import datetime
import socket


HOST = "2001:db8::1"
PORT = 49280

with socket.socket(socket.AF_INET6, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT, 0, 0))
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
                    print(
                        "[{} ({})]: {}".format(
                            rx_time.isoformat(), delta, frame
                        )
                    )
        print("disconnected")


import socket
import time

HOST = '172.20.10.3'  # Standard loopback interface address (localhost)
PORT = 65431      # Port to listen on (non-privileged ports are > 1023)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    with conn:
        print('Connected by', addr)
        conn.sendall(b"125,561,511,5518,51212,848,52121,51651")
        time.sleep(1)
        # conn.send(b"FUCKER")
        # time.sleep(1)
        # conn.send(b"FULLER")

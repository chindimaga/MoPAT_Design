
import socket
import time

HOST = '192.168.43.135'  # Standard loopback interface address (localhost)
PORT = 65431      # Port to listen on (non-privileged ports are > 1023)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    with conn:
        print('Connected by', addr)
        conn.sendall(b"I MAKE ELEVATING MUSIC!")
        time.sleep(1)

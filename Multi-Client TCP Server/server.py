"""
This program is made to create a TCP Server \
Server will assign a unique ID to each of its client, \
Thread-based parallelism is used to handle multiple clients \
customize data for each client and send them selectively.  
"""

import socket 
import threading 
global clientID 
global parameters

global clientOFF    #set this cclientOFF=1 to close all clients connection and then server 
clientOFF = 0		#control it from those programs who are sending data for clients  


parameters = [[]]		# parameters contains data for clients

parameters[0] = [0,0,0]	# parameters[0] is a default data that server will send to those clients \
						# whose data is not provised by sensors ( this situation can arise when client(robot) is \ 
						# connected to server but not visible to camera to take its coordinate (x,y,theta) )



#parameters for Client 1
parameters.append([10,11,12]) #["X", "Y", "Theta"]
#parameters for Client 2
parameters.append([20,21,22]) #["X", "Y", "Theta"]
#parameters for Client 3
parameters.append([30,31,32]) #["X", "Y", "Theta"]
#parameters for Client 4
parameters.append([40,41,42]) #["X", "Y", "Theta"]

clientID = 0

def create_socket():
	try:
		global host
		global port 
		global s

		host = "127.0.0.1"
		port = 9999
		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
		print ("Socket successfully created")

	except socket.error as msg :
		print ("Socket Creation Error " + str(msg) )

def bind_socket():

	try:
		global host
		global port
		global s

		print("Binding the port " + str(port))
		s.bind((host,port))
		s.listen(5)
		print("Server is listening ...")

	except socket.error as msg:
		print("Socket Binding Error " + str(msg) + "\n" + "Retrying...")
		bind_socket()

def socket_accept():
	conn, address = s.accept()
	print("Connection has been established! |" + "IP " + address[0] + " | port " + str(address[1]))
	global clientID
	clientID+=1

	t = threading.Thread(target=socket_accept)
	t.daemon = True
	t.start()


	send_commands(conn, clientID)
	conn.close()

def send_commands(conn, id):
	conn.send(str.encode(str(clientID)))
	msg = conn.recv(1024)
	if len(msg)>0:
		msg = msg.decode("utf-8")
		print(msg)
	

	while True:

		if clientOFF==1:
			break
		else: 	

			if id <= len(parameters)-1:

				for x in parameters[id]:

					if len(str.encode(str(x)))>0:
						conn.send(str.encode(str(x)))
						msg = conn.recv(1024)
						if len(msg)>0:
							msg = msg.decode("utf-8")
							print(msg)
			else:

				for x in parameters[0]:

					if len(str.encode(str(x)))>0:
						conn.send(str.encode(str(x)))
						msg = conn.recv(1024)
						if len(msg)>0:
							msg = msg.decode("utf-8")
							print(msg)


def main():
	create_socket()
	bind_socket()

main()
socket_accept()


t.join()

print("Done")	


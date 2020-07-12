import socket 

try: 
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
    print ("Socket successfully created")
except socket.error as err: 
    print ("socket creation failed with error %s" %(err))

s = socket.socket()
HOST = "127.0.0.1"
PORT = 9999
parameters = [0,0,0]
clientOFF = 0

s.connect((HOST,PORT))

clientID = s.recv(1024)
if len(clientID)>0:
	clientID = clientID.decode("utf-8")
	ack = f"Client receiver : {clientID}"
	s.send(bytes(ack,"utf-8"))
	print(f"Client ID : {clientID}")

while True:
	print(parameters)

	if clientOFF ==1:
		break

	for x in range(3):
		data = s.recv(1024)

		if len(data)>0:
			data = data.decode("utf-8")

			if data == "off":
				clientOFF =1
				break 


			ack = f"Client received : {data}"
			s.send(bytes(ack,"utf-8"))
			parameters[x] = int(data)
			
s.close()        

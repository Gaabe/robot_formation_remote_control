import socket
import time
import random

if __name__ == '__main__':
	# port = int(input("Enter the port number that you would like to connect: "))
	sensor = input("Enter the sensor id to send: ")
	print("Connecting to the localhost at port 5000")
	server_address = ('192.168.0.108', 5000)
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	sock.connect(server_address)
	print("Connected to the server")
	sock.sendall("id{sensor}".format(sensor=sensor).encode('utf-8'))
	while True:
		# msg = str(input("Send your message to the server: "))
		msg = str(random.randint(60, 70))
		sock.sendall(msg.encode('utf-8'))
		data = sock.recv(16)
		print("The server answered: {answer}".format(answer=data))
		time.sleep(2)

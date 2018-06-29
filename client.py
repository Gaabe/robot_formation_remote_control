import socket
import time
import random

if __name__ == '__main__':
	port = int(input("Enter the port number that you would like to connect: "))
	print("Connecting to the localhost at port {port}".format(port=port))
	server_address = ('localhost', port)
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	sock.connect(server_address)
	print("Connected to the server")
	while True:
		# msg = str(input("Send your message to the server: "))
		msg = str(random.randint(0, 60))
		sock.sendall(msg.encode('utf-8'))
		data = sock.recv(16)
		print("The server answered: {answer}".format(answer=data))
		time.sleep(0.5)
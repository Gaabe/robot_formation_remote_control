import socket

if __name__ == '__main__':
	port = int(input("Enter the port number that you would like to connect: "))
	print("Connecting to the localhost at port {port}".format(port=port))
	server_address = ('localhost', port)
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	sock.connect(server_address)
	print("Connected to the server")
	while True:
		msg = str(input("Send your message to the server: "))
		sock.sendall(msg)
		data = sock.recv(16)
		print("The server answered: {answer}".format(answer=data))
from threading import Thread
import socket


 
class ServerThread(Thread):
 
	def __init__(self, port):
		Thread.__init__(self)
		self.port = port
 
 
	def run(self):
		self.open_socket()

	def log_temperatures(self):
		data = self.connection.recv(16)
		print("Received: ", data)
		if data:
			if int(data) > 50:
				print("High temperature detected at sensor {sensor}".format(sensor=self.getName()[-1]))
			self.connection.sendall("Message received")
			return 0
		else:
			print("Connection closed by sensor {sensor}".format(sensor=self.getName()[-1]))
			return -1	

	def open_socket(self):
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		server_address = ('localhost', self.port)
		print("Starting {name} at port {port}".format(name=self.getName(), port=self.port))
		self.socket.bind(server_address)

		# Listen for incoming connections
		self.socket.listen(1)

		while True:
			print("Waiting for incoming connections")
			self.connection, self.client_address = self.socket.accept()
			if self.connection:
				break

		try:
			print("Incoming connection from {client}".format(client=self.client_address))

			while True:
				status = self.log_temperatures()
				if status != 0:
					break

		finally:
			self.connection.close()
	 
 
if __name__ == '__main__':
   temp_monitor_server_1 = ServerThread(4001)
   temp_monitor_server_1.setName('Temperature Monitor Server 1')

   temp_monitor_server_2 = ServerThread(4002)
   temp_monitor_server_2.setName('Temperature Monitor Server 2')

   temp_monitor_server_3 = ServerThread(4003)
   temp_monitor_server_3.setName('Temperature Monitor Server 3')

   temp_monitor_server_4 = ServerThread(4004)
   temp_monitor_server_4.setName('Temperature Monitor Server 4')

   temp_monitor_server_1.start()
   temp_monitor_server_2.start()
   temp_monitor_server_3.start()
   temp_monitor_server_4.start()
 
   temp_monitor_server_1.join()
   temp_monitor_server_2.join()
   temp_monitor_server_3.join()
   temp_monitor_server_4.join()
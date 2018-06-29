from threading import Thread
import socket
from collections import deque
import time
import matplotlib.pyplot as plt

sensor_1_temps = deque(30*[0], 30)
sensor_2_temps = deque(30*[0], 30)
sensor_3_temps = deque(30*[0], 30)
sensor_4_temps = deque(30*[0], 30)

sensor_temps = {
"Temperature Monitor Server 1" : sensor_1_temps,
"Temperature Monitor Server 2" : sensor_2_temps,
"Temperature Monitor Server 3" : sensor_3_temps,
"Temperature Monitor Server 4" : sensor_4_temps,}

class TempMonitorThread(Thread):

	def __init__(self, socket, client_address):
		Thread.__init__(self)
		self.client_address = client_address
		self.daemon = True
		self.socket = socket

	def run(self):
		print("Socket thread criado")

 
class ServerThread(Thread):
 
	def __init__(self, port):
		Thread.__init__(self)
		self.port = port
		self.daemon = True
 
 
	def run(self):
		self.open_socket()

	def register_temp(self, temp):
		global sensor_temps
		sensor_temps[self.getName()].append(temp)
		print(sensor_temps)

	def log_temperatures(self):
		data = self.connection.recv(16)
		print("Received: ", data)
		if data:
			self.register_temp(data)
			if float(data) > 50.0:
				print("High temperature detected at sensor {sensor}".format(sensor=self.getName()[-1]))
			self.connection.sendall("Message received")
			return 0
		else:
			print("Connection closed by sensor {sensor}".format(sensor=self.getName()[-1]))
			return -1   

	def open_socket(self):
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		server_address = ('0.0.0.0', self.port)
		print("Starting server at port {port}".format(port=self.port))
		self.socket.bind(server_address)

		# Listen for incoming connections
		self.socket.listen(1)

		while True:
			print("Waiting for incoming connections")
			self.connection, self.client_address = self.socket.accept()
			if self.connection:
				print(self.connection, self.client_address)
				temp_monitor = TempMonitorThread(self.connection, self.client_address)
				temp_monitor.start()
	 
 
if __name__ == '__main__':
	temp_monitor_server = ServerThread(5500)
	temp_monitor_server.setName('Temperature Monitor Server')


	temp_monitor_server.start()


	# plt.plot(range(len(sensor_temps["Temperature Monitor Server 1"])), list(sensor_temps["Temperature Monitor Server 1"]))
	# plt.show()
	# while True:
	# 	time.sleep(3)
	# 	print("plotting graph")
	# 	print(range(len(sensor_temps["Temperature Monitor Server 1"])), list(sensor_temps["Temperature Monitor Server 1"]))
	# 	plt.plot(range(len(sensor_temps["Temperature Monitor Server 1"])), list(sensor_temps["Temperature Monitor Server 1"]))
	# 	plt.draw()

	temp_monitor_server.join()




	
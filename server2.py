# Socket server in python using select function
 
import socket, select
import traceback
from collections import deque
import matplotlib.pyplot as plt

    
CONNECTION_LIST = []
TEMP_MONITORS = []
RECV_BUFFER = 4096
PORT = 4001


def get_temp_monitor(monitors, socket):
    for monitor in monitors:
        if monitor.socket == socket:
            return monitor

def assign_new_monitor_number(monitors):
    if not monitors:
        return 1
    else:
        number = 1
        for monitor in monitors:
            if monitor.number >= number:
                number = monitor.number
        return number + 1



class TempMonitor():

    def __init__(self, socket):
        self.temps = deque(30*[0], 30)
        self.socket = socket

    def log_temperature(self, temp):
        self.temps.append(float(temp))

    def update_line(self):
        plt.clf()
        for monitor in TEMP_MONITORS:
            plt.plot(range(len(monitor.temps)), monitor.temps)
            plt.pause(0.05)

  
if __name__ == "__main__":
         
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('0.0.0.0', PORT))
    server_socket.listen(10)
 
    CONNECTION_LIST.append(server_socket)
 
    print("Server started on port " + str(PORT))
 
    while 1:
        read_sockets,write_sockets,error_sockets = select.select(CONNECTION_LIST,[],[])
 
        for sock in read_sockets:
             
            if sock == server_socket:
                sockfd, addr = server_socket.accept()
                CONNECTION_LIST.append(sockfd)
                monitor = TempMonitor(sockfd)
                monitor.number = assign_new_monitor_number(TEMP_MONITORS)
                TEMP_MONITORS.append(monitor)
                print("Client (%s, %s) connected" % addr)
                 
            else:
                try:
                    data = sock.recv(RECV_BUFFER)
                    monitor = get_temp_monitor(TEMP_MONITORS, sock)
                    monitor.log_temperature(data)
                    monitor.update_line()
                    print("Received {data} from monitor {monitor}".format(data=data, monitor=monitor.number))
                    if data:
                        sock.send('OK ... '.encode('utf-8'))
                 
                except Exception as e:
                    monitor = get_temp_monitor(TEMP_MONITORS, sock)
                    TEMP_MONITORS.remove(monitor)
                    traceback.print_exc()
                    print("Client (%s, %s) is offline" % addr)
                    sock.close()
                    CONNECTION_LIST.remove(sock)
                    continue
         
    server_socket.close()
import socket, select
import traceback
from collections import deque
import matplotlib.pyplot as plt
from threading import Thread
import time
import serial
from serial.serialutil import SerialException
import seaborn as sns

    
CONNECTION_LIST = []
TEMP_MONITORS = []
RECV_BUFFER = 4096
PORT = 5000
GRAPH_WAIT_TIME = 1
UPDATE_TIME_WAIT_TIME = 10
USB_PORT = "/dev/ttyUSB0"
MAX_TEMP = 45
HEAT_MAP = [[26,26,26,26,26,26,26,26,26],
            [26,26,26,26,26,26,26,26,26],
            [26,26,26,26,26,26,26,26,26],
            [26,26,26,26,26,26,26,26,26],
            [26,26,26,26,26,26,26,26,26],
            [26,26,26,26,26,26,26,26,26],
            [26,26,26,26,26,26,26,26,26],
            [26,26,26,26,26,26,26,26,26],
            [26,26,26,26,26,26,26,26,26]]
fig = plt.figure()

sensor0_pos=(0, 4)
sensor1_pos=(4, 8)
sensor2_pos=(8, 4)
sensor3_pos=(4, 0)

sensors_pos = (sensor0_pos, sensor1_pos, sensor2_pos, sensor3_pos)
TEMP_CONST = 0.002

def pretty_print(matrix):
    s = [[str(e) for e in row] for row in matrix]
    lens = [max(map(len, col)) for col in zip(*s)]
    fmt = '\t'.join('{{:{}}}'.format(x) for x in lens)
    table = [fmt.format(*row) for row in s]
    print('\n'.join(table))

def interpolate_heatmap():
    global HEAT_MAP
    for s in sensors_pos:
        if HEAT_MAP[s[0]][s[1]] != 26:
            for i in range(-4, 5):
                for j in range(-4, 5):
                    try:
                        pos_x = s[0] + i
                        pos_y = s[1] + j
                        if pos_x >= 0 and pos_y >= 0 and (i != 0 or j != 0):
                            HEAT_MAP[pos_x][pos_y] += round(( (HEAT_MAP[s[0]][s[1]] - HEAT_MAP[pos_x][pos_y]) * ( 9 - ( abs(i) + abs(j) ) ) * TEMP_CONST), 2)
                    except IndexError:
                        pass


def update_heatmap():
    global HEAT_MAP
    for monitor in TEMP_MONITORS:
        if monitor.number == "id0":
            HEAT_MAP[0][4] = monitor.temps[29]
        elif monitor.number == "id1":
            HEAT_MAP[4][8] = monitor.temps[29]
        elif monitor.number == "id2":
            HEAT_MAP[8][4] = monitor.temps[29]
        elif monitor.number == "id3":
            HEAT_MAP[4][0] = monitor.temps[29]
    # print("antes:")
    # pretty_print(HEAT_MAP)
    interpolate_heatmap()
    # print("depois:")
    # pretty_print(HEAT_MAP)


def update_graph():
    plt.clf()
    ax1 = fig.add_subplot(211)
    for monitor in TEMP_MONITORS:
        plt.plot(range(len(monitor.temps)), monitor.temps, label=monitor.number)
    leg = plt.legend(loc="upper right", shadow=True, fancybox=True)
    leg.get_frame().set_alpha(0.5)
    plt.title("Monitor de temperatura")
    plt.ylim(0, 150)

    ax2 = fig.add_subplot(212)
    update_heatmap()
    ax = sns.heatmap(HEAT_MAP, vmin=20, vmax=200, square=True)
    plt.pause(0.05)


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

def update_time_on_sensors():
    print("updating time")
    for temp in TEMP_MONITORS:
        temp.socket.send(time.strftime('%Y%m%d%H%M%S', time.localtime()).encode('utf-8'))



class TempMonitor():

    def __init__(self, socket):
        self.temps = deque(30*[0], 30)
        self.socket = socket
        self.number = ""

    def log_temperature(self, temp):
        self.temps.append(float(temp.decode("utf-8")[:5]))

    def check_and_notify(self, serial, temp):
        if float(temp) > MAX_TEMP:
            print("warning robots")
            if serial:
                serial.write(self.number.encode("utf-8"))


  
if __name__ == "__main__":

    try:
        serial = serial.Serial(USB_PORT, 57600)

        if not serial.isOpen():
            serial.open()
    except SerialException:
        serial = None
        print("Serial não conectada")
         
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('0.0.0.0', PORT))
    server_socket.listen(10)
 
    CONNECTION_LIST.append(server_socket)
 
    print("Server started on port " + str(PORT))

    last_time_graph = time.time()
    last_time_update_time = time.time()
 
    while 1:

        if time.time() - last_time_graph >= GRAPH_WAIT_TIME:
            update_graph()
            last_time_graph = time.time()

        if time.time() - last_time_update_time >= UPDATE_TIME_WAIT_TIME:
            update_time_on_sensors()
            last_time_update_time = time.time()

        read_sockets,write_sockets,error_sockets = select.select(CONNECTION_LIST,[],[])
 
        for sock in read_sockets:
             
            if sock == server_socket:
                sockfd, addr = server_socket.accept()
                CONNECTION_LIST.append(sockfd)
                monitor = TempMonitor(sockfd)
                TEMP_MONITORS.append(monitor)
                print("Client (%s, %s) connected" % addr)
                 
            else:
                try:
                    data = sock.recv(RECV_BUFFER)
                    monitor = get_temp_monitor(TEMP_MONITORS, sock)
                    if not monitor.number:
                        monitor.number = data.decode("utf-8")[:3]
                        sock.send('OK1'.encode('utf-8'))
                    else:
                        monitor.log_temperature(data)
                        monitor.check_and_notify(serial, data)
                        print("Received {data} from monitor {monitor}".format(data=data, monitor=monitor.number))
                        if data:
                            sock.send('OK'.encode('utf-8'))
                 
                except Exception as e:
                    monitor = get_temp_monitor(TEMP_MONITORS, sock)
                    TEMP_MONITORS.remove(monitor)
                    traceback.print_exc()
                    print("Client (%s, %s) is offline" % addr)
                    sock.close()
                    CONNECTION_LIST.remove(sock)
                    continue
         
    server_socket.close()

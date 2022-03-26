#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import socket, signal, sys
import threading

class SocketManager:
    def __init__(self, port):
        self.running = True
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind(('127.0.0.1', port))
        self.sock.listen(5)
        self.sock.settimeout(1)
        self.connected = False

        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def shutdown(self):
        self.running = False
        self.sock.close()
        self.thread.join()

    def run(self):
        while not self.connected and self.running:
            try:
                conn, addr = self.sock.accept()
                self.connected = True
            except:
                pass
        while self.running:
            try:
                data = conn.recv(10)
            except:
                pass
            if data:

                msg = Float64()
                msg.data(data)
                pub.publish(msg)

def shutdown(sig, frame):
    global sock

    print('please')
    sock.shutdown()
    rospy.signal_shutdown('now')

if __name__ == '__main__':
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    sock = SocketManager(int(sys.argv[1]))

    rospy.init_node('depth_setpoint', disable_signals=True)

    pub = rospy.Publisher('/depth_control/setpoint', Float64, queue_size=10)

    print('ready')

    rospy.spin()

# coding=utf-8
import socket
from threading import Thread
import sys
import pickle
import re


class Client_Server():
    def __init__(self):
        self.rigidbodies = []

    def create_new_client_by_server(self, conn, addr):
        # print("print from thread")

        while True:
            to_send_rigidbodies = self.rigidbodies
            conn.sendall(pickle.dumps(to_send_rigidbodies))
            recv = conn.recv(16)
            if pickle.loads(recv) == "got it":
                print("client got them")
            elif pickle.loads(recv) == "got er":
                print("client got error")
                # print(self.rigidbodies)

    def as_client(self):

        RB = []
        rigidbodies = []
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect(('192.168.3.43', 1234))
        except socket.error as msg:
            print(msg)
            sys.exit(1)

        while True:
            buffer = s.recv(325)
            # print("I receive from motive")
            if len(buffer) == 325:
                buffer = bytes.decode(buffer)
                # print("buffer:", buffer)
                string_sep_rb = re.split(r'\*+', buffer)
                string_sep_rb = list(filter(None, string_sep_rb))
                # print(string_sep_rb[0])

                if string_sep_rb[0] == "1234":
                    del string_sep_rb[0]
                    # print("string_sep_rb:", string_sep_rb)
                    rigidbodies.clear()
                    RB.clear()
                    RB = [[-1, -1, -1, -1] for i in range(len(string_sep_rb))]
                    # print(RB)
                    for i in range(0, len(string_sep_rb)):
                        RB[i] = re.split(r'\#+', string_sep_rb[i])
                        RB[i]=list(filter(None, RB[i]))
                        RB[i] = list(map(int, RB[i]))
                    if RB[-1] == [0]:
                        del RB[-1]

                    # print(len(RB))
                    rigidbody = {"ID": 0, "ax": -1, "ay": -1, "yaw": -1}
                    rigidbodies = [rigidbody for i in range(len(RB))]
                    # print(RB)
                    for i in range(len(RB)):
                        rigidbody = {"ID": 0, "ax": 0, "ay": 0, "yaw": 0}
                        rigidbody["ID"] = RB[i][0]
                        rigidbody["ax"] = RB[i][1]
                        rigidbody["ay"] = RB[i][2]
                        rigidbody["yaw"] = RB[i][3]
                        rigidbodies[i] = rigidbody
                    self.rigidbodies = rigidbodies

                    print(self.rigidbodies)



if __name__ == '__main__':
    cs = Client_Server()

    """
    客户端内容
    """
    c = Thread(target=cs.as_client)
    c.start()

    """
    服务端内容
    """
    server = socket.socket(s-ocket.AF_INET, socket.SOCK_STREAM)
    server.bind(('192.168.3.39', 10000))
    server.listen(12)  # 操作系统可以挂起的最大连接数量

    while True:
        conn, addr = server.accept()
        print(conn, addr)
        m = Thread(target=cs.create_new_client_by_server, args=(conn, addr))
        m.start()

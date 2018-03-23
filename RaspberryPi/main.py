from collections import deque
import datetime
import serial
import socket
import sys
import time
from Crypto import Random
from Crypto.Cipher import AES
import base64
import checksum as cs
import predict as predict

useServer = 1
collect_test_data = 0
testing_samples = 1

def readlineCR(port): 
    rv="" 
    while True: 
        ch=port.read() 
        rv+=ch 
        if ch=='\r': 
            return rv

class Data():
    def __init__(self, socket):
        self.bs = 32
        self.secret_key = "1234512345123451"
        self.voltage = 0 
        self.current = 0 
        self.power = 0  #voltage * current
        self.cumpower=0
        self.sock = socket
        self.sample_queue = deque([], 20)

    def pad(self, msg):
        return msg + (self.bs - len(msg)%self.bs)*chr(self.bs - len(msg)%self.bs)

    def encryptText(self, msg, secret_key):
        raw = self.pad(msg)
        iv = Random.new().read(AES.block_size)
        cipher = AES.new(secret_key,AES.MODE_CBC,iv)
        return base64.b64encode(iv + cipher.encrypt(raw))

    def sendData(self, predictedMove):
        formattedAnswer = ("#"+ str(predictedMove) + "|" + str(self.voltage)\
            + "|" + str(self.current) + "|" + str(self.power) + "|" + str(self.cumpower) + "|")
        print(formattedAnswer)
        encryptedText = self.encryptText(formattedAnswer, self.secret_key)
        self.sock.send(encryptedText)

class RaspberryPi():
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.test_counter = 0
        self.isHandshakeDone = False

    def connectToServer(self):
        IPAddress = sys.argv[1]
        Port = int(sys.argv[2])
        server_address = (IPAddress, Port)
        self.sock.connect(server_address)

    def connectToArduino(self):
        #self.serial_port= serial.Serial('/dev/ttyAMA0', baudrate=9600, timeout=3.0) #For linux
        self.serial_port=serial.Serial("/dev/serial0", baudrate=115200, timeout=0) #For the Rpi
        print("Port Open!")
        self.serial_port.reset_input_buffer()
        self.serial_port.reset_output_buffer()

    def run(self):
        try:
            #Connections
            if(useServer):
                self.connectToServer()
                print("Connected to test server")
                data = Data(self.sock)

            self.connectToArduino()

            #Handshaking with Arduino
            while(self.isHandshakeDone == False):
                self.serial_port.write('H')
                print("H sent")
                time.sleep(0.5)
                reply = self.serial_port.read(1)
                if(reply == 'B'):
                    self.isHandshakeDone = True
                    self.serial_port.write('F')
                    print("Connected to Arduino")
                    self.serial_port.readline()
                    time.sleep(1)
                else:
                    time.sleep(0.5)
                    
            #if no byte to read
            #if(self.serial_port.in_waiting != 0 or self.serial_port.read() ):
            #   print("Disconnected")

            #Receive data from Arduino(periodically) and save to CSV
            send_flag = True
            while(collect_test_data):
                #instruction = raw_input("Type the next command")
                if (send_flag == True):
                    self.serial_port.write('R')
                    send_flag = False
                time.sleep(0.1)
                if(self.serial_port.inWaiting() > 0):
                    arduino_data = readlineCR(self.serial_port)
                    print(arduino_data)
                    #cs.save_data(arduino_data)
                    send_flag = True
                    cs.calc_checksum(arduino_data)
            
            #Read and Predict the results
            while(testing_samples):
                if (send_flag == True):
                    self.serial_port.write('R')
                    send_flag = False
                time.sleep(0.1)
                if(self.serial_port.inWaiting() > 0):
                    send_flag = True
                    arduino_data = readlineCR(self.serial_port)
                    #print(arduino_data)
                    if(cs.calc_checksum(arduino_data)):
                        last_comma_index = arduino_data.rfind(",");
                        arduino_data = arduino_data[:last_comma_index] #strip out the checksum
                        arduino_data_list = arduino_data.split("\n")[:-1]
                        for item in arduino_data_list:
                            item.replace('\x00', '')
                            data.sample_queue.appendleft(item)
                    
                    if(len(data.sample_queue) == 20):
                        predict.predict_data(data.sample_queue)
                        data.sample_queue.clear()
                                
            # deserialise_data = tds.get_data(arduino_data)
            # print(deserialise_data)
            # self.test_counter += 1
            # line_to_send = "A" + str(self.test_counter)
            # print(line_to_send)
            # self.serial_port.write(line_to_send)
                    
            #test communication with server.
##          while(True):
##              name = raw_input("What is the dance move?")
##              data.voltage = 12
##              data.current = 1
##              data.power = 100
##              data.cumpower
##              data.sendData(name)

        except KeyboardInterrupt:
            sys.exit(1)

if __name__ == '__main__':
    pi = RaspberryPi()
    pi.run()

import serial
import socket
import sys
import time
from Crypto import Random
from Crypto.Cipher import AES
import base64
import testdeserial as tds

def readlineCR(port): 
	rv="" 
	while True: 
		ch=port.read() 
		rv+=ch 
		if ch=='\r' or ch =='': 
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
			#self.connectToServer()
			#print("Connected to test server")
			#data = Data(self.sock)

			self.connectToArduino()

			#Handshaking with Arduino
			while(self.isHandshakeDone == False):
                self.serial_port.write('H')
                print("H sent")
                time.sleep(0.5)
                #print(self.serial_port.in_waiting)
                reply = self.serial_port.read(1)
                #print(reply)
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
			#	print("Disconnected")

			#Receive data from Arduino(periodically)
			while(True):
                instruction = raw_input("Type the next command")
                self.serial_port.write('R')
                time.sleep(1)
				if(self.serial_port.inWaiting() > 0):
					arduino_data = readlineCR(self.serial_port)
					print(arduino_data)
					deserialise_data = tds.get_data(arduino_data)
					print(deserialise_data)
					tds.calc_checksum(deserialise_data)
##					self.test_counter += 1
##					line_to_send = "A" + str(self.test_counter)
##					print(line_to_send)
##					self.serial_port.write(line_to_send)
					
			#test communication with server.
			while(True):
				name = raw_input("What is the dance move?")
				data.voltage = 12
				data.current = 1
				data.power = 100
				data.cumpower
				data.sendData(name)

		except KeyboardInterrupt:
			sys.exit(1)

if __name__ == '__main__':
	pi = RaspberryPi()
	pi.run()

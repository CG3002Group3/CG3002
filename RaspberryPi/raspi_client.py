import serial
import socket
import sys
import time
import random
from Crypto.Cipher import AES
import base64

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
		self.serial_port=serial.Serial("/dev/serial0", baudrate=115200) #For linux
		self.serial_port=serial.Serial("/dev/serial0", baudrate=115200) #For the Rpi

	def connectToServer(self):
		IPAddress = sys.argv[1]
		Port = sys.argv[2]
		server_address = (IPAddress, Port)
		sock.connect(server_address)

	def connectToArduino(self):
		serial_port.reset_input_buffer()
		serial_port.reset_output_buffer()

	def run(self):
		try:
			#Connections
			self.connectToServer()
			print("Connected to test server")
			data = Data(sock)

			self.connectToArduino()

			#Handshaking with Arduino
			while(self.serial_port.in_waiting == 0 or self.serial_port.read() != "Ack"):
				print("connecting..")
				self.serial_port.write("Hello!")
				time.sleep(1)

			#Acknowledge the arduino back
			self.serial_port.write("Ack")
			print("Connected to Arduino")


		except KeyboardInterrupt:
			sys.exit(1)

if __name__ == '__main__':
	pi = RaspberryPi()
	pi.run()
import socket
import sys
import random
from Crypto.Cipher import AES
import base64

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect the socket to the port where the server is listening
server_address = ('localhost', 8888)
print('connecting to %s port %s' % server_address, file=sys.stderr)
sock.connect(server_address)


iv = Random.new().read(AES.block_size)
cipher = AES.new(secret_key,AES.MODE_CBC,iv)
encoded = base64.b64encode(iv + cipher.encrypt(msg))
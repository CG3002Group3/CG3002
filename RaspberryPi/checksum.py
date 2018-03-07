import binascii


# calculate checksum & compare then compare with the received checksum
def cal_check_sum(incoming):
	msg_byte = hex_str_to_btye(incoming)
	check =0
	for i in msg_btye:
		check = add_to_crc(i, check)
	return check
	
def add_to_crc(b, crc):
	b2 = b2
	if(b < 0):
		b2 = b +256
	for i in xrange(8):
		odd = ((b2^crc) & 1) ==1
		crc >>= 1
		b2 >>= 1
		if (odd):
			crc ^= 0x8C #this means crc ^= 140
		return crc

def hex_str_to_btye(msg):
	hex_data = msg.decode("hex")	#returns a bytes() string?
	msg = btyearray(hex_data) 		#bytearray: mutable sequence of integers in the range 0 <= x <256
	return msg
	
#if __name__ == '__main__':

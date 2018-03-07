import checksum
import struct

# what kind of message do i need to generate to send to arduino?
def generate_msg(pac_type, id):
    type_hex = format(pac_type, '02x')
    id_hex = format(id, '02x')
    crc = checksum.calc_check_sum(type_hex + id_hex)
    return [chr(pkt_type), chr(id), chr(crc)]
	
def get_data(bytes_ls):
    data = []
    for i in range(12):
	data_bytes = bytes_ls[i:i+1]
	test_reading = struct.unpack('<b', data_bytes)[0]	#h is short int
	data.append(test_reading)
    return data
	
def checkCRC(bytes_ls):
    hex_values = map(lambda x: format(ord(x), '02x'), bytes_ls[:-1])
    hex_string = "".join(hex_values)  	#join using ""
    crc = crc8.calc_check_sum(hex_string)
    return crc == ord(bytes_ls[-1])

def calc_checksum(deserialise_data):
    checksum_value = 0
    for i in range (1, 11):
        checksum_value ^= deserialise_data[i]
        
    print(checksum_value)
    if(checksum_value == deserialise_data[11]):
        checksum_flag = 1
        print("Tally! checksum = ", deserialise_data[11])


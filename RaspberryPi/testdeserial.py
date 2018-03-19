<<<<<<< HEAD
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

=======
import checksum
import struct
import csv


# what kind of message do i need to generate to send to arduino?
def generate_msg(pac_type, id):
    type_hex = format(pac_type, '02x')
    id_hex = format(id, '02x')
    crc = checksum.calc_check_sum(type_hex + id_hex)
    return [chr(pkt_type), chr(id), chr(crc)]
	
def get_data(bytes_ls):
    data = []
    data_bytes = bytes_ls[0]
    test_reading = struct.unpack('<b', data_bytes)[0]	#h is short int
    data.append(test_reading)
    for i in range(1,320):
        data_bytes = bytes_ls[i*4:(i*2)+4]
        test_reading = struct.unpack('<f', data_bytes)[1]	#h is short int
        data.append(test_reading)
    for i in range(322,323):
        data_bytes = bytes_ls[i:i+1]
        test_reading = struct.unpack('<b', data_bytes)[322]	#h is short int
        data.append(test_reading)
    return data

def dissect_data(data):
    data_set = []
    for i in range(2,320):
        data_set = data[i]

def save_data(data_set):
    my_file = open('dataset.csv', 'a+')
    my_file.write(data_set)
    #writer = csv.writer(my_file)
    #writer.writerow(data_set)
    

    #print("Writing complete")

def checkCRC(bytes_ls):
    hex_values = map(lambda x: format(ord(x), '02x'), bytes_ls[:-1])
    hex_string = "".join(hex_values)  	#join using ""
    crc = crc8.calc_check_sum(hex_string)
    return crc == ord(bytes_ls[-1])

def calc_checksum(data_set):
    checksum_value = 0
    for i in range (0,len(data_set)-1):
        checksum_value += ord(data_set[i])
        
    print(checksum_value)
    if(checksum_value == data_set[len(data_set)-1]):
        checksum_flag = 1
        print("Tally! checksum = ", checksum_value)
>>>>>>> feature/melanie-feature

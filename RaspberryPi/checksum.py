import csv

def save_data(data_set):
    my_file = open('turnclap.csv', 'a+')
    my_file.write(data_set)
    #writer = csv.writer(my_file)
    #writer.writerow(data_set)

def calc_checksum(data_set):
    checksum_value = 0
    value_to_tally = 0
    last_comma_index = data_set.rfind(",");
    value_to_tally = data_set[last_comma_index+1:] #splice the checksum out of the received string
    #print "value to tally is: ", value_to_tally
    for i in range (0, len(data_set[:last_comma_index])):
        checksum_value += ord(data_set[i])

    #print"Calculated checksum is ", checksum_value
    if(int(checksum_value) == int(value_to_tally)):
        print("Tally! checksum = ", checksum_value)
        return True
    else:
        print("Checksum FAILED = ", checksum_value)
        return False
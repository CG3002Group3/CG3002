#! /usr/bin/env python
import os
import time
import random
import datetime
import telepot
from telepot.loop import MessageLoop

#'\' is used to splite pythone line
LAN_ip = os.popen('ip addr show enxb827eb2299d2 | grep "\<inet\>" | awk \'{ print $2 }\' | awk -F "/" \'{ print $1 }\'').read()
ipv4 = os.popen('ip addr show wlan0 | grep "\<inet\>" | awk \'{ print $2 }\' | awk -F "/" \'{ print $1 }\'').read().strip()
ssid = os.popen("iwconfig wlan0 \
                | grep 'ESSID' \
                | awk '{print $4}' \
                | awk -F\\\" '{print $2}'").read()
ip = ("PI2-LAN IP   : %sWIFI SSID: %sWIFI IP  : %s"%(LAN_ip,ssid,ipv4))
#print("ssid: " + ssid)
#print("ip address: " + your_ip)
print (ip)

#time.sleep(10)
##
##def handle(msg):
##    chat_id = msg['chat']['id']
##    command = msg['text']
###    print ('Got command: %s' % command)
##    bot.sendMessage(chat_id, str(chat_id))
##    
##    if command == '/time':
##        bot.sendMessage(chat_id, str(datetime.datetime.now()))
##        
##    elif command == '/ip':
##        bot.sendMessage(chat_id, ip)
        
def handle2(msg):
    chat_id = msg['chat']['id']
    command = msg['text']
    print ('Got command: %s' % command)
#    bot2.sendMessage(chat_id, str(chat_id))

    
    if command == '/time':
        bot2.sendMessage(chat_id, str(datetime.datetime.now()))
        
    elif command == '/ip':
        bot2.sendMessage(chat_id, ip)

while(1):
    if "172" in ipv4:
#        bot2 = telepot.Bot('471057020:AAHEM8QNgeHqYpl9fTJA_zM1oMsO72sOtyk')
#        bot = telepot.Bot('402795367:AAFMQDKGdYfqLouLsJkFy4CSIKgvzu7XxsI')
        bot2 = telepot.Bot('511570036:AAHOCndgOGtMaE4fWA930s2lV9tU53D1wSM')
#        MessageLoop(bot, handle).run_as_thread()
        MessageLoop(bot2, handle2).run_as_thread()
#        MessageLoop(bot3, handle2).run_as_thread()

        chat_id2 = 59475371
#        bot2.sendMessage(375887332, ip)
#        bot.sendMessage(333548211, ip)
        bot2.sendMessage(chat_id=chat_id2, text=ip)
        break
    elif "192" in LAN_ip:
        print("Local")
        break
        
    else:
         print("no wifi connection")
         time.sleep(5)
         LAN_ip = os.popen('ip addr show enxb827eb2299d2 | grep "\<inet\>" | awk \'{ print $2 }\' | awk -F "/" \'{ print $1 }\'').read()
         ipv4 = os.popen('ip addr show wlan0 | grep "\<inet\>" | awk \'{ print $2 }\' | awk -F "/" \'{ print $1 }\'').read().strip()
         ssid = os.popen("iwconfig wlan0 \
                | grep 'ESSID' \
                | awk '{print $4}' \
                | awk -F\\\" '{print $2}'").read()
         ip = ("LAN IP   : %sWIFI SSID: %sWIFI IP  : %s"%(LAN_ip,ssid,ipv4))

#while 1:
#    time.sleep(10)

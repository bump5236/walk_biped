import serial
import time
import csv
import os 
import re


def save(file_name, head, data):
    # normal save
    f = open(file_name,'w')
    csvWriter = csv.writer(f, lineterminator='\n')
    csvWriter.writerows(head)
    csvWriter.writerows(data)
    f.close()
    print("--- Saved ---")

def file_count():
    dir = os.getcwd()
    files = os.listdir(dir)
    count = 0
    for file in files:
        index = re.search('.csv', file)
        if index:
            count = count +1
    return count

ser =serial.Serial("COM5", 115200)
time.sleep(2)

for _ in range(20):
    c = ser.readline()

org_data = []
save_data = []
val = []
timer = [0, 0, 0]

try:
    timer[0] = time.time()
    print("--- Start ---")

    while True:
        timer[1] = time.time()
        c = ser.readline()
        c = c.strip().decode('utf-8')
        org_data.append(c.split(","))

        if len(org_data[-1]) == 14:
            val = org_data[-1]
            save_data.append(val)

            print(save_data[-1])
        
        elif org_data[-1] == ["--- Catching Error ---"]:
            break
        
except KeyboardInterrupt:
    print("--- Ctrl+C ---")


ser.close()
header = [["time [ms]",
                "sync",
                "R_heel",
                "R_toe",
                "R_phase",
                "R_tgt_cur",
                "R_cur",
                "R_pos",
                "L_heel",
                "L_toe",
                "L_phase",
                "L_tgt_cur",
                "L_cur",
                "L_pos"]]

num = file_count()
save("output" + str(num) + ".csv", header, save_data)
print("--- Fin ---")

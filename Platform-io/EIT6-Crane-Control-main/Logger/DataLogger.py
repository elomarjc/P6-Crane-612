import serial
import time

logTime = time.time() #Generates timestamp for file name
arduino_port = "COM9" #serial port of Arduino
baud = 115200 #arduino mega runs at 9600 baud
fileName="LoggedData.csv" #name of the CSV file generated (+ time stamp)

ser = serial.Serial(arduino_port, baud)
print("Connected to crane:" + arduino_port)
file = open(str(int(logTime))+"_"+fileName, "w") #Write new file
print("Created file")

all_data = []

while (1):
    try:
        getData=str(ser.readline().decode())  
        getData=getData.replace("\r\n", "")
        print(getData)

        all_data.append(getData)

        #file = open(str(int(logTime))+"_"+fileName, "a")
        #file.write(getData.replace("\r", "")) #write data with a newline

    except KeyboardInterrupt:
        for data in all_data:
            #print(data)
            file = open(str(int(logTime))+"_"+fileName, "a")
            file.write(data+"\n") #write data with a newline

        file.close()
        print("Data saved to log")
        #print(all_data)
        break
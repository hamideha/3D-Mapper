import serial

s = serial.Serial('COM4',115200) # Serial Port COM4 with 115200 BPS Baud Rate
s.open

f = open("FinalProjectData.xyz", "a") # Open file to write data to
writeFlag = False

for line in s:  
    line = line.decode("utf-8") # Decode Data as string
    line = line.rstrip("\r\n") # Remove newline character
    checkIfMeasurement = line.split(' ', 1)[0].replace('.', '', 1).isdigit() # Check if the line of data being read is a coordinate point
    
    if (checkIfMeasurement): # If it is a measurement write the line to the .xyz file
        writeFlag = True
        f.write(line + '\n')
        
    if (checkIfMeasurement == False and writeFlag == True):
        f.close()
        f = open("FinalProjectData.xyz", "a")

    print(line)

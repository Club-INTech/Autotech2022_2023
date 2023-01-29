import serial 

#To open the USB port
#arduino = serial.Serial(port = '/dev/ttyACM0', timeout=0)
S_MAXI = 3 #Maximum of the speed available
def rot(arduino, angle):
    '''Function to send an angular position to the MC'''
    type = 0x01 
    frame = type << 8 #Frame creation 
    angle += 90 #the angle can be between -90 and 90 deg
                #We reduce this to positive values (between 0 and 90 deg)
    value = 255 * (angle/180)
    value = round(value) #We round to the nearest whole value
    frame = (frame | value)
    arduino.write(int.to_bytes(frame, 2, byteorder ='big')) #We send the requested frame


def emergency_stop(arduino, value):
    '''Function to send to the MC if we need an emergency stop or not '''
    if value == False: #No need of emergency stop
        frame = 0x0400
        arduino.write(int.to_bytes(frame, 2, byteorder ='big')) #We send the requested frame
    else : #Need an emergency stop
        frame = 0x0401
        arduino.write(int.to_bytes(frame, 2, byteorder ='big')) #We send the requested frame


def forward_speed(value):
    '''Function to send the requested speed to move forward to the MC'''
    type = 0x02
    frame = type << 8 #Frame creation
    value = 255 * (value/(S_MAXI))
    value = round(value) #We round to the nearest whole value
    frame = (frame | value)
    arduino.write(int.to_bytes(frame, 2, byteorder ='big')) #We send the requested frame


def backward_speed(value):
    '''Function to send the requested speed to move back<ard to the MC'''
    type = 0x03
    frame = type << 8 #Frame creation
    value = 255 * (value/(S_MAXI))
    value = round(value) #We round to the nearest whole value
    frame = (frame | value)
    arduino.write(int.to_bytes(frame, 2, byteorder ='big')) #We send the requested frame


def actual_speed():
    '''Function to request the actual speed'''
    while True :
        type=arduino.read()
        value=arduino.read()
        if type==0x06 :
            return value*(S_MAXI/255)
        

def actual_angle():
    '''Function to request the actual angle'''
    while True :
        type=arduino.read()
        value=arduino.read()
        if type==0x05 :
            return value*(180/255)

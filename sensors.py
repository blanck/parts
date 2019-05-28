"""
sensors.py
Classes to receive sensordata from arduino
"""

import time

class Sensors:
    '''
    Installation:
    sudo apt install python3-smbus
    or
    sudo apt-get install i2c-tools libi2c-dev python-dev python3-dev
    git clone https://github.com/pimoroni/py-smbus.git
    cd py-smbus/library
    python setup.py build
    sudo python setup.py install

    pip install serial
    '''

    def __init__(self, port='/dev/ttyACM1', baud=115200, poll_delay=0.0166):
        import serial

        try:
            self.serial = serial.Serial(port, baud)
        except:
            print("Could not open serial port %", port)

        self.distance = { 'front' : 0., 'rear' : 0., 'left' : 0. , 'right' : 0. }
        self.steering = 0.
        self.poll_delay = poll_delay
        self.on = True

    def update(self):
        while self.on:
            self.poll()
            time.sleep(self.poll_delay)
                
    def poll(self):
        serline = self.serial.readline().strip()
        try:
            serline = serline.decode()
        except:
            serline = ""
        sensordata = serline.split(",")
        if len(sensordata) > 4 and sensordata[0] != '':
            self.distance['front'] = int(sensordata[0])
            self.distance['rear'] = int(sensordata[1])
            self.distance['left'] = int(sensordata[2])
            self.distance['right'] = int(sensordata[3])
            self.steering = int(sensordata[4])


    def run_threaded(self):
        return self.distance['front'], self.distance['rear'], self.distance['left'], self.distance['right'], self.steering

    def run(self):
        self.poll()
        return self.distance['front'], self.distance['rear'], self.distance['left'], self.distance['right'], self.steering

    def shutdown(self):
        self.on = False


if __name__ == "__main__":
    iter = 0
    s = Sensors()
    while iter < 100:
        data = s.run()
        print(data)
        time.sleep(0.1)
        iter += 1
     

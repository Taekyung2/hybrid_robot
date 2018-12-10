import RPi.GPIO as GPIO
import time
import tx_Beacon
import tcpServer
import tcpServerThread

# -----------------------------------------------------------------------------------------------
class pi_car:

	def __init__(self,in1=12,in2=13,ena=6,in3=20,in4=21,enb=26):
		self.IN1 = in1
		self.IN2 = in2
		self.IN3 = in3
		self.IN4 = in4
		self.ENA = ena
		self.ENB = enb
		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)
		GPIO.setup(self.IN1,GPIO.OUT)
		GPIO.setup(self.IN2,GPIO.OUT)
		GPIO.setup(self.IN3,GPIO.OUT)
		GPIO.setup(self.IN4,GPIO.OUT)
		GPIO.setup(self.ENA,GPIO.OUT)
		GPIO.setup(self.ENB,GPIO.OUT)
		self.pwm = 100
		self.PWMA = GPIO.PWM(self.ENA,self.pwm)
		self.PWMB = GPIO.PWM(self.ENB,self.pwm)
		self.PWMA.start(self.pwm)
		self.PWMB.start(self.pwm)
		self.stop()
	    
# -----------------------------------------------------------------------------------------------


	def forward(self):                
                GPIO.output(self.IN1,GPIO.LOW)
                GPIO.output(self.IN2,GPIO.HIGH)
                GPIO.output(self.IN3,GPIO.HIGH)
                GPIO.output(self.IN4,GPIO.LOW)

# -----------------------------------------------------------------------------------------------

	def stop(self):
		GPIO.output(self.IN1,GPIO.LOW)
		GPIO.output(self.IN2,GPIO.LOW)
		GPIO.output(self.IN3,GPIO.LOW)
		GPIO.output(self.IN4,GPIO.LOW)

# -----------------------------------------------------------------------------------------------

	def backward(self):
                GPIO.output(self.IN1,GPIO.HIGH)
                GPIO.output(self.IN2,GPIO.LOW)
                GPIO.output(self.IN3,GPIO.LOW)
                GPIO.output(self.IN4,GPIO.HIGH)

# -----------------------------------------------------------------------------------------------

	def left(self):
		GPIO.output(self.IN1,GPIO.LOW)
		GPIO.output(self.IN2,GPIO.HIGH)
		GPIO.output(self.IN3,GPIO.LOW)
		GPIO.output(self.IN4,GPIO.LOW)

# -----------------------------------------------------------------------------------------------

	def right(self):
		GPIO.output(self.IN1,GPIO.LOW)
		GPIO.output(self.IN2,GPIO.LOW)
		GPIO.output(self.IN3,GPIO.HIGH)
		GPIO.output(self.IN4,GPIO.LOW)

# -----------------------------------------------------------------------------------------------

	def setPWM_up(self):
                self.pwm = self.pwm + 20
                if self.pwm > 100 :
                    self.pwm = 100
                self.PWMB.ChangeDutyCycle(self.pwm)
                self.PWMA.ChangeDutyCycle(self.pwm)

# -----------------------------------------------------------------------------------------------

	def setPWM_down(self):
                self.pwm = self.pwm -20
                if self.pwm < 0 :
                    self.pwm = 0
                self.PWMA.ChangeDutyCycle(self.pwm)
                self.PWMB.ChangeDutyCycle(self.pwm)

# -----------------------------------------------------------------------------------------------

	def setMotor(self, left, right):
		if((right >= 0) and (right <= 100)):
			GPIO.output(self.IN1,GPIO.HIGH)
			GPIO.output(self.IN2,GPIO.LOW)
			self.PWMA.ChangeDutyCycle(right)
		elif((right < 0) and (right >= -100)):
			GPIO.output(self.IN1,GPIO.LOW)
			GPIO.output(self.IN2,GPIO.HIGH)
			self.PWMA.ChangeDutyCycle(0 - right)
		if((left >= 0) and (left <= 100)):
			GPIO.output(self.IN3,GPIO.HIGH)
			GPIO.output(self.IN4,GPIO.LOW)
			self.PWMB.ChangeDutyCycle(left)
		elif((left < 0) and (left >= -100)):
			GPIO.output(self.IN3,GPIO.LOW)
			GPIO.output(self.IN4,GPIO.HIGH)
			self.PWMB.ChangeDutyCycle(0 - left)

# -----------------------------------------------------------------------------------------------

class Executer:

    def __init__(self, tcpServer):
        self.andRaspTCP = tcpServer
        self.motor = pi_car()
        self.tx_Beacon = tx_Beacon.Beacon_tx()

# -----------------------------------------------------------------------------------------------

    def startCommand(self, command):
        if command == "1\n":
            self.motor.forward()
        if command == "2\n":
            self.motor.backward()
        if command == "3\n":
            self.motor.left()
        if command == "4\n":
            self.motor.right()
        if command == "5\n":
            self.motor.stop()
        if command == "6\n":
            self.motor.setPWM_up()
            print(self.motor.pwm)
        if command == "7\n":
            self.motor.setPWM_down()
            print(self.motor.pwm)
        if command == "8\n":
            print("A")
            self.tx_Beacon.formation = "0A"
        if command == "9\n":
            print("B")
            self.tx_Beacon.formation = "0B"
        if command == "10\n":
            print("C")
            self.tx_Beacon.formation = "0C"
            
# -----------------------------------------------------------------------------------------------
  
if __name__ == "__main__" :
    motor1 = pi_car()
    motor1.forward()
    time.sleep(2)
    motor1.backward()
    time.sleep(2)
    motor1.left()
    time.sleep(2)
    motor1.right()
    time.sleep(2)
    motor1.stop()
#    time.sleep(3)
#    motor.backward
#    time.sleep(3)
#    motor.stop
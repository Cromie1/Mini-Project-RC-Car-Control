# Mini-Project-RC-Car-Control
Names: 
Alex Cromar,
Kyla Kunz,
Mike Mercer

# E-Maxx Mirage

# Purpose and Functionality

# Instructions
To begin use with the breadboard design the user must connect the stm microcontroller to the following pins. Red 5v to the 5v pin. Black gnd to the gnd pin. Yellow first yellow wire to the SCL pin. Orange wire to SDA pin. Second yellow wire to the 3v pin. 

After connecting all the necessary wires the user must then plug in the transmitter to power and turn on the transmitter. At this point, the LEDs on the breakout boards should be on and the transmitter LED should be on. The user can then turn on the RC car and adjust the trim potentiometers on the transmitter to ensure alignment is correct. After this, the care should be fully functional. The user must make sure that the antenna is not fully extended otherwise radio interference will cause the I2C  Singals to not work properly.
# Wiring Diagram

# Schematics
Two schematics were created for this project unfortunately neither design worked and would require a redesign.
![alt text](PCB_Design.png)
This is the schematic for the basic implementation. There is not change being done to the input voltage for the digital potentiometer.
![alt text](PCB_Soldered.png)
This is the finished version of the first design.
![alt text](PCB_ZenerDiodes_Design.png)
This design utilizes zener diodes to drop the input voltages into the digital potentiometers. 
This was done to decrease the range so that we could input finer granularity into the control of the car.
![alt text](PCB_ZenerDiodes_Soldered.png)
This is the completed board for the second design.
# Flowcharts

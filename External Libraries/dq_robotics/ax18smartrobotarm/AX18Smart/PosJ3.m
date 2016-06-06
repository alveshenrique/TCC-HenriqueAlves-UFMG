% Position control for AX18 Smart Robot Arm joint 3

% AX-18A Dynamixel servo positioning example

clear all
close all
clc
Port = serial('/dev/ttyUSB0','BaudRate',1000000,'Terminator',[]);  % Creates the object for serial comunication


fopen(Port);  % Opens the serial port

Pos1 = 512;  % Desired position 0 to 1023

Hpos1 = dec2hex(bitand(768,Pos1));
Hpos1 = hex2dec(Hpos1(1));
Lpos1 = bitand(255,Pos1);

Pos2 = 1023 - Pos1;  % Desired position 0 to 1023

Hpos2 = dec2hex(bitand(768,Pos2));
Hpos2 = hex2dec(Hpos2(1));
Lpos2 = bitand(255,Pos2);


DynRegWrite(Port,4,30,[Lpos2 Hpos2]);
DynRegWrite(Port,5,30,[Lpos1 Hpos1]);

DynAction(Port,254);

fclose(Port); % Closes the serial port


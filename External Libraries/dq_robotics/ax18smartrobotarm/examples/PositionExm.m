% AX-18A Dynamixel servo positioning example

clear all
close all
clc
Port = serial('/dev/ttyUSB0','BaudRate',1000000,'Terminator',[]);  % Creates the object for serial comunication


fopen(Port);  % Opens the serial port

Pos = 512;  % Desired position 0 to 1023
id = 7;  % Device ID

Hpos = dec2hex(bitand(768,Pos));
Hpos = hex2dec(Hpos(1));
Lpos = bitand(255,Pos);

DynWrite(Port,id,30,[Lpos Hpos]);


fclose(Port); % Closes the serial port


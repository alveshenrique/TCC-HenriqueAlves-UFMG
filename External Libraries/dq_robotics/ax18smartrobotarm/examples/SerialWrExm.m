% Dynamixel Write Example

clear all
close all
clc
Port = serial('/dev/ttyUSB0','BaudRate',1000000,'Terminator',[]);  % Creates the object for serial comunication


fopen(Port);  % Opens the serial port

Pos = 100;  % Desired position 0 to 1023

data = DynWrite(Port,1,30,[bitand(255,Pos) bitand(768,Pos)]);

fclose(Port); % Closes the serial port


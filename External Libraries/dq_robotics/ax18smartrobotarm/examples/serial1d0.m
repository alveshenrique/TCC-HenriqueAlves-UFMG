% Serial Communication

clear all
close all
clc
Port = serial('/dev/ttyUSB0','BaudRate',1000000,'OutputBufferSize',8,'Terminator',[]);  % Creates the object for serial comunication


fopen(Port);  % Opens the serial port

data = DynRead(Port,1,3,1); % Reads the device ID

fclose(Port); % Closes the serial port


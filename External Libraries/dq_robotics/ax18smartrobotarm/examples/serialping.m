% Serial Communication

clear all
close all
clc
Port = serial('/dev/ttyUSB0','BaudRate',1000000,'Terminator',[]);  % Creates the object for serial comunication


fopen(Port);  % Opens the serial port

data = DynPing(Port,1); % Ping test to retrieve an empty status packet

fclose(Port); % Closes the serial port


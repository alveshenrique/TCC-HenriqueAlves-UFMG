% Serial Communication

clear all
close all
clc
Port = serial('COM4','BaudRate',1000000,'Terminator',[]);  % Creates the object for serial comunication


fopen(Port);  % Opens the serial port

data = DynPing(Port,1); % Reads the device ID

fclose(Port); % Closes the serial port


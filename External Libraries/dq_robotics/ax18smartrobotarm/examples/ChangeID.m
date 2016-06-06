%% Changes the ID of a single AX-18A servo

clear all
close all
clc
% Port = serial('/dev/ttyUSB0','BaudRate',1000000,'Terminator',[]);  % Creates the object for serial comunication
Port = serial('COM3','BaudRate',1000000,'Terminator',[]);  % Creates the object for serial comunication
%% Executes the ID setting

fopen(Port);  % Opens the serial port

% id = 9;  % New ID (range from 0 to 253)
id = 3;  % New ID (range from 0 to 253)  %Fredy

DynWrite(Port,254,3,id); % Sets the new ID

data = DynPing(Port,id); % Checks the set ID

display(['set ID: ' num2str(data(3))]);


fclose(Port); % Closes the serial port


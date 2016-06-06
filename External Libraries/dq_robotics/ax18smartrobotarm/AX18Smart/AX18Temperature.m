% Temperature Monitor

% AX-18A Dynamixel servo positioning example

clear all
close all
clc
Port = serial('/dev/ttyUSB0','BaudRate',1000000,'Terminator',[]);  % Creates the object for serial comunication


fopen(Port);  % Opens the serial port
temp = zeros(9,1);

for i=1:9
    data = DynRead(Port,i,43,1);
    temp(i) = data(6);
    display(['Temperature in motor ' num2str(i) ': ' num2str(temp(i))])
end



fclose(Port); % Closes the serial port


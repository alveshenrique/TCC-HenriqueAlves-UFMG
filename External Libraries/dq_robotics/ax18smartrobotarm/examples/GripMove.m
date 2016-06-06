% AX-18A Dynamixel gripper 

clear all
close all
clc
Port = serial('/dev/ttyUSB0','BaudRate',1000000,'Terminator',[]);  % Creates the object for serial comunication


fopen(Port);  % Opens the serial port


 AxJoint1(Port,0)
 AxJoint2(Port,0)
 AxJoint3(Port,0)
 AxJoint4(Port,0)
 AxJoint5(Port,0)
 AxGrip(Port,10)

% 
% for i=0:50
%     AxJoint2(Port,i)
%     pause(0.01)
% end

data1 = DynRead(Port,2,40,2);
data2 = DynRead(Port,3,40,2);

fclose(Port); % Closes the serial port


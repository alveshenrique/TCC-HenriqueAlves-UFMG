% AX18 Smart Robot Arm limits
clear all
close all

Port = serial('/dev/ttyUSB0','BaudRate',1000000,'Terminator',[]);  % Creates the object for serial comunication


fopen(Port);  % Opens the serial port

% Limits for Joints 2,3,4,5

d = 150;    % Angle variation from center ( + and - )
d = fix((d*1023)/300); 

% Lower limit
lim1 = 512 - d;

Hlim1 = dec2hex(bitand(768,lim1));
Hlim1 = hex2dec(Hlim1(1));
Llim1 = bitand(255,lim1);

% Higher limit
lim2 = 512 + d; 

Hlim2 = dec2hex(bitand(768,lim2));
Hlim2 = hex2dec(Hlim2(1));
Llim2 = bitand(255,lim2);

[L6 H6] = DynPos(0);
[L6h H6h] = DynPos(1023);


DynWrite(Port,2,6,[Llim1 Hlim1 Llim2 Hlim2]); % 
DynWrite(Port,3,6,[Llim1 Hlim1 Llim2 Hlim2]); % Joint 3

DynWrite(Port,4,6,[Llim1 Hlim1 Llim2 Hlim2]);
DynWrite(Port,5,6,[Llim1 Hlim1 Llim2 Hlim2]); % Joint 3

DynWrite(Port,6,6,[L6 H6 L6h H6h]); % Joint 4

DynWrite(Port,7,6,[Llim1 Hlim1 Llim2 Hlim2]); % Joint 5

fclose(Port)

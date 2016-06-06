% Gripper angle limits
clear all
close all

Port = serial('/dev/ttyUSB1','BaudRate',1000000,'Terminator',[]);  % Creates the object for serial comunication


fopen(Port);  % Opens the serial port

Dang = 208;  % Angle variation

% Right servo limit (ID = 8)
limR = 512 + Dang;  % Angle range for the gripper

HlimR = dec2hex(bitand(768,limR));
HlimR = hex2dec(HlimR(1));
LlimR = bitand(255,limR);

% Central position

Hcen = dec2hex(bitand(768,512));
Hcen = hex2dec(Hcen(1));
Lcen = bitand(255,512);

% Left servo limit  (ID = 9)
limL = 512 - Dang;

HlimL = dec2hex(bitand(768,limL));
HlimL = hex2dec(HlimL(1));
LlimL = bitand(255,limL);

% Writes the limits to the EEPROM

DynWrite(Port,8,6,[Lcen Hcen LlimR HlimR]);
DynWrite(Port,9,6,[LlimL HlimL Lcen Hcen]);
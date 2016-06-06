function data = DynRead(s,id,addr,n)
%DYNREAD reads data from a Dynamixel device
%   DYNREAD(s,id,addr,n) reads data from device with ID 'id' starting
%   at register location 'addr' and taking the 'n' following register
%   bytes through the communcation port 's'.
%   'id' is the device ID from 0 to 253, 254 is the broadcast address.

% Ernesto Lana
% Version 1.1 August, 2012
% Version 1.0 July, 2012

% Clears the buffer before sending read command
if s.BytesAvailable ~= 0
    fread(s,s.BytesAvailable);
end

ln = length(n)+3;  % Length of data packet
inst = 2; % Instruction, 2 = read data
params = [addr n];  % Instruction parameters

chksm = DynChecksum(id,ln,inst,params);  % Computes the packet checksum

pckt = [255 255 id ln inst params chksm];  % Builds the dynamixel packet

fwrite(s,pckt)  % Writes the dynamixel packet to the specified device
data = fread(s,6+n);  % Reads data coming from the device



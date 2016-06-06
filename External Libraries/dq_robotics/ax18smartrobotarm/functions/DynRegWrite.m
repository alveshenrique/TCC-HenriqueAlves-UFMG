function DynRegWrite(s,id,addr,params)
%DYNREGWRITE writes data to a Dynamixel device
%   DYNREGWRITE(s,id,addr,params) writes data to device with ID 'id' at
%   register address 'addr' with parameters 'params' through the communcation port 's'.
%   The written data is held but not executed until an action
%   packet arrives.
%   'id' is the device ID from 0 to 253, 254 is the broadcast address.

% Ernesto Lana
% Version 1.0
% July, 2012

ln = length(params)+3;  % Length of data packet
inst = 4; % Instruction, 4 = register write data
params1 = [addr params];  % Instruction parameters

chksm = DynChecksum(id,ln,inst,params1);  % Computes the packet checksum

pckt = [255 255 id ln inst params1 chksm];  % Builds the dynamixel packet

fwrite(s,pckt)  % Writes the dynamixel packet to the specified device

% Clears the data buffer
if s.BytesAvailable ~= 0
    fread(s,s.BytesAvailable);
end

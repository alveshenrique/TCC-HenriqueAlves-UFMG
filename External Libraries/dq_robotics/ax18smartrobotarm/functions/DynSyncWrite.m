function DynSyncWrite(s,id,addr,params)
%DYNSYNCWRITE writes data to several Dynamixel devices in one packet
%   DYNSYNCWRITE(s,id,addr,params) writes data to devices with IDs defined by the vector 'id'
%   at register address 'addr' with parameters vector 'params' through the communcation port 's'.
%   'id' is a vector of device IDs, each one from 0 to 253.
%   The 'params' vector must be a value vector ordered according to the 'id'
%   vector.
%   The command must be the same for every device (i.e. the same register
%   address will be written using this function)

% Ernesto Lana
% Version 1.0
% July, 2012

ln = length(params)+length(id)+4;  % Length of data packet
inst = 131; % Instruction, 131(0x83) = sync write
id1 = 254; % Broadcast ID



params1 = zeros(1,ln-4);  % Initializes a parameters vector
np = fix(length(params)/length(id))+1;  % Number of parameters per device

for i=1:length(id)
    params1(1,(i-1)*np+1:i*np) = [id(i) params((i-1)*(np-1)+1:i*(np-1))]; % Builds a parameters vector
end

params1 = [addr params1];

chksm = DynChecksum(id1,ln,inst,params1);  % Computes the packet checksum

pckt = [255 255 id1 ln inst (np-1) params1 chksm];  % Builds the dynamixel packet

fwrite(s,pckt)  % Writes the dynamixel packet to the specified device

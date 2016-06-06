classdef AX18 < handle
    % CLASS AX18
    % Defines the AX18 Smart Robot Arm motion

% Version 1.0 of july 2012 by Ernesto P Lana
% Version 1.1 of october 2012 by Ernesto P Lana
% Version 1.3 of january 2013 by Ernesto P Lana
% Version 1.4 of april 2013 by Ernesto P Lana
    properties
        device
        port
        DQmodel
    end
    methods
        function obj = AX18(varargin)
            if nargin == 0
                obj.device = '/dev/ttyUSB0';
            else
                obj.device = varargin{1};
            end
            obj.port = serial(obj.device,'BaudRate',1000000,'Terminator',[]);  % Creates the object for serial comunication
            % == DQ model for AX18 Smart Robot Arm ==
            % D&H Parameters Vectors
            DH_Alpha   =  [-pi/2  0      -pi/2   0   -pi/2   0 ];  % Alpha
            DH_A       =  [  0    0.159    0   0.02225    0    0 ];  % A           
            DH_Theta   =  [  0    0      -pi/2   0   -pi/2   0 ];  % Theta         
            DH_D      =  [ 0.167   0        0   0.0815    0.041    0 ];  %             
            DH_virtual =  [0,0,0,1,0,0]; % Virtual joint definition
            
%             % D&H Parameters Vectors  
%             %Juancho( se multiplica por 0.001 para convertir en mil�metros)
%             DH_Alpha  =   [-pi/2  0    -pi/2    -pi/2   0];  % alpha
%             DH_A      =   [0      156     22.25   0      0]*0.001;  % medida con metro
%             %DH_A      =   [0      159     22.25   0      0]*0.001;  % basada en otros
%             DH_Theta  =   [0      0      -pi/2   -pi/2      0 ];  % theta
%             DH_D      =   [167    0       0      122.5      0]*0.001;  % d
%             DH_virtual =  [0,0,0,0,0,0]; % Virtual joint definition
  
            
            
            
            DH_RobotMatrix = [ DH_Theta ; DH_D ; DH_A ; DH_Alpha ; DH_virtual ]; % D&H parameters matrix for the arm model
            lef = 0.17;  % Length of end effector (from last link)
            eff = (1 + DQ.E*(-0.5*lef*DQ.j))*(cos(pi/4)-DQ.k*sin(pi/4));  % End effector definition
            AX18DQ = DQ_kinematics(DH_RobotMatrix,'standard'); % Defines robot model using dual quaternions
            AX18DQ.set_effector(eff)
            obj.DQmodel = AX18DQ;
        end
        function start(obj)
            %START establishes the communication for AX18 Smart Robot Arm
            fopen(obj.port);
        end
        function stop(obj)
            %STOP closes the communication for AX18 Smart Robot Arm
            fclose(obj.port);
        end
        function delete(obj)
            %DELETE erases the communication port for AX18 Smart Robot Arm
            fclose(obj.port);
            delete(obj.port);
        end
        function joint(obj,j,p,v)
            %JOINT(j,p,v) moves joint 'j' (1 to 5) to position 'p' (in radians) at velocity 'v' (0 to 0.1)
            AXJoint(obj.port,j,p,v)
        end
        function J = getjoints(obj)
            j1 = DynRead(obj.port,1,36,2);
            j1 = DynInvPos(j1(6),j1(7));
            j2 = DynRead(obj.port,2,36,2);
            j2 = -DynInvPos(j2(6),j2(7))-pi/2;
            j3 = DynRead(obj.port,5,36,2);
            j3 = -DynInvPos(j3(6),j3(7));
            j4 = DynRead(obj.port,6,36,2);
            j4 = DynInvPos(j4(6),j4(7));
            j5 = DynRead(obj.port,7,36,2);
            j5 = DynInvPos(j5(6),j5(7));
            J = [j1;j2;j3;j4;j5];
        end
        function J = getjointsVel(obj)  %% bien
            j1 = DynRead(obj.port,1,38,2);
            j1 = DynInvVel(j1(6),j1(7));
            j2 = DynRead(obj.port,2,38,2);
            j2 = DynInvVel(j2(6),j2(7));
            j3 = DynRead(obj.port,5,38,2);
            j3 = DynInvVel(j3(6),j3(7));
            j4 = DynRead(obj.port,6,38,2);
            j4 = DynInvVel(j4(6),j4(7));
            j5 = DynRead(obj.port,7,38,2);
            j5 = DynInvVel(j5(6),j5(7));
            J = [j1;j2;j3;j4;j5];
        end
        function J = getgrip(obj) % gets gripper independent joint values
            j1 = DynRead(obj.port,8,36,2);
            j1 = DynInvPos(j1(6),j1(7));
            j2 = DynRead(obj.port,9,36,2);
            j2 = DynInvPos(j2(6),j2(7));
            J = [j1;j2];
        end
        function varargout = temperature(obj,varargin)
            %TEMPERATURE(obj,disp) returns motor temperature. If a
            %parameter 't' is given, temperature(t) sets the highest limit
            %temperature as being 't' Celsius degrees (default t = 75)
            T = zeros(9,1);
            
            for i=1:9
                data = DynRead(obj.port,i,43,1);
                T(i) = data(6);
                display(['Temperature in motor ' num2str(i) ': ' num2str(T(i))])
            end
            if nargout == 1
            varargout(1) = {T};
            end
            if nargin == 2
                a = cell2mat(varargin(1));
                DynWrite(obj.port,254,11,a);
                ts = 'Set';
            else
                ts = 'Current';
            end
            tl = DynRead(obj.port,1,11,1);
                display([ts ' highest limit temperature: ' num2str(tl(6)) ...
                    ' Celsius degrees'])
        end
        function baudrate(obj,varargin)
            %BAUDRATE() returns the communication port speed for the AX18
            %Smart Robot Arm. If a parameter 'd' is given (from 0 to 255), baudrate(d) sets
            %the communication speed to a baudrate of 2000000/(d+1) (bps)
            %(default d = 1)
            if nargin == 1
                spd = DynRead(obj.port,1,4,1); % Reads baud rate registry address
                spd = 2000000/(spd(6)+1); % Computes the baud rate in BPS
                display(['Baud rate: ' num2str(spd) '(BPS)']);
            end
            if nargin == 2
                % <<disabled function>>
                %a = cell2mat(varargin);
                %DynWrite(obj.port,254,4,a);
                %spd = 2000000/(a+1); % Computes the baud rate in BPS
                %display(['Set baud rate: ' num2str(spd) '(BPS)']);
            end
        end
        function delay(obj,varargin)
            % DELAY() returns the delay time per data value from the
            % transmission of an instruction packet until the return of a
            % status packet. If a parameter 't', from 0 to 254, is given,
            % delay(t) sets the delay time to 2*t microseconds. The default
            % value is 250 (500 microseconds)
            if nargin == 1
                dly = DynRead(obj.port,1,5,1); % Reads baud rate registry address
                dly = dly(6)*2; % Computes the baud rate in BPS
                display(['Return delay time: ' num2str(dly) '(usec)']);
            end
            if nargin == 2
                a = cell2mat(varargin);
                DynWrite(obj.port,254,5,a);
                dly = 2*a;
                display(['Set return delay time: ' num2str(dly) '(usec)']);
            end
        end
%         function jlims(obj)
%             % <<função pendente>>
%         end
        function varargout = load(obj,varargin)
            %LOAD() returns each AX18 Smart Robot Arm motor load in the (counter) 
            %clockwise (C)CW direction. If a parameter 'l' is given, load(l)
            %sets the highest load, from 0 to 100%.
            L = zeros(9,1);
            
            for i=1:9
                data = DynRead(obj.port,i,40,2);
                l = data(6) + 256*data(7);
                if l<1024
                    l = (l*100)/1023;
                    L(i) = l;
%                     dir = 'CCW';
                else
                    l = ((l-1024)*100)/1023;
                    L(i) = -l;
%                     dir = 'CW';
                end
                display(['Load in motor ' num2str(i) ': ' num2str(L(i),'%4.2f') '% '])
            end
            if nargout == 1
            varargout(1) = {L};
            end
            if nargin == 2
                a = cell2mat(varargin(1));
                a = fix((a*1023)/100);
                [L H] = DynLH(a);
                DynWrite(obj.port,254,14,[L H]);
                ts = 'Set';
            else
                ts = 'Current';
            end
            tl = DynRead(obj.port,1,14,2);
            tl = tl(6)+256*tl(7);
            tl = (tl*100)/1023;
                display([ts ' load limit: ' num2str(tl,'%4.2f') ...
                    ' (% of maximum)'])
        end
        function voltage(obj,vargin)
            %VOLTAGE could be used to indicate (along with the current voltage)
            %or to modify the voltage range of operation
            %of the servos. If two parameters are given, from 0 to 15, then
            %voltage(min,max) sets the operation voltage range from min to
            %max volts.
            if nargin == 3
                v1 = cell2mat(vargin(1));
                v2 = cell2mat(vargin(2));
                if v1<=15 && v2>=0 && v2>v1
                    DynWrite(obj.port,254,12,[fix(v1*10) fix(v2*10)])
                    s1 = 'Set';
                end
            else
                s1 = 'Current';
            end
            v1 = DynRead(obj.port,1,12,2);
            v2 = v1(7);
            v1 = v1(6);
            display([s1 ' voltage range: ' num2str(v1/10) ' to ' num2str(v2/10)])
            vp = DynRead(obj.port,1,42,1);
            disp(['Present voltage: ' num2str(vp(6)/10)])
        end
        function grip(obj,pct,v)
            % GRIP sets the percentage of openess of the AX18 gripper,
            % between 0 (fully closed) to 100% (fully opened), at velocity
            % vel. grip(percentage,vel)
            AxGrip(obj.port,pct)
            vel = fix(v*1023);
            if vel==0
                vel=1;
            end
            [Lvel Hvel] = DynLH(vel);
            DynWrite(obj.port,8,32,[Lvel Hvel]);
            DynWrite(obj.port,9,32,[Lvel Hvel]);
        end
        function compliance(obj,varargin)
            %COMPLIANCE() returns the AX18 Smart Robot Arm motor compliance (looseness). If a
            %parameter 'c' is given, compliance(c) sets the compliance
            %angle.
            if nargin == 2
                a = cell2mat(varargin(1));
                a = a/2;
                a = fix((a*1023)/300);
                if a>255
                    a=255;
                end
                if a<1
                    a=1;
                end
                DynWrite(obj.port,254,26,[a a]);
                ts = 'Set';
            else
                ts = 'Current';
            end
            tl = DynRead(obj.port,1,26,1);
            tl = tl(6)*2;
            tl = (tl*300)/1023;
                display([ts ' motor compliance: ' num2str(tl,'%4.2f') ...
                    ' degrees'])
        end
        function m = moving(obj)
            % Indicates if AX18 Smart Robot Arm is moving (1) or not (0)
            ids = [1,2,4,6,7,8,9];
            m = 0;
            for i=1:5
                st = DynRead(obj.port,ids(i),46,1);
                if st(6) == 1
                    m = 1;
                end
            end
        end
    end
end

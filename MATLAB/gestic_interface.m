% INTERFACE COM O KIT HILLSTAR MGC3130 DA MICROCHIP COM O MATLAB.
% PARA SEU FUNCIONAMENTO, ADICIONE O PATH DESDE .m NO CÓDIGO EM C++ EM SEU
% TRECHO ADEQUADO.
% PARA INFORMAÇÕES SOBRE O CÓDIGO EM C++ UTILIZADO, VIDE O CÓDIGO EM ANEXO.
% VARIÁVEIS COLETADAS DO SENSOR E ARMAZENADAS EM MATRIZES NO MATLAB:
% 
% dX, dY, dZ: Posição (X,Y,Z) da mão do usuário no espaço, com origem no
% centro da placa. Seus valores variam de 0 a 65535.
%
% dLastGesture: Último gesto detectado pelo sensor. Seus valores são:
%     1 : East>West flick
%     2 : West>East flick
%     3 : South>North flick
%     4 : North>South flick
%     0 : Sem gestos detectados.
%     
%     O valor desta variável voltará para 0 se nenhum gesto tiver sido 
%     detectado num período de 1 segundo. A taxa de atualização desta
%     variável é de 10ms no programa em C++. Tanto a taxa de atualização,
%     quanto o intervalo de tempo necessário para zerar a variável podem
%     ser editados no programa em C++ anexado.
%
% dGesture: Retém o último valor de dLastGesture, até que um gesto
% diferente do guardado seja detectado, substituindo o valor anterior.
% Repare que neste caso, dGesture valerá 0 apenas enquanto nenhum gesto
% tiver sido detectado após o início da rotina.
%
% dAirWheel: Quando detectado o gesto de AirWheel, esta variável irá crescer
%            caso o giro seja em sentido horário, e decrescer caso o giro
%            seja em sentido antihorário. Seus valores variam de 0 a 255.
% 
% dTouch: número inteiro que indica qual eletrodo foi tocado pelo usuário.
%         0: SOUTH
%         1: WEST
%         2: NORTH
%         3: EAST
%         4: CENTER
%         5: NO TOUCH
%         O valor 5 será o inicializado pela variável, sendo que a partir do
%         momento que algum eletrodo for tocado, seu valor será guardado na
%         matriz até que outro eletrodo seja tocado. Portanto nunca mais
%         valerá 5, até que o programa (em C++) seja resetado.
% REESCREVER PARA DTOUCH E DTOUTCHD!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% 
% Para que este script funcione, é necessário adicionar o path (com
% subdiretórios) das bibliotecas contidas nas pastas:
% 
%     ICREATE
%     dq_robotics
% O driver do robô deve também estar instalado na máquina:
%     PL2303_Vista_32_64_332102.exe
%     PL2303_Prolific_GPS_1013_20090319.exe (se necessário).
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % CÓDIGO UTILIZADO PARA CONTROLAR O ROBÔ "JOÃOZINHO"
% 
% clc
% display('Executing Kinematic Control...');
% 
% %% Robot initialization
% % Communication ports definition
% arm_port = {'COM9'}; %USB Serial Port
% base_port = 4; % Prolific USB-to-Serial Comm Port
% 
% % Robot definition
% serPort = RoombaInit(base_port);
% ax18 = AX18(arm_port);
% ax18.start
% 
% % Joint angles definitions
% % initial_theta_manipulator = [0 1.97*-pi/3 1.81*pi/3 0 0]' => r0 = - 0.70463 + 0.059169i + 0.059169j - 0.70463k; p0 = 0.041938i + 0.34949k
% theta0 = [0 1.97*-pi/3 1.81*pi/3 0 0]';
% q = [0 0 0 theta0']'; % [x_position_Base y_position_Base phi_angle_Base theta_manipulator']'
% 
% %% Initial end effector's pose definition
% theta_arm = theta0;
% 
% % Robot velocity (movement velocity)
% vel = 0.05;
% 
% % Sets the robot to the initial position
% for i=1:5
%     ax18.joint(i,theta_arm(i),vel);
% end
% ax18.grip(1,vel);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% CODIGO 2: CONTROLE DO ROBO
% 
% clear all;
clc;
close all;

%% Portas de comunicação para o braço e a base móvel
arm_port = {'COM9'};
base_port = 7;

%% Manipulator (physical) object
Ax18 = AX18(arm_port);
Ax18.start

%% D&H Parameters Vectors
DH_Alpha   =  [-pi/2     0       -pi/2      0       -pi/2    0 ];  % Alpha
DH_A          =  [  0    0.159    0      0.02225     0      0 ];  % A
DH_Theta   =  [  0        0      -pi/2      0       -pi/2     0 ];  % Theta
DH_D         =  [ 0.167   0        0     0.0815   0.041    0 ];  % D
DH_virtual =  [0,         0,       0,        1,         0,      0]; % Virtual joint definition

DH_RobotMatrix = [ DH_Theta ; DH_D ; DH_A ; DH_Alpha ; DH_virtual ]; % D&H parameters matrix for the arm model
AX18DQ = DQ_kinematics(DH_RobotMatrix,'standard'); % Defines robot model using dual quaternions

%% Initial Position for the Robot Arm
theta0 = [0    1.97*-pi/3   1.81*pi/3    0    0]';
thetab = theta0;
x_arm = AX18DQ.fkm(thetab);

%% Esta parte do código permite colocar ao robô manipulador na pose inicial
for i=1:5
    Ax18.joint(i,theta0(i),0.05);
end
% Ax18.grip(1,0.5);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
% clear;

display('Executing Kinematic Control...');

%Create a new DQ_kinematics object with the AX18 arm standard Denavit-Hartenberg parameters           
ax18 = DQ_AX18;

%Initial configuration
theta=[0    1.97*-pi/3   1.81*pi/3    0    0]';

%Configuring the end-effector:
ax18.set_effector(cos(pi/4)+DQ.i*sin(pi/4))

p = translation(ax18.fkm(theta));

% XYZ_inicial = [dX dY dZ]/65535;
% K=50;


% XYZ_final = [dX dY dZ]/65535;
% 
% if norm(XYZ_final - XYZ_inicial)>0.1
%     pd = p+DQ.i*XYZ_final(1)*K+DQ.j*XYZ_final(2)*K+DQ.k*XYZ_final(3)*K;
%     [theta,flag] = ax18_translate(ax18,theta,pd,0.1);
%     p = translation(ax18.fkm(theta));
% end
% 
% 
% pause(0.05);
% XYZ_inicial = XYZ_final;

% for i=0:50
%     pause(1);
%     disp(dLastGesture);
%     disp(dGesture);
% end


while(1)
    disp('Selecione um eixo para executar um movimento...');
while (dFlagTouch == 0) 
    pause(0.1);
end
disp('Eixo selecionado.');
pause(1);
if (dTouch == 1) || (dTouch == 3)
    mode = 1;
    disp('Movimento no eixo X.');
elseif (dTouch == 2) || (dTouch == 0)
    mode = 2;
    disp('Movimento no eixo Y.');
elseif (dTouch == 4)
    mode = 0;   % Se o eletrodo do centro que foi tocado, entao movimento no eixo Z
    disp('Movimento no eixo Z.');
end
disp('Selecione o movimento desejado. O movimento será dado com sinal, em torno do eixo selecionado.');
disp('Flicks do norte para o sul, e do leste para o oeste: Sinal Negativo.');
disp('Flicks do sul para o norte, e do oeste para o leste: Sinal Positivo.');
while (dGesture ~= dLastGesture)
    pause(0.1);
end

K = 30; % Amplitude do movimento de translação.

if (dGesture == 2) ||(dGesture == 4)
    K = -K;
    disp('Movimento Negativo.');
else
    disp('Movimento Positivo.');
end
switch mode
    case 1 
        pd = p+DQ.i*K; 
    case 2
        pd = p+DQ.j*K;
    case 0
        pd = p+DQ.k*K;
        
end
flag = dGesture;
error = pd - p;
while (norm(vec4(error)) > 0.1) && (flag == dGesture)
    x = ax18.fkm(theta);
    p = translation(x);
    error1 = pd - p;
%     if max(abs(rdivide(theta_new-theta,theta+0.1)))>100
%         flag=dGesture;
%         disp('Movimento cancelado.');
%         break;
%     end

 if (norm(vec4(error1))>norm(vec4(error)))
        flag=dGesture;
        disp('Movimento cancelado.');
        break;
 end
    error = error1;
    Jp = ax18.jacobp(ax18.jacobian(theta),x);
    theta = theta + pinv(Jp)*0.5*vec4(error);
    plot(ax18,theta);
    drawnow;
    
    for i=1:5
    Ax18.joint(i,theta(i),0.05);
    end
    
end
disp('Fim do movimento.');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % CÓDIGO DE TESTES:
% 
% hAxes = axes;  
% hData = plot(hAxes,nan,nan,nan,nan,'*');
% axis(hAxes,[0 65535 0 65535 0 65535]);
% axis manual;
% 
% % hAxes = axes;  
% % hData = plot(hAxes,nan,nan,'*');
% % axis(hAxes,[-2 2 -2 2]);
% % axis manual;
% 
% while (dLastGesture <4)
% %    disp('X =');
% %    disp(dX);
% %    disp('Y =');
% %    disp(dY);
% %    disp('Z =');
% %    disp(dZ);
% %    disp('Gesto: ');
% %    disp(dLastGesture);
% 
% % deg = (dAirWheel/255)*2*pi;
% % set(hData,'XData',cos(deg),'YData',sin(deg));
% 
% set(hData,'XData',dX,'YData',dY,'ZData',dZ);
% drawnow;
% % scatter3(dX,dY,dZ);
% 
% % disp(dTouch);
% 
% pause(0.05);
%     
% end


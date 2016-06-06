function [temp] = Brazo_AX18(n,v)
robot = AX18('COM4');
robot.start;
for i=n:-1:1
    if i~=2
    robot.joint(i,0,v);
    else
    robot.joint(i,-pi/4,v+0.03);
        for j=0.2:0.05:0.9
        robot.grip(j);
        pause(0.1)
        end
    end
end  
pause(3)
robot.joint(5,0,v);
robot.joint(4,-pi/8,v);
robot.joint(3,-pi/8,v);
robot.joint(2,-5*pi/8,v);
robot.joint(1,0,v);
robot.joint(3,-pi/4,v);
for j=0.9:-0.05:0.3
     robot.grip(j);
     pause(0.5)
 end
temp=robot.temperature;
robot.delete;
end
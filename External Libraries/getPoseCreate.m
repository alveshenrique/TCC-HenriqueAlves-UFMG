function xm = getPoseCreate(xm_old,serPort)
    phi = AngleSensorRoomba(serPort);
    d = DistanceSensorRoomba(serPort);
    px = d*cos(phi);
    py = d*sin(phi);
    xm = cos(phi/2)+DQ.k*sin(phi/2)+DQ.E*0.5*(DQ.i*(px*cos(phi/2)+py*sin(phi/2))+DQ.j*(-px*sin(phi/2)+py*cos(phi/2)));
    xm = xm_old*xm;
end
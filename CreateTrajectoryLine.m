function roverPath = CreateTrajectoryLine(p0,L,vel,alpha,N)       
    dl = L/(N-1);
    r = 0 : dl : L;
    phi = p0(3);
    psi = phi+alpha;
    cPsi = cos(psi);
    sPsi = sin(psi);
    R = [cPsi -sPsi;
            sPsi cPsi];
    for ind = 1 : N
        r_ = R*[r(ind);0];
        x(ind) = r_(1);
        y(ind) = r_(2);
    end
    
    x = p0(1)+x;
    y = p0(2)+y;
    
    tTotal = L/vel;
    dt = tTotal/(N-1);
    t = 0:dt:tTotal;
    phi = x;
    phi(:) = p0(3);

    roverPath = Path();

    roverPath.x = [x ];
    roverPath.y = [y ];
    roverPath.phi = [phi];
    roverPath.t = [t];
    
    v = roverPath.x;
    vSigned = vel*sign(L);
    v(:) = vSigned;
    v(2) = vSigned/2;
    v(end) = vSigned/2;
    roverPath.v = v;   
    roverPath.alpha = alpha;
end

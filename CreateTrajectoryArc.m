function roverPath = CreateTrajectoryArc(p0,R,arcL,vel,N)       
%     N = 10;
    sR = sign(R);
    arc = arcL/R;
    dAlpha = arc/(N-1)*sR;

    phi = p0(3);
    phi0 = -(phi+pi/2)*sR;
    
    phiEnd = phi0 + arc*sR;
    alpha = phi0:dAlpha:phiEnd;

    P = [p0(1); p0(2)];
    cPhi = cos(phi+pi/2*sR);
    sPhi = sin(phi+pi/2*sR);
    Rm = [cPhi -sPhi;
            sPhi cPhi];
    aR = abs(R);
    C = P+Rm*[aR; 0];

    N = length(alpha);
    x = zeros(1,N);
    y = x; PHI = x;
    for ind = 1 : N
        cPhi_ = cos(alpha(ind));
        sPhi_ = sin(alpha(ind));
        Rphi_ = [cPhi_ -sPhi_;
            sPhi_ cPhi_];
        p_ = C + Rphi_*[aR; 0];
        x(ind) = p_(1);
        y(ind) = p_(2);
        PHI(ind) = phi + (alpha(ind)-alpha(1)); 
    end

    %plot(x,y)
    
    tTotal = arc*aR/vel;
    dt = tTotal/(N-1);
    t = 0:dt:tTotal;

    roverPath = Path();

    roverPath.x = [x ];
    roverPath.y = [y ];
    roverPath.phi = [PHI];
    roverPath.t = [t];
    
    v = roverPath.x;

    %v(:) = vel;
    v(:) = vel*sign(arcL);
    %v(end) = 0;
    roverPath.v = v;
end

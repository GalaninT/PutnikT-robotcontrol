function stateWatcher(P,V,A,W,Acc,feedbackFR,feedbackMR,feedbackML,feedbackFL,reactionForces,elapsedTime,T_D)
    global rover

    rover.state.elapsedTime = elapsedTime;
    rover.state.bodyPosition = P;
    rover.state.bodySpeed = V;
    rover.state.bodyOrientation = A;
    rover.state.bodyAngularSpeed = W;


    actualSpeed = zeros(4,4);
    actualPosition = zeros(4,4);
    iSpeed = zeros(4,4);

    P = 3;
    L = 4; J = 4;
    feedback = [feedbackFR';
        feedbackMR';
        feedbackML';
        feedbackFL'];   

    for lInd = 1 : L
        for jInd = 1 : J
            actualPosition(lInd,jInd) = feedback(lInd,(jInd-1)*P+1);
            actualSpeed(lInd,jInd) = feedback(lInd,(jInd-1)*P+2);
        end
    end    

    offset = [0 pi 0 0;
            pi -pi 0 0;
            pi -pi 0 0
            0 pi 0 0];    
    
    rover.state.position = actualPosition+offset;
    rover.state.speed = actualSpeed;
    rover.state.reactionForce = reactionForces;

    r = [];
    N = length(rover.legSet);
    Q = rover.state.position;
    angles = rover.state.bodyOrientation';

    for lInd = 1 : N
        treeBaseName = strcat('b',rover.legSet{lInd},'1');
        leg = subtree(rover.tree,treeBaseName); 
   
        q = Q(lInd,:);        
        pointName = strcat('b',rover.legSet{lInd},'3');
        T = getTransform(leg,q,pointName);
        rInd = T(1:3,4)'; 
        r = [r; rInd];       
    end
    
    [n, d] = rover.FindPlane(r); %n is the plane normal / abc
    rover.state.contactPts = rover.FindContactPoints(r,n,rover.rWheel);

    rover.state.forceIntegral = rover.state.forceIntegral + rover.state.reactionForce*T_D;
    %rover.state.speedIntegral = rover.state.speedIntegral + actualSpeed*T_D;

                    for lInd = 1 : L
                        treeBaseName = strcat('b',rover.legSet{lInd},'1');
                        leg = subtree(rover.tree,treeBaseName); 
            
                        q = actualPosition(lInd,:);
                        qd = actualSpeed(lInd,:);

                        pointName = strcat('b',rover.legSet{lInd},'3');                    
                        Jtotal = geometricJacobian(leg,q,pointName);
                        %J = Jtotal([4 6],1:2);

                        vLeg = Jtotal*qd';
                        rover.state.speedIntegral(lInd,:) = vLeg(:);                       
                    end    
    
    rover.state.isSet = 1;

end
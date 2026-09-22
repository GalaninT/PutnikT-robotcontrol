function [output] = roverController(t,feedbackFR,feedbackMR,feedbackML,feedbackFL,reactionForces,WLRover)
%roverController Summary of this function goes here
%   Output - joint torques, [hip,knee,steer,wheel] - for all legs, in order
%   [FR,RR,RL,FL]
J = 4; L = 4;
N = J*L;
output = zeros(1,N);
% T = 0;
% output(4) = T;
% output(8) = T;
% output(12) = T;
% output(16) = T;

phi = 0;
qDes = [phi pi-phi 0 0;
        (pi-phi) -(pi-phi) 0 0;
        (pi-phi) -(pi-phi) 0 0;
        phi pi-phi 0 0]; %FR

qdDes = zeros(4,4);
if (t>0.05)
    qdDes = [0 0 0 1;
         0 0 0 1;
         0 0 0 0;
         0 0 0 0];
end

actualSpeed = zeros(4,4);
actualPosition = zeros(4,4);

u_sign = [1 1 1 1;
        1 1 1 1;
        1 1 1 1;
        1 1 1 1];

P = 3;
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
S = [1 1 1 1;
     1 1 1 1;
     1 1 1 1;
     1 1 1 1];
% 
% actualPosition = deg2rad(actualPosition).*S+offset;
% actualSpeed = deg2rad(actualSpeed).*S;

actualPosition = (actualPosition).*S+offset;
actualSpeed = (actualSpeed).*S;

% p = actualPosition
% v = actualSpeed

% type = 'Zero';
type = 'ID';
tauRequired = zeros(L,J);

switch type
    case 'Zero'
        
    case 'ID'
        Kp = ones(L,J)*2000;
%         Kp(:,3) = 5; %steer
        Kp(:,4) = 0; %wheels
        Kd = ones(L,J)*100;  
%         Kd(:,3) = 3000;  
        Kd(:,4) = 100;  
        disp('next')

        for lInd = 1 : L
            treeBaseName = strcat('b',WLRover.legSet{lInd},'1');
            leg = subtree(WLRover.tree,treeBaseName); 
            leg.Gravity = WLRover.tree.Gravity;

            q = actualPosition(lInd,:);
            qd = actualSpeed(lInd,:);
            desiredAcceleration = Kp(L,:).*(qDes(lInd,:)-q)+Kd(L,:).*(qdDes(lInd,:)-qd);

            fInd = (lInd-1)*3+1;
            wrench = reactionForces(fInd:fInd+2)'
            wrench = [0 0 0 wrench];
            wheelName = strcat('Wh',WLRover.legSet{lInd});
            fExt = externalForce(leg,wheelName,wrench,q);

            T = inverseDynamics(leg,q,qd,desiredAcceleration,fExt);
%             T = T.*u_sign
            
            tauRequired(lInd,:) = T(:);    
        end
end
% disp('T')
% disp(tauRequired)
output = reshape(tauRequired',[1 L*J]);

end
classdef FullMotionTask < Task
    %   FullMotionTask Summary of this class goes here
    %   Detailed explanation goes here

    properties
        totalStateDes
        totalStateInit
        totalStep
        qInit

        kPath
        kV
        kVe
        kPhi
        kPi
        kE
        vLim
        accLim
        deadSpeed        
        kF
        fDes
        vFLim
        fDeadScale
        s

        path
        pathType
        posture
        lastKeyPointInd
    end

    methods
        function obj = FullMotionTask(controlledVars,varsValue,minTime,lineType)
            %   FullMotionTask Construct an instance of this class
            %   Detailed explanation goes here
            obj.moveType = 'FullMotion';
            obj.varsValue = varsValue{1};
            obj.minTime = minTime;
            obj.pathType = lineType{1};
            obj.lastKeyPointInd = 1;

            %check!
            obj.kPi = 100;           %Proportional error coefs
            obj.kE = 5;
            obj.Kd = 50;            %ID coeffs
            obj.Kp = 1000;

            obj.kPath = 2;
            obj.kVe = 1;
            obj.vLim = 1;
            obj.accLim = 100; 
            obj.kPhi = 3;
            obj.deadSpeed = 0.02;
            
            obj.kF = 0.001;
            obj.vFLim = 0.03;    
            obj.fDes = -45;  
            obj.fDeadScale = 1/3;
            
            obj.endTolerance = [0.01 0.0015];  %path error  - posture error
            
            obj.allSet = 0;
        end

        function torque = FindControl(obj,rover,tcur)
            %   FindControl Summary of this method goes here
            %   Detailed explanation goes here
            global vDesLog vActualLog wDesLog tLog

            state = rover.state;
            actualPosition = state.position;
            actualSpeed = state.speed;
            F = -reshape(rover.state.reactionForce,[3,4])'; 

            LN = size(actualSpeed,1);
            JN = size(actualSpeed,2);
            tauRequired = zeros(LN,JN);
            torque = tauRequired;          
            
            %first iteration - preparation
            if ~obj.allSet
                obj.PrepareTask(rover);
            end

            %computed part
            %path part
            [keyPointInd,vActual] = obj.path.FindClosestPointInd(state,obj.lastKeyPointInd);
            vDes = obj.FindNominalSpeed(keyPointInd,vActual);

            if (keyPointInd == length(obj.path.x))
                %special case for the last point
                vDes = obj.FindLimitedSpeed(vDes,keyPointInd,state);                       
            end
            %find desired speed value with P-feedback
            vDes = vDes + obj.kVe.*(vDes-vActual);                 
            
            %find arc parameters, steer angles and corrected steer angles
            [R,O] = obj.path.FindArcByTwoPoints(keyPointInd,state.bodyPosition,obj.s);                    
            wWheelDes = rover.FindWheelDesiredSpeed(vDes,O); 
            [deltaDes,rotationSign] = rover.FindDeltaDesired(O); 
            deltaDes = obj.CorrectForHeading(deltaDes,state,O,rotationSign);

            %posture part
            p = rover.GetTotalState();
            pathProgress = obj.path.FindProgress(state.bodyPosition); %per cent
            pDes = obj.totalStateInit + pathProgress*obj.totalStep;
            qdDes = obj.FindDesiredJointsSpeed(pDes,p,rover); 

            wWheelDes = qdDes(:,4) + wWheelDes(:);                    
            wWheelDes = obj.BalanceWheelSpeed(wWheelDes);
            qdDes(:,4) = wWheelDes(:);

            qDes = obj.qInit;
            qDes(:,3) = deltaDes(:);

            %actual controller
            controllerType = 'HybridTraj';
            switch controllerType
                case 'Zero'
                    %nothing to do
                case {'ID';'IDTraj'}
%                     wDes = qdDes(:,4)'; %for log
% 
%                     for lInd = 1 : LN
%                         treeBaseName = strcat('b',rover.legSet{lInd},'1');
%                         leg = subtree(rover.tree,treeBaseName); 
%                         leg.Gravity = rover.tree.Gravity;
%             
%                         q = actualPosition(lInd,:);
%                         qd = actualSpeed(lInd,:);
% 
%                         desiredAcceleration = obj.Kd(lInd,:).*(qdDes(lInd,:)-qd)+obj.Kp(lInd,:).*(qDes(lInd,:)-q);
%             
%                         fInd = (lInd-1)*3+1;
%                         wrench = rover.state.reactionForce(fInd:fInd+2)';
%                         wrench = [0 0 0 wrench];
%                         wheelName = strcat('Wh',rover.legSet{lInd});
%                         fExt = externalForce(leg,wheelName,wrench,q);
%             
%                         %T = inverseDynamics(leg,q,qd,desiredAcceleration,fExt);
%                         T = inverseDynamics(leg,q,qd,desiredAcceleration);
%                         Tmax = [200 100 50 50];
%                         Tmin = -Tmax;
%                         T = min(T,Tmax);
%                         T = max(T,Tmin);                        
%                         tauRequired(lInd,:) = T(:);                         
%                     end                    
                case 'HybridTraj'  
                    wDes = qdDes(:,4)'; %for log
                    qdDesTotal = zeros(LN,JN);

                    for lInd = 1 : LN
                        treeBaseName = strcat('b',rover.legSet{lInd},'1');
                        leg = subtree(rover.tree,treeBaseName);           
                        q = actualPosition(lInd,:);

                        pointName = strcat('b',rover.legSet{lInd},'3');
                        Jtotal = geometricJacobian(leg,q,pointName);
                        J = Jtotal([4 6],1:2);

                        f = F(lInd,[1 3])';
                        fDesV = [0; obj.fDes];
                        dF = (fDesV - f);
                        if abs(dF(2)) < abs(obj.fDes*obj.fDeadScale)
                            vDesFZ = 0;
                        else
                            vDesF = obj.kF*dF;
                            vDesFZ = vDesF(2);
                            vDesFZ = max(vDesFZ,-obj.vFLim);
                            vDesFZ = min(vDesFZ,obj.vFLim);
                        end
                        
                        vDesF = [0;vDesFZ];
                        qdDesF = J\vDesF;
                        qdDesF = [qdDesF' 0 0];
                        qdDesJ = qdDes(lInd,:);
                        qdDesJ = qdDesJ + qdDesF;
                        qdDesTotal(lInd,:) = qdDesJ(:);                       
                    end 
            
                    withForce = 0;
                    tauRequired = rover.SolveIDController(qDes,qdDesTotal,obj.Kp,obj.Kd,withForce);                    
                              
            end    
            obj.lastKeyPointInd = keyPointInd;
            torque = reshape(tauRequired',1,[]);
%             vDesLog = [vDesLog vDes];
%             vActualLog = [vActualLog Vactual];
%             wDesLog = [wDesLog; wDes];
%             tLog = [tLog tcur];            
        end

        function SetInitialState(obj,state)
            obj.initialState = state;
        end

        function vDes = FindNominalSpeed(obj,keyPointInd,Vactual)
            global T_D
            keyPoint = obj.path.GetPoint(keyPointInd);
            vDes = keyPoint.v;
            if vDes*Vactual < 0 
                return;
            else
                vDes = min([vDes,Vactual+obj.accLim*T_D]); 
                vDes = max([vDes,Vactual-obj.accLim*T_D]);                
            end
        end       
  
        function vDes = FindLimitedSpeed(obj,vDes,keyPointInd,state)
            keyPoint = obj.path.GetPoint(keyPointInd);
            error = [keyPoint.x-state.bodyPosition(1) keyPoint.y-state.bodyPosition(2)];
            phi = state.bodyOrientation(3);
            v1 = [cos(phi) sin(phi)];
            errAlong = dot(v1,error);
            r = norm(errAlong);
            vDesLeft = obj.kPath*r;                        
            vDesNorm = min([obj.vLim,norm(vDes),vDesLeft]); 

            %нужно учесть направление движения, чтобы не уехать от точки                        
            sV = sign(errAlong);
            vDes = vDesNorm*sV; 
        end        

        function wWheelDes = BalanceWheelSpeed(obj,wWheel)
            wWheelDes = wWheel;
            wF = wWheel(1) + wWheel(4);
            wR = wWheel(2) + wWheel(3);
            wM = (wF+wR)/2;
            if abs(wM) > 0.01 
                ratF = 1+(wF-wM)/wM;
                wWheelDes(1) = wWheel(1)/ratF;
                wWheelDes(4) = wWheel(4)/ratF;
                ratR = 1+(wR-wM)/wM;
                wWheelDes([2 3]) = wWheel([2 3])/ratR;
            end 
        end

        function deltaDes = CorrectForHeading(obj,deltaDes,state,O,rotationSign)
            phi = state.bodyOrientation(3);
            V = state.bodySpeed;
            Vactual = dot(V(1:2),[cos(phi) sin(phi)]);                    
            sL = sign(Vactual); %actual motion direction (forward/backward)

            r = state.bodyPosition(1:2)-O;
            phiR = atan2(r(2),r(1));
            phiDesGross = phiR+rotationSign*pi/2;

            phiDes = wrapToPi(phiDesGross);
            phiError = phiDes - phi;
            %FR RR RL FL  %make 
            K = [0.5 -1 -1 0.5]*obj.kPhi*sL;
            if abs(Vactual) < obj.deadSpeed
                K(:) = 0;
            end
            deltaDes = deltaDes + K.*phiError;             
        end 

        function qdDes = FindDesiredJointsSpeed(obj,pDes,p,rover)
            [L,J] = rover.FindRoverMatrices();
            v = rover.state.speed;
            LN = size(v,1);
            JN = size(v,2);
            K = [ones(1,6)*obj.kPi ones(1,4)*obj.kE];
%             Kpi = obj.kPi*eye(length(pDes));
            Kpi = diag(K);
            ksiDot = Kpi*(pDes - p);
            ksiDot([1 2 6]) = 0;
            u = -(J\L)*ksiDot;
            u = reshape(u,[3 4])';
            qdDes = zeros(LN,JN);
            kW = 0.1;
            for lInd = 1 : LN
                qdDes(lInd,1) = u(lInd,1);
                qdDes(lInd,2) = u(lInd,2);
                qdDes(lInd,4) = u(lInd,3)*kW;
            end             
        end        

        function PrepareTask(obj,rover)
            state = rover.state;
            obj.qInit = obj.initialState.position;
            LN = size(obj.qInit,1);
            JN = size(obj.qInit,2);                

            obj.Kd = ones(LN,JN)*obj.Kd; %ID coeffs
            obj.Kp = ones(LN,JN)*obj.Kp;
            obj.Kp(:,3) = 200;
            obj.Kd(:,3) = 15;
            obj.Kp(:,4) = 0;   
            obj.Kd(:,4) = 200;  

            p0 = state.bodyPosition;
            p0(3) = state.bodyOrientation(3); %yaw

            NP = 10;
            switch obj.pathType
                case 'L'
                    alpha = obj.varsValue(1,4);
                    obj.path = CreateTrajectoryLine(p0,obj.varsValue(1,2),obj.varsValue(1,1),alpha,NP);
                case 'A'
                    obj.path = CreateTrajectoryArc(p0,obj.varsValue(1,3),obj.varsValue(1,2),obj.varsValue(1,1),NP);
            end

            obj.totalStateInit = rover.GetTotalState(); 
            step = zeros(size(obj.totalStateInit));
            step(3) = obj.varsValue(2,1);
            step(7:10) = obj.varsValue(2,4:7);
            stateDes = obj.totalStateInit + step;
            stateDes(4:5) = obj.varsValue(2,2:3);  

            stateDes(1) = obj.path.x(end);
            stateDes(2) = obj.path.y(end);
            
            obj.totalStateDes = stateDes;
            obj.totalStep = stateDes - obj.totalStateInit;
            
            obj.s = sign(obj.path.v(1));
            obj.allSet = 1;            
        end

        function isCompleted = Completed(obj,rover,t)
            isCompleted = 0;
            if obj.isCompleted
                isCompleted = 1;
                return;
            end 
            if ~obj.allSet
                return;
            end

            tTask = t - obj.tStart;
            if tTask < obj.minTime
                return;
            end

            p = rover.GetTotalState();           

            errorSize = obj.totalStateDes - p; 
            errorPosture = errorSize([3 4 5 7 8 9 10]);
            errPosture = norm(errorPosture);

            errorPath = errorSize([1 2]);
            phi = obj.path.phi(end);

            %нужно учесть направление движения, чтобы не уехать от точки
            v1 = [cos(phi) sin(phi)];
            errorPath = dot(v1,errorPath);                            
            errPath = norm(errorPath);

            if errPath < obj.endTolerance(1) && errPosture < obj.endTolerance(2)
                disp('next')
                obj.isCompleted = 1;
                isCompleted = 1;
            end 
        end

    end
end
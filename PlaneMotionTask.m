classdef PlaneMotionTask < Task
    %PlaneMotionTask Summary of this class goes here
    %   Detailed explanation goes here

    properties
        totalStateDes
        qInit

        kV
        kPath
        kVe
        kPhi
        vLim
        accLim
        deadSpeed

        path
        pathType
        lastKeyPointInd
        s
    end

    methods
        function obj = PlaneMotionTask(controlledVars,varsValue,minTime,lineType)
            %   PlaneMotionTask Construct an instance of this class
            %   Detailed explanation goes here
            obj.moveType = 'PlaneMotion';
            obj.varsValue = varsValue{1};
            obj.minTime = minTime;
            obj.pathType = lineType{1};
            obj.lastKeyPointInd = 1;

            %controlledVars not used yet - now we control position in trajectory mode

            %check!
            obj.kV = 20;            %Proportional error coefs
            obj.Kd = 200;           %ID coeffs
            obj.Kp = 3000;     

            obj.kPath = 2;
            obj.kVe = 1;
            obj.vLim = 1;              
            obj.accLim = 25; 
            obj.kPhi = 3;
            obj.deadSpeed = 0.05;

            obj.endTolerance = [0.01 15];  
            obj.isCompleted = 0;
            obj.allSet = 0;
        end

        function [torque] = FindControl(obj,rover,tcur)
            % global vDesLog vActualLog wDesLog tLog
            % FindControl Summary of this method goes here
            % Detailed explanation goes here

            state = rover.state;
            actualSpeed = state.speed;
            LN = size(actualSpeed,1);
            JN = size(actualSpeed,2);
            tauRequired = zeros(LN,JN);
            torque = tauRequired;          
            
            %first iteration - preparation
            if ~obj.allSet
                obj.PrepareTask(state);
            end  

            %actual controller
            controllerType = 'ID';
            switch controllerType
                case 'Zero'
                    
                case {'ID';'IDTraj'}
                    [keyPointInd,vActual] = obj.path.FindClosestPointInd(state,obj.lastKeyPointInd);
                    vDes = obj.FindNominalSpeed(keyPointInd,vActual);

                    if (keyPointInd == length(obj.path.x))
                        vDes = obj.FindLimitedSpeed(vDes,keyPointInd,state);                 
                    end
                    vDes = vDes + obj.kVe.*(vDes-vActual);

                    [R,O] = obj.path.FindArcByTwoPoints(keyPointInd,state.bodyPosition,obj.s);                    
                    wWheelDes = rover.FindWheelDesiredSpeed(vDes,O); 
                    [deltaDes,rotationSign] = rover.FindDeltaDesired(O); 

                    deltaDes = obj.CorrectForHeading(deltaDes,state,O,rotationSign);

                    qdDes = zeros(LN,JN);                     
                    wWheelDes = obj.BalanceWheelSpeed(wWheelDes);
                    qdDes(:,4) = wWheelDes(:);

                    qDes = obj.qInit;
                    qDes(:,3) = deltaDes(:);

                    withForce = 0;
                    tauRequired = rover.SolveIDController(qDes,qdDes,obj.Kp,obj.Kd,withForce);
                case 'HybridTraj'  
                    %XXX
            end    
            obj.lastKeyPointInd = keyPointInd;
            torque = reshape(tauRequired',1,[]);

%             vDesLog = [vDesLog vDes];
%             vActualLog = [vActualLog vDes];
%             wDes = wWheelDes;
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

        function PrepareTask(obj,state)
            obj.qInit = obj.initialState.position;
            LN = size(obj.qInit,1);
            JN = size(obj.qInit,2);            

            obj.Kd = ones(LN,JN)*obj.Kd; %ID coeffs
            obj.Kp = ones(LN,JN)*obj.Kp;
            obj.Kp(:,3) = 500;
            obj.Kd(:,3) = 15;
            obj.Kp(:,4) = 0;   
            obj.Kd(:,4) = 200;  

            p0 = state.bodyPosition;
            p0(3) = state.bodyOrientation(3);       %yaw

            %make switch: Line, Arc, etc
            NP = 10;
            switch obj.pathType
                case 'L'
                    alpha = 0;
                    obj.path = CreateTrajectoryLine(p0,obj.varsValue(1,2),obj.varsValue(1,1),alpha,NP);
                case 'A'
                    obj.path = CreateTrajectoryArc(p0,obj.varsValue(1,3),obj.varsValue(1,2),obj.varsValue(1,1),NP);
            end

            obj.totalStateDes = [obj.path.x(end) obj.path.y(end)];
            obj.s = sign(obj.path.v(1));
            obj.allSet = 1;            
        end

        function isCompleted = Completed(obj,rover,t)
            isCompleted = 0;

            if ~obj.allSet
                return;
            end

            if obj.isCompleted
                isCompleted = 1;
                return;
            end            

            tTask = t - obj.tStart;
            if tTask < obj.minTime
                return;
            end

            p = rover.GetTotalState(); 
            error = obj.totalStateDes - p(1:2)'; 
            
            %нужно учесть направление движения, чтобы не уехать от точки
            phi = obj.path.phi(end);
            v1 = [cos(phi) sin(phi)];
            errorSize = dot(v1,error);             
                            
            tol = obj.endTolerance(1);
            %tolForce = sqrt(L)*obj.endTolerance(2);
            err = norm(errorSize);

            if err < tol 
%                 disp('next')
                isCompleted = 1;
                obj.isCompleted = 1;                
            end 
        end

    end
end
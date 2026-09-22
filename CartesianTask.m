classdef CartesianTask < Task
    %   CartesianTask Summary of this class goes here
    %   Detailed explanation goes here

    properties
        qInit
        legStep
        pDes
        kV
        kVe

        Tt
        gamma
        s
        KpJ3
        KdJ3

        fDes
        kF 
        kFi
        kFv
        sf

        traj
        trajd
        trajT

        tact
        forceTact
        LD
        muX
        g
        dm
        vDesIntegral
        fDesIntegral
    end

    methods
        function obj = CartesianTask(activeLeg,controlledVars,varsValue)
            %   CartesianTask Construct an instance of this class
            %   Detailed explanation goes here
            obj.moveType = 'Cartesian';
            obj.activeLeg = activeLeg;
            lIndArray = obj.GetLegIndex();
            obj.controlledVars = cell(1,4);
            for lInd = 1 : length(obj.controlledVars)
                ind = find(lIndArray==lInd);
                if isempty(ind)
                    obj.controlledVars(lInd) = {'P'};
                else
                    obj.controlledVars(lInd) = controlledVars{ind};
                end
            end
            %obj.controlledVars = controlledVars;
            obj.varsValue = varsValue;
            obj.paramsValue = [];    
            obj.controllerType = 'IDTraj';

            obj.kV = 10;
            obj.kVe = 0.5;            
            obj.Kp = 2000;
            obj.Kd = 100;
            obj.KpJ3 = 1000;
            obj.KdJ3 = 50;

            obj.Tt = [0.05 0.05 0.02 0.1 0.1 0.1];
            obj.gamma = eye(6)*5;    
            obj.s = [0 0 0 1 0 1];

            obj.sf = [0 0 0 0 0 1];
            obj.kF = 0.2;
            obj.kFi = 5;
            obj.kFv = 200;

            obj.vDesIntegral = zeros(4,6);
            obj.fDesIntegral = zeros(4,3);
            obj.endTolerance = [0.005 0.005 15];
            obj.tact = 0;
            obj.forceTact = 0;
            obj.isCompleted = 0;
            obj.allSet = 0;
        end

        function torque = FindControl(obj,rover,tcur)
            %FindControl Summary of this method goes here
            %Detailed explanation goes here

            actualSpeed = rover.state.speed;
            L = size(actualSpeed,1);
            J = size(actualSpeed,2);
            tauRequired = zeros(L,J);
            torque = tauRequired;        
            
            %first iteration - preparation
            if ~obj.allSet
                obj.PrepareTask(rover);
            end
                   
            global T_D
            %actual controller        
            for lInd = 1 : L   
                mode = obj.controlledVars{lInd};
                switch mode
                    case 'P'
                        finalControllerType = obj.controllerType;
                    case 'H'
                        finalControllerType = 'CartHybrid';
                end                  

                [pActual,vActual,J] = rover.FindLegMotion(lInd);
                pDesLeg = obj.pDes(lInd,:)';
                qDes = obj.qInit(lInd,:);

                switch finalControllerType
                    case 'Zero'
                        %nothing to change
                    case 'ID'    
                        %find control for the leg
                        withForce = 0;                             
                        
                        vDes = obj.kV*(pDesLeg-pActual);
                        vDes = vDes + obj.kVe*(vDes - vActual);

                        qdDes = J\vDes;
                        qdDes = [qdDes' 0 0];
                        
                        T = rover.SolveLegID(lInd,qDes,qdDes,obj.Kp(lInd,:),obj.Kd(lInd,:),withForce);

                    case 'IDTraj'
                        %find control for the leg
                        withForce = 0;  

                        [r,v] = obj.GetLegTrajectoryPoint(tcur,lInd);
                        vDes = v+obj.kV*(r-pActual);                       
                        vDes = vDes + obj.kVe*(vDes - vActual);
                        qdDes = J\vDes;
                        qdDes = [qdDes' 0 0];                        

                        T = rover.SolveLegID(lInd,qDes,qdDes,obj.Kp(lInd,:),obj.Kd(lInd,:),withForce);   
                    case 'CartImpV'
                        %find control for the leg
                        withForce = 0;  

                        [r,v] = obj.GetLegTrajectoryPoint(tcur,lInd); 

                        vDes = v+obj.kV*(r-pActual);                       
                        vDes = vDes + obj.kVe*(vDes - vActual);
                        vDes6 = [0 0 0 vDes(1) 0 vDes(2)];
                        iVDesStep = vDes6*T_D; 
                        iVDes = obj.vDesIntegral(lInd,:)+iVDesStep;      
                        obj.vDesIntegral(lInd,:) = iVDes(:);

                        if ~obj.tact
                            obj.FindLegDynamics(rover,lInd);
                        end 

                        legDyn.L = obj.LD{lInd};
                        legDyn.muX = obj.muX{lInd};
                        legDyn.g = obj.g{lInd};
                        legDyn.dm = obj.dm{lInd};

                        T = rover.SolveLegIDCartesian(lInd,legDyn,vDes,iVDes,obj.Tt,obj.gamma,obj.s);

                        qdDes = [0 0 0 0];
                        obj.Kp(lInd,3) = obj.KpJ3;
                        obj.Kd(lInd,3) = obj.KdJ3;
                        Tq = rover.SolveLegID(lInd,qDes,qdDes,obj.Kp(lInd,:),obj.Kd(lInd,:),withForce);   
                        T(3:4) = Tq(3:4);

                    case 'CartHybrid'
                        %find control for the leg
                        withForce = 0;  

                        [r,v] = obj.GetLegTrajectoryPoint(tcur,lInd); 
                        obj.Kp(lInd,3) = 1000;
                        obj.Kd(lInd,3) = 50;

                        vDes = v+obj.kV*(r-pActual);                       
                        vDes = vDes + obj.kVe*(vDes - vActual);
                        vDes6 = [0 0 0 vDes(1) 0 vDes(2)];
                        iVDesStep = vDes6*T_D; 
                        iVDes = obj.vDesIntegral(lInd,:)+iVDesStep;      
                        obj.vDesIntegral(lInd,:) = iVDes(:);                        

                        Fd = obj.fDes(lInd);
                        fDes3 = [0;0;Fd];
                        iFDesStep = fDes3'*T_D; 
                        iFDes = obj.fDesIntegral(lInd,:)+iFDesStep;      
                        obj.fDesIntegral(lInd,:) = iFDes(:);

                        if ~obj.tact
                            obj.FindLegDynamics(rover,lInd);
                        end 

                        legDyn.L = obj.LD{lInd};
                        legDyn.muX = obj.muX{lInd};
                        legDyn.g = obj.g{lInd};
                        legDyn.dm = obj.dm{lInd};

                        KF.kF = obj.kF;
                        KF.kFi = obj.kFi;
                        KF.KFv = obj.KFv;
                        T = SolveLegIDCartesianHybrid(rover,lInd,legDyn,vDes,iVDes,fDes3,iFDes,obj.Tt,obj.gamma,obj.s,obj.sf,KF);

                        qdDes = [0 0 0 0];
                        Tq = rover.SolveLegID(lInd,qDes,qdDes,obj.Kp(lInd,:),obj.Kd(lInd,:),withForce);   
                        T(3:4) = Tq(3:4);
                end
                tauRequired(lInd,:) = T(:);
            end              
            torque = reshape(tauRequired',1,[]);

            obj.tact = obj.tact+1;
            if obj.tact > 100
                obj.tact = 0;
            end            
        end

        function SetInitialState(obj,state)
            obj.initialState = state;
        end

        function PrepareTask(obj,rover)
            params = obj.paramsValue;
            obj.qInit = obj.initialState.position;
            if ~isempty(params)
                obj.kV = params(1); 
            end
            L = size(obj.qInit,1);
            J = size(obj.qInit,2);            
            obj.Kp = ones(L,J)*obj.Kp;
            obj.Kd = ones(L,J)*obj.Kd;    
            obj.Kp(:,3) = 500;
            obj.Kd(:,3) = 30;           

            obj.legStep = zeros(L,2);
            obj.fDes = zeros(L,1);

            tmax = obj.minTime;
            tpts = [0 tmax];
            timeStamps = 0:0.01:tmax;
            obj.traj = cell(1,L);
            obj.trajd = cell(1,L);
            obj.trajT = cell(1,L);

            lIndArray = obj.GetLegIndex();               

            %set variable values and parameters (if needed)
            for ind = 1 : length(lIndArray)
                lInd = lIndArray(ind);                   
                values = obj.varsValue{ind};   
                mode = obj.controlledVars{lInd};
                
                obj.Kp(lInd,1:2) = 0;               %active legs work in velocity mode
                switch mode
                    case 'P'
                        obj.legStep(lInd,:) = values(:);    %x z
                    case 'H'
                        obj.legStep(lInd,:) = values(:);    %x z(not used)
                        obj.fDes(lInd) = values(2);
                end                     
            end

            %find trajectories (PVT) for moving legs TCPs
            obj.pDes = zeros(size(obj.legStep));
            for lInd = 1 : L    
                pInit = rover.FindLegMotion(lInd);    
                pDesLeg = pInit+obj.legStep(lInd,:)';
                obj.pDes(lInd,:) = pDesLeg;

                wpts = [pInit, pDesLeg];
                [r,v,acc,pp] = quinticpolytraj(wpts, tpts, timeStamps);
                obj.traj(lInd) = {r};
                obj.trajd(lInd) = {v};
                obj.trajT(lInd) = {timeStamps};                                 
            end

            obj.allSet = 1;            
        end

        function [pDes,vDes] = GetLegTrajectoryPoint(obj,tcur,lInd)
            TR = obj.traj{lInd};
            TV = obj.trajd{lInd};
            timeStamps = obj.trajT{lInd};
            t = tcur - obj.tStart;
            pDes = interp1(timeStamps,TR',min(t,timeStamps(end)))';
            vDes = interp1(timeStamps,TV',min(t,timeStamps(end)))';
        end

        function FindLegDynamics(obj,rover,lInd)
            wheelName = strcat('Wh',rover.legSet{lInd});
            treeBaseName = strcat('b',rover.legSet{lInd},'1');
            leg = subtree(rover.tree,treeBaseName); 

            q = rover.state.position(lInd,:);  
            qd = rover.state.speed(lInd,:);  
            Jleg = geometricJacobian(leg,q,wheelName);
            Jinv = pinv(Jleg); 

            M = massMatrix(leg,q);  %RST
            obj.LD{lInd} = Jinv'*M*Jinv;

            c = velocityProduct(leg,q,qd);
            obj.muX{lInd} = Jinv'*c';
            obj.g{lInd} = gravityTorque(leg,q)';                
            %dm = FindRepulsiveTorque(leg,actualPosition)'
            obj.dm{lInd} = zeros(size(obj.g{lInd}))'; %XXX            
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

            lIndArray = obj.GetLegIndex();
            L = length(lIndArray);
            taskSuccess = 1;
            for ind = 1 : L                
                taskSuccess = obj.CheckLegSuccess(rover,ind);
                if taskSuccess == 0
                    break;
                end
            end                       

            if taskSuccess 
                disp('next')
                obj.isCompleted = 1;
                isCompleted = 1;
            end 
        end

        function taskSuccess = CheckLegSuccess(obj,rover,ind)
            taskSuccess = 1;
            lIndArray = obj.GetLegIndex();
            lInd = lIndArray(ind);
            pActual = rover.FindLegMotion(lInd);    
            F = -reshape(rover.state.reactionForce,[3,4])';
            fActual = F(lInd,3);

            S = eye(2);
            if obj.controlledVars{lInd} == 'H'
                S(end) = 0;
                
                forceError = norm(obj.fDes(lInd)-fActual);
                if abs(forceError) > obj.endTolerance(3)
                    taskSuccess = 0;
                    obj.forceTact = 0;
                    return;
                else
                    obj.forceTact = obj.forceTact+1;
                    if obj.forceTact < 3
                        taskSuccess = 0;
                        return;
                    end
                end
            end

            pDesLeg = obj.pDes(lInd,:)';
            cartesianErrorR = S*(pDesLeg-pActual);
            cartesianError = norm(cartesianErrorR);

            q = rover.state.position(lInd,:); 
            qInitLeg = obj.qInit(lInd,:);
            jointError = norm(q(3)-qInitLeg(3));
            if abs(cartesianError) > obj.endTolerance(1) || abs(jointError) > obj.endTolerance(2)
                taskSuccess = 0;
            end            
        end
    end
end
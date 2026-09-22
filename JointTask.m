classdef JointTask < Task
    %   JointTask Summary of this class goes here
    %   Detailed explanation goes here

    properties
        qDes
        qdDes

        traj
        trajd
        trajT
    end

    methods
        function obj = JointTask(activeLeg,controlledVars,varsValue,minTime)
            %JointTask Construct an instance of this class
            %   Detailed explanation goes here
            obj.moveType = 'Joint';
            obj.activeLeg = activeLeg;
            obj.controlledVars = controlledVars;
            obj.varsValue = varsValue;
            obj.minTime = minTime;

            obj.paramsValue = [];
            %obj.controllerType = 'ID';
            obj.controllerType = 'IDTraj';

            obj.Kp = 5000;
            obj.Kd = 200; 

            obj.Kp = 10000;
            obj.Kd = 300;             

            obj.isCompleted = 0;
            %obj.endTolerance = 0.015; 
            obj.endTolerance = 0.05; %XXX
            obj.allSet = 0;
        end

        function torque = FindControl(obj,rover,t)
            %   FindControl Summary of this method goes here
            %   Detailed explanation goes here

            L = rover.numberOfLegs;
            J = length(obj.controlledVars{1}); %only for the joint task
            tauRequired = zeros(L,J);
            torque = tauRequired;
            
            %first iteration - preparation
            if ~obj.allSet
                obj.PrepareTask(L,J);
            end
                   
            %actual controller
            switch obj.controllerType
                case 'Zero'
                    %already zeros
                case 'ID'
                    withForce = 0;
                    tauRequired = rover.SolveIDController(obj.qDes,obj.qdDes,obj.Kp,obj.Kd,withForce);
                case 'IDTraj'
                    withForce = 0;
                    [qDesT,qdDesT] = obj.GetJointTrajectoryPoint(t);
                    obj.Kp(:,3) = 1000; %XXX
                    obj.Kd(:,3) = 50;
                    tauRequired = rover.SolveIDController(qDesT,qdDesT,obj.Kp,obj.Kd,withForce);
            end  
            torque = reshape(tauRequired',1,[]);
        end

        function SetInitialState(obj,state)
            obj.initialState = state;
        end

        function PrepareTask(obj,L,J)
            obj.qDes = obj.initialState.position;
            obj.qdDes = zeros(size(obj.qDes));
            obj.Kp = ones(L,J)*obj.Kp;
            obj.Kd = ones(L,J)*obj.Kd; 
            lIndArray = obj.GetLegIndex();
            jModes = obj.controlledVars;
            params = obj.paramsValue;

            %for trajectory
            tmax = obj.minTime;
            tpts = [0 tmax];
            timeStamps = 0:0.01:tmax;
            obj.traj = cell(L,J);
            obj.trajd = cell(L,J);
            obj.trajT = cell(L,J);

            obj.FillJointTrajectoryInitial(L,J,tmax);

            for ind = 1 : length(lIndArray)
                lInd = lIndArray(ind);
                values = obj.varsValue{ind};
                thisLegJointModes = jModes{1};
                
                for jInd = 1 : J
                    jointMode = thisLegJointModes{jInd};
                    qInit = obj.initialState.position(lInd,jInd);

                    switch jointMode
                        case '-'
                            %nothing to change
                        case 'P'
                            obj.qDes(lInd,jInd) = values(1,jInd);
                            obj.qdDes(lInd,jInd) = values(2,jInd);
                            if ~isempty(params)
                                obj.Kp(lInd,jInd) = params(1,jInd);
                                obj.Kd(lInd,jInd) = params(2,jInd);
                            end
                            
                            wpts = [qInit, obj.qDes(lInd,jInd)];
                            [r,v,acc,pp] = quinticpolytraj(wpts, tpts, timeStamps);
                            obj.traj(lInd,jInd) = {r};
                            obj.trajd(lInd,jInd) = {v};
                            obj.trajT(lInd,jInd) = {timeStamps};

                        case 'V'
                            obj.qdDes(lInd,jInd) = values(2,jInd);
                            obj.Kp(lInd,jInd) = 0;
                            if ~isempty(params)
                                obj.Kd(lInd,jInd) = params(2,jInd);                        
                            end

                            %position ignored by Kp
                            r = timeStamps;
                            r(:) = qInit;
                            v = timeStamps;
                            v(:) = obj.qdDes(lInd,jInd);
                            obj.traj(lInd,jInd) = {r};
                            obj.trajd(lInd,jInd) = {v};
                            obj.trajT(lInd,jInd) = {timeStamps};
                    end                
                end
            end
            obj.allSet = 1;            
        end

        function FillJointTrajectoryInitial(obj,L,J,tmax)
            tpts = [0 tmax];
            timeStamps = 0:0.01:tmax;            
            for lInd = 1 : L
                for jInd = 1 : J
                    qInit = obj.initialState.position(lInd,jInd);
                    wpts = [qInit, qInit];
                    [r,v,acc,pp] = quinticpolytraj(wpts, tpts, timeStamps);
                    obj.traj(lInd,jInd) = {r};
                    obj.trajd(lInd,jInd) = {v};
                    obj.trajT(lInd,jInd) = {timeStamps};
                end
            end
        end

        function [qDes,qdDes] = GetJointTrajectoryPoint(obj,tcur)
            L = size(obj.traj,1);
            J = size(obj.traj,2);
            qDes = zeros(L,J);
            qdDes = zeros(L,J);
            for lInd = 1 : L
                for jInd = 1 : J
                    TR = obj.traj{lInd,jInd};
                    TV = obj.trajd{lInd,jInd};
                    timeStamps = obj.trajT{lInd,jInd};
                    t = tcur - obj.tStart;
                    qDes(lInd,jInd) = interp1(timeStamps,TR',min(t,timeStamps(end)))';
                    qdDes(lInd,jInd) = interp1(timeStamps,TV',min(t,timeStamps(end)))';                    
                end
            end
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

            %only for the position mode
            %need to check speed mode
            state = rover.state;
            actualPosition = state.position;
            lInd = obj.GetLegIndex();
            errorSize = norm(obj.qDes(lInd,:)-actualPosition(lInd,:));

            if errorSize < obj.endTolerance
                isCompleted = 1;
                obj.isCompleted = 1;
                disp('next')
            end            
        end
    end
end
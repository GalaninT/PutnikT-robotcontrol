classdef HomeTask < Task
    %   HomeTask Summary of this class goes here
    %   Detailed explanation goes here

    properties
        qDes
        qdDes        
        subTasks
        currentSubTaskIndex

        qInit 
    end

    methods
        function obj = HomeTask(minTime)
            %   HomeTask Construct an instance of this class
            %   Detailed explanation goes here
            obj.moveType = 'Home';
            obj.minTime = minTime;
            obj.controllerType = 'IDTraj';

            phi = 0;
            qFR = [phi pi-phi 0 0];
            qRR = [(pi-phi) -(pi-phi) 0 0];
            qRL = [pi-phi -(pi-phi) 0 0];    
            qFL = [phi pi-phi 0 0];
            q = [qFR; qRR; qRL; qFL;];
            obj.qDes = q;
            obj.qdDes = zeros(size(q));

            obj.endTolerance = 0.03;
            obj.allSet = 0;

            obj.isCompleted = 0;
            obj.currentSubTaskIndex = 1;
        end

        function torque = FindControl(obj,rover,t)
            %   FindControl Summary of this method goes here
            %   Detailed explanation goes here
            if ~obj.allSet
                obj.PrepareTask(rover,t);
            end
            
            subTask = obj.GetCurrentSubTask();
            torque = subTask.FindControl(rover,t);
        end

        function SetStartTime(obj,t)
            obj.tStart = t;
            obj.subTasks{1}.tStart = t;
        end      

        function SetInitialState(obj,state)
            obj.initialState = state;
            obj.subTasks{1}.initialState = state;
        end

        function SetParameters(obj,params)
            %XXX
            if ~isempty(params)
                L = length(obj.subTasks);
                obj.kV = params(1); 
                for ind = 1 : L
                    obj.subTasks{ind}.paramsValue = [obj.kV];
                end
            end            
        end        

        function subTask = GetCurrentSubTask(obj)
            subTask = obj.subTasks{obj.currentSubTaskIndex};
        end

        function PrepareTask(obj,rover,t)
            activeLeg = {'FR','RR','RL','FL'};
            eachJointControlledVars = {'P','P','P','P'};

            state = rover.state;
            actualSpeed = state.speed;
            obj.qInit = state.position;            

            L = size(actualSpeed,1);  
            obj.qDes(:,4) = state.position(:,4);

            %set required home angles
            subVarsValue = cell(1,L);
            jointControlledVars = cell(1,L);
            for lInd = 1 : L                       
                subVarsValueJoint = [obj.qDes(lInd,:); obj.qdDes(lInd,:)];
                subVarsValue{lInd} = subVarsValueJoint;
                jointControlledVars{lInd} = eachJointControlledVars;
            end       
            
            mTime = obj.minTime;
            homeTask = JointTask(activeLeg,jointControlledVars,subVarsValue,mTime); 
            homeTask.controllerType = obj.controllerType;

            obj.subTasks{1} = homeTask;
            obj.SetInitialState(state);
            obj.SetStartTime(t);
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
        
            currentTask = obj.subTasks{obj.currentSubTaskIndex};
            if currentTask.Completed(rover,t)
                if obj.currentSubTaskIndex == length(obj.subTasks)
                    isCompleted = 1;
                    obj.isCompleted = 1;
                    return;
                end
                obj.currentSubTaskIndex = obj.currentSubTaskIndex+1;
                obj.subTasks{obj.currentSubTaskIndex}.tStart = t;
                obj.subTasks{obj.currentSubTaskIndex}.minTime = obj.minTime;
                obj.subTasks{obj.currentSubTaskIndex}.initialState = rover.state;
            end
        end
    end
end
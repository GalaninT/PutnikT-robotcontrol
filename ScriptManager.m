classdef ScriptManager < handle
    %ScriptManager Summary of this class goes here
    %   Detailed explanation goes here

    properties
        taskList
        currentTaskIndex
        currentTask
        started
        lastTaskFinished
    end

    methods
        function obj = ScriptManager()
            %ScriptManager Construct an instance of this class
            %   Detailed explanation goes here
            obj.taskList = table('Size',[0 6],'VariableTypes',{'categorical','cell','cell','double','cellstr','cell'},...
                'VariableNames',{'Type','VarType','VarValue','MinTime','ActiveLeg','Param'});
            obj.currentTaskIndex = 0;
            obj.started = 0;
            obj.lastTaskFinished = 0;
        end

        function AddTask(obj,varargin)
            %AddTask Summary of this method goes here
            %   Detailed explanation goes here
            type = varargin{1};
            varType = varargin{2};
            varValue = varargin{3};
            nominalTime = varargin{4};                       

            L = size(obj.taskList,1)+1;
            obj.taskList(L,1) = type;            
            obj.taskList(L,2) = {varType};
            obj.taskList(L,3) = {varValue};            
            obj.taskList(L,4) = {nominalTime};
            if nargin > 5
                leg = varargin{5}; 
                obj.taskList(L,5) = {leg};
            else
                obj.taskList(L,5) = {'-'};              
            end            
            if nargin > 6
                paramValue = varargin{6};
                obj.taskList(L,6) = paramValue;
            else
                obj.taskList(L,6) = {[]};
            end            
            
        end

        function task = GetCurrentTask(obj,t)
            %GetCurrentTask Summary of this method goes here

            if (obj.lastTaskFinished)
                task = obj.currentTask;
                return;
            end

            %   Detailed explanation goes here
            ind = obj.currentTaskIndex;
            c = obj.taskList(ind,2).VarType{1}
            controlledVars  = obj.taskList(ind,2).VarType;
            varsValue = obj.taskList(ind,3).VarValue{1};
            minTime = obj.taskList(ind,4).MinTime;
            paramsValue = obj.taskList(ind,6).Param{1}; 

            switch obj.taskList(ind,1).Type
                case 'Joint'
                    activeLeg  = obj.taskList(ind,5).ActiveLeg{1};
                    task = JointTask(activeLeg,controlledVars,varsValue,minTime);
                    task.tStart = t;
                    task.paramsValue = paramsValue; 
                case 'Home'
                    task = HomeTask(minTime);
                    task.tStart = t;                
                    task.paramsValue = paramsValue; 
                case 'Cartesian'
                    activeLeg  = obj.taskList(ind,5).ActiveLeg{1};
                    task = CartesianTask(activeLeg,controlledVars,varsValue);
                    task.tStart = t;
                    task.minTime = minTime;
                    task.paramsValue = paramsValue; 
                case 'SlideStep'
                    activeLeg  = obj.taskList(ind,5).ActiveLeg{1};
                    task = SlideStepTask(activeLeg,controlledVars,varsValue,minTime);
                    task.tStart = t;
                    task.paramsValue = paramsValue; 
                    
                case 'Step'
                    activeLeg  = obj.taskList(ind,5).ActiveLeg{1};
                    task = StepTask(activeLeg,varsValue);
                    task.SetStartTime(t); %remove to setting phase of control
                    task.SetMinTime(obj.taskList(ind,4).MinTime); %remove to setting phase of control
                    task.SetParameters(paramsValue);
                case 'SpotTurn'
                    task = SpotTurnTask(controlledVars,varsValue,minTime);      
                case 'Steer'
                    task = SteerTask(controlledVars,varsValue,minTime);                       

                case 'Posture'
                    task = PostureTask(controlledVars,varsValue,minTime);
                    task.tStart = t;
                case 'PlaneMotion'
                    lineType = obj.taskList(ind,5).ActiveLeg{1};
                    task = PlaneMotionTask(controlledVars,varsValue,minTime,lineType);
                    task.tStart = t;
                case 'FullMotion'
                    lineType = obj.taskList(ind,5).ActiveLeg{1};
                    task = FullMotionTask(controlledVars,varsValue,minTime,lineType);
                    task.tStart = t;
                case 'CrabLine'   
                    task = CrabLineTask(controlledVars,varsValue,minTime);
                    task.tStart = t;
                case 'Crab'   
                    task = CrabTask(controlledVars,varsValue,minTime);
                    task.tStart = t;                    
            end                           
                        
            obj.currentTask = task;
        end
                               
        function ProceedTask(obj,t)
            %ProceedTask Summary of this method goes here
            %   Detailed explanation goes here
            if obj.lastTaskFinished
                return;
            end

            disp(strcat('Task',num2str(obj.currentTaskIndex),'Finished, t = ',num2str(t)));
           
            if (obj.currentTaskIndex == size(obj.taskList,1))
                obj.lastTaskFinished = 1;
            end
             
            obj.currentTaskIndex = min(obj.currentTaskIndex+1,size(obj.taskList,1));
        end
                                      
        function StartScript(obj)
            %StartScript Summary of this method goes here
            %   Detailed explanation goes here
            obj.currentTaskIndex = 1;
            obj.started = 1;
        end

        function started = Started(obj)
            started = obj.started;
        end
    end
end
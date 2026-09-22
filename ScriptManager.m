classdef ScriptManager < handle
    %ScriptManager Summary of this class goes here
    %   Detailed explanation goes here

    properties
        taskList
        currentTaskIndex
        currentTask
        started
        lastTaskFinished
        empty
    end

    methods
        function obj = ScriptManager()
            %ScriptManager Construct an instance of this class
            %   Detailed explanation goes here
            obj.taskList = table('Size',[0 7],'VariableTypes',{'categorical','cell','cell','double','cellstr','cell','int16'},...
                'VariableNames',{'Type','VarType','VarValue','MinTime','ActiveLeg','Param','Index'});
            obj.currentTaskIndex = 0;
            obj.started = 0;
            obj.empty = 1;
            obj.lastTaskFinished = 0;
        end

        function AddTask(obj,varargin)
            %AddTask Summary of this method goes here
            %   Detailed explanation goes here
            index = varargin{1};
            [R,C] = size(obj.taskList);

            %find index before
            rInd = 0;
            for ind = 1 : R
                if (index < obj.taskList(ind,7).Index)
                    rInd = ind;
                    break;
                end
            end  

            if rInd
                %copy rows
                for ind = R : -1 : rInd
                    for c = 1 : C
                        obj.taskList(ind+1,c) = obj.taskList(ind,c);
                    end
                end                
            else
                rInd = size(obj.taskList,1)+1;
            end            
            
            type = varargin{2};
            varType = varargin{3};
            varValue = varargin{4};
            nominalTime = varargin{5};                       
            
            %L = size(obj.taskList,1)+1;
            L = rInd;
            obj.taskList(L,1) = type;            
            obj.taskList(L,2) = {varType};
            obj.taskList(L,3) = {varValue};            
            obj.taskList(L,4) = {nominalTime};
            obj.taskList(L,7) = {index};
            if nargin > 6
                leg = varargin{6}; 
                obj.taskList(L,5) = {leg};
            else
                obj.taskList(L,5) = {'-'};              
            end            
            if nargin > 7
                paramValue = varargin{7};
                obj.taskList(L,6) = paramValue;
            else
                obj.taskList(L,6) = {[]};
            end            
            obj.empty = 0;
        end

        function task = GetCurrentTask(obj,t)
            %GetCurrentTask Summary of this method goes here

            if (obj.lastTaskFinished)
                task = obj.currentTask;
                return;
            end

            %   Detailed explanation goes here
            ind = obj.currentTaskIndex;
            %c = obj.taskList(ind,2).VarType{1}
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
                case 'Steer'
                    task = SteerTask(controlledVars,varsValue,minTime);   
                case 'Drive'
                    task = DriveTask(varsValue,minTime);   
                case 'Rotation' %requires a special initial configuration
                    task = RotationTask(controlledVars,varsValue,minTime);                      
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

        function [fine,msg,index] = EntryToString(obj,ind)
            if ind > size(obj.taskList,1)
                fine = 0;
                msg = '';
                index = 0;
            else
                fine = 1;
                cma = ',';
                entry = obj.taskList(ind,:);
                motionType = string(entry{1,1});
                activeLegStr = obj.WriteLegs(entry{1,5});
                controlVarsStr = obj.WriteVars(entry{1,2});
                valuesStr = obj.WriteValues(entry{1,3});
                minTime = entry{1,4};
                index = entry{1,7};
                msg = strcat(num2str(index),' ',motionType,cma,activeLegStr,cma,controlVarsStr,cma,valuesStr,cma,num2str(minTime));                
            end
        end

        function lastIndex = GetLastIndex(obj)
            lastIndex = obj.taskList(size(obj.taskList,1),7).Index;
        end

        function str = WriteLegs(obj,legCell)
            str = '';
            for ind = 1 : length(legCell)                
                str = strcat(str,legCell{ind},',');              
            end            
        end

        function str = WriteVars(obj,varCell)
            str = '';
            for ind = 1 : length(varCell) 
                varInd = varCell{ind};
                str = strcat(str,varInd{1},',');              
            end 
        end

        function str = WriteValues(obj,valuesCell)
            L = length(valuesCell);
            str = '';
            for lInd = 1 : L
                values = valuesCell{lInd};
                newStr = values{1};
                if (isnumeric(newStr))
                    newStr = strcat(num2str(newStr(1,:)),',',num2str(newStr(2,:)));
                end
                str = strcat(str,',',newStr);
            end            
        end

        function DeleteLastTask(obj)
            T = obj.taskList;
            obj.taskList = T(1:size(T,1)-1,:); %XXX
            if(~size(obj.taskList,1))
                obj.empty = 1;
            end
        end

        function DeleteTask(obj,index)
            [R,C] = size(obj.taskList);

            %find row index for Index
            rInd = 0;
            for ind = 1 : R
                if (index == obj.taskList(ind,7).Index)
                    rInd = ind;
                end
            end

            if ~rInd
                return;
            end

            %rewrite tail of the list
            for ind = rInd : R-1
                for c = 1 : C
                    obj.taskList(ind,c) = obj.taskList(ind+1,c);
                end
            end
            obj.DeleteLastTask();
        end

        function ClearTaskList(obj)
            obj.taskList = table('Size',[0 6],'VariableTypes',{'categorical','cell','cell','double','cellstr','cell'},...
                'VariableNames',{'Type','VarType','VarValue','MinTime','ActiveLeg','Param'});
            T = obj.taskList;
            obj.taskList = T(1:size(T,1)-1,:);
            obj.currentTaskIndex = 0;
            obj.empty = 1;
        end

        function Reset(obj)
            obj.currentTaskIndex = 0;
            obj.started = 0;
            obj.lastTaskFinished = 0;            
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

        function out = IsEmpty(obj)
            out = obj.empty;
        end        
    end
end
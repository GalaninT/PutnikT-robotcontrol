classdef EmptyTask < Task
    %EmptyTask Summary of this class goes here
    %   Detailed explanation goes here

    properties
        
    end

    methods
        function obj = EmptyTask()
            %EmptyTask Construct an instance of this class
            %   Detailed explanation goes here
            obj.moveType = 'Empty';
        end

        function torque = FindControl(obj,rover,controllerType)
%             L = rover.numberOfLegs;
%             J = length(obj.controlledVars);
%             torque = zeros(L,J);

            torque = 0;
        end


        function SetInitialState(obj,state)
            obj.initialState = state;
        end

        function isCompleted = Completed(obj,rover,t)
            isCompleted = 0;
        end

%         function outputArg = method1(obj,inputArg)
%             %METHOD1 Summary of this method goes here
%             %   Detailed explanation goes here
%             outputArg = obj.Property1 + inputArg;
%         end
    end
end
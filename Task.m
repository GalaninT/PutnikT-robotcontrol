classdef (Abstract) Task < handle
    %Task Summary of this class goes here
    %   Detailed explanation goes here

    properties 
        moveType %Joint,
        controllerType %Zero, ID, ESP, IDTraj, ESPTraj, etc
        activeLeg
        controlledVars
        varsValue
        paramsValue
        initialState
        endTolerance
        allSet

        tStart
        minTime

        Kp
        Kd

        isCompleted
    end

    methods (Abstract)

        torque = FindControl(obj)
        isCompleted = Completed(obj,rover,t)
        SetInitialState(obj,state)   

    end

    methods
        function lInd = GetLegIndex(obj)
            L = length(obj.activeLeg);
            lInd = [];
            for ind = 1 : L
                switch obj.activeLeg{ind}
                    case 'FR'
                        lInd = [lInd 1];
                    case 'RR'
                        lInd = [lInd 2];
                    case 'RL'
                        lInd = [lInd 3];
                    case 'FL'
                        lInd = [lInd 4];                    
                end
            end
        end
    end
end
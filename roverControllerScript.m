function [output] = roverControllerScript(t)
%roverControllerScript 
%   Output - joint torques, [hip,knee,steer,wheel] - for all legs, in order
%   [FR,RR,RL,FL]
J = 4; L = 4;
N = J*L;
output = zeros(1,N);

global currentTask rover

if strcmp(currentTask.moveType,'Empty') 
    return;
elseif isempty(currentTask.initialState)
    if rover.state.isSet
        currentTask.SetInitialState(rover.state);
    end
    return;
end

output = currentTask.FindControl(rover,t);

end
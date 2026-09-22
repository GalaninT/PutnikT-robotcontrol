function roverManager(t)
%roverManager 
%   Output - no output, just currentTask to fill

global scriptMaster currentTask rover

    if (~scriptMaster.Started())
        scriptMaster.StartScript();
        currentTask = scriptMaster.GetCurrentTask(t);
    end

    if (currentTask.Completed(rover,t))
        scriptMaster.ProceedTask(t);
        currentTask = scriptMaster.GetCurrentTask(t);
    end

    global elapsedTime
    elapsedTime = t;

end
function d = readDone(modelName)
    d = 0;  % default "not done" if nothing readable yet
    try
        runObj = Simulink.sdi.getCurrentSimulationRun(modelName);
        sig = getSignalsByName(runObj, 'done');
        if ~isempty(sig) && ~isempty(sig.Values.Data)
            d = sig.Values.Data(end);   % latest streamed sample
        end
    catch
        d = 0;   % run not started / signal not present yet
    end
end
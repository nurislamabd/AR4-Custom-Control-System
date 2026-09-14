 %% 
%% Setup - run once
cam = webcam;
cropRegion = [932.3023  467.2261  243.1213  335.2515];
load('cameraCalibration.mat', 'tform');

% Declare Simulink parameters (must match names referenced in your model)
targetX = Simulink.Parameter(0);
targetX.StorageClass = 'ExportedGlobal';

targetY = Simulink.Parameter(0);
targetY.StorageClass = 'ExportedGlobal';

targetZ = Simulink.Parameter(67);
targetZ.StorageClass = 'ExportedGlobal';

targetGo = Simulink.Parameter(0);
targetGo.StorageClass = 'ExportedGlobal';

targetColor = Simulink.Parameter(0);
targetColor.StorageClass = 'ExportedGlobal';

%% Start External Mode on your model first (manually, via Monitor & Tune,
%% or set_param(modelName,'SimulationCommand','start'))

%% Main loop - run after External Mode is live
while true
    % 1. wait until robot idle at camera pose
    while readDone("AR4FullPositionControl") ~= 1, pause(0.05); end

    % 2. capture + detect
    [X, Y, colorID, found] = getVisionTargetValidated(cam, cropRegion, tform);
    if ~found, pause(1); continue; end
    
    % 3. command
    targetX.Value = X; targetY.Value = Y; targetZ.Value = 67;
    targetGo.Value = 1; targetColor.Value = colorID;
    set_param("AR4FullPositionControl",'SimulationCommand','update');

    % 4. wait for FSM to start (done drops)
    while readDone("AR4FullPositionControl") ~= 0, pause(0.05); end

    % 5. clear trigger so it doesn't re-fire
    targetGo.Value = 0;
    set_param("AR4FullPositionControl",'SimulationCommand','update');

    % 6. wait for completion (done back high, robot at camera pose)
    while readDone("AR4FullPositionControl") ~= 1, pause(0.05); end
end





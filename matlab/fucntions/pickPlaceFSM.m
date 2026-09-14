function [cmdX, cmdY, cmdZ, gripperClose, done, state] = pickPlaceFSM(...
        targetX, targetY, targetZ, targetColor, newTarget, moveComplete)
%#codegen
% Pick-and-place FSM. Outputs a Cartesian target for the IK block and
% advances one state only after the commanded move has GENUINELY completed.
%
% MOVE DETECTION -- two-phase handshake (see notes in prior version):
%   phase 0 = wait for move to START (signal LOW, or START_TIMEOUT for
%             near-zero moves), phase 1 = wait to SETTLE (high for
%             STABLE_STEPS). MOVE_TIMEOUT is a hard safety cap.
%   moveComplete should be CoordMove's encoder-based done, NOT pathStep done.
%
% COLOR SORTING: targetColor is latched alongside the target coordinates at
% pickup time (objColor), so a mid-cycle change to the pushed parameter can
% never re-route a block already in the gripper. S_PLACE_A / S_PLACE_B are
% separate states so each drop-off can be tuned independently later.

    Ts = 0.03;   % must match this block's actual sample time (s)

    % Setpoints [x y z] mm, robot base frame
    INTERMEDIATE = [-50, 350, 220];    % fill with real values
    PLACE_A      = [-100, 250, 220];   % drop-off for color 1  
    PLACE_B      = [0, 250, 220];      % drop-off for color 2  
    HOME         = [-50, 250, 220];
    CAM          = [-50, 250, 220];
    HOVER_OFF    = 40;

    GRIP_CLOSE_DWELL = round(0.8 / Ts);
    GRIP_OPEN_DWELL  = round(0.8 / Ts);

    % Move handshake timing (all in samples)
    START_TIMEOUT = round(0.50 / Ts);   % if signal never drops, assume started
    STABLE_STEPS  = round(0.20 / Ts);   % continuous-high to count as settled
    MOVE_TIMEOUT  = round(6.0  / Ts);   % hard safety cap per move-state

    S_WAIT=1; S_HOVER=2; S_PICK=3; S_CLOSE=4; S_LIFT=5;
    S_INTER=6; S_PLACE_A=7; S_OPEN=8; S_HOME=9; S_PLACE_B=10;

    persistent st cmd objX objY objZ objColor dwell stableCnt phase
    if isempty(st)
        st = S_WAIT; cmd = CAM;
        objX = 0; objY = 0; objZ = 150; objColor = 1;
        dwell = 0; stableCnt = 0;
        phase = 0;                 % 0 = waiting for move to START, 1 = waiting to SETTLE
    end

    % Settle debounce
    if moveComplete
        stableCnt = stableCnt + 1;
    else
        stableCnt = 0;
    end

    dwell = dwell + 1;             % samples since this state (move) was entered

    if phase == 0
        if (~moveComplete) || (dwell >= START_TIMEOUT)
            phase = 1;             % move has started (or assumed started)
            stableCnt = 0;         % reset settle debounce for the settle phase
        end
    end

    moveDoneStable = false;
    if phase == 1 && stableCnt >= STABLE_STEPS
        moveDoneStable = true;
    end
    if dwell >= MOVE_TIMEOUT
        moveDoneStable = true;     % safety fallback -- never hang
    end

    gripperClose = 0;
    done = 0;

    switch st
        case S_WAIT                         % at camera pose, gripper open
            cmd = CAM; gripperClose = 0;
            if moveDoneStable
                done = 1;
                if newTarget
                    objX = targetX; objY = targetY; objZ = targetZ;
                    objColor = targetColor;         % latch color with the target
                    cmd = [objX, objY, objZ + HOVER_OFF];
                    st = S_HOVER; dwell = 0; stableCnt = 0; phase = 0; done = 0;
                end
            end

        case S_HOVER                        % above object
            gripperClose = 0;
            if moveDoneStable
                cmd = [objX, objY, objZ];   % descend onto object
                st = S_PICK; dwell = 0; stableCnt = 0; phase = 0;
            end

        case S_PICK                         % down onto object
            gripperClose = 0;
            if moveDoneStable
                st = S_CLOSE; dwell = 0; stableCnt = 0; phase = 0;
            end

        case S_CLOSE                        % grip, hold, wait (no move)
            gripperClose = 1;
            if dwell >= GRIP_CLOSE_DWELL
                cmd = [objX, objY, objZ + HOVER_OFF];   % lift straight up
                st = S_LIFT; dwell = 0; stableCnt = 0; phase = 0;
            end

        case S_LIFT
            gripperClose = 1;
            if moveDoneStable
                cmd = INTERMEDIATE;
                st = S_INTER; dwell = 0; stableCnt = 0; phase = 0;
            end

        case S_INTER                        % route by latched color
            gripperClose = 1;
            if moveDoneStable
                if objColor == 2
                    cmd = PLACE_B;
                    st = S_PLACE_B;
                else
                    cmd = PLACE_A;          
                    st = S_PLACE_A;
                end
                dwell = 0; stableCnt = 0; phase = 0;
            end

        case S_PLACE_A                      % drop-off for color 1
            gripperClose = 1;
            if moveDoneStable
                st = S_OPEN; dwell = 0; stableCnt = 0; phase = 0;
            end

        case S_PLACE_B                      % drop-off for color 2
            gripperClose = 1;
            if moveDoneStable
                st = S_OPEN; dwell = 0; stableCnt = 0; phase = 0;
            end

        case S_OPEN                         % release, wait (no move)
            gripperClose = 0;
            if dwell >= GRIP_OPEN_DWELL
                cmd = HOME;
                st = S_HOME; dwell = 0; stableCnt = 0; phase = 0;
            end

        case S_HOME
            gripperClose = 0;
            if moveDoneStable
                st = S_WAIT; dwell = 0; stableCnt = 0; phase = 0;
            end

        otherwise
            st = S_WAIT; cmd = CAM;
    end

    cmdX = cmd(1); cmdY = cmd(2); cmdZ = cmd(3);
    state = st;
end
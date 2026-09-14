function [modeOut, startNextOut, stage, done] = HomingSequencer( ...
    startHoming, enableIK, skipHoming, ...
    homed1, homed2, homed3, homed4, homed5, homed6, ...
    next1,  next2,  next3,  next4,  next5,  next6)
%HOMINGSEQUENCER  Staged initialization for the AR4 MK3 (paired homing).
%
%   Homes joints in THREE pairs to avoid mechanical cross-coupling
%   (notably J3's zero being disturbed when homed alongside J2):
%       Pair A: J1 & J2
%       Pair B: J3 & J5
%       Pair C: J4 & J6
%
%   Per-pair sequence: HOME to switch (mode 2) -> GOTO home angle (mode 1)
%   -> HOLD at home angle (mode 1) for the rest of calibration.
%
%   mode: 0 = POSITION (IK), 1 = INITIALIZE (drive to HOME_TARGET), 2 = HOMING
%
%   STAGES (phase):
%     0 : IDLE
%     1 : HOME  J1,J2            (wait homed1 & homed2)
%     2 : GOTO  J1,J2 -> home    (wait next1  & next2)
%     3 : HOME  J3,J5            (wait homed3 & homed5)  J1,J2 held
%     4 : GOTO  J3,J5 -> home    (wait next3  & next5)   J1,J2 held
%     5 : HOME  J4,J6            (wait homed4 & homed6)  J1,J2,J3,J5 held
%     6 : GOTO  J4,J6 -> home    (wait next4  & next6)   others held
%     7 : DONE  hold all at home (mode 1) until enableIK -> all mode 0
%
%   INPUTS
%     startHoming - rising edge (0->1) arms the sequence from IDLE
%     enableIK    - when 1 at stage 7, hands control to IK (all mode 0)
%     skipHoming  - when 1, jump straight to stage 7 (DONE) trusting the
%                   already-loaded encoder positions (NO homing performed).
%                   Only safe if the robot's true position is known/loaded.
%     homed1..6   - 'homed' output from each joint block
%     next1..6    - 'next'  output from each joint block
%
%   OUTPUTS (1x6, order J1..J6)
%     modeOut, startNextOut, stage, done

    persistent phase lastStart justEntered lastSkip;

    if isempty(phase)
        phase       = 0;
        lastStart   = 0;
        justEntered = true;
        lastSkip    = 0;
    end

    % -------------------------------------------------------------------------
    % SKIP-HOMING: jump straight to DONE (stage 7) on rising edge of skipHoming
    % Trusts loaded encoder positions; performs no homing motion.
    % -------------------------------------------------------------------------
    if skipHoming == 1 && lastSkip == 0
        phase       = 7;
        justEntered = true;
    end
    lastSkip = skipHoming;

    % -------------------------------------------------------------------------
    % RISING EDGE TRIGGER (normal homing) - only from IDLE
    % -------------------------------------------------------------------------
    if startHoming == 1 && lastStart == 0 && phase == 0
        phase       = 1;
        justEntered = true;
    end
    lastStart = startHoming;

    % -------------------------------------------------------------------------
    % DEFAULTS
    % -------------------------------------------------------------------------
    modeOut      = [0, 0, 0, 0, 0, 0];
    startNextOut = [0, 0, 0, 0, 0, 0];

    % order in arrays: [J1 J2 J3 J4 J5 J6]

    switch phase

        case 0
            % IDLE - waiting for startHoming (or skipHoming) rising edge

        % ================= PAIR A : J1 & J2 =================
        case 1
            % HOME J1, J2
            modeOut      = [2, 2, 0, 0, 0, 0];
            startNextOut = [1, 1, 0, 0, 0, 0];
            if homed1 && homed2
                phase       = 2;
                justEntered = true;
            end

        case 2
            % GOTO J1, J2 -> home angle
            modeOut      = [1, 1, 0, 0, 0, 0];
            startNextOut = [1, 1, 0, 0, 0, 0];
            if justEntered
                justEntered = false;      % skip stale next=true from homing
            elseif next1 && next2
                phase       = 3;
                justEntered = true;
            end

        % ================= PAIR B : J3 & J5 =================
        case 3
            % HOME J3, J5   (J1,J2 held at home angle via mode 1)
            modeOut      = [1, 1, 2, 0, 2, 0];
            startNextOut = [1, 1, 1, 0, 1, 0];
            if homed3 && homed5
                phase       = 4;
                justEntered = true;
            end

        case 4
            % GOTO J3, J5 -> home angle   (J1,J2 still held)
            modeOut      = [1, 1, 1, 0, 1, 0];
            startNextOut = [1, 1, 1, 0, 1, 0];
            if justEntered
                justEntered = false;
            elseif next3 && next5
                phase       = 5;
                justEntered = true;
            end

        % ================= PAIR C : J4 & J6 =================
        case 5
            % HOME J4, J6   (J1,J2,J3,J5 held at home angle)
            modeOut      = [1, 1, 1, 2, 1, 2];
            startNextOut = [1, 1, 1, 1, 1, 1];
            if homed4 && homed6
                phase       = 6;
                justEntered = true;
            end

        case 6
            % GOTO J4, J6 -> home angle   (others still held)
            modeOut      = [1, 1, 1, 1, 1, 1];
            startNextOut = [1, 1, 1, 1, 1, 1];
            if justEntered
                justEntered = false;
            elseif next4 && next6
                phase = 7;
            end

        otherwise
            % STAGE 7: DONE - hold all at home until IK enabled, then hand over
            startNextOut = [1, 1, 1, 1, 1, 1];
            if enableIK
                modeOut = [0, 0, 0, 0, 0, 0];   % IK / positionAngle takes over
            else
                modeOut = [1, 1, 1, 1, 1, 1];   % hold at home position
            end
    end

    stage = phase;
    done  = (phase == 7);

end
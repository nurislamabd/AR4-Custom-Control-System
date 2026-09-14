function [count, displaySpeed, dir, pul, next, limit, homed] = J2(startNext, clk, dt, rst, speedRPM, positionAngle, currCount, accelRPMperSec, mode)
% J2 - Joint 2 position/speed controller with S-curve acceleration + HOMING
%
% AR4-MK3 robot. SINGLE-LIMIT joint.
% Limit switch at the most-negative end (-42 deg) = count 0.
% Count increases as the joint moves AWAY from the switch.
%
% MODE (single input):
%   mode = 0  ->  POSITION  : normal operation, drive to positionAngle input
%   mode = 1  ->  INITIALIZE: drive to HOME_TARGET, ignore positionAngle
%   mode = 2  ->  HOMING    : drive toward limit switch, zero count on contact
%
% Typical sequence: 2 (home) -> 1 (go to home angle) -> 0 (position control)
%
% INPUTS
%   startNext, clk, dt, rst, t, speedRPM, positionAngle, currCount,
%   accelRPMperSec, mode  (mode: 0=position, 1=initialize, 2=homing)
% OUTPUTS
%   count, displaySpeed, dir, pul, next, limit, homed
%
%   >>> J2: span=132 deg (-42..90)  maxCount=18300

    persistent currentCount lastClkState lastCount spTimer smoothedSpeed ...
               currentRPM distTraveled totalMoveFrozen lastTarget pulseAccum ...
               homedFlag lastMode;

    % -------------------------------------------------------------------------
    % CONSTANTS  -  per-joint values
    % -------------------------------------------------------------------------
    Ts           = 3e-5;
    PPR          = 1000;
    CPR          = PPR * 4;
    maxCount     = 18300;          % <<< J2
    LIMIT_BUFFER = 50;
    TUNE_C2R     = 0.8;            % <<< J2

    TOTAL_DEG      = 132;          % <<< J2 span: 90-(-42)
    COUNTS_PER_REV = maxCount / TOTAL_DEG * 360;
    GEAR_RATIO     = COUNTS_PER_REV / CPR;
    C2R            = (GEAR_RATIO * CPR) / 60 * TUNE_C2R;

    speedUpdateInterval = 0.2;
    ALPHA               = 0.3;
    MIN_RPM             = 0.5;
    STOP_COUNTS         = 1;

    HOME_RPM    = 2;               % <<< tune: homing speed (RPM)
    HOME_TARGET = 0;               % <<< angle commanded in mode=1 (deg)

    % Single-limit angle->count mapping (anchored at switch)
    J_MIN_DEG    = -42;            % <<< J2
    J_MAX_DEG    =  90;            % <<< J2
    SWITCH_DEG   = -42;            % <<< J2 switch at -42 deg = count 0
    countsPerDeg = maxCount / TOTAL_DEG;

    % -------------------------------------------------------------------------
    % INITIALISATION (first call only)
    % -------------------------------------------------------------------------
    if isempty(currentCount)
        currentCount    = currCount;
        lastCount       = currCount;
        lastClkState    = clk;
        spTimer         = 0;
        smoothedSpeed   = 0;
        currentRPM      = MIN_RPM;
        distTraveled    = 0;
        totalMoveFrozen = 0;
        lastTarget      = -1;
        pulseAccum      = 0;
        homedFlag       = false;
        lastMode        = 0;
    end

    limit = false;
    homed = homedFlag;
    dir   = 0;
    pul   = 0;
    next  = false;

    % -------------------------------------------------------------------------
    % 1. ENCODER COUNTING
    % -------------------------------------------------------------------------
    if clk ~= lastClkState && clk == 1
        if dt ~= clk
            currentCount = currentCount - 1;
        else
            currentCount = currentCount + 1;
        end
    end

    % Reset S-curve state on any mode change (once, on the transition)
    if mode ~= lastMode
        distTraveled    = 0;
        totalMoveFrozen = 0;
        pulseAccum      = 0;
        lastTarget      = -1;
        currentRPM      = MIN_RPM;
        spTimer         = 0;
        smoothedSpeed   = 0;
    end
    lastMode = mode;

    % =========================================================================
    % MODE 2: HOMING
    % =========================================================================
    if mode == 2
        if rst == 1
            currentCount = -235;
            homedFlag    = true;
        end
        homed = homedFlag;          % limit NOT asserted during homing

        if homedFlag
            currentCount = -235;
            pul          = 0;
            next         = true;
        elseif startNext
            dir        = 0;         % toward switch = decreasing count
            pulseAccum = pulseAccum + HOME_RPM * C2R * Ts;
            if pulseAccum >= 1.0
                pul        = 1;
                pulseAccum = pulseAccum - 1.0;
            end
        end

        count        = currentCount;
        displaySpeed = HOME_RPM;
        lastClkState = clk;
        return;
    end
    % =========================================================================

    % -------------------------------------------------------------------------
    % 2. SPEED DISPLAY
    % -------------------------------------------------------------------------
    spTimer = spTimer + Ts;
    if spTimer >= speedUpdateInterval
        smoothedSpeed = ALPHA * currentRPM + (1 - ALPHA) * smoothedSpeed;
        spTimer       = 0;
    end

    % -------------------------------------------------------------------------
    % 3. TARGET FROM ANGLE
    %    mode 1 (INITIALIZE): command = HOME_TARGET
    %    mode 0 (POSITION)  : command = positionAngle input
    % -------------------------------------------------------------------------
    if mode == 1
        cmdAngle = HOME_TARGET;
    else
        cmdAngle = positionAngle;
    end

    % Clamp to physical range, then map (single-limit, anchored at switch)
    if cmdAngle < J_MIN_DEG, cmdAngle = J_MIN_DEG; end
    if cmdAngle > J_MAX_DEG, cmdAngle = J_MAX_DEG; end
    targetCount = round(abs(SWITCH_DEG - cmdAngle) * countsPerDeg);
    targetCount = max(LIMIT_BUFFER, min(maxCount - LIMIT_BUFFER, targetCount));

    % -------------------------------------------------------------------------
    % 4. DIRECTION
    % -------------------------------------------------------------------------
    countError = targetCount - currentCount;
    absError   = abs(countError);
    if countError >= 0
        dir = 1;
    else
        dir = 0;
    end

    % -------------------------------------------------------------------------
    % 5. FREEZE totalMove ON NEW TARGET
    % -------------------------------------------------------------------------
    if targetCount ~= lastTarget
        totalMoveFrozen = absError;
        distTraveled    = 0;
        pulseAccum      = 0;
        lastTarget      = targetCount;
    end

    % -------------------------------------------------------------------------
    % 6. S-CURVE SPEED PROFILE
    % -------------------------------------------------------------------------
    accel = max(accelRPMperSec, 0.1);
    fullRampCounts = 0.5 * (MIN_RPM + speedRPM) * C2R * (speedRPM - MIN_RPM) / accel;

    if totalMoveFrozen > 0 && 2 * fullRampCounts >= totalMoveFrozen
        peakRPM    = sqrt(max(MIN_RPM^2 + totalMoveFrozen * accel / C2R, MIN_RPM^2));
        peakRPM    = max(MIN_RPM, min(speedRPM, peakRPM));
        rampCounts = 0.5 * (MIN_RPM + peakRPM) * C2R * (peakRPM - MIN_RPM) / accel;
        rampCounts = max(1, rampCounts);
    else
        peakRPM    = speedRPM;
        rampCounts = max(1, fullRampCounts);
    end

    if totalMoveFrozen <= 0
        targetRPM = MIN_RPM;
    elseif absError <= rampCounts
        x         = max(0.0, min(1.0, absError / rampCounts));
        targetRPM = MIN_RPM + (peakRPM - MIN_RPM) * 0.5 * (1 - cos(pi * x));
    elseif distTraveled <= rampCounts
        x         = max(0.0, min(1.0, distTraveled / rampCounts));
        targetRPM = MIN_RPM + (peakRPM - MIN_RPM) * 0.5 * (1 - cos(pi * x));
    else
        targetRPM = peakRPM;
    end
    currentRPM = max(MIN_RPM, min(speedRPM, targetRPM));

    % -------------------------------------------------------------------------
    % 7. DDS PULSE ACCUMULATOR
    % -------------------------------------------------------------------------
    if startNext && absError > STOP_COUNTS
        pulseAccum = pulseAccum + currentRPM * C2R * Ts;
        if pulseAccum >= 1.0
            pul          = 1;
            pulseAccum   = pulseAccum - 1.0;
            distTraveled = distTraveled + 1;
        end
        next = false;
    else
        next       = (absError <= STOP_COUNTS);
        pulseAccum = 0;
    end

    % -------------------------------------------------------------------------
    % 8. LIMIT SWITCH SAFETY (modes 0 and 1 only)
    % -------------------------------------------------------------------------
    if homedFlag && currentCount > LIMIT_BUFFER
        homedFlag = false;
    end
    if rst == 1 && ~homedFlag && dir == 0
        limit = true;
        pul   = 0;
    end
    if currentCount > maxCount
        limit = true;
        pul   = 0;
    end

    % -------------------------------------------------------------------------
    % 8.5 MODE-0 PULSE SUPPRESSION
    %     In position mode, CoordMove owns all pulse/dir generation.
    %     Force this block's own pul/dir to 0 so it can't leak a second
    %     pulse stream to the driver. Encoder counting + limit safety above
    %     still run normally every tick.
    % -------------------------------------------------------------------------
    if mode == 0
        pul = 0;
    end


    % -------------------------------------------------------------------------
    % 9. OUTPUTS
    % -------------------------------------------------------------------------
    count        = currentCount;
    displaySpeed = smoothedSpeed;
    homed        = homedFlag;
    lastClkState = clk;

end
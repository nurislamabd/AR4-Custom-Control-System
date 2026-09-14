function [count, displaySpeed, dir, pul, next, limit, homed] = J5(startNext, clk, dt, rst, speedRPM, positionAngle, currCount, accelRPMperSec, mode)
% J5 - Joint 5 position/speed controller with S-curve acceleration + HOMING
%
%   >>> J5: span=208.5 deg (108.5-...-100)  maxCount=5800  HOME_TARGET=90
%   >>> Backlash compensation added (~7 deg measured play)

    persistent currentCount lastClkState lastCount spTimer smoothedSpeed ...
               currentRPM distTraveled totalMoveFrozen lastTarget pulseAccum ...
               homedFlag lastMode lastRawCmd backlashSign;

    % -------------------------------------------------------------------------
    % CONSTANTS  -  per-joint values
    % -------------------------------------------------------------------------
    Ts           = 3e-5;
    PPR          = 1000;
    CPR          = PPR * 4;
    maxCount     = 5800;           % <<< J5
    LIMIT_BUFFER = 50;
    TUNE_C2R     = 1.6;            % <<< J5

    TOTAL_DEG      = 208.5;          % <<< J5 span: 108.5-(-100)
    COUNTS_PER_REV = maxCount / TOTAL_DEG * 360;
    GEAR_RATIO     = COUNTS_PER_REV / CPR;
    C2R            = (GEAR_RATIO * CPR) / 60 * TUNE_C2R;

    speedUpdateInterval = 0.2;
    ALPHA               = 0.3;
    MIN_RPM             = 0.5;
    STOP_COUNTS         = 1;

    HOME_RPM    = 5;               % <<< tune: homing speed (RPM)
    HOME_TARGET = 90;              % <<< J5 home angle (deg)

    % Single-limit angle->count mapping (anchored at switch)
    J_MIN_DEG    = -100;           % <<< J5
    J_MAX_DEG    =  108.5;           % <<< J5
    SWITCH_DEG   = -100;           % <<< J5 switch at -100 deg = count 0
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
        lastRawCmd      = -999;    % force first target to register as "new"
        backlashSign    = 0;
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
            currentCount = currentCount + 1;
        else
            currentCount = currentCount - 1;
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
            currentCount = 0;
            homedFlag    = true;
        end
        homed = homedFlag;          % limit NOT asserted during homing

        if homedFlag
            currentCount = 0;
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
    % -------------------------------------------------------------------------
    if mode == 1
        cmdAngle = HOME_TARGET;
    else
        cmdAngle = positionAngle;
    end

    % ---------------------------------------------------------------------
    % BACKLASH COMPENSATION (J5: ~7 deg measured play, stable)
    % Latch compensation direction ONCE when a NEW target arrives (based on
    % travel direction from current physical position), then HOLD it for the
    % whole move. Holding (not recomputing every cycle) is what makes it
    % work -- otherwise arrival at the target erases the offset.
    % ---------------------------------------------------------------------
    BACKLASH_DEG = 7.0;             % <<< measured J5 backlash (tune)
    halfBacklash = BACKLASH_DEG / 2;

    rawTarget = cmdAngle;           % uncompensated commanded angle

    if abs(rawTarget - lastRawCmd) > 0.5     % a NEW target was commanded
        currentAngle = SWITCH_DEG + (currentCount / countsPerDeg);
        if rawTarget > currentAngle
            backlashSign = 1;       % must move + -> shift target +
        elseif rawTarget < currentAngle
            backlashSign = -1;      % must move - -> shift target -
        else
            backlashSign = 0;
        end
        lastRawCmd = rawTarget;
    end
    % backlashSign persists (held) across calls for the whole move

    cmdAngle = rawTarget + backlashSign * halfBacklash;

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
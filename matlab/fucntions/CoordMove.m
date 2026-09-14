function [pul1, pul2, pul3, pul4, pul5, pul6, ...
          dir1, dir2, dir3, dir4, dir5, dir6, ...
          done] = CoordMove(angle1, angle2, angle3, angle4, angle5, angle6, ...
                            count1, count2, count3, count4, count5, count6, ...
                            speedRPM, accelRPMperSec, enable)
% CoordMove - Coordinated 6-joint motion with single shared S-curve master clock

%
% Replaces per-joint pulse generation in mode=0 (position control).
% J1-J6 blocks still own: encoder counting, homing (mode 2),
% initialize (mode 1), limit safety, and count/homed/limit outputs.
% This block owns: pul and dir for all 6 joints in mode=0 only.
%
% Architecture matches original AR4 firmware (driveMotorsJ):
%   1. Convert target angles -> target counts (per-joint mapping)
%   2. Compute step deltas from current counts
%   3. Find HighStep (limiting joint with most steps)
%   4. Run ONE shared S-curve (accel->cruise->decel) over HighStep ticks
%   5. Each joint fires its pulses proportionally via Bresenham algorithm
%      so all 6 joints arrive together, guaranteed by construction
%
% INPUTS:
%   angle1..6       - target joint angles from IK block (degrees)
%   count1..6       - current encoder counts from J1-J6 blocks
%   speedRPM        - cruise speed for the LIMITING joint (RPM)
%   accelRPMperSec  - acceleration rate (RPM/s)
%   enable          - gate: true = run coordinated move, false = hold
%
% OUTPUTS:
%   pul1..6         - step pulse per joint (0 or 1) -- wire to stepper drivers
%   dir1..6         - direction per joint (0 or 1)  -- wire to stepper drivers
%   done            - true when all joints have reached their targets
%
% WIRING NOTE:
%   In mode=0, wire CoordMove pul/dir outputs to stepper drivers.
%   In mode=1/2, wire J1-J6 pul/dir outputs to stepper drivers instead.
%   Use a Switch block (or Multiport Switch) gated on mode to select.
%   J1-J6 pul/dir outputs in mode=0 can be left disconnected or ignored.

    % -------------------------------------------------------------------------
    % PERSISTENT STATE
    % -------------------------------------------------------------------------
    persistent highStep;           % total master-clock ticks for this move
    persistent masterTick;         % current master-clock tick (0..highStep)
    persistent stepTarget;         % per-joint target step count [1x6]
    persistent dirs;               % per-joint direction [1x6], frozen at move start
    persistent ddaAccum;           % DDA accumulator per joint [1x6] - exact stepping
    persistent targetCountFrozen;  % frozen absolute target count per joint [1x6]
    persistent pulseTarget;        % pulses each joint must fire (countDelta*TUNE) [1x6]
    persistent lastTargetCount;    % frozen target counts from last tick [1x6]
    persistent currentRPM;         % current RPM on the shared S-curve
    persistent C2R_master;         % counts-to-RPM factor for the master (HighStep) joint
    persistent masterAccum;        % DDS accumulator on the master clock
    persistent masterIdx;          % index of the joint that defines highStep
    persistent remSmooth;          % low-pass filtered master remaining pulses
    persistent j5BacklashSign;     % J5 backlash direction, latched per move
    persistent runawayFlag;        % [1x6] joint is moving AWAY from target
    persistent j5LastRawCmd;       % J5 last raw commanded angle (for new-target detect)

    % -------------------------------------------------------------------------
    % PER-JOINT CONSTANTS (from J1-J6 source files)
    % -------------------------------------------------------------------------
    % J1: switch at +170 deg, count increases away from switch
    J1_maxCount    = 37500;  J1_TOTAL_DEG = 340;  J1_SWITCH_DEG =  170;
    J1_LIMIT_BUF   = 50;     J1_TUNE_C2R  = 0.8;
    J1_countsPerDeg = J1_maxCount / J1_TOTAL_DEG;

    % J2: switch at -42 deg, count increases away from switch
    J2_maxCount    = 18300;  J2_TOTAL_DEG = 132;  J2_SWITCH_DEG = -42;
    J2_LIMIT_BUF   = 50;     J2_TUNE_C2R  = 0.8;
    J2_countsPerDeg = J2_maxCount / J2_TOTAL_DEG;

    % J3: switch at +52 deg, count increases away from switch
    J3_maxCount    = 19720;  J3_TOTAL_DEG = 141;  J3_SWITCH_DEG =  52;
    J3_LIMIT_BUF   = 50;     J3_TUNE_C2R  = 0.8;
    J3_countsPerDeg = J3_maxCount / J3_TOTAL_DEG;

    % J4: ZERO_COUNT anchor
    J4_maxCount    = 42860;  J4_TOTAL_DEG = 344;  J4_ZERO_COUNT = 22238;
    J4_LIMIT_BUF   = 50;     J4_TUNE_C2R  = 0.8;
    J4_countsPerDeg = J4_maxCount / J4_TOTAL_DEG;

    % J5: switch at -100 deg, count increases away from switch. Has backlash comp.
    J5_maxCount    = 5800;   J5_TOTAL_DEG = 208.5;  J5_SWITCH_DEG = -100;
    J5_LIMIT_BUF   = 50;     J5_TUNE_C2R  = 1.6;
    J5_J_MIN_DEG   = -100;   J5_J_MAX_DEG = 108.5;
    J5_BACKLASH_DEG = 7.0;   % measured J5 play (must match J5.m)
    J5_countsPerDeg = J5_maxCount / J5_TOTAL_DEG;

    % J6: full rotation, internal +180 remap. TARGET_MARGIN=0 (reaches both ends).
    J6_maxCount    = 20000;
    J6_TARGET_MARGIN = 0;    J6_TUNE_C2R  = 0.8;
    J6_countsPerDeg = J6_maxCount / 360;

    % Per-joint pulse:count ratio (one pulse moves 1/TUNE_C2R encoder counts)
    tuneArr = [J1_TUNE_C2R, J2_TUNE_C2R, J3_TUNE_C2R, ...
               J4_TUNE_C2R, J5_TUNE_C2R, J6_TUNE_C2R];

    % Shared constants
    Ts    = 3e-5;
    CPR   = 1000 * 4;
    MIN_RPM    = 0.5;
    % -------------------------------------------------------------------------
    % PER-JOINT ARRIVAL TOLERANCE
    % Specified in DEGREES (physically meaningful) and converted to counts,
    % because counts/deg varies 5x across joints (J5 ~28, J3 ~140).
    %
    % +/-3 counts asks for repeatability the mechanism does not have: for J1
    % that is 0.027 deg. After its last pulse a joint settles back a few counts
    % from backlash/compliance, and since one pulse moves ~1.25 counts it lands
    % on a discrete grid. Usually inside the band, occasionally not -- which is
    % why every joint fails sometimes and J4 (most lash, smallest moves) fails
    % most.
    %
    % Tolerances are chosen by ROLE, not uniformly:
    %   J1,J2,J3 positioning -> tight, these set TCP position directly
    %   J5       wrist pitch -> moderate, sets gripper tilt
    %   J4,J6    rolls       -> loose, only affect gripper YAW, which is
    %                          immaterial for a straight-down gripper
    % -------------------------------------------------------------------------
    TOL_DEG_J1 = 0.15;   % ~17 counts  (~1.0 mm TCP at 400 mm reach)
    TOL_DEG_J2 = 0.15;   % ~21 counts
    TOL_DEG_J3 = 0.15;   % ~21 counts
    TOL_DEG_J4 = 1.00;   % ~125 counts (roll: yaw only)
    TOL_DEG_J5 = 0.30;   % ~8 counts
    TOL_DEG_J6 = 1.00;   % ~56 counts  (roll: yaw only)

    MIN_STOP_COUNTS = 3; % floor: never tighter than ~2x one pulse's movement

    % RUNAWAY GUARD: if a joint's error ever exceeds the distance it was asked
    % to travel by this margin, it is moving AWAY from target -- its direction
    % sense is wrong. Stop pulsing that joint rather than driving it into a
    % hard limit. 'done' will not fire, which is the correct, visible failure.
    RUNAWAY_MARGIN = 400;   % counts (~3.2 deg on J4, ~3.6 deg on J1)

    % ACC/DCC ramp fraction of total move (matching original firmware defaults)
    ACC_FRAC = 0.25;   % 25% of move for accel ramp
    DCC_FRAC = 0.25;   % 25% of move for decel ramp

    % -------------------------------------------------------------------------
    % INITIALISATION (first call only)
    % -------------------------------------------------------------------------
    if isempty(highStep)
        highStep        = 0;
        masterTick      = 0;
        stepTarget      = zeros(1,6);
        dirs            = zeros(1,6);
        ddaAccum        = zeros(1,6);
        targetCountFrozen = zeros(1,6);
        pulseTarget       = zeros(1,6);
        lastTargetCount = zeros(1,6);
        currentRPM      = MIN_RPM;
        C2R_master      = 1;
        masterAccum     = 0;
        masterIdx       = 1;
        remSmooth       = 0;
        j5BacklashSign  = 0;
        j5LastRawCmd    = -999;   % force first J5 target to register as new
        runawayFlag     = false(1,6);
    end

    % Default outputs every tick (Simulink requires all outputs assigned)
    pul1 = 0; pul2 = 0; pul3 = 0; pul4 = 0; pul5 = 0; pul6 = 0;
    dir1 = dirs(1); dir2 = dirs(2); dir3 = dirs(3);
    dir4 = dirs(4); dir5 = dirs(5); dir6 = dirs(6);
    done = 0;

    if ~enable
        done = 0;
        return;
    end

    % -------------------------------------------------------------------------
    % 1. ANGLE -> TARGET COUNT (matching each J-file's own mapping exactly)
    % -------------------------------------------------------------------------
    % J1: single-limit, switch-anchored at +170
    tc1 = round(abs(J1_SWITCH_DEG - angle1) * J1_countsPerDeg);
    tc1 = max(J1_LIMIT_BUF, min(J1_maxCount - J1_LIMIT_BUF, tc1));

    % J2: single-limit, switch-anchored at -42 (moving away increases count)
    tc2 = round(abs(J2_SWITCH_DEG - angle2) * J2_countsPerDeg);
    tc2 = max(J2_LIMIT_BUF, min(J2_maxCount - J2_LIMIT_BUF, tc2));

    % J3: single-limit, switch-anchored at +52
    tc3 = round(abs(J3_SWITCH_DEG - angle3) * J3_countsPerDeg);
    tc3 = max(J3_LIMIT_BUF, min(J3_maxCount - J3_LIMIT_BUF, tc3));

    % J4: ZERO_COUNT anchor
    tc4 = round(J4_ZERO_COUNT + angle4 * J4_countsPerDeg);
    tc4 = max(J4_LIMIT_BUF, min(J4_maxCount - J4_LIMIT_BUF, tc4));

    % J5: single-limit, switch-anchored at -100, WITH backlash compensation.
    % Must replicate J5.m exactly: latch backlash direction on a new target
    % based on travel direction from current physical position, then HOLD it.
    j5HalfBacklash = J5_BACKLASH_DEG / 2;
    j5RawTarget    = angle5;                 % uncompensated commanded angle

    if abs(j5RawTarget - j5LastRawCmd) > 0.5   % a NEW target was commanded
        j5CurrentAngle = J5_SWITCH_DEG + (count5 / J5_countsPerDeg);
        if j5RawTarget > j5CurrentAngle
            j5BacklashSign = 1;
        elseif j5RawTarget < j5CurrentAngle
            j5BacklashSign = -1;
        else
            j5BacklashSign = 0;
        end
        j5LastRawCmd = j5RawTarget;
    end

    j5CmdAngle = j5RawTarget + j5BacklashSign * j5HalfBacklash;
    if j5CmdAngle < J5_J_MIN_DEG, j5CmdAngle = J5_J_MIN_DEG; end
    if j5CmdAngle > J5_J_MAX_DEG, j5CmdAngle = J5_J_MAX_DEG; end
    tc5 = round(abs(J5_SWITCH_DEG - j5CmdAngle) * J5_countsPerDeg);
    tc5 = max(J5_LIMIT_BUF, min(J5_maxCount - J5_LIMIT_BUF, tc5));

    % J6: full rotation, internal +180 remap
    tc6 = round((angle6 + 180) * J6_countsPerDeg);
    tc6 = max(J6_TARGET_MARGIN, min(J6_maxCount - J6_TARGET_MARGIN, tc6));

    newTargetCounts = [tc1, tc2, tc3, tc4, tc5, tc6];
    currentCounts   = [count1, count2, count3, count4, count5, count6];

    % -------------------------------------------------------------------------
    % 2. NEW MOVE DETECTION
    %    If target counts changed from last tick, freeze everything and
    %    set up a fresh move. Matching the J-file "lastTarget" pattern.
    % -------------------------------------------------------------------------
    targetChanged = any(newTargetCounts ~= lastTargetCount);

    if targetChanged
        % Per-joint count deltas
        deltas     = newTargetCounts - currentCounts;
        stepTarget = abs(deltas);          % counts each joint must travel

        % ---------------------------------------------------------------
        % PULSE-BASED DDA SIZING (fixes J5 lag / arrives-late bug)
        % Each pulse moves the encoder 1/TUNE_C2R counts, so the number of
        % PULSES a joint needs is countDelta * TUNE_C2R. J5 (TUNE=1.6) needs
        % MORE pulses per count than the others (TUNE=0.8); if the DDA is
        % sized in raw counts, J5 is starved of pulses and finishes late.
        % Distribute the DDA by PULSES NEEDED so all joints finish together.
        % ---------------------------------------------------------------
        pulseTarget = zeros(1,6);
        for i = 1:6
            pulseTarget(i) = round(stepTarget(i) * tuneArr(i));
        end

        % highStep = max PULSES across joints (limiting joint sets duration).
        % DDA increment pulseTarget(i) <= highStep guarantees <=1 pulse/tick.
        highStep = double(max(pulseTarget(:)));

        % C2R per joint (pulse-rate factor, includes per-joint TUNE_C2R)
        c2r_all = zeros(1,6);
        c2r_all(1) = ((J1_maxCount/J1_TOTAL_DEG*360)/CPR * CPR)/60 * J1_TUNE_C2R;
        c2r_all(2) = ((J2_maxCount/J2_TOTAL_DEG*360)/CPR * CPR)/60 * J2_TUNE_C2R;
        c2r_all(3) = ((J3_maxCount/J3_TOTAL_DEG*360)/CPR * CPR)/60 * J3_TUNE_C2R;
        c2r_all(4) = ((J4_maxCount/J4_TOTAL_DEG*360)/CPR * CPR)/60 * J4_TUNE_C2R;
        c2r_all(5) = ((J5_maxCount/J5_TOTAL_DEG*360)/CPR * CPR)/60 * J5_TUNE_C2R;
        c2r_all(6) = ((J6_maxCount/360      *360)/CPR * CPR)/60 * J6_TUNE_C2R;

        % Master clock rate from the joint with the most PULSES
        masterJoint = 1;
        for mj = 1:6
            if pulseTarget(mj) >= pulseTarget(masterJoint)
                masterJoint = mj;
            end
        end
        C2R_master = c2r_all(masterJoint);
        masterIdx  = masterJoint;              % joint that defines the timebase
        remSmooth  = double(pulseTarget(masterJoint));   % seed = full move

        % Reset master clock and per-joint DDA counters
        masterTick = 0;
        ddaAccum   = zeros(1,6);
        currentRPM = MIN_RPM;

        runawayFlag = false(1,6);      % fresh move, nobody is running away yet

        % Freeze absolute target counts for encoder-based stop
        targetCountFrozen = newTargetCounts;

        lastTargetCount = newTargetCounts;
    end

    % -------------------------------------------------------------------------
    % 3. PER-TICK DIRECTION SERVO + ENCODER-BASED STOP
    %    Direction is recomputed EVERY tick from live encoder error (like the
    %    J-blocks), so a joint that overshoots reverses and settles instead of
    %    running away. "Reached" is within STOP_COUNTS. The DDA (below) still
    %    governs coordinated TIMING; this only governs direction + stop.
    %
    %    countError(i) = targetCountFrozen(i) - liveCount(i)
    %    Per-joint dir convention (from each J-file, NOT uniform):
    %      J1: err>=0 -> dir=1 | J2: err>=0 -> dir=1 | J3: err>=0 -> dir=0
    %      J4: err>=0 -> dir=1 | J5: err>=0 -> dir=1 | J6: err>=0 -> dir=1
    % -------------------------------------------------------------------------
    liveCounts   = [count1, count2, count3, count4, count5, count6];
    jointReached = false(1,6);
    countErr     = zeros(1,6);
    % Per-joint arrival tolerance: degrees -> counts, floored
    stopBand = zeros(1,6);
    stopBand(1) = max(MIN_STOP_COUNTS, round(TOL_DEG_J1 * J1_countsPerDeg));
    stopBand(2) = max(MIN_STOP_COUNTS, round(TOL_DEG_J2 * J2_countsPerDeg));
    stopBand(3) = max(MIN_STOP_COUNTS, round(TOL_DEG_J3 * J3_countsPerDeg));
    stopBand(4) = max(MIN_STOP_COUNTS, round(TOL_DEG_J4 * J4_countsPerDeg));
    stopBand(5) = max(MIN_STOP_COUNTS, round(TOL_DEG_J5 * J5_countsPerDeg));
    stopBand(6) = max(MIN_STOP_COUNTS, round(TOL_DEG_J6 * J6_countsPerDeg));

    for i = 1:6
        countErr(i) = targetCountFrozen(i) - liveCounts(i);
        if abs(countErr(i)) <= stopBand(i)
            jointReached(i) = true;
        end
    end

    % Runaway detection: error larger than the commanded travel + margin means
    % this joint is being driven the wrong way.
    for i = 1:6
        if abs(countErr(i)) > (stepTarget(i) + RUNAWAY_MARGIN)
            runawayFlag(i) = true;
        end
    end

    % Recompute direction each tick (servo). Convention per joint.
    if countErr(1) >= 0, dirs(1) = 1; else, dirs(1) = 0; end   % J1
    if countErr(2) >= 0, dirs(2) = 1; else, dirs(2) = 0; end   % J2
    if countErr(3) >= 0, dirs(3) = 0; else, dirs(3) = 1; end   % J3
    if countErr(4) >= 0, dirs(4) = 1; else, dirs(4) = 0; end   % J4
    if countErr(5) >= 0, dirs(5) = 1; else, dirs(5) = 0; end   % J5
    if countErr(6) >= 0, dirs(6) = 1; else, dirs(6) = 0; end   % J6

    if highStep <= 0 || all(jointReached)
        done = 1;
        dir1 = dirs(1); dir2 = dirs(2); dir3 = dirs(3);
        dir4 = dirs(4); dir5 = dirs(5); dir6 = dirs(6);
        return;
    end

    % -------------------------------------------------------------------------
    % 4. SHARED S-CURVE: compute currentRPM for this master tick
    %    One accel->cruise->decel profile driven by masterTick vs highStep.
    %    Uses same cosine-blend S-curve shape as J1-J6 blocks for consistency.
    % -------------------------------------------------------------------------
    ACCStep = highStep * ACC_FRAC;
    DCCStep = highStep * DCC_FRAC;

    % fullRampCounts: how many master ticks a full accel ramp takes at speedRPM
    accel = max(accelRPMperSec, 0.1);
    fullRampCounts = 0.5 * (MIN_RPM + speedRPM) * C2R_master * (speedRPM - MIN_RPM) / accel;

    % If move is short, reduce peakRPM so ramps fit
    if highStep > 0 && 2 * fullRampCounts >= highStep
        peakRPM = sqrt(max(MIN_RPM^2 + highStep * accel / C2R_master, MIN_RPM^2));
        peakRPM = max(MIN_RPM, min(speedRPM, peakRPM));
        rampCounts = 0.5*(MIN_RPM+peakRPM)*C2R_master*(peakRPM-MIN_RPM)/accel;
        rampCounts = max(1, rampCounts);
    else
        peakRPM    = speedRPM;
        rampCounts = max(1, fullRampCounts);
    end

    % ---------------------------------------------------------------------
    % SELF-CORRECTING DECEL (mirrors the J-file method)
    %
    % Decel is driven by the MASTER joint's LIVE remaining error, converted to
    % pulses, instead of (highStep - masterTick). This makes the profile
    % closed-loop like the J-blocks: if the joint lags, decel waits; if it
    % arrives early, decel is already finished. masterTick can no longer run
    % past highStep and pin the clock at MIN_RPM.
    %
    % Only the MASTER joint's error is used -- deliberately NOT max() across
    % all six. A max() switches which joint it tracks mid-move, which makes the
    % signal jump and destroys smoothness. One joint = one monotonic signal.
    %
    % Encoder counts are integers, so the raw signal is quantised; it is
    % low-pass filtered (same idea as ALPHA on displaySpeed in the J-files)
    % so the ramp glides instead of stepping.
    % ---------------------------------------------------------------------
    REM_ALPHA = 0.02;   % smoothing on the measured error (smaller = smoother)

    remRaw    = abs(countErr(masterIdx)) * tuneArr(masterIdx);   % pulses to go
    remSmooth = remSmooth + REM_ALPHA * (remRaw - remSmooth);

    % Decel checked FIRST (as the J-files do): arrival always wins over ramp-up.
    if remSmooth <= rampCounts
        x = max(0.0, min(1.0, remSmooth / rampCounts));
        targetRPM = MIN_RPM + (peakRPM - MIN_RPM) * 0.5 * (1 - cos(pi * x));
    elseif masterTick <= rampCounts
        x = max(0.0, min(1.0, masterTick / rampCounts));
        targetRPM = MIN_RPM + (peakRPM - MIN_RPM) * 0.5 * (1 - cos(pi * x));
    else
        targetRPM = peakRPM;
    end
    currentRPM = max(MIN_RPM, min(speedRPM, targetRPM));

    % -------------------------------------------------------------------------
    % 5. MASTER TICK: should we fire a pulse this Simulink timestep?
    %    DDS accumulator on the MASTER clock only.
    %    Each Simulink tick, accumulate currentRPM * C2R_master * Ts.
    %    When accumulator >= 1.0, fire one master tick and run Bresenham.
    % -------------------------------------------------------------------------
    if targetChanged
        masterAccum = 0;
    end

    masterAccum = masterAccum + currentRPM * C2R_master * Ts;

    if masterAccum < 1.0
        % No master tick this Simulink timestep — hold outputs
        dir1 = dirs(1); dir2 = dirs(2); dir3 = dirs(3);
        dir4 = dirs(4); dir5 = dirs(5); dir6 = dirs(6);
        return;
    end
    masterAccum = masterAccum - 1.0;
    masterTick  = masterTick + 1;

    % -------------------------------------------------------------------------
    % 6. DDA PROPORTIONAL STEPPING with ENCODER-BASED STOP
    %    The DDA governs proportional TIMING (keeps joints coordinated), but a
    %    joint stops firing once its LIVE encoder count reaches target -- NOT
    %    when a pulse tally is met. This self-corrects for pulse:count ratio
    %    (TUNE_C2R etc.), exactly like the individual J-blocks do.
    % -------------------------------------------------------------------------
    pulses = zeros(1,6);

    for i = 1:6
        if jointReached(i)
            continue;                      % encoder reached target -> stop
        end
        if runawayFlag(i)
            continue;                      % wrong direction -> refuse to drive
        end

        % A joint that is NOT yet reached must always be able to step. If its
        % frozen pulseTarget rounded to 0 (its delta was ~0 when the move was
        % set up) the old code skipped it forever, so it could never correct
        % drift and 'done' could never fire. Give it the full rate instead.
        inc = pulseTarget(i);
        if inc <= 0
            inc = highStep;
        end

        ddaAccum(i) = ddaAccum(i) + inc;
        if ddaAccum(i) >= highStep
            ddaAccum(i) = ddaAccum(i) - highStep;
            pulses(i)   = 1;               % fire a step this tick
        end
    end

    % -------------------------------------------------------------------------
    % 7. ASSIGN OUTPUTS
    % -------------------------------------------------------------------------
    pul1 = pulses(1); pul2 = pulses(2); pul3 = pulses(3);
    pul4 = pulses(4); pul5 = pulses(5); pul6 = pulses(6);
    dir1 = dirs(1);   dir2 = dirs(2);   dir3 = dirs(3);
    dir4 = dirs(4);   dir5 = dirs(5);   dir6 = dirs(6);

    done = double(all(jointReached));

end
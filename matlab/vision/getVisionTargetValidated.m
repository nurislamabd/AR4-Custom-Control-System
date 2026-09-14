function [X, Y, colorID, found] = getVisionTargetValidated(cam, cropRegion, tform)
% Two captures separated by WAIT_S. Accept only if both find the same color
% at approximately the same world position -- guards against motion blur,
% a block still settling, or a transient false detection.

TOL_MM = 5;     % max allowed disagreement between the two readings
WAIT_S = 1;     % pause between the two photos

X = 0; Y = 0; colorID = 0; found = false;

[u1, v1, c1, f1] = detectBlockColor(imcrop(snapshot(cam), cropRegion));
if ~f1, return; end

pause(WAIT_S);

[u2, v2, c2, f2] = detectBlockColor(imcrop(snapshot(cam), cropRegion));
if ~f2, return; end

% Compare in world mm, not pixels, so TOL_MM is a physical tolerance
w1 = transformPointsForward(tform, [u1 v1]);
w2 = transformPointsForward(tform, [u2 v2]);
d  = hypot(w1(1)-w2(1), w1(2)-w2(2));

if c1 == c2 && d <= TOL_MM
    X = (w1(1) + w2(1)) / 2;    % average the two readings
    Y = (w1(2) + w2(2)) / 2;
    colorID = c1;
    found = true;
end
% else: leave found = false -> caller skips this cycle and retries
end
function [X, Y, colorID, found] = getVisionTarget(cam, cropRegion, tform)
    img = imcrop(snapshot(cam), cropRegion);
    [u, v, colorID, found] = detectBlockColor(img);   % <-- new function, new colorID output

    if ~found
        X = 0; Y = 0; colorID = 0;
        return;
    end

    worldPt = transformPointsForward(tform, [u v]);
    X = worldPt(1);
    Y = worldPt(2);
end
function [u, v, colorID, found] = detectBlockColor(img)
    [maskA, ~] = createMaskA(img);
    [maskB, ~] = createMaskB(img);
    maskA = bwareaopen(maskA, 200);
    maskB = bwareaopen(maskB, 200);
    
    statsA = regionprops(maskA, 'Centroid', 'Area');
    statsB = regionprops(maskB, 'Centroid', 'Area');
    
    % Decide which color to service this cycle: prefer A, fall back to B
    if ~isempty(statsA)
        stats = statsA; colorID = 1;
    elseif ~isempty(statsB)
        stats = statsB; colorID = 2;
    else
        u = 0; v = 0; colorID = 0; found = false; return;
    end
    
    % --- selection rule among same-color blocks ---
    % pick the LEFT-MOST in the image (smallest u pixel).
    centroids = cat(1, stats.Centroid);   % Nx2, columns [u v]
    [~, i] = min(centroids(:,1));          % <-- change this line to change the rule
    
    u = centroids(i,1);
    v = centroids(i,2);
    found = true;
end
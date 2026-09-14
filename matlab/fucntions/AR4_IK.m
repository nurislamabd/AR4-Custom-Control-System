function [J1, J2, J3, J4, J5, J6] = AR4_IK(px, py, pz)
%AR4_IK  Analytical inverse kinematics for the AR4 MK3 6-axis robot arm.
%
%   [J1,J2,J3,J4,J5,J6] = AR4_IK(px, py, pz)
%
%   INPUTS
%     px, py, pz  – TCP position in mm
%
%   OUTPUTS
%     J1 … J6  – Joint command angles in DEGREES
%                (zero = mechanical home, matches AR4 software convention)
%
%   ORIENTATION
%     Hardcoded for top-down pick & place:
%       Rz = 180°, Ry = 0°, Rx = 180°  (gripper pointing straight down)
%     Change Rz_deg / Ry_deg / Rx_deg below if a different tool orientation
%     is needed.
%
%   DH PARAMETERS  (standard DH, from AR4 MK3 spreadsheet)
%     Joint | alpha      | d (mm)  | a (mm)
%     ------+------------+---------+-------
%       1   | -pi/2      | 169.77  |  64.2
%       2   |  0         |   0     | 305.0
%       3   |  pi/2      |   0     |   0
%       4   | -pi/2      | 222.63  |   0
%       5   |  pi/2      |   0     |   0
%       6   |  0         |  41.0   |   0
%
%   DH THETA OFFSETS  (DH_theta = cmd_angle_deg + offset_deg)
%     J1:  0°   J2: -90°   J3: 180°   J4: 0°   J5: 0°   J6: 0°
%
%     NOTE on J5: the offset is 0°, NOT +90°. This matches the AR4
%     software output. At mechanical home (J5_cmd = 90°), DH_theta = 90°.
%
%   JOINT LIMITS (command degrees)
%     J1 ±170   J2 -42/+90   J3 -89/+52
%     J4 ±165   J5 ±105      J6 ±155

% ── Link parameters ──────────────────────────────────────────────────────
a1 = 64.2;
d1 = 169.77;
a2 = 305.0;
d4 = 222.63;
d6 = 41.0;

% ── Tool orientation (change here if needed) ─────────────────────────────
Rz_deg = 180;
Ry_deg = 0;
Rx_deg = 180;
elbow_up = true;   % true = elbow-up (normal AR4 posture)

% ── Build R0_6 from ZYX extrinsic Euler angles ───────────────────────────
rz = Rz_deg * (pi/180);
ry = Ry_deg * (pi/180);
rx = Rx_deg * (pi/180);

Crz = cos(rz);  Srz = sin(rz);
Cry = cos(ry);  Sry = sin(ry);
Crx = cos(rx);  Srx = sin(rx);

R06 = [Crz*Cry,  Crz*Sry*Srx - Srz*Crx,  Crz*Sry*Crx + Srz*Srx; ...
       Srz*Cry,  Srz*Sry*Srx + Crz*Crx,  Srz*Sry*Crx - Crz*Srx; ...
      -Sry,      Cry*Srx,                 Cry*Crx               ];

% ── Wrist centre ─────────────────────────────────────────────────────────
% Pw = P_tcp - d6 * z_tool   (z_tool = 3rd column of R0_6)
approach = R06(:, 3);
Pw = [px; py; pz] - d6 * approach;
Wx = Pw(1);  Wy = Pw(2);  Wz = Pw(3);

% ── Joint 1 ──────────────────────────────────────────────────────────────
J1_rad = atan2(Wy, Wx);   % DH offset for J1 = 0

% ── Joints 2 & 3 (planar arm solution) ───────────────────────────────────
r = sqrt(Wx^2 + Wy^2) - a1;   % horizontal reach beyond J1 offset
s = Wz - d1;                   % vertical height above J2 axis

D = (r^2 + s^2 - a2^2 - d4^2) / (2 * a2 * d4);
D = max(-1.0, min(1.0, D));    % clamp numerical noise to [-1,1]

if elbow_up
    J3_raw = atan2(-sqrt(1 - D^2),  D);   % elbow-up: negative sqrt
else
    J3_raw = atan2( sqrt(1 - D^2),  D);   % elbow-down
end

J2_raw = atan2(s, r) - atan2(d4 * sin(J3_raw), a2 + d4 * cos(J3_raw));

% Convert geometric angles → command angles (remove DH theta offsets):
%   J2: DH offset = -pi/2  →  cmd = pi/2 - J2_raw
%   J3: DH offset =  pi    →  cmd = -J3_raw - pi/2
J2_rad = pi/2 - J2_raw;
J3_rad = -J3_raw - pi/2;

% ── Joints 4, 5, 6 (spherical wrist) ─────────────────────────────────────
R03 = dh_R03(J1_rad, J2_rad, J3_rad, a1, d1, a2);

R36 = R03' * R06;

% ZYZ-like wrist decomposition from R3_6:
J5_raw = atan2(sqrt(R36(1,3)^2 + R36(2,3)^2), R36(3,3));

if abs(sin(J5_raw)) > 1e-6
    J4_rad = atan2( R36(2,3) / sin(J5_raw),  R36(1,3) / sin(J5_raw));
    J6_rad = atan2( R36(3,2) / sin(J5_raw), -R36(3,1) / sin(J5_raw));
else
    % Gimbal lock (J5 ≈ 0° or 180°): absorb singularity into J4 = 0
    J4_rad = 0;
    if R36(3,3) > 0
        J6_rad = atan2(-R36(2,1),  R36(1,1));
    else
        J6_rad = atan2( R36(2,1), -R36(1,1));
    end
end

% J5 DH theta offset = 0  →  cmd = raw  (matches AR4 software)
J5_rad = J5_raw;

% ── Convert to degrees ────────────────────────────────────────────────────
J1 = J1_rad * (180/pi);
J2 = J2_rad * (180/pi);
J3 = J3_rad * (180/pi);
J4 = J4_rad * (180/pi);
J5 = J5_rad * (180/pi);
J6 = J6_rad * (180/pi);

end  % AR4_IK


% =========================================================================
%  Helper: rotation matrix R0_3  (joints 1-3 only)
% =========================================================================
function R03 = dh_R03(q1, q2, q3, a1, d1, a2)
%DH_R03  3×3 rotation submatrix of T0_3.
%  q1,q2,q3 are COMMAND angles in radians.

% DH theta offsets for joints 1-3
off = [0, -pi/2, pi];

% alpha for joints 1-3
al = [-pi/2, 0, pi/2];

% d for joints 1-3
dv = [d1, 0, 0];

% a for joints 1-3
av = [a1, a2, 0];

T = eye(4);
qs = [q1, q2, q3];

for i = 1:3
    th = qs(i) + off(i);
    ct = cos(th);  st = sin(th);
    ca = cos(al(i));  sa = sin(al(i));
    Ai = [ct, -st*ca,  st*sa, av(i)*ct; ...
          st,  ct*ca, -ct*sa, av(i)*st; ...
           0,     sa,     ca,     dv(i); ...
           0,      0,      0,        1];
    T = T * Ai;
end

R03 = T(1:3, 1:3);
end
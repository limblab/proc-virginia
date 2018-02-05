function[R_y] = yaw2Rmat(y)
% Obtains rotation matrix (Rmat) from yaw (y) around Z axis

R_y = [cosd(y) -sind(y) 0; sind(y) cosd(y) 0; 0 0 1];

end
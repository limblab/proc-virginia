function[R_y] = yaw2Rmat(y)

R_y = [cosd(y) -sind(y) 0; sind(y) cosd(y) 0; 0 0 1];

end
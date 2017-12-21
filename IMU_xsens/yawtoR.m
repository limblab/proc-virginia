function[R_y] = yawtoR(y)

R_y = [cosd(y) -sind(y) 0; sind(y) cosd(y) 0; 0 0 1];

end
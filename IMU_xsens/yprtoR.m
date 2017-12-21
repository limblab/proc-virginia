function[Rmat] = yprtoR(y,p,r) 

R_y = [cosd(y) -sind(y) 0; sind(y) cosd(y) 0; 0 0 1];
R_p = [cosd(p) 0 sind(p); 0 1 0; -sind(p) 0 cosd(p)];
R_r = [1 0 0; 0 cosd(r) -sind(r); 0 sind(r) cosd(r)];

Rmat = R_y*R_p*R_r;

% Rmat =
% [ cos(p)*cos(y), cos(y)*sin(p)*sin(r) - cos(r)*sin(y), sin(r)*sin(y) + cos(r)*cos(y)*sin(p)]
% [ cos(p)*sin(y), cos(r)*cos(y) + sin(p)*sin(r)*sin(y), cos(r)*sin(p)*sin(y) - cos(y)*sin(r)]
% [       -sin(p),                        cos(p)*sin(r),                        cos(p)*cos(r)]

%% Roll around y
% R_y = [cosd(y) -sind(y) 0; sind(y) cosd(y) 0; 0 0 1];
% R_r = [cosd(r) 0 sind(r); 0 1 0; -sind(r) 0 cosd(r)];
% R_p = [1 0 0; 0 cosd(p) -sind(p); 0 sind(p) cosd(p)];

% Rmat =
% [ cos(r)*cos(y) - sin(p)*sin(r)*sin(y), -cos(p)*sin(y), cos(y)*sin(r) + cos(r)*sin(p)*sin(y)]
% [ cos(r)*sin(y) + cos(y)*sin(p)*sin(r),  cos(p)*cos(y), sin(r)*sin(y) - cos(r)*cos(y)*sin(p)]
% [                       -cos(p)*sin(r),         sin(p),                        cos(p)*cos(r)]

end

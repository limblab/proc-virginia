function[Rmat] = ypr2Rmat(y,p,r) 
% Obtains rotation matrix (Rmat) from Euler angles (y, p, r) using XYZ
% order

R_y = [cosd(y) -sind(y) 0; sind(y) cosd(y) 0; 0 0 1];
R_p = [cosd(p) 0 sind(p); 0 1 0; -sind(p) 0 cosd(p)];
R_r = [1 0 0; 0 cosd(r) -sind(r); 0 sind(r) cosd(r)];

Rmat = R_y*R_p*R_r;

% Rmat =
% [ cos(p)*cos(y), cos(y)*sin(p)*sin(r) - cos(r)*sin(y), sin(r)*sin(y) + cos(r)*cos(y)*sin(p)]
% [ cos(p)*sin(y), cos(r)*cos(y) + sin(p)*sin(r)*sin(y), cos(r)*sin(p)*sin(y) - cos(y)*sin(r)]
% [       -sin(p),                        cos(p)*sin(r),                        cos(p)*cos(r)]

end

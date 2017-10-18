function[Rmat] = Rotypr(y,p,r)

R_y = [cosd(y) -sind(y) 0; sind(y) cosd(y) 0; 0 0 1];
R_p = [cosd(p) 0 sind(p); 0 1 0; -sind(p) 0 cosd(p)];
R_r = [1 0 0; 0 cosd(r) -sind(r); 0 sind(r) cosd(r)];

Rmat = R_y*R_p*R_r;

% Rmat = [cosd(y).*cosd(p), cosd(y).*sind(p).*sind(r)-sind(y).*cosd(r), cosd(y).*sind(p).*cosd(r)+sind(y).*sind(r);...
%     sind(y).*cosd(p), sind(y).*sind(p).*sind(r)+cosd(y).*cosd(r), sind(y).*sind(p).*cosd(r)-cosd(y).*sind(r);...
%     -sind(p), cosd(p).*sind(r), cosd(p).*cosd(r)];

end

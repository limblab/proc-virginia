function[yw,pt,rl] = Rmat2ypr(Rmat) 
% Obtains Euler angles (yw, pt, rl) from rotation matrix (Rmat) taking into
% account possible singularities at +/-90 deg pitch

if (Rmat(3,1)~=1) || (Rmat(3,1)~=-1)
    pt = asin(-Rmat(3,1));
    rl = atan2(Rmat(3,2)/cos(pt), Rmat(3,3)/cos(pt));
    yw = atan2(Rmat(2,1)/cos(pt), Rmat(1,1)/cos(pt));
else
    if Rmat(3,1)==-1 
        yw = 0;
        pt = pi/2;
        rl = yw + atan2(Rmat(1,2),Rmat(1,3));
    else
        yw = 0;
        pt = -pi/2;
        rl = -yw + atan2(-Rmat(1,2),-Rmat(1,3));
    end
end

yw = rad2deg(yw);
pt = rad2deg(pt);
rl = rad2deg(rl);

% p = asind(-Rmat(3,1));
% r = acosd(Rmat(3,3)/cosd(p));
% y = acosd(Rmat(1,1)/cosd(p));

% Rmat =
% [ cos(p)*cos(y), cos(y)*sin(p)*sin(r) - cos(r)*sin(y), sin(r)*sin(y) + cos(r)*cos(y)*sin(p)]
% [ cos(p)*sin(y), cos(r)*cos(y) + sin(p)*sin(r)*sin(y), cos(r)*sin(p)*sin(y) - cos(y)*sin(r)]
% [       -sin(p),                        cos(p)*sin(r),                        cos(p)*cos(r)]

end
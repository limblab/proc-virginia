function[Rmat] = vec2Rmat(v1,v2)
% Obtains rotation matrix (Rmat) between two vectors (v1, v2)

angle = acos((v1*v2')/(norm(v1)*norm(v2)));
axis = cross(v1,v2)/norm(cross(v1,v2));

axismat = [ 0 -axis(3) axis(2) ; axis(3) 0 -axis(1) ; -axis(2) axis(1) 0];

% Rodrigues formula for rotation matrix
Rmat = eye(3) + sin(angle)*axismat + (1-cos(angle))*axismat*axismat;

end
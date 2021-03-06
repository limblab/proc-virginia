function[Rmat] = quat2Rmat(q0,q1,q2,q3) 
% Obtains rotation matrix (Rmat) from quaternion (q0, q1, q2, q3)

qw = q0;
qx = q1;
qy = q2;
qz = q3;

sqx = qx*qx;
sqy = qy*qy;
sqz = qz*qz;

Rmat = [1-2*(sqy+sqz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw);...
    2*(qx*qy+qz*qw), 1-2*(sqx+sqz), 2*(qy*qz-qx*qw);...
    2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(sqx+sqy)];

end
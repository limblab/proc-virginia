function[yw,pt,rl] = quat2ypr(q) 
% Obtains Euler angles (yw, pt, rl) from quaternion (q containing q0, q1, q2
% q3)

q1.w = q.q0;
q1.x = q.q1;
q1.y = q.q2;
q1.z = q.q3;

test = q1.x*q1.y + q1.z*q1.w;

sqw = q1.w*q1.w;
sqx = q1.x*q1.x;
sqy = q1.y*q1.y;
sqz = q1.z*q1.z;

unit = sqx + sqy + sqz + sqw;

if (test > 0.499*unit) % Singularity at +90 deg pitch
    yw = 2 * atan2(q1.x,q1.w);
    pt = pi/2;
    rl = 0;
elseif (test < -0.499*unit) % Singularity at -90 deg pitch
    yw = -2 * atan2(q1.x,q1.w);
    pt = - pi/2;
    rl = 0;
else
    yw = atan2(2*q1.y*q1.w-2*q1.x*q1.z , sqx - sqy - sqz + sqw);
    pt = asin(2*test/unit);
    rl = atan2(2*q1.x*q1.w-2*q1.y*q1.z , -sqx + sqy - sqz + sqw);
end

yw = rad2deg(yw);
pt = rad2deg(pt);
rl = rad2deg(rl);

end
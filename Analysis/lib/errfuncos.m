function [e,ymod] = errfuncos(par)

global binsg fb

% Define parameters from parameter vector
a = par(1);
b = par(2);
c = par(3);

% Define estimated y: ymod
ymod = a*cosd(binsg-c)+b;      

% Define error: e
e = (fb-ymod);  
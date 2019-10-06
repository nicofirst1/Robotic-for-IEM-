function [p,R] = fk(q)
%FK Summary of this function goes here
%   Detailed explanation goes here

[robot, serial]=initializer();

T=trans_mat(robot,[1 robot.ndof]);
T=vec_subs(T,"all",q);
p=T(1:3,4);
R=T(1:3,1:3);
end


function [robot,serial] = initializer()


% Link(DH, OPTIONS) is a link object using the specified kinematic
% convention  and with parameters:
%  - DH = [THETA D A ALPHA SIGMA OFFSET] where SIGMA=0 for a revolute and 1
%    for a prismatic joint; and OFFSET is a constant displacement between the
%    user joint variable and the value used by the kinematic model.
%  - DH = [THETA D A ALPHA SIGMA] where OFFSET is zero.
%  - DH = [THETA D A ALPHA], joint is assumed revolute and OFFSET is zero.
%Options::
%
% 'standard'    for standard D&H parameters (default).
% 'modified'    for modified D&H parameters.
% 'revolute'    for a revolute joint, can be abbreviated to 'r' (default)
% 'prismatic'   for a prismatic joint, can be abbreviated to 'p'


[DH,mdh]=dh_params();

L(1)=Link(DH(1,1:6), mdh);
L(2)=Link(DH(2,1:6), mdh);
L(3)=Link(DH(3,1:6), mdh);
L(4)=Link(DH(4,1:6), mdh);
L(5)=Link(DH(5,1:6), mdh);

% use robot with tool tip 
robot=cork2mine(DH(1:5,:),3,mdh,DH(6,:));
% initialize tool tip 
serial= SerialLink(L);
% get homo of last joint

% set it to tool tip 
serial.tool=SE3(robot.tool);





end




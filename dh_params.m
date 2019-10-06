

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

l1=17;
l2=17;
l3=7;
l4=4;
l5=4;
l6=9;

type='revolute';
mdh='modified';

DH=[
%THETA  D     A     ALPHA   SIGMA   OFFSET
   0    l1    0     0       0       0   ;
   0    0     0     pi/2    0       pi/2;
   0    0     l2    0       0       pi/2;
   0    l3+l5 l4    pi/2    0       pi/2;
   0    0     0     pi/2    0       0;
   %pi/2 l6    0     -pi/2   0       0;
   
   ];

L(1)=Link(DH(1,1:6), mdh);
L(2)=Link(DH(2,1:6), mdh);
L(3)=Link(DH(3,1:6), mdh);
L(4)=Link(DH(4,1:6), mdh);
L(5)=Link(DH(5,1:6), mdh);

robot=cork2mine(DH,3,mdh);


serial= SerialLink(L);
serial.tool=[0 0 0];


%serial.plotopt={'workspace',[-20 20 -20 20 -10 20]};
serial.qlim(2,:)=[0,10];
figure(1);
view(3);
%serial.plot([0,0,0,0,0],'scale',.5);
serial.teach();



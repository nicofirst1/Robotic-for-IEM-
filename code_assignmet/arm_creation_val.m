
% plot robot 
[robot,pArb]=initializer();

view(3);
figure(1);
pArb.plot([0,0,0,0,0],'scale',.5);

view(3);
figure(2);
pArb.plot([0,-pi/2,-pi/2,0,0],'scale',.5);





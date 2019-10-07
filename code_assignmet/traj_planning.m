[robot,pArb]=initializer();
   
% getting feasable cartesian position with forward kin
% this is done in order to get a feaseble end point in the cartesian space
Qf = [pi/3,pi/2,pi/4,pi/4,pi/6];
Tf=pArb.fkine(Qf);
Pf=Tf.t;


% configuration at start
Qi = [0, 0, 0, 0,0 ];
% transformation at start/end
Ti=pArb.fkine(Qi);
Pi=Ti.t;
% transformation at end
Tf = SE3(Pf);

% remove last joint from the inverse 
m = [1 1 1 1 0 0];   
Qf=pArb.ikine(Tf,Ti,'mask',m);


% Getting the time 
step=0.3;
total_time=3;
t =[0:step:total_time];

% estimate trak
[q,qd,qdd]=jtraj(Qi,Qf,t);

%plot_qs(q,qd,qdd,t,pArb);



function p=get_feasable_point(pArb,q)



end



function plot_qs(q,qd,qdd,t,pArb)

view(3);
pArb.plot(q);


% Configuration plots
figure(1);

% thetas position
nexttile;
plot(t,q);
title('q');

% thetas vel
nexttile;
plot(t,qd);
title('qd');

% thetas accel
nexttile;
plot(t,qdd);
title('qdd');


nexttile;
tmp=plot(t,q);
title('Legends');

legHandle =legend('q1','q2','q3','q4','q5');
set(tmp, 'visible', 'off');
legHandle.TextColor = [0 0 0];
legHandle.FontSize = 16;
end


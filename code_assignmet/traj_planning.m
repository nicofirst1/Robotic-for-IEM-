clear;

[robot,pArb]=initializer();
   
%% Initial configuration
% getting feasable cartesian position with forward kin
% this is done in order to get a feaseble end point in the cartesian space

Qi=[0,pi/2, 0, pi, pi/4];
Qf=[0,-pi/2,-pi/4,0,pi/7];

Pi= get_feasable_point(pArb,Qi);
Pf = get_feasable_point(pArb,Qf);

t=get_t(0,3,0.3);

%% Joint Trajectory

% get the values of configuration 
[q,qd,qdd]=jtraj(Qi,Qf,t);
% extract the coordinates in the end effector frame
pe=pArb.fkine(q).tv;

% plot 
figure(1);
plot_qs(q,qd,qdd,t,pe);


figure(3);

pcshow(pe);


% plot robot movement
figure(2);
view(3);
pArb.plot(q, 'loop');



%% Cartesian Trajectory

%cartesian_traj(Pi,Pf,pArb,t)


%% Utils Functions

function cartesian_traj(Pi,Pf,pArb,t)
% Plan trajectory in the cartesian space,
% Pi/Pf: start and end for the end effector cartesian position
% pArb: serialLink
% t: is the timing step


% transformation at end
Tf = SE3(Pf);
Ti = SE3(Pi);


Tc=ctraj(Ti,Tf,length(t))
Pe = transl(Tc)

plot(t,Pe);
title('End effecotr position');
legend('x','y','z');



end





function t=get_t(start_t,end_t,step)

    t =[start_t:step:end_t];
end
function p=get_feasable_point(pArb,q)
% return a feasable cartesian point given the robot and a configuration
% the configuration can be zero and then a random one in range [0, pi/2]
% will be taken


if q==0
    qs=length(pArb.a);
    q=(pi/2).*rand(qs,1) ;
end

p=pArb.fkine(q).t;

end



function plot_qs(q,qd,qdd,t,pe)


% thetas position
nexttile;
plot(t,q);
title('Joint position');
legend('q1','q2','q3','q4','q5');
xlabel("seconds");
ylabel("rad");

% thetas vel
nexttile;
plot(t,qd);
title('Joint velocity');
legend('qd1','qd2','qd3','qd4','qd5');
xlabel("seconds");
ylabel("rad/s");


% thetas accel
nexttile;
plot(t,qdd);
title('Joint accleration');
legend('qdd1','qdd2','qdd3','qdd4','qdd5');
xlabel("seconds");
ylabel("rad/s^2");


% endeffector position 
nexttile;
plot(t,pe);
title('End effector coordinates');
legend('x','y','z');
xlabel("seconds");
ylabel("coord");




end


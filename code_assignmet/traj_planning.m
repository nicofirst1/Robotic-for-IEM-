clear;

[robot,pArb]=initializer();
   
%% Initial configuration
% getting feasable cartesian position with forward kin
% this is done in order to get a feaseble end point in the cartesian space

Qi=[0,pi/2, 0, pi, pi/4];
Qf=[0,-pi/4,-pi/4,0,pi/7];

Pi= get_feasable_point(pArb,Qi);
Pf = get_feasable_point(pArb,Qf);

t=get_t(0,3,0.1);


%% Joint Trajectory


joint_traj(Qi,Qf,t,pArb);

%% Cartesian Trajectory

cartesian_traj(Pi,Pf,t,pArb)


%% Utils Functions


function joint_traj(Qi,Qf,t,pArb)
% Plan and plot trajectory in the cartesian space,
% Qi/Qf: start and end for the robot configuration
% pArb: serialLink
% t: is the timing step


% get the values of configuration 
[q,qd,qdd]=jtraj(Qi,Qf,t);
% extract the coordinates in the end effector frame
pe=pArb.fkine(q).tv;

% plot 
figure(1);
plot_qs(q,qd,qdd,t,pe);


figure(2);
scatter3(pe(1,:),pe(2,:),pe(3,:));
zlabel("Z");
ylabel("Y");
xlabel("X");
title("Cartesian coord. for end-effector")


% plot robot movement
figure(3);
view(3);
pArb.plot(q, 'loop');


end

function cartesian_traj(Pi,Pf,t,pArb)
% Plan trajectory in the cartesian space,
% Pi/Pf: start and end for the end effector cartesian position
% pArb: serialLink
% t: is the timing step


% transformation at end
Tf = SE3(Pf);
Ti = SE3(Pi);

% get list of transformation matrices
Tc=ctraj(Ti,Tf,length(t));
% get end effector position for plotting
pe = transl(Tc)';
% get robot configurations 
mask=[1 1 1 1 0 0];
q = pArb.ikine(Tc,'mask',mask);



figure(4);
scatter3(pe(1,:),pe(2,:),pe(3,:));
zlabel("Z");
ylabel("Y");
xlabel("X");
title("Cartesian coord. for end-effector")

figure(7);
zrs=zeros(1,size(q,2));
qd=[diff(q) ;zrs ];
qdd=[diff(qd); zrs];
plot_qs(q,qd,qdd,t,pe);


% plot robot movement
figure(6);
view(3);
pArb.plot(q, 'loop');


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


clear;
clc;

[robot,pArb_j]=initializer("ax18-joint");
[robot,pArb_c]=initializer("ax18-cartesian");

   
%% Initial configuration
% getting feasable cartesian position with forward kin
% this is done in order to get a feaseble end point in the cartesian space

Qi=[0,pi/2, pi/2, pi,0 ];
Qf=[pi,-pi/4,-pi/4,pi,pi];

Pi= get_feasable_point(pArb_j,Qi);
Pf = get_feasable_point(pArb_j,Qf);

t=get_t(0,3,0.1);

fprintf("Trajectory will consist of %d points\n",length(t));


%% Joint Trajectory


joint_traj(Qi,Qf,t,pArb_j);

%% Cartesian Trajectory

cartesian_traj(Pi,Pf,t,pArb_c)


%% Utils Functions


function joint_traj(Qi,Qf,t,pArb)
% Plan and plot trajectory in the cartesian space,
% Qi/Qf: start and end for the robot configuration
% pArb: serialLink
% t: is the timing step

disp("############################################")
disp("Estimating trajectory in Joint space...")
disp("############################################")

% get the values of configuration 
[q,qd,qdd]=jtraj(Qi,Qf,t);
% extract the coordinates in the end effector frame
pe=pArb.fkine(q).tv;

%% Plots 
disp("Plotting position, velocities and accelaration...")
figure(1);
plot_qs(q,qd,qdd,t,pe,'[Joint Traj]');


figure(2);
disp("Plotting end point position in cartesian space...")
scatter3(pe(1,:),pe(2,:),pe(3,:));
zlabel("Z");
ylabel("Y");
xlabel("X");
title("[Joint Traj] Cartesian coord. for end-effector in Joint")


% plot robot movement
disp("Plotting robot movement...")
figure(3);
view(3);
title("[Joint Traj] Joint Trajectory")
pArb.plot(q);



end

function cartesian_traj(Pi,Pf,t,pArb)
% Plan trajectory in the cartesian space,
% Pi/Pf: start and end for the end effector cartesian position
% pArb: serialLink
% t: is the timing step

disp("############################################")
disp("Estimating trajectory in Cartesian space...")
disp("############################################")

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

%% Plots 

disp("Plotting end point position in cartesian space...")
figure(4);
scatter3(pe(1,:),pe(2,:),pe(3,:));
zlabel("Z");
ylabel("Y");
xlabel("X");
title("[Cart. Traj] Cartesian coord. for end-effector")


disp("Plotting position, velocities and accelaration...")
figure(7);
zrs=zeros(1,size(q,2));
qd=[diff(q) ;zrs ];
qdd=[diff(qd); zrs];
plot_qs(q,qd,qdd,t,pe,'[Cart. Traj]');


% plot robot movement
disp("Plotting robot movement...")
figure(6);
view(3);
title("[Cart. Traj] Cartesian Trajectory")
pArb.plot(q);


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



function plot_qs(q,qd,qdd,t,pe,tit)


% thetas position
nexttile;
plot(t,q);
title(strcat(tit,' Joint position'));
legend('q1','q2','q3','q4','q5');
xlabel("seconds");
ylabel("rad");

% thetas vel
nexttile;
plot(t,qd);
title(strcat(tit,' Joint velocity'));
legend('qd1','qd2','qd3','qd4','qd5');
xlabel("seconds");
ylabel("rad/s");


% thetas accel
nexttile;
plot(t,qdd);
title(strcat(tit,' Joint accleration'));
legend('qdd1','qdd2','qdd3','qdd4','qdd5');
xlabel("seconds");
ylabel("rad/s^2");


% endeffector position 
nexttile;
plot(t,pe);
title(strcat(tit,' End effector coordinates'));
legend('x','y','z');
xlabel("seconds");
ylabel("coord");




end


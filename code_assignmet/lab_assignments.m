clear;
clc;

[robot,pArb]=initializer("ax18");

   
%% Initial configuration
% getting feasable cartesian position with forward kin
% this is done in order to get a feaseble end point in the cartesian space
% below you can find the use of custom function ik, which is not effective
% since for some points the robot may be outside of its working space.
% During the lab course the Qi/Qf lines can be discarded

Qi=[0,0, pi/2, 0,0 ];
Qf=[pi/2,-pi/4,-pi/6,pi/2,pi/3];

Pi= get_feasable_point(pArb,Qi);
Pf = get_feasable_point(pArb,Qf);

qi=ik(Pi);
qf=ik(Pf);


disp("Starting point:");
disp(Pi);
disp("Ending point:");
disp(Pf);

start_t=0;
end_t=3;
step=0.1;

sprintf("Trajectory time starts at %d and ends at %d with a step of %d seconds",start_t,end_t,step)

t=get_t(0,3,0.1);



%% Cartesian Trajectory
cartesian_traj(Pi,Pf,t,pArb,robot)


%% Utils Functions



function cartesian_traj(Pi,Pf,t,pArb,robot)
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

%%%%%%%%%%%
% Uncomment get_joint_coord to get different movements, it's slow
%%%%%%%%%%%

%j_points=get_joint_coord(q,robot);
load('j_points.mat');
plot_joint_pos(j_points);

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
title("Cartesian Trajectory")
pArb.plot(q);


end



function plot_joint_pos(j_points)


figure(8)
title("Joints trajectory in Cartesian space")
view(3);
hold on;

label=["Joint 1","Joint 2","Joint 3","Joint 4","Joint 5","tool"];
S = repmat([70,50,40,30,20,10],size(j_points,2),1);
C = repmat([1,2,3,4,5,7],size(j_points,2),1);
%iterate over every theta
for idx=1:size(j_points,3)
    
   
    scatter3(j_points(1,:,idx),j_points(2,:,idx),j_points(3,:,idx),S(:,idx),C(:,idx),'fill')
   
end
legend('label',label)


end

function j_points=get_joint_coord(qs,robot)

    j_points=zeros(3,size(qs,1),size(qs,2));
    disp("Estimating joint position for plotting...")

    % iterate on all joints
    for idx=1:size(qs,2)+1

        fprintf("\r")
        fprintf('Iteration %d/%d',idx,size(qs,2)+1); % delete previous counter display


        % getting transaltion matrix for current joint
        pe=trans_from_robot(robot,[1 idx]);
        % get position
        pe=pe*[0,0,0,1]';
        % remove last elem to have xyz
        pe=pe(1:3,:);

        %iterate on all configurations
        for jdx= 1:size(qs,1)
            % get the current configuation
            q=qs(jdx,:);
            % use it in the pe and assign it
            num_pe=double(vec_subs(pe,"all",q));
            j_points(:,jdx,idx)=num_pe;

        end

    end

    disp("\n")
    disp("Joint positoin estimated")
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


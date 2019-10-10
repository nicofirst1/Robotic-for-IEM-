clc;
clear;
% plot robot 
[robot,pArb]=initializer("ax18");

plot_f=1;
validate_f=1;

if plot_f
    
    q0=[0,0,0,0,0];
    qs=[0,pi/2,pi/2,0,0];
    

    figure(1);
    title("Robot in zero configuration");
    view(3);
    pArb.plot(q0,'scale',.5);

    figure(2);
    [robot,pArb2]=initializer("bob");

    title("Robot in default configuration");
    view(3);
    pArb2.plot(qs,'scale',.5);
end


if validate_f
    qs=length(pArb.a);
    qr=(pi/2).*rand(qs,1) ;
    
    disp("Random configuration q :")
    disp(qr')
  
    disp("########################")
    disp("Forward kinematic")
    disp("########################")
    
      
    pe_corke=pArb.fkine(qr).t;
    pe_mine=fk(qr);
    pe_mine=double(pe_mine);
    
    
    
    disp("Corke end effector position:")
    disp(pe_corke)
    
    disp("Ours end effector position:")
    disp(pe_mine)
    
    
    
    disp("########################")
    disp("Jacobian")
    disp("########################")
    
    j_corke=pArb.jacob0(qr);
    j_mine=jac_from_robot(robot);
    j_mine=vec_subs(j_mine,"all",qr);
    j_mine=double(j_mine);
    
    
    disp("Corke:")
    disp(j_corke)
    
    disp("Ours:")
    disp(j_mine)
    
    
    

end









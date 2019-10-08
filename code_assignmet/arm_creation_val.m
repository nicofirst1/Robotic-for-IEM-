
% plot robot 
[robot,pArb]=initializer();

plot_f=0;
validate_f=1;

if plot_f
    view(3);
    figure(1);
    pArb.plot([0,0,0,0,0],'scale',.5);

    view(3);
    figure(2);
    pArb.plot([0,-pi/2,-pi/2,0,0],'scale',.5);
end


if validate_f
    qs=length(pArb.a);
    qr=(pi/2).*rand(qs,1) ;
    
    disp("Random configuration q :")
    disp(qr')
  
    disp("########################")
    disp("End effector position")
    disp("########################")
    
      
    pe_corke=pArb.fkine(qr).t;
    pe_mine=pe_from_robot(robot);
    pe_mine=vec_subs(pe_mine,"all",qr);
    pe_mine=double(pe_mine);
    
    
    
    disp("Corke:")
    disp(pe_corke)
    
    disp("Ours:")
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










function [DH mdh] = dhp_params()

    l1=17;
    l2=17;
    l3=7;
    l4=4;
    l5=4;
    l6=9;

    mdh='modified';

    DH=[
    %THETA  D     A     ALPHA   SIGMA   OFFSET
       0    l1    0     0       0       0   ;
       0    0     0     pi/2    0       pi/2;
       0    0     l2    0       0       pi/2;
       0    l3+l5 l4    pi/2    0       pi/2;
       0    0     0     pi/2    0       0;
       pi/2 l6    0     -pi/2   0       0;

       ];

end

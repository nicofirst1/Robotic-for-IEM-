clear;
clc;

[robot,pArb]=initializer("ax18");

figure(1);
view(3);
pArb.teach();
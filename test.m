close all 
clc
clear all


%Intialize values
a = [1 2; 3 2];
b = [4 3; 0.2 3];
c = [1 1];
d = [0 0];


%Stability through poles
[num,denum] = ss2tf(a,b,c,d,1);
poles = roots(denum);
disp('Poles are')
poles

%Stability through eigen values
disp('Eigenvalues are: ');
eig(a)

%Stability through root locus
figure;
sys = tf(num,denum);
rlocus(sys)
title('Root locus of the system');

gaink = (0:1:10);
rlocus(sys,gaink)


%Stability through Step response
figure;
step(a,b,c,d);
title('Step Response of System');
ylabel('Amplitude in response to unit step');

%Check pre-requisites for controllability and observability

p = ctrb(a,b);
disp('Rank of Controller')
ctrb_rank = rank(p)

disp('Rank of Observer')
q = obsv(a,c);
obsv_rank = rank(q)

n = size(a,1)

%place observer values and controller values
desired.ob.egnvalues = [-10, -60, -90];
desired.ctrl.egnvlaues =[-2, -12, -18];

k = acker(a,b,desired.ctrl.egnvlaues);
l = acker(a',c',desired.ob.egnvalues)';




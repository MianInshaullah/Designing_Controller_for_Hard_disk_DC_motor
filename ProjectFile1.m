% A hard disk is a data storage device. It uses magnetic storage system
% along with electronic hardware to access the data. The electronic circuit
% consists of a dc motor. The dc motor has the following state space. 

%% first clear the command window and workspace
clear; 
clc; 
close all; 

%Declare the values for J, b, k, R, and L
j = 3.2; b = 3.5; k = 0.0274; r = 4; l = 2.75;

%% Create array A, B, C, and D 

mA = [0 1 0; 0 -b/j k/j; 0 k/l -r/l];
mB = [0;0; 1/l];
mC = [1 0 0];
mD = [0];


%% Checking stability of the system by finding pole of the Transfer Func. 
%Method: Convert the State-space model to a Transfer function and find if
%any values are positive. 

[num,denum] = ss2tf(mA,mB,mC,mD,1);
poles_of_tf = roots(denum);
disp('The Poles of the Transfer Function are the following '); 
poles_of_tf

if (any(poles_of_tf<0)~= 0) 
    disp ('Poles contain non-negative value(s); therefore, the system is unstable');
end



%% Checking stability of the system by finding eigenvalues of matrix A
%Method: Find the eigen value of matrix A and see if its negative or
%non-negative

eigen_values = eig(mA); 
disp('The eigenvalues of matrix A are');
eigen_values

if (any(eigen_values<0) ~= 0)
    disp ('Eigenvaluese contain non-negative value(s); therefore the system is unstable');
end


%% Checking the stability of the system using step response; 
%Method: Check if the step response of the system is unbounded

step(mA,mB,mC,mD)
title('Step response of the system')
ylabel('Amplitude in response to unit step');


%% Checking the stability of the system using root locus method
%Method: The root locus method finds the roots of the Characteristics
%equation. 

figure;
sys = tf(num(1,:),denum)
rlocus(sys)
title('Root locus of the system')


figure;
gaink = (0:1:10);
rlocus(sys,gaink)

%% Compute the controllability matrix for the system
%If the system is controllable, place the controller eigenvalues at
%(-14,-33,-33) and observer eigenvales at a location which is faster than
%the controller eigenvalues

ctrl = ctrb(mA, mB);
rank_ctrl = rank(ctrl);
disp('The rank of the controllability matrix is')
rank_ctrl  

obsrvr = obsv(mA, mC);
rank_obsrvr = rank(obsrvr);
disp('The rank of the observer matrix is')
rank_obsrvr

order = size(mA,1);
disp('The order of the system is')
order

if (rank_ctrl && rank_obsrvr == order)
    disp('The rank of system is equal to order of the system. The system is controllable and observable');
end

%% Designing a Observer based State feedback controller
%note matrix C is not an identity matrix; therefore, we need observer values

desired.ob.egnvalues = [-15,-34, -34];
mL = acker(mA',mC', desired.ob.egnvalues)';

desired_egnvalues = [-14, -33, -33];
mK = acker(mA, mB, desired_egnvalues);

%steady state errror after designing controller
ACL = mA - mB*mK;   %A-L*C gives you eig(ACL) the observer desired values
[nu,de] = ss2tf(ACL,mB,mC,mD);
g = tf(nu,de)    %stable transfer function 
eig(ACL)         %stable after designing controller,


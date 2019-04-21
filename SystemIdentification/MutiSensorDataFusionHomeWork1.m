%function [theta1,theta2,theta]=MutiSensorDataFusionHomeWork1(sigma)
%function [theta1,theta2,theta]=MutiSensorDataFusionHomeWork1(L)
%function [t1,t2]=MutiSensorDataFusionHomeWork1(L)
function [theta1,theta2,theta]=MutiSensorDataFusionHomeWork1()
%% this scrpt is to get the answer of those two questions.
%theta1: the result using LSbatchProcessing
%theta2: the result using LSRecurrence
%=============================warning======================================
% 1.You need have .m file as:inv_M_XuLie.m, leastSquare_batchProcessingSI.m
% and leastSquare_RecurrenceSI.m
% 2.......
%==========================================================================
%    author liangzid. Student ID: 20163933. Class:Automation 1609.
%                                                   time:2019.3.30
%==========================================================================
%% question 1
% generate a M Sequence
init_state=[1 0 1 0];
AXuLie=[0 0 1 1];
inv_M=zeros(1,2*(2^length(AXuLie)-1));% the P of the inv_M sequence is 30.
[inv_M,M]=inv_M_XuLie(AXuLie,init_state);

%display the result
M=M.*2-1;
inv_M=inv_M.*2-1;

%% question 2

%====================get Z(k)

% init
%default Z(0)=Z(1)=0;U(0)=U(1)=0;sigma=1;
theta_T=[1.532 0.75 0.43 0.35];
sigma=0.04;
miu=0; %the qiwang(junzhi) of white noise is zero.

%transfrom them to the matrix.
na=2;nb=2;% write na and nb depend on the difference eqution. 
L=50;
U=zeros(L,nb);
Z=zeros(L,na);

lengthNeedGenerate=L+na;
v_U=[];
for i=1:(floor(L/30)+1)
    v_U=[v_U,inv_M];
end

 

v_Z=zeros(1,length(v_U));
% get the output sequence v_Z
for i=3:length(v_U)
    h=[-1*v_Z(i-1),-1*v_Z(i-2),v_U(i-1),v_U(i-2)];
    white_noise=randn(1)*sigma + miu;
    v_Z(i)=h*theta_T'+white_noise;
    % donnot use noise
%     v_Z(i)=h*theta_T';

end
% give up the first two elements.
%v_U=v_U(3:end);
%v_Z=v_Z(3:end);

v_U=v_U(1:lengthNeedGenerate);  
v_Z=v_Z(1:lengthNeedGenerate);


% transfer the squence (from 0 to L+na) to (from 1-i to x-i)
bias=na-1+1;
% transfer the sequence to the Matrix.
for i=1:na
    
    % bias=1-na;
    Z(:,i)=v_Z((1-i+bias):(L-i+bias));
end
for j=1:nb
    U(:,j)=v_U((1-j+bias):(L-j+bias));%tong shang
end
%Z %display the result of the difference eqution.
%U
%========================Least Square Method=============================== 

%batchProcessing
ZL=v_Z((1+bias):1:(1+bias+L-1));

ZL=ZL';
%tic;
theta1=leastSquare_batchProcessingSI(U,Z,eye(L),ZL);%didn't use weight.
%t1=toc
%tic;
theta2=leastSquare_RecurrenceSI(Z,U,ones(1,L),ZL);  %didn't use weight.
%t2=toc

%% ========================================================================
% theta3=leastSquare_ForgetFactorSI(Z,U,0.8,0,ZL)   %this is the forget--
% % % --factor method.
% % % this line cannot be used!!   theta4=leastSquare_LimitedMemory(Z,U,3,ZL)
% theta5=leastSquare_ZheXi(Z,U,ones(1,L),0.99,ZL)
% theta6=leastSquare_PRechange(Z,U,[10,20,30],1000,10000,ZL)

theta=theta_T';     

end
function theta=leastSquare_PRechange(Z,U,Kseries,amin,amax,ZL)
%% Zhe Xi least square mathod in System Identification.
%===========================input parameters===============================
% Z: the output matrix. (L)(number of observation)*na(number of the sequence)
% U: the input  matrix. (L)(number of observation)*nb(number of  observation)
% Kseries:the time when P need to be Rechanged.
%==========================output parameter================================
%theta: least square solution of the xishu(coefficents) of the system.
%==========================other parameters================================
%epsilon_forInit: the little value for get theta's initValue.
%alpha: a big value for get matrix P.
%
%============================example=======================================
%See the m.file:MutiSensorDataFusionHomeWork1.m
%    author liangzid. Student ID: 20163933. Class:Automation 1609.
%                                                   time:2019.4.21 
%==========================================================================

[L1,nb]=size(U);
[L2,na]=size(Z);

if L1~=L2
    string='ERROR: the size of U Must be equal to Z.'
    
else
    %prepare for the algothrim
    L=L1; % number of the data observe
    % theta=zeros(na+nb,1);

    %epsilon_forEnd =1e-3;
    epsilon_forInit=1e-3;
    alpha=10000;
    % errorLoss=0.001;
    
    In=eye(na+nb);
    zl=ZL;
    
    % init the initial value
    theta=epsilon_forInit.*rand(na+nb,1);
    P=(alpha).*eye(na+nb);
    % h=zeros(na+nb,1);
    %h=[Z(:,1);U(:,1)];
    
    %Lambda=Lambda/length(Lambda);
    % recurrence
    for i=1:L
        
        pre_h=[-1.*Z(i,:),U(i,:)];
        h=pre_h';
        K=P*h/((h')*P*h+1);
        theta=theta+K*(zl(i)-h'*theta);
        if isHasAny(i,Kseries)
            P=(rand(1)*(amax-amin)+amin).*In;
        else
            P=(In-K*h')*P;
        end
    end
    
end

end
function answer=isHasAny(a,vector)
answer=0;
for i=1:length(vector)
    if vector(i)==a
        answer=1;
        break;
    end
end
end 
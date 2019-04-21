function theta=leastSquare_ForgetFactorSI(Z,U,miu,isChangedWithTime,ZL)
%% Forget factors least square mathod in System Identification.
%===========================input parameters===============================
% Z: the output matrix. (L)(number of observation)*na(number of the sequence)
% U: the input  matrix. (L)(number of observation)*nb(number of  observation)
% miu: the forget parameter whose value is between 0 and 1.
% isChangedWithTime: a bool number whose value is 0 or 1.if it is 1, is mea
%   -ns that the miu is the changed value which is the (tu zu he) of miu0 and 1. 
%==========================output parameter================================
%theta: least square solution of the xishu(coefficents) of the system.

%============================example=======================================
%See the m.file:MutiSensorDataFusionHomeWork1.m
%    author liangzid. Student ID: 20163933. Class:Automation 1609.
%                                                   time:2019.3.30 
%==========================================================================

[L1,nb]=size(U);
[L2,na]=size(Z);

if L1~=L2
    string='ERROR: the size of U Must be equal to Z.'
    
else
    %prepare for the algothrim
    L=L1; % number of the data observe
    
    if miu>=0 && miu <=1
        epsilon_forInit=1e-3;
        alpha=10000;
        miu_k0=0.95;
        miu0=0.99;
        In=eye(na+nb);

        % init the initial value
        theta=epsilon_forInit.*rand(na+nb,1);
        P=(alpha).*eye(na+nb);
        
        % h=zeros(na+nb,1);
        %h=[Z(:,1);U(:,1)];

        %Lambda=Lambda/length(Lambda);
        % recurrence
        
        for i=1:L
            
            if isChangedWithTime
                miu_k0=miu0*miu_k0+(1-miu0);
                miu=miu_k0;
            end
     
            pre_h=[-1.*Z(i,:),U(i,:)];
            h=pre_h';
            %K=P*h/((h')*P*h+(1/Lambda(i)));
            K=P*h/((h')*P*h+(1));
            theta=theta+K*(ZL(i)-h'*theta);
            P=(1/miu)*(In-K*h')*P;
        end
    else
        a1='Error:miu must be in 0~1.'
        Lambda=ones(1,L);
        theta=leastSquare_RecurrenceSI(Z,U,Lambda,ZL);
    end
    
    end
end
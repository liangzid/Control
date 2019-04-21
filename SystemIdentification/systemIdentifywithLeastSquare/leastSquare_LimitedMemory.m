function theta=leastSquare_LimitedMemory(Z,U,lenNeedLimit,ZL)
%% Limited Memory least square mathod in System Identification.
%===========================input parameters===============================
% Z: the output matrix. (L)(number of observation)*na(number of the sequence)
% U: the input  matrix. (L)(number of observation)*nb(number of  observation)
% lenNeedLimit: the max length of the information.
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

    %epsilon_forEnd =1e-3;
    epsilon_forInit=1e-3;
    alpha=10000;
    
    In=eye(na+nb);
    
    % init the initial value
    theta=epsilon_forInit.*rand(na+nb,1);
    P=(alpha).*eye(na+nb);
    
    % recurrence
    for i=1:(L-lenNeedLimit)
        pre_h=[-1.*Z(i,:),U(i,:)];
        h=pre_h';
        
        % add new data
        K=P*h/((h')*P*h+(1));
        theta=theta+K*(ZL(i+lenNeedLimit)-h'*theta);
        P=(In-K*h')*P;
        
        %get out the old data
        K=P*h/(1-h'*P*h);
        P=(In+K*h')*P;
        theta=theta-K*(ZL(i)-h'*theta);
      
    end
    
end


end
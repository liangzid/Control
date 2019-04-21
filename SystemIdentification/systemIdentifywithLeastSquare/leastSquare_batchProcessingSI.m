function theta=leastSquare_batchProcessingSI(U,Z,Lambda,ZL)
%% batch processing least square mathod in System Identification.
%===========================input parameters===============================
% Z: the output matrix. (L)(number of observation)*na(number of the sequence)
% U: the input  matrix. (L)(number of observation)*nb(number of  observation)
% Lambda: the weighted matrix of the different time(sequences).
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
    string='ERROR: the number of U Must be equal to Z.'
    
else
    if ~isdiag(Lambda)
        string='error: the weighted matirx must be diag.'
    else
        L=L1; % number of the data observe
        %theta=zeros(na+nb,1);
        H_L=zeros(L,na+nb);
        % ZL=ZZ(2:end,1);
        H_L(:,1:na)           =-1.*Z;
        H_L(:,(na+1):(na+nb)) =U;
        
        theta=(H_L'*H_L)\H_L'*ZL;
        %theta=inv(H_L'*Lambda*H_L)*H_L'*Lambda*ZL;
    end
end
end

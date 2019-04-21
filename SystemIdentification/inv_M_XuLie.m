 function [Inv_M_Xulie,M_Xulie]=inv_M_XuLie(AXuLie,InitValue)
%% Inv_M_XuLie is a function to generate inverse-M sequence.
%========================the input parameters============================== 
%1)AXuLie: a row vector consists of 1 or 0,where the last number of it must
%be 1; 
%2)InitValue: a row vector represents the initial number of the register.
%========================the output parameter==============================
%Inv_M_Xulie:the inverse M Xulie
%===============================example====================================
% Inv_M_Xulie=inv_M_XuLie([0 0 1 1],[1 0 1 0]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% the size of these two vector must be equal.
if ~(size(AXuLie)==size(InitValue))
    string='EORROR:the size of AXuLie must be equal to InitValue.'
    Inv_M_Xulie=NaN;
else
    if AXuLie(end)~=1
        string='error:the last element of Vector AXuLie must be 1.'
        Inv_M_Xulie=NaN;
    else
        len=length(AXuLie);
        M=zeros(1,2^len-1);
        inv_M=zeros(1,2*length(M));
        for i=1:len
            M(i)=InitValue(len-i+1);
        end
        for i=(len+1):length(M)
            M(i)=0;
            for j=1:len
                M(i)=mod_2_add(M(i),M(i-len+j-1)*AXuLie(len-(j-1))); % core steps.
            end
        end
        
        % shi zhong fang bo xin hao
        % FangBo1=[0,1];FangBo2=[1,0];
        using_FangBo=zeros(size(inv_M));
        for i=1:2:length(inv_M)
            using_FangBo(i)=1;
        end
        inv_M=[M,M];
        for i=1:length(inv_M)
            inv_M(i)=mod_2_add(inv_M(i),using_FangBo(i));
        end
        Inv_M_Xulie=inv_M;
        M_Xulie=M;
    end
end
 end

% mo 2 he function
function result= mod_2_add(a,b)
        result=mod(a+b,2);
    end

function qiankuiqiankui(setpoint,Ty_ofWm,Tx_ofWm,KofWm,timeDelayofWm,kp,ki,TofWo,KofWo,timeDelayofWo,interfere,
                        TofWf,KofWf,timeDelayofWf,runningTime,T)
{
    var timee=runningTime/T;

    var setValue=new initArray(timee,setpoint);
    var outputofWm=new initArray(timee,0);
    var ouputofControlPI=new initArray(timee,0);
    var inputofWo=new initArray(timee,0);
    var outputofWo=new initArray(timee,0);
    var outputofWf=new initArray(timee,0);
    var mainoutput=new initArray(timee,0);

    var error=0;

    for(var i=0;i<timee;i++)
    {
        //PID控制器输出
        if(i==0)
        {
            ouputofControlPI[i]=0;
        }
        else
        {
            ouputofControlPI[i]=kp*(setValue[i-1]-mainoutput[i-1])+ki*error;
        }

        //前馈控制部分
        if(i==0)
        {
            outputofWm[i]==0;
        }
        else if((i-timeDelayofWm/T)<0)
        {
            outputofWm[i]=0;
        }
        else if(i-timeDelayofWm/T==0)
        {
            outputofWm[i]=-1*KofWm*(1/(1+2*Tx_ofWm/T))*((1+2*Ty_ofWm/T)*interfere[i-timeDelayofWm/T]);

        }
        else if(i-timeDelayofWm/T-1>=0)
        {
            outputofWm[i]=-1*KofWm*(1/(1+2*Tx_ofWm/T))*((1+2*Ty_ofWm/T)*interfere[i-timeDelayofWm/T]
                +(1-2*Ty_ofWm/T)*interfere[i-1-timeDelayofWm/T]
                -(1-2*Tx_ofWm/T)*outputofWm[i-1-timeDelayofWm/T]);
        }

        inputofWo[i]=ouputofControlPI[i]+outputofWm[i];

        //计算被控对象输出
        if(i==0)
        {
            outputofWo[i]=0;
        }
        else if((i-timeDelayofWo/T)<0)
        {
            outputofWo[i]=0;
        }
        else if((i-timeDelayofWo/T)==0)
        {
            outputofWo[i]=(1+2*TofWo/T)*(KofWo*(inputofWo[i-timeDelayofWo/T]));//此处存疑
        }
        else
        {
            outputofWo[i]=1/(1+2*TofWo/T)*(KofWo*(inputofWo[i-timeDelayofWo/T]+inputofWo[i-1-timeDelayofWo/T])
                -(1-2*TofWo/T)*mainoutput[i-1-timeDelayofWo/T]);
        }

        //计算干扰项输入
        if(i==0)
        {
            outputofWf[i]=0;
        }
        else if((i-timeDelayofWf/T)<0)
        {
            outputofWf[i]=0;
        }
        else if((i-timeDelayofWf/T)==0)
        {
            outputofWf[i]=(1+2*TofWf/T)*(KofWf*(interfere[i-timeDelayofWf/T]));//此处存疑
        }
        else
        {
            outputofWf[i]=1/(1+2*TofWf/T)*(KofWf*(interfere[i-timeDelayofWf/T]+interfere[i-1-timeDelayofWf/T])
                -(1-2*TofWf/T)*outputofWf[i-1-timeDelayofWf/T]);
        }

        mainoutput[i]=outputofWo[i]+outputofWf[i];

        error+=setValue[i]-mainoutput[i];

    }

    return mainoutput;

}

/*对数组进行初始化,返回该数组的函数*/
function initArray(lens,value)
{
    array=new Array(lens);
    for (i=0;i<lens;i++)
    {
        array[i]=value;
    }
    return array;
}


var setpoint=1;
var kp=1.9956;
var ki=0.0195963675100431;

var TofWo=100;
var KofWo=1;
var timeDelayofWo=0;

var TofWf=10;
var KofWf=1;
var timeDelayofWf=0;

var TyofWm=100;
var TxofWm=10;
var KofWm=1;
var timeDelayofWm=0;

var runningTime=500;
var T=0.01;

var interfere=new initArray(runningTime/T,1);

mainoutput=qiankuiqiankui(setpoint,TyofWm,TxofWm,KofWm,timeDelayofWm,kp,ki,TofWo,KofWo,timeDelayofWo,interfere,
    TofWf,KofWf,timeDelayofWf,runningTime,T);

console.log(mainoutput);

var fs = require('fs');
let str=JSON.stringify(mainoutput,"","\t");
fs.writeFile('前馈反馈.json',str);


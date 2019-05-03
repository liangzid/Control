function qiankui_FanKui(setpoint,kp,ki,TofWo,KofWo,timeDelayofWo,interfere,TofWf,KofWf,timeDelayofWf,runningTime) {

    var T=0.01;

    var setValue=new initArray(runningTime/T,setpoint);
    var mainoutput=new initArray(setValue.length,0);
    var outputofPI=new initArray(setValue.length,0);
    var outputofWf=new initArray(setValue.length,0);
    var inputofWo=new initArray(setValue.length,0);

    var error=0;

    for(var i=0;i<runningTime/T;i++)
    {
        //计算控制器输出
        if(i==0)
        {
            outputofPI[i]=0;
        }
        else
        {
            outputofPI[i]=kp*(setValue[i-1]-mainoutput[i-1])+ki*error;
        }
        console.log("outputofPI : "+outputofPI[i]);

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

        inputofWo[i]=outputofPI[i]+outputofWf[i];
        //计算被控对象输出
        if(i==0)
        {
            mainoutput[i]=0;
        }
        else if((i-timeDelayofWo/T)<0)
        {
            mainoutput[i]=0;
        }
        else if((i-timeDelayofWo/T)==0)
        {
            mainoutput[i]=(1+2*TofWo/T)*(KofWo*(inputofWo[i-timeDelayofWo/T]));//此处存疑
        }
        else
        {
            mainoutput[i]=1/(1+2*TofWo/T)*(KofWo*(inputofWo[i-timeDelayofWo/T]+inputofWo[i-1-timeDelayofWo/T])
                -(1-2*TofWo/T)*mainoutput[i-1-timeDelayofWo/T]);
        }

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
var runningTime=500;
var T=0.01;

var interfere=new initArray(runningTime/T,1);
var mainoutput=qiankui_FanKui(setpoint,kp,ki,TofWo,KofWo,timeDelayofWo,interfere,TofWf,KofWf,timeDelayofWf,runningTime);

console.log(mainoutput);

var fs = require('fs');
let str=JSON.stringify(mainoutput,"","\t");
fs.writeFile('反馈.json',str);
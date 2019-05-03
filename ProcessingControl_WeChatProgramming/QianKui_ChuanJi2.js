function qiankui_cchuanji(setpoint,TyofWm,TxofWm,KofWm,timeDelayofWm,kp1,ki1,kp2,ki2,TofWo1,KofWo1,timeDelayofWo1,
                          TofWf1,KofWf1,timeDelayofWf1,interfere1,TofWo2,KofWo2,timeDelayofWo2,TofWf2,KofWf2,timeDelayofWf2,
                          interfere2,runningtime,T)
{
    var timee=runningtime/T;

    var setValue=new initArray(timee,setpoint);
    var outputofControl1=new initArray(timee,0);
    var inputofControl2=new initArray(timee,0);
    var outputofWm=new initArray(timee,0);
    var outputofControl2=new initArray(timee,0);

    var outputofWo1=new initArray(timee,0);
    var outputofWf1=new initArray(timee,0);

    var suboutput=new initArray(timee,0);

    var outputofWo2=new initArray(timee,0);
    var outputofWf2=new initArray(timee,0);

    var mainoutput=new initArray(timee,0);

    var error1=0;
    var error2=0;

    for (var i=0;i<timee;i++)
    {
        //PID控制器1输出
        if(i==0)
        {
            outputofControl1[i]=0;
        }
        else
        {
            outputofControl1[i]=kp1*(setValue[i-1]-mainoutput[i-1])+ki1*error1;
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
            outputofWm[i]=-1*KofWm*(1/(1+2*Tx_ofWm/T))*((1+2*Ty_ofWm/T)*interfere2[i-timeDelayofWm/T]);

        }
        else if(i-timeDelayofWm/T-1>=0)
        {
            outputofWm[i]=-1*KofWm*(1/(1+2*TxofWm/T))*((1+2*TyofWm/T)*interfere2[i-timeDelayofWm/T]
                +(1-2*TyofWm/T)*interfere2[i-1-timeDelayofWm/T]
                -(1-2*TxofWm/T)*outputofWm[i-1-timeDelayofWm/T]);
        }

        inputofControl2[i]=outputofControl1[i]+outputofWm[i];

        //PID控制器2输出
        if(i==0)
        {
            outputofControl2[i]=0;
        }
        else
        {
            outputofControl2[i]=kp2*(inputofControl2[i-1]-suboutput[i-1])+ki2*error2;
        }

        //计算被控对象1输出
        if(i==0)
        {
            outputofWo1[i]=0;
        }
        else if((i-timeDelayofWo1/T)<0)
        {
            outputofWo1[i]=0;
        }
        else if((i-timeDelayofWo1/T)==0)
        {
            outputofWo1[i]=(1+2*TofWo1/T)*(KofWo1*(outputofControl2[i-timeDelayofWo1/T]));//此处存疑
        }
        else
        {
            outputofWo1[i]=1/(1+2*TofWo1/T)*(KofWo1*(outputofControl2[i-timeDelayofWo1/T]+outputofControl2[i-1-timeDelayofWo1/T])
                -(1-2*TofWo1/T)*outputofWo1[i-1-timeDelayofWo1/T]);
        }

        //计算干扰1
        if(i==0)
        {
            outputofWf1[i]=0;
        }
        else if((i-timeDelayofWf1/T)<0)
        {
            outputofWf1[i]=0;
        }
        else if((i-timeDelayofWf1/T)==0)
        {
            outputofWf1[i]=(1+2*TofWf1/T)*(KofWf1*(interfere1[i-timeDelayofWf1/T]));//此处存疑
        }
        else
        {
            outputofWf1[i]=1/(1+2*TofWf1/T)*(KofWf1*(interfere1[i-timeDelayofWf1/T]+interfere1[i-1-timeDelayofWf1/T])
                -(1-2*TofWf1/T)*outputofWf1[i-1-timeDelayofWf1/T]);
        }

        suboutput[i]=outputofWo1[i]+outputofWf1[i];

        //被控对象2输出计算
        if(i==0)
        {
            outputofWo2[i]=0;
        }
        else if((i-timeDelayofWo2/T)<0)
        {
            outputofWo2[i]=0;
        }
        else if((i-timeDelayofWo2/T)==0)
        {
            outputofWo2[i]=(1+2*TofWo2/T)*(KofWo2*(suboutput[i-timeDelayofWo2/T]));//此处存疑
        }
        else
        {
            outputofWo2[i]=1/(1+2*TofWo2/T)*(KofWo2*(suboutput[i-timeDelayofWo2/T]+suboutput[i-1-timeDelayofWo2/T])
                -(1-2*TofWo2/T)*outputofWo2[i-1-timeDelayofWo2/T]);
        }

        //计算干扰2
        if(i==0)
        {
            outputofWf2[i]=0;
        }
        else if((i-timeDelayofWf2/T)<0)
        {
            outputofWf2[i]=0;
        }
        else if((i-timeDelayofWf2/T)==0)
        {
            outputofWf2[i]=(1+2*TofWf2/T)*(KofWf2*(interfere2[i-timeDelayofWf2/T]));//此处存疑
        }
        else
        {
            outputofWf2[i]=1/(1+2*TofWf2/T)*(KofWf2*(interfere2[i-timeDelayofWf2/T]+interfere2[i-1-timeDelayofWf2/T])
                -(1-2*TofWf2/T)*outputofWf2[i-1-timeDelayofWf2/T]);
        }

        mainoutput[i]=outputofWo2[i]+outputofWf2[i];

        error1+=setValue[i]-mainoutput[i];
        error2+=inputofControl2[i]-suboutput[i];
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

//类似于Python中的range方法
function generate_RangeSeries(maxnumber,lengthOfArray)
{
    var a=new Array(lengthOfArray);
    var per=maxnumber/lengthOfArray;
    for(var i=0;i<lengthOfArray;i++)
    {
        a[i]=i*per;
    }
    return a;
}

function generate2dimensionArray(array1,array2)
{
    var a=new Array(array1.length);
    for(var i=0;i<a.length;i++)
    {
        a[i]=new Array(2);
        a[i][0]=array1[i];
        a[i][1]=array2[i];
    }
    return a;
}



var setpoint=1;

var kp1=2.16987137619709;
var ki1=0.0155852616583419;

var kp2=0.368740274216487;
var ki2=0;

var TofWf1=15;
var KofWf1=1;
var timeDelayofWf1=0;

var TofWf2=10;
var KofWf2=1;
var timeDelayofWf2=30;

var TofWo1=30;
var KofWo1=1;
var timeDelayofWo1=0;

var TofWo2=100;
var KofWo2=1;
var timeDelayofWo2=30;

var TyofWm=0;
var TxofWm=0;
var KofWm=3090/0.37/8.8;
var timeDelayofWm=0;

var runningtime=500;
var T=0.01;

var interfere1=initArray(runningtime/T,1);
var interfere2=initArray(runningtime/T,1);

mainoutput=qiankui_cchuanji(setpoint,TyofWm,TxofWm,KofWm,timeDelayofWm,kp1,ki1,kp2,ki2,TofWo1,KofWo1,timeDelayofWo1,TofWf1,
    KofWf1,timeDelayofWf1,interfere1,TofWo2,KofWo2,timeDelayofWo2,TofWf2,KofWf2,timeDelayofWf2,interfere2,runningtime,T);

console.log(mainoutput);

var fs = require('fs');
let str=JSON.stringify(mainoutput,"","\t");
fs.writeFile('data_ChuanJi2.json',str);
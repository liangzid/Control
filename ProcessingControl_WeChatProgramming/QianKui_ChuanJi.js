/**********************************************************************************
 * 前馈串级控制数据输出
 * 功能:模拟控制系统运行,获得输出信号和控制信号等关键信号. 
 * 输入参数:
 * 1)setpoint:给定值
 * 2)Kp1,Ki1:主控制器的PI参数;
 * 2.5)YOfWQianKui,XOfWQianKui:前馈环节的分子分母,数据均为一维数组,且是降阶排列;
 * 3)Kp2,Ki2:副回路的PI参数;
 * 4)YOfWo1,XOfWo1:副回路被控对象的分子,分母.数据类型均为一维数组,且是降阶排列;
 * 4.5)timeDelayOfWo1:副回路如果存在时滞环节的话,时滞数值;
 * 5)interfere1:副回路的施加扰动;
 * 5.5)YOfWf1,XOfWf1:扰动信号的分子分母,数据类型均为一维数组,且是降阶排序;
 * 6)interfere2:加在主被控对象上的扰动;
 * 6.5)YOfWf2,XOfWf2:第二个扰动信号作用时的分子分母,数据类型均为一维数组,且是降阶排序;
 * 7)YOfWo2,XOfWo2:主回路被控对象的分子,分母.数据类型均为一维数组,且是降阶排列;
 * 7.5)timeDelayOfWo2:主回路如果存在时滞环节的话,时滞数值;
 * 8)主回路的施加扰动;
 * 9)是否使用模型建立不准确,如果为真,表示模型建立不准确,这样被控对象的参数可能会发生一些变化;
 * 10)runningTime:数据采集的总时间长度
 * 输出参数:
 * 目前的输出参数计划设计为一个object对象,包含一下属性:
 * 1)被控量向量;
 * 2)主控制器控制量向量;
 * 3)副控制器控制量;
 * 4)副回路输出;
 * 包含一下方法:
 * 1)画图?
 * 
 * liangzi,2019,4,29
 ***********************************************************************************/ 
 function qianKui_ChuanJi(setpoint,Kp1,Ki1,TYOfWQianKui,TXOfWQianKui,KOfQianKui,timeDelayOfQianKui,
    Kp2,Ki2,TYOfWo1,KOfWo1,timeDelayOfWo1,interfere1,TYOfWf1,KOfWf1,timeDelayOfWf1,TYOfWo2,KOfWo2,
    timeDelayOfWo2,interfere2,TYOfWf2,KOfWf2,timeDelayOfWf2,runningTime,isUseModelingInaccurate)
{
    //超参设置
    var T=0.01;//采样时间,这个就默认吧.
    
    //输入输出与状态变量初始化
    var setValue  =initArray(runningTime/T,setpoint);//用给定值来初始化输入
    //setValue=new Float64Array(runningTime/T);
    var OutputOfControlPI1=new Float64Array(setValue.length);
    var OutputOfControlPI2=new Float64Array(setValue.length);
    var OutputOfWo1=new Float64Array(setValue.length);
    var OutputOfWo2=new Float64Array(setValue.length);
    var OutputOfWf1=new Float64Array(setValue.length);
    var OutputOfWf2=new Float64Array(setValue.length);
    //mainOutput=new Float64Array(setValue.length);
    //subOutput =new Float64Array(mainOutput.length);
    var QianKuiForinterfere1=new Float64Array(setValue.length);


    var mainOutput=initArray(setValue.length,0.00);
    var subOutput =initArray(setValue.length,0.00);

    //初始化控制器中的两个积分项的偏差值
    var error1=0;
    var error2=0;

    //模拟整个回路的状态
    for(var i=0;i<setValue.length;i++)
    {
        //主回路控制器输出
        if(i-1<0)
        {
            OutputOfControlPI1[i]=0;
        }
        else
        {
            //console.log(mainOutput[i-1]);
            OutputOfControlPI1[i]=Kp1*(setValue[i-1]-mainOutput[i-1])+Ki1*error1;
        }

        //输出控制信号一直以来的变化;

        //console.log('输出信号PI1为:'+OutputOfControlPI1[i]);

        //计算前馈补偿项
        /*********************************************************************
         * ===========这一部分是完全准确时的补偿,如果需要自定义时可以使用============ * 
        if(i-1<0)//将零时刻之前的项取为0
        {
            QianKuiForinterfere1[i]=-KOfWf1/KOfWo1*(1+2*TYOfWf1[0]/T)*
            ((1+2*TYOfWo1[0]/T)*interfere1[i]);
        }
        else
        {
            QianKuiForinterfere1[i]=-KOfWf1/KOfWo1*(1+2*TYOfWf1[0]/T)*
            ((1+2*TYOfWo1[0]/T)*interfere1[i]+(1-2*TYOfWo1/T)*interfere1[i-1]
            -(1-2*TYOfWf1/T)*QianKuiForinterfere1[i-1]);
        }
        *******************************************************************/
        //计算前馈补偿
        var taoQianKui=~~(timeDelayOfQianKui/T);
        console.log("taoqiankui:"+taoQianKui);
        if(i-1-taoQianKui<0)
        {
            if(i==taoQianKui)
            {
                QianKuiForinterfere1[i]=-KOfQianKui/(1+2*TXOfWQianKui/T)*
                ((1+2*TYOfWQianKui/T)*interfere1[i-taoQianKui]);
            }
            else
            {
                QianKuiForinterfere1[i]=0;
            }
        }
        else if(i-1-taoQianKui>=0)
        {
            console.log(-(1-2*TXOfWQianKui/T)*QianKuiForinterfere1[i-1-taoQianKui]
                );

            QianKuiForinterfere1[i]=(-KOfQianKui/(1+(2*TXOfWQianKui/T)))*
            ((1+2*TYOfWQianKui/T)*interfere1[i-taoQianKui]
            +(1-2*TYOfWQianKui/T)*interfere1[i-1-taoQianKui]
            -(1-2*TXOfWQianKui/T)*QianKuiForinterfere1[i-1-taoQianKui]);
        }

        //console.log("----------:"+KOfQianKui+TXOfWQianKui+TYOfWQianKui);
        console.log("interfere1:"+interfere1[i]);
        console.log("qiankuiforinterfere1:"+QianKuiForinterfere1[i]);

        //副回路控制器输出
        if(i-1<0)
        {
            OutputOfControlPI2[i]=0;
        }
        else
        {
            OutputOfControlPI2[i]=Kp2*(OutputOfControlPI1[i-1]+QianKuiForinterfere1[i-1]-subOutput[i-1])
            +Ki2*error2;
        }
        console.log("=====outputofcontrolpi1:"+OutputOfControlPI1[i]);





        //计算副回路输出值(分为两部分:控制器作用于被控对象结果+干扰项)
        var taowo1=~~(timeDelayOfWo1/T);
        if(i<1+taowo1)
        {
            if(i>=taowo1)
            {
                OutputOfWo1[i]=1/(1+2*TYOfWo1/T)*(KOfWo1*(OutputOfControlPI2[i-taowo1]));
            }
            else
            {
                OutputOfWo1[i]=0;
            }
            
        }
        else
        {
            OutputOfWo1[i]=1/(1+2*TYOfWo1/T)*(KOfWo1*(OutputOfControlPI2[i-taowo1]+OutputOfControlPI2[i-1-taowo1])
            -(1-2*TYOfWo1/T)*OutputOfWo1[i-1-taowo1]);
            
        }
        console.log("outputofcontrolpi2:"+OutputOfControlPI2[i]);
        //console.log("outputofwo1:"+OutputOfWo1);



        var taowf1=~~(timeDelayOfWf1/T);
        if(i<1+taowf1)
        {
            if(i>=taowf1)
            {
                OutputOfWf1[i]=1/(1+2*TYOfWf1/T)*(KOfWf1*(interfere1[i-taowf1]));
            }
            else
            {
                OutputOfWf1[i]=0;
            }
        }
        else
        {
            OutputOfWf1[i]=1/(1+2*TYOfWf1/T)*(KOfWf1*(interfere1[i-taowf1]+interfere1[i-1-taowf1])
            -(1-2*TYOfWf1/T)*OutputOfWf1[i-1-taowf1]);
        }

        console.log("outputofwo1:"+OutputOfWo1[i]);
        console.log("outputofwf1:"+OutputOfWf1[i]);


        subOutput[i]=OutputOfWo1[i]+OutputOfWf1[i];

        console.log("suboutput:"+subOutput[i]);
        //计算主回路的输出值
        var taowo2=~~(timeDelayOfWo2/T);
        console.log("taowo2:"+taowo2);
        if(i<1+taowo2)
        {
            if(i>=taowo2)
            {
                OutputOfWo2[i]=1/(1+2*TYOfWo2/T)*(KOfWo2*(subOutput[i-taowo2]));
            }
            else
            {
                OutputOfWo2[i]=0;
            }
            
        }
        else
        {
            OutputOfWo2[i]=1/(1+2*TYOfWo2/T)*(KOfWo2*(subOutput[i-taowo2]+subOutput[i-1-taowo2])
            -(1-2*TYOfWo2/T)*OutputOfWo2[i-1-taowo2]);
            
        }

        var taowf2=~~(timeDelayOfWf2/T);
        //console.log("taowf2:"+taowf2);
        if(i<1+taowf2)
        {
            if(i>=taowf2)
            {
                //console.log("interfere2:"+interfere2[i])
                OutputOfWf2[i]=1/(1+2*TYOfWf2/T)*(KOfWf2*(interfere2[i-taowf2]));
            }
            else
            {
                //console.log("hahahahahahhahahahaaaaaaaaaaaaaaaaaa");
                OutputOfWf2[i]=0;
            }
        }
        else
        {


            OutputOfWf2[i]=1/(1+2*TYOfWf2/T)*(KOfWf2*(interfere2[i]+interfere2[i-1-taowf2])
            -(1-2*TYOfWf2/T)*OutputOfWf2[i-1-taowf2]);
        }

        console.log('Outputofwo2:'+OutputOfWo2[i]);
        //console.log('outputofwf2:'+OutputOfWf2[i]);
        mainOutput[i]=OutputOfWo2[i]+OutputOfWf2[i];
        console.log("mainoutput:"+mainOutput[i]);
//-------------------------------------------------------------------------------------------------        
        /* 没有考虑时滞,暂时废弃
        if(i-1<0)
        {
            OutputOfWo2[i]=1/(1+2*TYOfWo2/T)*(KOfWo2*(subOutput[i]));
            outputofwf2[i]=1/(1+2*TYOfWf2/T)*(KOfWf2*(interfere2[i]));
            mainOutput[i]=OutputOfWo2[i]+outputofwf2[i];
        }
        else
        {
            OutputOfWo2[i]=1/(1+2*TYOfWo2/T)*(KOfWo2*(subOutput[i]+subOutput[i-1])
            -(1-2*TYOfWo2/T)*OutputOfWo2[i-1]);
            outputofwf2[i]=1/(1+2*TYOfWf2/T)*(KOfWf2*(interfere2[i]+interfere2[i-1])
            -(1-2*TYOfWf2/T)*outputofwf2[i-1]);
            mainOutput[i]=OutputOfWo2[i]+outputofwf2[i];
        }
        */
//-----------------------------------------------------------------------------------------------------
        //改变两个积分项的差值,为下一次PI控制器计算做准备.
        error1+=setValue[i]-mainOutput[i];
        error2+=OutputOfControlPI1[i]+OutputOfWf1[i]-subOutput[i];
    }

    //将数据进行输出
    liangzi=new DataFormer(generate_RangeSeries(runningTime,T),mainOutput,subOutput,
    OutputOfControlPI1,OutputOfControlPI2,OutputOfWf1,OutputOfWf2);
    
    return liangzi;
    //return OutputOfControlPI1;
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
    a=new Array;
    per=maxnumber/lengthOfArray;
    for(i=0;i<lengthOfArray;i++)
    {
        a.push(i*per);
    }
    return a;
}

//输出格式确立
function DataFormer(timeSeries,mainOutput,subOutput,OutputOfControlPI1,OutputOfControlPI2,
    OutputOfWf1,OutputOfWf2)
{
    this.timeSeries=timeSeries;

    this.mainOutput=mainOutput;
    this.subOutput=subOutput;
    
    this.OutputOfControlPI1=OutputOfControlPI1;
    this.OutputOfControlPI2=OutputOfControlPI2;
    
    this.OutputOfWf1=OutputOfWf1;
    this.OutputOfWf2=OutputOfWf2;
};

//主函数部分
var setpoint=1.0;
var Kp1=2.17;
var Ki1=0.015585;
var TYOfWQianKui=0;
var TXOfWQianKui=0;
var KOfQianKui=5;
var timeDelayOfQianKui=0;
var Kp2=0.369;
var Ki2=0;
var TYOfWo1=30;
var KOfWo1=1;
var timeDelayOfWo1=0;

var TYOfWf1=0;
var KOfWf1=0;
var timeDelayOfWf1=0;

var TYOfWo2=100;
var KOfWo2=1;
var timeDelayOfWo2=0;

var TYOfWf2=0;
var KOfWf2=0;
var timeDelayOfWf2=0;

var runningTime=5;
var isUseModelingInaccurate=0;
var T=0.01

var interfere1=initArray(runningTime/T,1);
var interfere2=initArray(interfere1.length,1);
//console.log(interfere1[0]);

var isUseModelingInaccurat=0;
liangzi=qianKui_ChuanJi(setpoint,Kp1,Ki1,TYOfWQianKui,TXOfWQianKui,KOfQianKui,timeDelayOfQianKui,
    Kp2,Ki2,TYOfWo1,KOfWo1,timeDelayOfWo1,interfere1,TYOfWf1,KOfWf1,timeDelayOfWf1,TYOfWo2,KOfWo2,
    timeDelayOfWo2,interfere2,TYOfWf2,KOfWf2,timeDelayOfWf2,runningTime,isUseModelingInaccurat);

//liangzi2="...";

//console.log(liangzi.mainOutput)

var fs = require('fs');
let str=JSON.stringify(liangzi,"","\t");
fs.writeFile('data.json',str);
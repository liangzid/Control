function qianKui_ChuanJi(setpoint,Kp,Ki,TYOfWQianKui,TXOfWQianKui,KOfQianKui,timeDelayOfQianKui,
    TOfWo,KOfWo,timeDelayOfWo,interfere,TOfWf,KOfWf,timeDelayOfWf,runningTime,isUseModelingInaccurate)
{
    //超参设置
    var T=0.01;//采样时间,这个就默认吧.
    
    //输入输出与状态变量初始化
    setValue=new Float64Array(runningTime/T);
    OutputOfControlPI=new Float64Array(setValue.length);
    OutputOfWo=new Float64Array(setValue.length);
    OutputOfWf=new Float64Array(setValue.length);
    mainOutput=new Float64Array(setValue.length);
    QianKuiForinterfere=new Float64Array(setValue.length);

    setValue  =initArray(setValue,setpoint);//用给定值来初始化输入
    mainOutput=initArray(mainOutput,0.00);
    subOutput =initArray(subOutput,0.00);

    //初始化控制器中的两个积分项的偏差值
    var error=0;

    //模拟整个回路的状态
    for(i=0;i<setValue.length;i++)
    {
        //主回路控制器输出
        if(i-1<0)
        {
            OutputOfControlPI[i]=0;
        }
        else
        {
            OutputOfControlPI[i]=Kp*(setValue[i-1]-mainOutput[i-1])+Ki*error;
        }

        //计算前馈补偿
        taoQianKui=floor(timeDelayOfQianKui/T);
        if(i-1-taoQianKui<0)
        {
            if(i==taoQianKui)
            {
                QianKuiForinterfere[i]=-KOfQianKui/(1+2*TXOfWQianKui[0]/T)*
                ((1+2*TYOfWQianKui[0]/T)*interfere[i-taoQianKui]);
            }
            else
            {
                QianKuiForinterfere[i]=0;
            }
        }
        else if(i-1-taoQianKui>=0)
        {
            QianKuiForinterfere[i]=-KOfQianKui/(1+2*TXOfWQianKui[0]/T)*
            ((1+2*TYOfWQianKui[0]/T)*interfere[i-taoQianKui]
            +(1-2*TYOfWQianKui/T)*interfere[i-1-taoQianKui]
            -(1-2*TXOfWQianKui/T)*QianKuiForinterfere[i-1-taoQianKui]);
        }

        //计算输出值
        taowo=timeDelayOfWo/T;
        if(i<1+taowo)
        {
            if(i>=taowo)
            {
                OutputOfWo[i]=1/(1+2*TYOfWo/T)*(Kofwo*(OutputOfControlPI[i-taowo1]+QianKuiForinterfere));
            }
            else
            {
                OutputOfWo[i]=0;
            }
            
        }
        else
        {
            OutputOfWo[i]=1/(1+2*TYOfWo/T)*(KOfWo*(OutputOfControlPI[i-taowo]+OutputOfControlPI[i-1-taowo]
                +QianKuiForinterfere[i-taowo]+QianKuiForinterfere[i-i-taowo])
            -(1-2*TOfWo/T)*OutputOfWo1[i-1-taowo]);
            
        }

        taowf1=timeDelayOfWf1/T;
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

        subOutput[i]=OutputOfWo1[i]+OutputOfWf1[i];
        //计算主回路的输出值
        taowo2=timeDelayOfWo2/T;
        if(i<1+taowo2)
        {
            if(i>=taowo2)
            {
                OutputOfWo2[i]=1/(1+2*TYOfWo2/T)*(Kofwo2*(subOutput[i-taowo2]));
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

        taowf2=timeDelayOfWf2/T;
        if(i<1+taowf2)
        {
            if(i>=taowf2)
            {
                OutputOfWf2[i]=1/(1+2*TYOfWf2/T)*(KOfWf2*(interfere2[i-taowf2]));
            }
            else
            {
                OutputOfWf2[i]=0;
            }
        }
        else
        {
            OutputOfWf2[i]=1/(1+2*TYOfWf2/T)*(KOfWf2*(interfere2[i]+interfere2[i-1-taowf2])
            -(1-2*TYOfWf2/T)*OutputOfWf2[i-1-taowf2]);
        }

        mainOutput[i]=OutputOfWo2[i]+OutputOfWf2[i];
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
    OutputOfControlPI1,OutputOfControlPI2,OutputOfWf1,outputofwf2);
    
    return liangzi;
}


/*对数组进行初始化,返回该数组的函数*/
function initArray(array,value)
{
    for (everyValue in array)
    {
        everyValue=value;
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
    this.OutputOfWf2=outputofwf2;  
};

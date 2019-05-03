
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

array1=[1,2,3,4,5];
array2=[11,22,33,44,55];

cc=generate2dimensionArray(array1,array2);
console.log(cc);
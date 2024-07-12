 /************************************线性回归计算斜率************************************/
/*   
   *  @brief 最小二乘法拟合直线斜率
   * @param 输入点集
   * @param startline 开始的行数
   * @param endline 结束的行数
   * @return 返回拟合的斜率
*/
float regression(vector<POINT> &v_edge,int startline,int endline)
{
  
  int i=0,SumX=0,SumY=0,SumLines = 0; 
  float SumUp=0,SumDown=0,avrX=0,avrY=0,B,A;
   int t = 0;
    if (startline > endline) // 都是从上往下计算的，反了就互换一下
    {
      t = startline;
      startline = endline;
      endline = t;
    }  
  SumLines=endline-startline;   // startline 为开始行， //endline 结束行 //SumLines
 
  for(i=startline;i<endline;i++)     
  { 
    SumX+=i;       
    SumY+=v_edge[i].y;    //这里Middle_black为存放中线的数组
  }         
  avrX=SumX/SumLines;     //X的平均值
  avrY=SumY/SumLines;     //Y的平均值       
  SumUp=0;      
  SumDown=0;  
  for(i=startline;i<endline;i++)   
  {       
    SumUp+=(v_edge[i].y-avrY)*(i-avrX);    
    SumDown+=(v_edge[i].y-avrY)^2;    
  }    
  if(SumDown==0) 
    B=0;  
  else 
    B=(SumUp/SumDown);       
    A=(SumY-B*SumX)/SumLines;  //截距
    return B;  //返回斜率
}
/**
   * @brief 边缘斜率计算部分
   *
   * @param v_edge
   * @param img_height
   * @return double
   */
  double Part_stdevEdgeCal(vector<POINT> &v_edge, int img_height,int start,int end ) {
      int i = 0, t = 0;
    if (start < end) // 都是从下往上计算的，反了就互换一下
    {
      t = start;
      start = end;
      end = t;
    }    
    if (start >= ROWSIMAGE - 1 - 5) // 数组越界保护
      start = ROWSIMAGE - 1 - 5;
    if (end <= 5)
      end = 5;
    if (v_edge.size() < img_height / 4) {
      return 1000;
    }
    vector<int> v_slope;
    int step = 4; ///这是间隔// v_edge.size()/10;
    for (int i = end; i < start; i += step) {
      if (v_edge[i].x - v_edge[i - step].x)
        v_slope.push_back((v_edge[i].y - v_edge[i - step].y) * 100 /
                          (v_edge[i].x - v_edge[i - step].x));
    }
    if (v_slope.size() > 1) {
      double sum = accumulate(begin(v_slope), end(v_slope), 0.0);
      double mean = sum / v_slope.size(); // 均值
      double accum = 0.0;
      for_each(begin(v_slope), end(v_slope),
               [&](const double d) { accum += (d - mean) * (d - mean); });

      return sqrt(accum / (v_slope.size() - 1)); // 方差
    } else
      return 0;
  }


  /*-------------------------------------------------------------------------------------------------------------------
  @brief     通过斜率，左边线定点补线--
  @param     k       输入斜率
             startY  输入起始点纵坐标
             startX  输入起始点横坐标
             endX    结束点纵坐标
  @return    null
  Sample     K_Add_Boundry_Left(float k,int startY,startX,int endX);
  @note      补得线直接贴在边线上
-------------------------------------------------------------------------------------------------------------------*/
void K_Add_Boundry_Left(float k,int startY,int startX,int endX)
{
    int i=0,t=0;
    if(startY>=COLSIMAGE)  //减去切行
        startY=COLSIMAGE;
    else if(startY<=0)
        startY=0;
    if(endX>=track.pointsEdgeLeft.size()-1)
        endX=track.pointsEdgeLeft.size()-1;
    else if(endX<=0)
        endX=0;
    if(startX<endX)//--操作，start需要大
    {
        t=startX;
        startX=endX;
        endX=t;
    }
//这里有bug，下方循环--循环，需要start要大，只进行y的互换，但是没有进行x的互换
//建议进行判断，如果start更小，那就进行++访问
//这里修改各位自行操作
    for(i=startX;i>=endX;i--)
    {
        track.pointsEdgeLeft[i].y=(int)((i-startX)/k+startY);//(y-y1)=k(x-x1)变形，x=(y-y1)/k+x1
        if(track.pointsEdgeLeft[i].y>=COLSIMAGE-1)
        {
           track.pointsEdgeLeft[i].y=COLSIMAGE-1;
        }
        else if(track.pointsEdgeLeft[i].y<=0)
        {
            track.pointsEdgeLeft[i].y[i]=0;
        }
    }
}
 /*-------------------------------------------------------------------------------------------------------------------
  @brief     通过斜率，右边线定点补线--
  @param     k       输入斜率
             startY  输入起始点纵坐标
             startX  输入起始点横坐标
             endX    结束点纵坐标
  @return    null
  Sample     K_Add_Boundry_Right(float k,int startY,startX,int endX);
  @note      补得线直接贴在边线上
-------------------------------------------------------------------------------------------------------------------*/
void K_Add_Boundry_Right(float k,int startY,int startX,int endX)
{
    int i=0,t=0;
    if(startY>=COLSIMAGE)  //减去切行
        startY=COLSIMAGE;
    else if(startY<=0)
        startY=0;
    if(endX>=track.pointsEdgeRight.size()-1)
        endX=track.pointsEdgeRight.size()-1;
    else if(endX<=0)
        endX=0;
    if(startX<endX)//--操作，start需要大
    {
        t=startX;
        startX=endX;
        endX=t;
    }
//这里有bug，下方循环--循环，需要start要大，只进行y的互换，但是没有进行x的互换
//建议进行判断，如果start更小，那就进行++访问
//这里修改各位自行操作
    for(i=startX;i>=endX;i--)
    {
        track.pointsEdgeRight[i].y=(int)((i-startX)/k+startY);//(y-y1)=k(x-x1)变形，x=(y-y1)/k+x1
        if(track.pointsEdgeRight[i].y>=COLSIMAGE-1)
        {
           track.pointsEdgeRight[i].y=COLSIMAGE-1;
        }
        else if(track.pointsEdgeRight[i].y<=0)
        {
            track.pointsEdgeRight[i].y=0;
        }
    }
}
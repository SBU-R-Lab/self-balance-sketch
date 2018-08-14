class Plot
{
  int x,y,w,h;
  GPlot plot ;
  int size =500;
  GPointsArray points = new GPointsArray(size);
  Plot(int x,int y,int w,int h,String title,PApplet parent)
  {
    plot = new GPlot(parent);
    plot.setPos(x, y);
    plot.setDim(200,100);
    plot.setTitleText(title);
    //plot.setLineColor(200);
  }
  void addPoint(float x , float y)
  {
    points.add(x,y);
    plot.setPoints(points);
    if (points.getNPoints()-size > 0)
      points.removeRange(0,points.getNPoints()-size);
  }
  void draw()
  {
    plot.defaultDraw();
  }
}

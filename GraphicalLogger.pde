class GraphicalLogger
{
  int x,y;
  int  fSize = 24;
  GraphicalLogger(int x , int y)
  {
     this.x = x;
     this.y = y;
  }
  
  ArrayList<String> items = new ArrayList<String>();
  void draw()
  {
    textSize(fSize);
    for (int i =0 ;i < items.size(); i++)
    {
      text(items.get(i),x,y+i*fSize);
    }
    items.clear();
  }
  void add(String s , float num)
  {
      items.add(s + " : " + str(num));
  }
  void add(String s , int num)
  {
      items.add(s + " : " + str(num));
  }
}

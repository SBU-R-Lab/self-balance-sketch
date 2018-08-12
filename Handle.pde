class Handle
{
  int x,y,w,h;
  Handle(int x, int y ,int w ,int h)
  {
    this.x = x;
    this.y = y;
    this.w = w;
    this.h = h;
  }
  void draw(float angle)
  {
    pushMatrix();
    translate(x + w/2,y + 4 * h/5);
    rotate(angle);
    fill(255,0,0);
    rect(-w/2,- 4 * h/5,w,h);
    popMatrix();
  }
}

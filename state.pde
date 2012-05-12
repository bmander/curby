class State{
  ProbabilityDensityFunction s;
  ProbabilityDensityFunction v;
  ProbabilityDensityFunction a;
  float t;
  
  float a_obs;
  
  State(float a_obs, float t){
    this.s=new DegenerateDensityFunction(0);
    this.v=new DegenerateDensityFunction(0);
    this.a_obs=a_obs;
    this.t=t;
    
    this.a=null;
  }
  
  void setA(ProbabilityDensityFunction a){
    this.a=a;
  }
  
  void draw(float zoom){
    strokeWeight(0.5);
    stroke(255,0,0);
    line(width/2,0,width/2,height);
    line(width/2-zoom,0,width/2-zoom,height);
    line(width/2+zoom,0,width/2+zoom,height);
    
    stroke(0);
    line(zoom*a_obs+width/2,0,zoom*a_obs+width/2,height/3);
    v.draw(-2.0, 2.0, width/2, 1*height/3, 10, 200.0);
    s.draw(-2.0, 2.0, width/2, 0*height/3, 10, 200.0);
    
    if(this.a!=null){
      strokeWeight(2);
      stroke(0,0,255);
      a.draw(-2.0, 2.0, width/2, 2*height/3, 10, 200.0);
    }
  }
}
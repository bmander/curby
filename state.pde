class State{
  ProbabilityDensityFunction bias;
  ProbabilityDensityFunction wbias;
  
  ProbabilityDensityFunction s;
  ProbabilityDensityFunction v;
  ProbabilityDensityFunction a;
  ProbabilityDensityFunction w;
  float t;
  
  float a_obs;
  float w_obs;
  
  State(float a_obs, float w_obs, float t, float v){
    this.bias = new UniformDensityFunction(-0.5,0.5);
    this.wbias = new UniformDensityFunction(-0.5,0.5);
    
    this.a_obs=a_obs;
    this.w_obs=w_obs;
    this.t=t;
    
    this.a=new DoubleExponentialDensityFunction( -v*TIMIDNESS, WANDERLUST );
    this.w = new DoubleExponentialDensityFunction( 0, SPINNYNESS );
  }
  
  void setA(ProbabilityDensityFunction a){
    this.a=a;
  }
  
  void draw_probpane(ProbabilityDensityFunction p, int pane, float xscale, float yscale, String caption){
    float functionwidth = (width/2)/xscale;
    p.draw(-functionwidth, functionwidth, width/2, pane*height/NPANES, yscale, xscale);
    text(caption, 5, height-((pane+1)*height/NPANES-20));
  }
  
  void draw_obs(float x, int pane, float xzoom, String caption){
    line(xzoom*x+width/2,
      height-((pane+1)*height/NPANES),
      xzoom*x+width/2,
      height-((pane)*height/NPANES));
    text(caption, 5, height-((pane+1)*height/NPANES-40) );
  }
  
  void draw(float zoom){
    strokeWeight(0.5);
    stroke(255,0,0);
    line(width/2,0,width/2,height);
    line(width/2-zoom,0,width/2-zoom,height);
    line(width/2+zoom,0,width/2+zoom,height);
    
    stroke(0);
    draw_probpane( v, 2, 200.0, 10.0, "argmax(v)="+fround(v.argmax(),3)+" ms^-1" );
    draw_probpane( s, 1, 200.0, 10.0, "argmax(s)="+fround(s.argmax(),3)+" m" );
    
    draw_probpane( w, 0, 3.0, 1000.0, "argmax(w)="+fround(w.argmax(),3)+" deg/s" );
    draw_obs(w_obs,0,3.0, "w_obs="+fround(w_obs,3)+" deg/s");
    
    strokeWeight(2);
    stroke(0,0,255);
    draw_obs(a_obs,3,zoom, "a_obs="+fround(a_obs,3)+" ms^-2");
    draw_probpane(a,3,200.0,10.0, "argmax(a)="+fround(a.argmax(),2)+" ms^-2");
    

  }
}

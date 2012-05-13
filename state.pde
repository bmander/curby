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
  

  
  void draw(float zoom){
    strokeWeight(0.5);
    stroke(255,0,0);
    //line(width/2,0,width/2,height);
    //line(width/2-zoom,0,width/2-zoom,height);
    //line(width/2+zoom,0,width/2+zoom,height);
    
    stroke(0);
    draw_probpane( v, 2, 200.0, 10.0, "argmax(v)="+fround(v.argmax(),3)+" ms^-1", 1.0 );
    draw_probpane( s, 1, 200.0, 10.0, "argmax(s)="+fround(s.argmax(),3)+" m", 1.0 );
    
    draw_probpane( w, 0, 3.0, 1000.0, "argmax(w)="+fround(w.argmax(),3)+" deg/s", 20.0 );
    draw_obs(w_obs,0,3.0, "w_obs="+fround(w_obs,3)+" deg/s", color(0,0,255),NPANES);
    
    draw_obs(a_obs,3,zoom, "a_obs="+fround(a_obs,3)+" ms^-2", color(0,0,255),NPANES);
    draw_probpane(a,3,200.0,10.0, "argmax(a)="+fround(a.argmax(),2)+" ms^-2", 1.0);
    

  }
}

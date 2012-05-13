class State{
  ProbabilityDensityFunction bias;
  
  ProbabilityDensityFunction s;
  ProbabilityDensityFunction v;
  ProbabilityDensityFunction a;
  ProbabilityDensityFunction omega;
  float t;
  
  float a_obs;
  
  State(float a_obs, float t, float v){
    this.bias = new UniformDensityFunction(-0.5,0.5);
    
    this.a_obs=a_obs;
    this.t=t;
    
    this.a=new DoubleExponentialDensityFunction( -v*TIMIDNESS, WANDERLUST );
    this.omega = new DoubleExponentialDensityFunction( 0, SPINNYNESS );
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
    line(zoom*a_obs+width/2,0,zoom*a_obs+width/2,height/NPANES);
    v.draw(-2.0, 2.0, width/2, 2*height/NPANES, 10, 200.0);
    s.draw(-2.0, 2.0, width/2, 1*height/NPANES, 10, 200.0);
    omega.draw(-300,300,width/2, 0*height/NPANES, 1000, 1);
    
    strokeWeight(2);
    stroke(0,0,255);
    a.draw(-2.0, 2.0, width/2, 3*height/NPANES, 10, 200.0);
    

  }
}

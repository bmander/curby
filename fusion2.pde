import processing.serial.*;
final String serialPort = "COM14"; // replace this with your serial port. On windows you will need something like "COM1".

int NPANES=5;
int NPROBPANES=5;

float MPERSSQUARED_PER_BIT = (1/256.0)*9.807; //(g/LSB)*(m*s^-2/g)=m*s^-2/LSB
float LSB_PER_DEGREE_PER_SECOND = 14.375;

int MODE_AVS = 0;
int MODE_PROB = 1;

//free paramters
float TIMIDNESS=5.0; //tendancy to decelerate given a velocity
float WANDERLUST=4.0; //tau of exponential that describes the acceleration prior
float SPINNYNESS=0.1; //tau of exponential that describes the tendancy to spin
float BIAS_WANDER=0.0001; //stddev of movement of bias between successive time slices
float ACCEL_NOISE_RMS=0.038; //data sheet value for accelerometer
float ACCEL_NOISE_FUDGE=1.2; //it seems slightly higher in practice
float GYRO_NOISE_RMS=0.038; //degrees per second

IMU imu;
PFont font;
boolean running;
boolean runonce;
boolean sampling;
int mode;

class Sampleset {
  Histogram accel_samples;
  Histogram bias_samples;
  Histogram w_samples;
  Histogram wbias_samples;
  Histogram theta_samples;
  
  Sampleset(State laststate){
    accel_samples=new Histogram(-10,10,0.02);
    bias_samples=new Histogram(-5,5,0.01);
    w_samples=new Histogram(-150,150,0.05);
    wbias_samples=new Histogram(-1,1,0.07);
    theta_samples=new Histogram(-90,90,0.1);
  }
}

class Graph{
  
  float dt;
  
  ProbabilityDensityFunction bias_movement_prior;
  ProbabilityDensityFunction noise_prior;
  ProbabilityDensityFunction wnoise_prior;
 
  State laststate;
  State state;
  
  Graph(){
    bias_movement_prior = new GaussianDensityFunction(0,BIAS_WANDER);
    noise_prior = new GaussianDensityFunction(0,ACCEL_NOISE_FUDGE*ACCEL_NOISE_RMS);
    wnoise_prior = new GaussianDensityFunction(0,GYRO_NOISE_RMS);
  }
  
  Sampleset sample(int n){
    Sampleset sampleset = new Sampleset(laststate);
  
    ProbabilityDensityFunction last_bias_proposal_dist = new UniformDensityFunction(laststate.bias.left(),laststate.bias.right());
    ProbabilityDensityFunction bias_movement_proposal_dist = graph.bias_movement_prior;//new UniformDensityFunction(bias_movement_prior.left(),bias_movement_prior.right());
    ProbabilityDensityFunction noise_proposal_dist = graph.noise_prior;//new UniformDensityFunction(noise_prior.left(),noise_prior.right());
    
    ProbabilityDensityFunction last_wbias_proposal_dist = new UniformDensityFunction(laststate.wbias.left(),laststate.wbias.right());
    ProbabilityDensityFunction wnoise_proposal_dist = graph.wnoise_prior;
    
    ProbabilityDensityFunction last_w_proposal_dist;
    ProbabilityDensityFunction last_theta_proposal_dist;
    if(laststate.w.left()==laststate.w.right()){
      last_w_proposal_dist=laststate.w; //it's a degenerate distribution
    } else {
      last_w_proposal_dist = new UniformDensityFunction(laststate.w.left(), laststate.w.right());
    }
    if(laststate.theta.left()==laststate.theta.right()){
      last_theta_proposal_dist=laststate.theta; //degenerate function
    } else {
      last_theta_proposal_dist=new UniformDensityFunction(laststate.theta.left(), laststate.theta.right());
    }
  
    for(int i=0; i<n; i++){
      float last_bias_proposal = last_bias_proposal_dist.sample();
      float bias_movement_proposal = bias_movement_proposal_dist.sample();
      float noise_proposal = noise_proposal_dist.sample();
      float bias_proposal = last_bias_proposal+bias_movement_proposal;
      float accel_proposal = state.a_obs-(bias_proposal+noise_proposal);
      
      float wbias_proposal = last_wbias_proposal_dist.sample();
      float wnoise_proposal = wnoise_proposal_dist.sample();
      float w_proposal = state.w_obs-wbias_proposal-wnoise_proposal;
      
      float last_w_proposal = last_w_proposal_dist.sample();
      float last_theta_proposal = last_theta_proposal_dist.sample();
      float theta_proposal = last_theta_proposal + last_w_proposal*dt;
    
      //likelihood of sample
      float likelihood = 1.0;
      likelihood *= state.a.probDensity(accel_proposal);
      likelihood *= laststate.bias.probDensity(last_bias_proposal)/last_bias_proposal_dist.probDensity(last_bias_proposal);
      likelihood *= laststate.wbias.probDensity(wbias_proposal)/last_wbias_proposal_dist.probDensity(wbias_proposal);
      likelihood *= state.w.probDensity(w_proposal);
      
      if(laststate.w.left()!=laststate.w.right()){
        likelihood *= laststate.w.probDensity(last_w_proposal)/last_w_proposal_dist.probDensity(last_w_proposal);
      }
      if(laststate.theta.left()!=laststate.theta.right()){
        likelihood *= laststate.theta.probDensity(last_theta_proposal)/last_theta_proposal_dist.probDensity(last_theta_proposal);
      }
      
      //likelihood *= bias_movement_prior.probDensity(bias_movement_proposal)/bias_movement_proposal_dist.probDensity(bias_movement_proposal);
      //likelihood *= noise_prior.probDensity(noise_proposal)/noise_proposal_dist.probDensity(noise_proposal);
      
      sampleset.accel_samples.add( accel_proposal, likelihood );
      sampleset.bias_samples.add( bias_proposal, likelihood );
      sampleset.wbias_samples.add( wbias_proposal, likelihood );
      sampleset.w_samples.add( w_proposal, likelihood );
      sampleset.theta_samples.add( theta_proposal, likelihood );
    }
    
    return sampleset;
  }
  
  void update(float a_obs, float w_obs, float t, int n){
    // move the current state to the past
    laststate=state;
      
    // pop a new state into the present, with measurements from the IMU
    // if the past didn't exist, then this is the first measurement; no further work to do this iteration
    if(laststate==null){
      state = new State(a_obs,w_obs,t,0);
      state.s=new DegenerateDensityFunction(0);
      state.v=new DegenerateDensityFunction(0);
      state.a=new DegenerateDensityFunction(0);
      state.w=new DegenerateDensityFunction(0);
      state.theta=new DegenerateDensityFunction(0);
      return;
    } else {
      state = new State(a_obs,w_obs,t,laststate.v.argmax());
    }
    
    //sample the portion of the graph connected to state.a
    Sampleset smp = sample(n);
    sampleset = smp; //export it to the global scope so we can draw it
    
    //update probability distriubtions using sample set
    state.bias = new HistogramDensityFunction( smp.bias_samples );
    state.a = new HistogramDensityFunction( smp.accel_samples );
    state.wbias = new HistogramDensityFunction( smp.wbias_samples );
    state.w = new HistogramDensityFunction( smp.w_samples );
    state.theta = new HistogramDensityFunction( smp.theta_samples );
    
    //update current state velocity and position using analytical methods
    dt = graph.state.t - graph.laststate.t;
    state.v = advance_gaussian( laststate.v, laststate.a, dt );
    state.s = advance_gaussian( laststate.s, laststate.v, dt );
    
    //advance rotation
    //state.theta = advance_gaussian( laststate.theta, laststate.w, dt );
  }
  
}

Graph graph;

Sampleset sampleset;

void keyPressed(){
  if(key==' '){ //reset
    graph.state.s=new DegenerateDensityFunction(0);
    graph.state.v=new DegenerateDensityFunction(0);
  } else if(key=='p'){ //pause
    if(running){
      running=false;
    } else {
      imu.clear();
      running=true;
    }
  } else if(key=='n'){  //next
    runonce=true;
  } else if(key=='m'){  //mode
    if(mode==MODE_AVS){
      mode=MODE_PROB;
    } else{
      mode=MODE_AVS;
    }
  } 
}

void setup(){
  size(800,850);
  smooth();
  font= loadFont("ArialMT-14.vlw");
  textFont(font);
  
  imu = new IMU(this, serialPort);
  
  
  running=true;
  runonce=false;
  mode=MODE_AVS;
  
  graph = new Graph();
}

ProbabilityDensityFunction advance_by_sampling( ProbabilityDensityFunction x0, ProbabilityDensityFunction dx, float dt, int n ){
  
  float leftsupport = x0.left()+dx.left()*dt;
  float rightsupport = x0.right()+dx.right()*dt;
  
  if(leftsupport>rightsupport){
    throw new RuntimeException("left support can't be righter than right support x0:("+x0.left()+"-"+x0.right()+") dx:("+dx.left()+"-"+dx.right()+") support:"+leftsupport+" "+rightsupport);
  }
  
  println( "support: "+leftsupport+"-"+rightsupport );
  
  float resolution = (rightsupport-leftsupport)/100;
  
  if(resolution<0.02){
    resolution=0.02;
  }
  
  Histogram x1 = new Histogram(leftsupport,rightsupport,resolution);
  
  //cache these in case calculating them is difficult; we'll need them lots
  float x0_left = x0.left();
  float x0_right = x0.right();
  float dx_left = dx.left();
  float dx_right = dx.right();
  
  float x0_proposal;
  float dx_proposal;
  float x1_proposal;
  float likelihood;
  
  for(int i=0; i<n; i++){
    x0_proposal = random(x0_left,x0_right);
    dx_proposal = random(dx_left,dx_right);
    
    x1_proposal = x0_proposal+dx_proposal*dt;
        
    likelihood = 1;
    if(x0_left!=x0_right){
      likelihood *= x0.probDensity(x0_proposal)/(1/(x0_right-x0_left));
    }
    if(dx_left!=dx_right){
      likelihood *= dx.probDensity(dx_proposal)/(1/(dx_right-dx_left));
    }
        
    try{
      x1.add( x1_proposal, likelihood );
    }catch(ArrayIndexOutOfBoundsException ex){
      println( "support "+leftsupport+"-"+rightsupport );
      println( x1_proposal );
      println( resolution );
      println( (x1_proposal-leftsupport)/resolution );
      throw ex;
    }
  }
  
  
  return new HistogramDensityFunction( x1 );
  
}

ProbabilityDensityFunction advance_degenerate( ProbabilityDensityFunction x0, ProbabilityDensityFunction dx, float dt ){
  
  return new DegenerateDensityFunction(x0.argmax() + dx.argmax()*dt);

}

ProbabilityDensityFunction advance_gaussian( ProbabilityDensityFunction x0, ProbabilityDensityFunction dx, float dt ){
  
  return new GaussianDensityFunction(x0.argmax() + dx.argmax()*dt, sqrt(sq(x0.stddev())+sq(dx.stddev()*dt)));

}

void draw_histogram(Histogram histogram, int pane, float xscale, float yscale, String caption, float tickpitch){
  float functionwidth = (width/2)/xscale;
  //draw scale ticks
  draw_obs(0,pane,xscale,null, color(200),NPROBPANES);
  for(float i=0; i<functionwidth; i+=tickpitch){
    draw_obs(i,pane,xscale,null, color(200),NPROBPANES);
    draw_obs(-i,pane,xscale,null, color(200),NPROBPANES);
  }
  
  histogram.draw(width/2,pane*height/NPROBPANES,xscale,yscale);
  text(caption,5,20+(NPROBPANES-pane-1)*height/NPROBPANES);
  

}

void draw_probpane(ProbabilityDensityFunction p, int pane, float xscale, float yscale, String caption, float tickpitch){
  draw_probpane(p,pane,xscale,yscale,caption,tickpitch,NPANES);
}

void draw_probpane(ProbabilityDensityFunction p, int pane, float xscale, float yscale, String caption, float tickpitch, int npanes){
  stroke(0);
  fill(0);
  float functionwidth = (width/2)/xscale;
  p.draw(-functionwidth, functionwidth, width/2, pane*height/npanes, yscale, xscale);
  text(caption, 5, height-((pane+1)*height/npanes-20));
    
  //draw scale ticks
  draw_obs(0,pane,xscale,null, color(200),npanes);
  for(float i=0; i<functionwidth; i+=tickpitch){
    draw_obs(i,pane,xscale,null, color(200),npanes);
    draw_obs(-i,pane,xscale,null, color(200),npanes);
  }
}
  
void draw_obs(float x, int pane, float xzoom, String caption, color strokecolor, int npanes){
  stroke(strokecolor);
  fill(strokecolor);
    
  line(xzoom*x+width/2,
    height-((pane+1)*height/npanes),
    xzoom*x+width/2,
    height-((pane)*height/npanes));
      
  if(caption!=null){
    text(caption, 5, height-((pane+1)*height/npanes-40) );
  }
}

void draw(){
  //float dt=0;
 
  running=false;
  runonce=true;
  // update the state until the serial stream runs dry
  while(running || runonce){
    
    try{
      // grab a reading from the IMU
      IMUReading reading = imu.read();
      
      if(reading==null){
        break;
      }
      
      // convert to SI units
      float a_obs = reading.ax*MPERSSQUARED_PER_BIT;
      float w_obs = -reading.wy/LSB_PER_DEGREE_PER_SECOND;
      float t = reading.t/1000.0;
      
      graph.update(a_obs, w_obs, t, 50000);
      
    } catch (IMUParseException e){
    }
    
    runonce=false;
    
  }
 
  //draw
  if(mode==MODE_PROB){
    background(255);
    
    //draw priors
    draw_probpane(graph.laststate.bias, 3, 200.0, 5.0, "'last_bias prior' distribution", 1, NPROBPANES);
    
    //draw posteriors
    if(sampleset.accel_samples!=null){
      fill(0);
      draw_histogram( sampleset.accel_samples, 4, 200, 0.002, "'a' sample histogram", 1.0 );
      draw_histogram( sampleset.bias_samples, 2, 200, 0.002, "'bias' sample histogram", 1.0 );
      draw_histogram( sampleset.w_samples, 1, 100, 0.002, "'w' sample histogram", 1.0 );
      draw_histogram( sampleset.wbias_samples, 0, 200, 0.002, "'wbias' sample histogram", 0.1 );
    }
    
  } else {
    if(graph.state!=null){
      background(255);
      fill(28);
      text("dt="+fround(graph.dt,3)+" s", width-200,height-20 );
      
      graph.state.draw(200.0);
    }
  }
  
  delay(1000);
}

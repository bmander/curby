import processing.serial.*;
final String serialPort = "COM14"; // replace this with your serial port. On windows you will need something like "COM1".

float MPERSSQUARED_PER_BIT = (1/256.0)*9.807; //(g/LSB)*(m*s^-2/g)=m*s^-2/LSB

int MODE_AVS = 0;
int MODE_PROB = 1;

//free paramters
float TIMIDNESS=5.0; //tendancy to decelerate given a velocity
float WANDERLUST=4.0; //tau of exponential that describes the acceleration prior
float BIAS_WANDER=0.0001; //stddev of movement of bias between successive time slices
float ACCEL_NOISE_RMS=0.038; //data sheet value for accelerometer
float ACCEL_NOISE_FUDGE=1.2; //it seems slightly higher in practice

IMU imu;
PFont font;
boolean running;
boolean runonce;
boolean sampling;
int mode;

class Sampleset {
  Histogram accel_posterior;
  Histogram last_bias_posterior;
  Histogram bias_posterior;
  
  Sampleset(State laststate){
    accel_posterior=new Histogram(-5,5,0.01);
    last_bias_posterior=new Histogram(laststate.bias.left(),laststate.bias.right(),0.01);
    bias_posterior=new Histogram(-5,5,0.01);
  }
}

ProbabilityDensityFunction accel_prior;
ProbabilityDensityFunction bias_movement_prior;
ProbabilityDensityFunction noise_prior;

Sampleset sampleset;

State laststate;
State state;

void keyPressed(){
  if(key==' '){ //reset
    state.s=new DegenerateDensityFunction(0);
    state.v=new DegenerateDensityFunction(0);
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
  size(800,500);
  smooth();
  font= loadFont("ArialMT-14.vlw");
  textFont(font);
  
  imu = new IMU(this, serialPort);
  
  state=null;
  running=true;
  runonce=false;
  mode=MODE_AVS;
  
  accel_prior=new DoubleExponentialDensityFunction( 0, WANDERLUST );
  //last_bias_prior=new UniformDensityFunction(-0.5,0.5);
  bias_movement_prior = new GaussianDensityFunction(0,BIAS_WANDER);
  noise_prior = new GaussianDensityFunction(0,ACCEL_NOISE_FUDGE*ACCEL_NOISE_RMS);
}

void sample(State laststate, State state, int n){
  
    ProbabilityDensityFunction last_bias_proposal_dist = new UniformDensityFunction(laststate.bias.left(),laststate.bias.right());
    ProbabilityDensityFunction bias_movement_proposal_dist = bias_movement_prior;//new UniformDensityFunction(bias_movement_prior.left(),bias_movement_prior.right());
    ProbabilityDensityFunction noise_proposal_dist = noise_prior;//new UniformDensityFunction(noise_prior.left(),noise_prior.right());
  
    for(int i=0; i<n; i++){
      float last_bias_proposal = last_bias_proposal_dist.sample();
      float bias_movement_proposal = bias_movement_proposal_dist.sample();
      float noise_proposal = noise_proposal_dist.sample();
      float bias_proposal = last_bias_proposal+bias_movement_proposal;
      float accel_proposal = state.a_obs-(bias_proposal+noise_proposal);
    
      //likelihood of sample
      float likelihood = 1.0;
      likelihood *= accel_prior.probDensity(accel_proposal);
      likelihood *= laststate.bias.probDensity(last_bias_proposal)/last_bias_proposal_dist.probDensity(last_bias_proposal);
      //likelihood *= bias_movement_prior.probDensity(bias_movement_proposal)/bias_movement_proposal_dist.probDensity(bias_movement_proposal);
      //likelihood *= noise_prior.probDensity(noise_proposal)/noise_proposal_dist.probDensity(noise_proposal);
      
      sampleset.accel_posterior.add( accel_proposal, likelihood );
      sampleset.last_bias_posterior.add( last_bias_proposal, likelihood );
      sampleset.bias_posterior.add( bias_proposal, likelihood );
    }
}

void update_state(State laststate, State state){
    sampleset = new Sampleset(laststate);
    
    sample(laststate, state, 5000);
    
    state.bias = new HistogramDensityFunction( sampleset.bias_posterior );
    
    state.a = new HistogramDensityFunction( sampleset.accel_posterior );
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

void draw(){
  float dt=0;
 
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
      float t = reading.t/1000.0;
      
      // move the current state to the past
      laststate=state;
      
      // pop a new state into the present, with measurements from the IMU
      state = new State(a_obs,t);
      
      // if the past didn't exist, then this is the first measurement; no further work to do this iteration
      if(laststate==null){
        continue;
      }
      
      update_state(laststate, state);
        
      dt = state.t - laststate.t;
        
      if(laststate.a!=null){
        state.v = advance_gaussian( laststate.v, laststate.a, dt );
        state.s = advance_gaussian( laststate.s, laststate.v, dt );
          
        accel_prior=new DoubleExponentialDensityFunction( -state.v.argmax()*TIMIDNESS, 4 );
      }
      
    } catch (IMUParseException e){
    }
    
    runonce=false;
    
  }
 
  //draw
  if(mode==MODE_PROB){
    background(255);
    
    //draw priors
    stroke(255,0,0);
    accel_prior.draw( -2.0, 2.0, width/2, 2*height/3, 75, 200.0);
    laststate.bias.draw(-2.0,2.0,width/2, 1*height/3, 15, 200.0);
    
    //draw posteriors
    if(sampleset.accel_posterior!=null&&sampleset.last_bias_posterior!=null){
      fill(0);
      sampleset.accel_posterior.draw(width/2,2*height/3,200,0.0002);
      sampleset.last_bias_posterior.draw(width/2,height/3,200,0.0002);
      sampleset.bias_posterior.draw(width/2,0,200,0.0002);
    }
    
    //draw text
    fill(255,0,0);
    text("'a' prior distribution",5,20);
    text("'last bias' prior distribution",5,20+height/3);
    fill(0,0,0);
    text("'a' posterior distribution",5,20+20);
    text("'last bias' posterior distribution",5,20+height/3+20);
    text("'bias' posterior distribution",5,20+2*height/3+20);
    fill(0,0,0);
  } else {
    if(state!=null){
      background(255);
      fill(28);
      text("dt="+fround(dt,3)+" s", width-200,height-20 );
      text("a_obs="+fround(state.a_obs,3)+" ms^-2", 5, 20 );
      text("argmax(v)="+fround(state.v.argmax(),3)+" ms^-1", 5, height/3+20);
      text("argmax(s)="+fround(state.s.argmax(),3)+" m", 5, 2*height/3+20);
      fill(0,0,255);
      text("argmax(a)="+fround(state.a.argmax(),2)+" ms^-2", 5, 20+20);
      state.draw(200.0);
    }
  }
  
  //delay(100);
}

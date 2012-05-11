import processing.serial.*;
final String serialPort = "COM14"; // replace this with your serial port. On windows you will need something like "COM1".

float MPERSSQUARED_PER_BIT = (1/256.0)*9.807; //(g/LSB)*(m*s^-2/g)=m*s^-2/LSB

int MODE_AVS = 0;
int MODE_PROB = 1;

IMU imu;
PFont font;
boolean running;
boolean runonce;
boolean sampling;
int mode;

ProbabilityDensityFunction accel_prior;
ProbabilityDensityFunction last_bias_prior;
ProbabilityDensityFunction bias_movement_prior;
ProbabilityDensityFunction noise_prior;
Histogram accel_posterior;
Histogram last_bias_posterior;
Histogram bias_posterior;
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
    last_bias_prior = new HistogramDensityFunction(last_bias_posterior);
  
    accel_posterior=new Histogram(-5,5,0.02);
    last_bias_posterior=new Histogram(-5,5,0.02);
    

  
    runonce=true;
  } else if(key=='m'){  //mode
    if(mode==MODE_AVS){
      mode=MODE_PROB;
    } else{
      mode=MODE_AVS;
    }
  } else if(key=='s'){ //sample
    if(!sampling){
      accel_posterior=new Histogram(-5,5,0.02);
      last_bias_posterior=new Histogram(-5,5,0.02);
      sampling=true;
    } else{
      sampling=false;
    }
  } 
}

void setup(){
  size(800,500);
  smooth();
  font= loadFont("ArialMT-14.vlw");
  textFont(font);
  
  //frameRate(1);
  
  imu = new IMU(this, serialPort);
  
  state=null;
  running=true;
  runonce=false;
  mode=MODE_PROB;
  
  accel_prior=new DoubleExponentialDensityFunction( 0, 4 );
  last_bias_prior=new UniformDensityFunction(-0.5,0.5);
  bias_movement_prior = new GaussianDensityFunction(0,0.0005);
  noise_prior = new GaussianDensityFunction(0,0.038);
}

void sampleonce(float a_obs){
      float last_bias_proposal = random(last_bias_prior.left(),last_bias_prior.right());
      float bias_movement_proposal = random(bias_movement_prior.left(), bias_movement_prior.right());
      float noise_proposal = random(noise_prior.left(),noise_prior.right());
      float bias_proposal = last_bias_proposal+bias_movement_proposal;
      float accel_proposal = a_obs-(bias_proposal+noise_proposal);
    
      //likelihood of sample
      float likelihood = 1.0;
      likelihood *= accel_prior.probDensity(accel_proposal);
      likelihood *= last_bias_prior.probDensity(last_bias_proposal);
      likelihood *= bias_movement_prior.probDensity(bias_movement_proposal);
      likelihood *= noise_prior.probDensity(noise_proposal);
      
      accel_posterior.add( accel_proposal, likelihood );
      last_bias_posterior.add( last_bias_proposal, likelihood );
      bias_posterior.add( bias_proposal, likelihood );
}

void sample(float a_obs, int n){
  
    for(int i=0; i<n; i++){
      sampleonce(a_obs);
    }
}

ProbabilityDensityFunction compute_accel(float a_obs){
    accel_posterior=new Histogram(-5,5,0.02);
    last_bias_posterior=new Histogram(last_bias_prior.left(),last_bias_prior.right(),0.02);
    bias_posterior=new Histogram(-5,5,0.02);
    
    sample(state.a_obs, 10000);
    
    last_bias_prior = new HistogramDensityFunction(bias_posterior);
    
    return new HistogramDensityFunction( accel_posterior );
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
      
      state.a = compute_accel(a_obs);
        
      dt = state.t - laststate.t;
        
      if(laststate.a!=null){
        state.v = new DegenerateDensityFunction(laststate.v.argmax() + laststate.a.argmax()*dt);
        state.s = new DegenerateDensityFunction(laststate.s.argmax() + laststate.v.argmax()*dt);
          
        accel_prior=new DoubleExponentialDensityFunction( -state.v.argmax()*0.75, 4 );
      }
      
    } catch (IMUParseException e){
    }
    
    runonce=false;
    
  }
  
  // update bayes net
  if(state!=null){
    //updateADist(state);
  }
 
 
  //draw
  if(mode==MODE_PROB){
    background(255);
    
    //draw priors
    stroke(255,0,0);
    accel_prior.draw( -2.0, 2.0, width/2, 2*height/3, 75, 200.0);
    last_bias_prior.draw(-2.0,2.0,width/2, 1*height/3, 15, 200.0);
    
    //draw posteriors
    if(accel_posterior!=null&&last_bias_posterior!=null){
      fill(0);
      accel_posterior.draw(width/2,2*height/3,200,0.000002);
      last_bias_posterior.draw(width/2,height/3,200,0.000002);
      bias_posterior.draw(width/2,0,200,0.000002);
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
      text("v="+fround(state.v.argmax(),3)+" ms^-1", 5, height/3+20);
      text("s="+fround(state.s.argmax(),3)+" m", 5, 2*height/3+20);
      fill(0,0,255);
      text("argmax(a)="+fround(state.a.argmax(),2)+" ms^-2", 5, 20+20);
      state.draw(200.0);
    }
  }
  
  //delay(1000);
}

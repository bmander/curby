import processing.serial.*;
final String serialPort = "COM14"; // replace this with your serial port. On windows you will need something like "COM1".

float MPERSSQUARED_PER_BIT = (1/256.0)*9.807; //(g/LSB)*(m*s^-2/g)=m*s^-2/LSB

int MODE_AVS = 0;
int MODE_PROB = 1;

class State{
  float s;
  float v;
  float a;
  float t;
  
  State(float a, float t){
    s=0;
    v=0;
    a=a;
    t=t;
  }
  
  void draw(float zoom){
    strokeWeight(0.5);
    stroke(255,0,0);
    line(width/2,0,width/2,height);
    line(width/2-zoom,0,width/2-zoom,height);
    line(width/2+zoom,0,width/2+zoom,height);
    
    stroke(0);
    line(zoom*a+width/2,0,zoom*a+width/2,height/3);
    line(zoom*v+width/2,height/3,zoom*v+width/2,2*height/3);
    line(zoom*s+width/2,2*height/3,zoom*s+width/2,height);
  }
}

IMU imu;
PFont font;
boolean running;
boolean runonce;
boolean sampling;
boolean autosample=true;
int mode;

ProbabilityDensityFunction accel_prior;
ProbabilityDensityFunction last_bias_prior;
ProbabilityDensityFunction bias_movement_prior;
Histogram accel_posterior;
Histogram last_bias_posterior;
Histogram bias_posterior;
State state;

void keyPressed(){
  if(key==' '){ //reset
    state.s=0;
    state.v=0;
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
  } else if(key=='a'){ //autosample
    autosample=!autosample;
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
  
  accel_prior=new DoubleExponentialDensityFunction( 4 );
  last_bias_prior=new UniformDensityFunction(-0.5,0.5);
  bias_movement_prior = new GaussianDensityFunction(0,0.002);
}

void sampleonce(){
      //float accel_proposal = random(-1.5,1.5);
      //float bias_proposal = state.a-accel_proposal;
      float last_bias_proposal = random(-0.5,0.5);
      float bias_movement_proposal = random(-0.1, 0.1);
      float bias_proposal = last_bias_proposal+bias_movement_proposal;
      float accel_proposal = state.a-bias_proposal;
    
      //likelihood of sample
      float likelihood = 1.0;
      likelihood *= accel_prior.probDensity(accel_proposal);
      likelihood *= last_bias_prior.probDensity(last_bias_proposal);
      likelihood *= bias_movement_prior.probDensity(bias_movement_proposal);
      
      accel_posterior.add( accel_proposal, likelihood );
      last_bias_posterior.add( last_bias_proposal, likelihood );
      bias_posterior.add( bias_proposal, likelihood );
}

void sample(int n){
  
    for(int i=0; i<n; i++){
      sampleonce();
    }
}

void draw(){
  float dt=0;
 
  
  // until the serial stream runs dry
  while(running || runonce){
    
    try{
      // grab a reading from the IMU
      IMUReading reading = imu.read();
      
      if(reading==null){
        break;
      }
      
      // convert to SI units
      float a = reading.ax*MPERSSQUARED_PER_BIT;
      float t = reading.t/1000.0;
      
      // if this is the first time-slice, apply initial conditions
      if(state==null){
        state = new State(a,t);
      
      // else apply the transformation model
      } else {
        dt = t - state.t;
        
        state.v = state.v + state.a*dt;
        state.s = state.s + state.v*dt;
        state.a = a;
        state.t = t;
      }
      
    } catch (IMUParseException e){
    }
    
    runonce=false;
    
  }
  
  if(state!=null){
    if(last_bias_posterior!=null){
      last_bias_prior = new HistogramDensityFunction(bias_posterior);
    }
    
    accel_posterior=new Histogram(-5,5,0.02);
    last_bias_posterior=new Histogram(-5,5,0.02);
    bias_posterior=new Histogram(-5,5,0.02);
    sample(20000);
  }
  
  
  if(sampling && state!=null){
    sample(40000);
  }
  
  if(mode==MODE_PROB){
    background(255);
    
    stroke(255,0,0);
    accel_prior.draw( -2.0, 2.0, width/2, 2*height/3, 75, 100.0);
    last_bias_prior.draw(-2.0,2.0,width/2, 1*height/3, 75, 100.0);
    
    if(accel_posterior!=null&&last_bias_posterior!=null){
      fill(0);
      accel_posterior.draw(width/2,2*height/3,200,0.0001);
      last_bias_posterior.draw(width/2,height/3,200,0.0001);
      bias_posterior.draw(width/2,0,200,0.0001);
    }
    
    text("'a' prior distribution",5,20);
    text("'last bias' prior distribution",5,20+height/3);
    fill(0,0,0);
    text("'a' posterior distribution",5,20+20);
    text("'last bias' posterior distribution",5,20+height/3+20);
    fill(255,0,0);
  } else {
    if(state!=null){
      background(255);
      text("dt="+fround(dt,3)+" s", width-200,height-20 );
      text("a="+fround(state.a,3)+" ms^-2", 5, 20 );
      text("v="+fround(state.v,3)+" ms^-1", 5, height/3+20);
      text("s="+fround(state.s,3)+" m", 5, 2*height/3+20);
      fill(28);
      state.draw(200.0);
    }
  }
  
  delay(20);
}

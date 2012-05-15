import processing.serial.*;
final String serialPort = "COM14"; // replace this with your serial port. On windows you will need something like "COM1".

int NPANES=5;
int NPROBPANES=5;

float EARTH_GRAVITY=9.807;
float MPERSSQUARED_PER_BIT = (1/256.0)*EARTH_GRAVITY; //(g/LSB)*(m*s^-2/g)=m*s^-2/LSB
float LSB_PER_DEGREE_PER_SECOND = 14.375;

int MODE_AVS = 0;
int MODE_PROB = 1;

//free paramters
float TIMIDNESS=5.0; //tendancy to decelerate given a velocity
float WANDERLUST=4.0; //tau of exponential that describes the acceleration prior
float SPINNYNESS=0.1; //tau of exponential that describes the tendancy to spin
float BIAS_WANDER=0.00001; //stddev of movement of bias between successive time slices
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
    wbias_samples=new Histogram(-5,5,0.02);
    theta_samples=new Histogram(-90,90,0.1);
  }
}

Graph graph;
Sampleset sampleset;

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

void draw(){
  //float dt=0;
 
  //running=false;
  //runonce=true;
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
      
      graph.update(a_obs, w_obs, t, 3000);
      
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
      draw_histogram( sampleset.accel_samples, 4, 200, 0.02, "'a' sample histogram", 1.0 );
      draw_histogram( sampleset.bias_samples, 2, 200, 0.02, "'bias' sample histogram", 1.0 );
      draw_histogram( sampleset.w_samples, 1, 100, 0.02, "'w' sample histogram", 1.0 );
      draw_histogram( sampleset.wbias_samples, 0, 200, 0.02, "'wbias' sample histogram", 0.1 );
    }
    
  } else {
    if(graph.state!=null){
      background(255);
      fill(28);
      text("dt="+fround(graph.dt,3)+" s", width-200,height-20 );
      
      graph.state.draw(200.0);
    }
  }
  
  //delay(100);
}

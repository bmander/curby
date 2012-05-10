import processing.serial.*;
final String serialPort = "COM14"; // replace this with your serial port. On windows you will need something like "COM1".

float MPERSSQUARED_PER_BIT = (1/256.0)*9.807; //(g/LSB)*(m*s^-2/g)=m*s^-2/LSB

int MODE_AVS = 0;
int MODE_PROB = 1;

IMU imu;
PFont font;
boolean running;
boolean runonce;
int mode;

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

State state;

void keyPressed(){
  if(key==' '){
    state.s=0;
    state.v=0;
  } else if(key=='p'){
    if(running){
      running=false;
    } else {
      imu.clear();
      running=true;
    }
  } else if(key=='n'){
    runonce=true;
  } else if(key=='m'){
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
      //float dt=0;
      
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
      
      // slap it all on the screen
      /*if(mode==MODE_AVS){
        background(255);
        text("dt="+fround(dt,3)+" s", width-200,height-20 );
        text("a="+fround(state.a,3)+" ms^-2", 5, 20 );
        text("v="+fround(state.v,3)+" ms^-1", 5, height/3+20);
        text("s="+fround(state.s,3)+" m", 5, 2*height/3+20);
        fill(28);
        state.draw(200.0);
      }*/
      
    } catch (IMUParseException e){
    }
    
    runonce=false;
    
  }
  
  if(mode==MODE_PROB){
    background(255);
    text("prob mode", 100,100);
    fill(28);
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
  
}

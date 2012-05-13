float INFINITY = 10000000000000000.0;

abstract class ProbabilityDensityFunction{
  void draw(float left, float right, float shiftx, float shifty, float sc, float zoom){
    
    float incr = 1/zoom;
    
    for(float i=left; i<right; i += incr){
      line(shiftx+i*zoom,
        height-(shifty+sc*this.probDensity(i)),
        shiftx+(i+incr)*zoom,
        height-(shifty+sc*this.probDensity(i+incr))
      );
    }
  }
  
  abstract float probDensity(float x);
  
  abstract float sample();
  
  abstract float left();
  abstract float right();
  
  abstract float argmax();
  
  abstract float stddev();
}

class DegenerateDensityFunction extends ProbabilityDensityFunction{
  float c;
  
  DegenerateDensityFunction(float c){
    this.c=c;
  }
  
  float probDensity(float x){
    if(x==c){
      return INFINITY;
    } else {
      return 0;
    }
  }
  
  float sample(){
    return c;
  }
  
  float argmax(){
    return c;
  }
  
  float left(){
    return this.c;
  }
  
  float right(){
    return this.c;
  }
  
  float stddev(){
    return 0;
  }
  
  void draw(float left, float right, float shiftx, float shifty, float sc, float zoom){
    line(shiftx+this.c*zoom,height-(shifty),shiftx+this.c*zoom,height-(shifty+sc));
  }
 
}

class DoubleExponentialDensityFunction extends ProbabilityDensityFunction{
  float tau;
  float mean;
  DoubleExponentialDensityFunction(float mean, float tau){
    this.tau=tau;
    this.mean=mean;
  }
  
  float probDensity(float x){
    return 0.5*tau*exp(-tau*abs(x-mean));
  }
  
  float sample(){
    throw new UnsupportedOperationException();
  }
  
  float left(){
    return -log(tau/(2*0.001))/tau;
  }
  
  float right(){
    return log(tau/(2*0.001))/tau;
  }
  
  float argmax(){
    return mean;
  }
  
  float stddev(){
    return 2*1/tau;
  }
  
  
}

class UniformDensityFunction extends ProbabilityDensityFunction{
  float low;
  float high;
  
  UniformDensityFunction(float low, float high){
    this.low=low;
    this.high=high;
  }
  
  float probDensity(float x){
    if(x<low || x >high){
      return 0;
    } else {
      return (1/(high-low));
    }
  }
  
  float sample(){
    return random(low,high);
  }
  
  float left(){
    return this.low;
  }
  
  float right(){
    return this.high;
  }
  
  float argmax(){
    return (low+high)/2;
  }
  
  float stddev(){
    throw new UnsupportedOperationException();
  }
}

class HistogramDensityFunction extends ProbabilityDensityFunction{
  Histogram histogram;
  
  // if the entire prob distribution is zero, rightval=this.histogram.left and leftval=this.histogram.right
  float leftval; //leftmost point with non-zero prob density
  float rightval; //rightmost point with non-zero prob density
  
  float argmax_cache; //place to cache argmax; it never changes after this object is created
  
  HistogramDensityFunction(Histogram histogram){
    this.histogram=histogram;
    
    for(int i=0; i<this.histogram.buckets.length; i++){
      this.leftval = this.histogram.left+i*this.histogram.pitch;
      if(this.histogram.buckets[i]>0){
        break;
      }
    }
    for(int i=this.histogram.buckets.length-1; i>=0; i--){
      this.rightval = this.histogram.left+(i+1)*this.histogram.pitch;
      if(this.histogram.buckets[i]>0){
        break;
      }
    }
    
    this.argmax_cache = this.find_argmax();
  }
  
  float probDensity(float x){
    if(x<histogram.left || x>histogram.right){
      return 0;
    }
    
    //find uniform probibility density of a single bucket
    float bucketdensity = 1/histogram.pitch;
    
    //get probability of picking this bucket
    int bucket=0;
    float bucketprob=0;
    try{
      bucket = histogram.bucket(x);
      bucketprob = histogram.count(bucket)/histogram.mass;
    } catch (ArrayIndexOutOfBoundsException  ex){
      println( this.histogram.left+"-"+this.histogram.right );
      println( x );
      println( bucket );
      println( this.histogram.buckets.length );
      throw ex;
    }
    
    //probability density is the product of the two
    return bucketprob*bucketdensity;
  }
  
  float sample(){
    float x = random(0,this.histogram.mass);
    
    float ll=0;
    for(int i=0; i<this.histogram.buckets.length; i++){
      float bucketmass = this.histogram.buckets[i];
      if( x>= ll && x<=ll+bucketmass){
        //the sample is inside this bucket
        float cc = (x-ll)/bucketmass;
        return this.histogram.left+(i+cc)*this.histogram.pitch;
      }
      ll += bucketmass;
    }
    
    return -1000000; //something went terribly wrong
  }
  
  float left(){
    return this.leftval;
  }
  
  float right(){
    return this.rightval;
  }
  
  float find_argmax(){
    int winner=0;
    float winneramount=-1;
    for(int i=0; i<this.histogram.buckets.length; i++){
      if(this.histogram.buckets[i]>winneramount){
        winneramount=this.histogram.buckets[i];
        winner=i;
      }
    }
    return this.histogram.left+this.histogram.pitch*(winner+0.5);
  }
  
  float argmax(){
    return this.argmax_cache;
  }
  
  float expectedvalue(){
    float ret=0;
    for(int i=0; i<this.histogram.buckets.length; i++){
      float bucketmass = this.histogram.buckets[i];
      float bucketprob = bucketmass/this.histogram.mass;
      float bucketval = this.histogram.left+i*this.histogram.pitch;
      ret += bucketval*bucketprob;
    }
    return ret;
  }
  
  float variance(){
    float mu = expectedvalue();
    float ret=0;
    for(int i=0; i<this.histogram.buckets.length; i++){
      float bucketmass = this.histogram.buckets[i];
      float bucketprob = bucketmass/this.histogram.mass;
      float bucketval = this.histogram.left+i*this.histogram.pitch;
      ret += bucketprob*sq(bucketval-mu);
    }
    return ret;
  }
  
  float stddev(){
    return sqrt(variance());
  }
}

class GaussianDensityFunction extends ProbabilityDensityFunction{
  float mean;
  float stddev;
  
  float A;
  
  GaussianDensityFunction( float mean, float stddev ){
    this.mean=mean;
    this.stddev=stddev;
    
    //density coefficient
    this.A = 1/sqrt(2*PI*sq(stddev));
  }
  
  float probDensity(float x){
    float B = -sq(x-mean);
    float C = 2*sq(stddev);
    return A*exp(B/C);
  }
  
  float sample(){
    PVector vec = gaussian(this.stddev);
    return this.mean+vec.x;
  }
  
  float left(){
    return -5*this.stddev;
  }
  
  float right(){
    return 5*this.stddev;
  }
  
  float argmax(){
    return mean;
  }
  
  float stddev(){
    return stddev;
  }
}

PVector gaussian(float stddev) {
  float x1, x2, w, y1, y2;
 
  do {
    x1 = 2.0 * random(0,1) - 1.0;
    x2 = 2.0 * random(0,1) - 1.0;
    w = x1 * x1 + x2 * x2;
  } while ( w >= 1.0 );

  w = sqrt( (-2.0 * log( w ) ) / w );
  y1 = (x1 * w)*stddev;
  y2 = (x2 * w)*stddev;
         
  return new PVector(y1,y2);
}

class Histogram {
  float left;
  float right;
  float pitch;
  float[] buckets;
  float mass;
  
  Histogram(float left, float right, float pitch){
    this.left = left;
    this.right = right;
    this.pitch = pitch;
    this.buckets = new float[int((right-left)/pitch)+1];
    for(int i=0; i<buckets.length; i++){
      buckets[i]=0;
    }
    mass=0;
  }
  
  void add(float x){
    this.add(x,1);
  }
  
  void add(float x, float weight){

    int bucket = int((x-this.left)/this.pitch);
    //if(bucket<=0 || bucket>=this.buckets.length){
    //  return;
    //}
    
    add_to_bucket(bucket,weight);

  }
  
  void add_to_bucket( int i, float weight ){
    this.buckets[i] += weight;
    mass += weight;
  }
  
  float count(int bucket){

      return this.buckets[bucket];

  }
  
  int bucket(float val){
    return int((val-this.left)/pitch);
  }
  
  void draw(float shiftx, float shifty, float scalex, float scaley){
    strokeWeight(1);
    fill(0);
    noStroke();
    for(int i=0; i<buckets.length; i++){
      float x = left+i*pitch;
      
      float y=buckets[i]/pitch;
      
      
      //line(shift+x,height-0,shift+x,height-y);
      rect(shiftx+x*scalex,height-shifty,pitch*scalex,-y*scaley);
    }
  }
  
  Histogram smooth(int bucketradius){
    Histogram ret = new Histogram(this.left,this.right,this.pitch);
    for(int i=0; i<this.buckets.length; i++){
      int ll = max(0,i-bucketradius);
      int rr = min(this.buckets.length-1,i+bucketradius);
      float ss=0;
      for(int j=ll; j<=rr; j++){
        ss += this.buckets[j];
      }
      ret.add_to_bucket( i, ss/(rr-ll) );
    }
    return ret;
  }
}

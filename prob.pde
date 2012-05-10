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
}

class DoubleExponentialDensityFunction extends ProbabilityDensityFunction{
  float tau;
  DoubleExponentialDensityFunction(float tau){
    this.tau=tau;
  }
  
  float probDensity(float x){
    return 0.5*tau*exp(-tau*abs(x));
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
}

class HistogramDensityFunction extends ProbabilityDensityFunction{
  Histogram histogram;
  
  HistogramDensityFunction(Histogram histogram){
    this.histogram=histogram;
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
    this.buckets[bucket] += weight;
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
}

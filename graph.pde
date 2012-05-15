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
  
    ProbabilityDensityFunction last_bias_proposal_dist = new GaussianDensityFunction(laststate.bias.mean(), laststate.bias.stddev());
    ProbabilityDensityFunction bias_movement_proposal_dist = graph.bias_movement_prior;//new UniformDensityFunction(bias_movement_prior.left(),bias_movement_prior.right());
    ProbabilityDensityFunction noise_proposal_dist = graph.noise_prior;//new UniformDensityFunction(noise_prior.left(),noise_prior.right());
    
    ProbabilityDensityFunction last_wbias_proposal_dist = new GaussianDensityFunction(laststate.wbias.mean(), laststate.wbias.stddev());
    ProbabilityDensityFunction wnoise_proposal_dist = graph.wnoise_prior;
    
    ProbabilityDensityFunction last_w_proposal_dist = new GaussianDensityFunction(laststate.w.mean(), laststate.w.stddev());
    ProbabilityDensityFunction last_theta_proposal_dist= new GaussianDensityFunction(laststate.theta.mean(), laststate.theta.stddev());
  
    for(int i=0; i<n; i++){
      float last_bias_proposal = last_bias_proposal_dist.sample();
      float bias_movement_proposal = bias_movement_proposal_dist.sample();
      float noise_proposal = noise_proposal_dist.sample();
      float bias_proposal = last_bias_proposal+bias_movement_proposal;
      float accel_total_proposal = state.a_obs-(bias_proposal+noise_proposal);
      
      float wbias_proposal = last_wbias_proposal_dist.sample();
      float wnoise_proposal = wnoise_proposal_dist.sample();
      float w_proposal = state.w_obs-wbias_proposal-wnoise_proposal;
      
      float last_w_proposal = last_w_proposal_dist.sample();
      float last_theta_proposal = last_theta_proposal_dist.sample();
      float theta_proposal = last_theta_proposal + last_w_proposal*dt;
      
      float a_gravsensed_proposal = sin(radians(theta_proposal))*EARTH_GRAVITY;
      float a_linsensed_proposal = accel_total_proposal-a_gravsensed_proposal;
      float a_linear_proposal = a_linsensed_proposal/cos(radians(theta_proposal));
    
      //likelihood of sample
      float likelihood = 1.0;
      likelihood *= state.a.probDensity(a_linear_proposal);
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
      
      sampleset.accel_samples.add( a_linear_proposal, likelihood );
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
      state.theta=new UniformDensityFunction(-3,3);
      return;
    } else {
      state = new State(a_obs,w_obs,t,laststate.v.argmax());
    }
    
    //sample the portion of the graph connected to state.a
    Sampleset smp = sample(n);
    sampleset = smp; //export it to the global scope so we can draw it
    
    //update probability distriubtions using sample set
    state.bias = new HistogramDensityFunction( smp.bias_samples.smooth(1) );
    state.a = new HistogramDensityFunction( smp.accel_samples.smooth(1) );
    state.wbias = new HistogramDensityFunction( smp.wbias_samples.smooth(1) );
    state.w = new HistogramDensityFunction( smp.w_samples.smooth(1) );
    state.theta = new HistogramDensityFunction( smp.theta_samples.smooth(1) );
    
    //update current state velocity and position using analytical methods
    dt = graph.state.t - graph.laststate.t;
    state.v = advance_gaussian( laststate.v, laststate.a, dt );
    state.s = advance_gaussian( laststate.s, laststate.v, dt );
    
    //advance rotation
    //state.theta = advance_gaussian( laststate.theta, laststate.w, dt );
  }
  
}

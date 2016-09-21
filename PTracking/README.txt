This version of PTracking has been adapted to the B-Human framework. It is general purpose but
inside such a framework does not compile and we preferred to use it as an external library.

- sudo apt-get install gcc-4.7-multilib g++-4.7-multilib (to compile PTracking 32-bit)

PTracking 32-bit

- Uncomment add_definitions(-m32)
- Rename libptracking.a to libptracking-32.a

PTracking 64-bit

- Comment add_definitions(-m32)

-$ mkdir build/
-$ cmake ../src/
-$ make -j(#cores +1)



- copy include/ and lib/ folder in /spqrnao/Util/PTracking


- compile the code and hope that no errors show up
  * change const -> constexpr where is needed
  * goto include/Utils/Utils.h : change #include<boost/random.hhp> ->
    #include <gsl/gsl_rng.h> e #include <boost/random/normal_distribution.hpp>->
    #include <gsl/gsl_randist.h>

- change:
	inline static float sampleGaussianSigma(float sigma)
	{
		static boost::mt19937 rng;
		static boost::normal_distribution<> nd(0.0,sigma);
		static boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > 
		
		var_nor(rng,nd);
				
		return var_nor();
	}
	
	in
	
	inline static float sampleGaussianSigma(float sigma)
	{
		static gsl_rng* r = 0;
		
		if (r == 0)
		{
			gsl_rng_env_setup();
			r = gsl_rng_alloc (gsl_rng_default); 
		}
		
		return gsl_ran_gaussian(r,sigma);
	}


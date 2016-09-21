/**
* @file DiveHandler.h
*
*	This header file contains the declaration of a module working as a dive handler for the goalie.
*   Such handler is activated when the ball gets in the own field side, and it computes an estimate of its projection toward the goal
*   with respect to the goalie reference frame. It also provides estimates for the amount of time needed to dive, save the ball and
*   then get back to the goalie position. This measure is compared against the estimated time the ball needs to reach the goal.
*   With the use of reinforcement learning techniques (Policy Gradient, Genetic Algorithms) this module seeks the optimal diving strategy
*   by minimizing a cost function defined with the above mentioned parameters.
*   Output of this module is the representation DiveHandle, comprising a timer to trigger the dive action and the type of dive to be
*   performed (long dive, close dive, no dive at all).
*
* @author Claudio Delli Bovi, Francesco Riccio
*
*/

#pragma once

// Includes

#include <string>
#include <vector>
#include <list>
#include <map>
#include <set>
#include <bitset>
#include <time.h>

#include "Tools/Module/Module.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/spqr_representations/ConfigurationParameters.h"
#include "Representations/spqr_representations/RobotPoseSpqrFiltered.h"
#include "Representations/spqr_representations/GlobalBallEstimation.h"
#include "Representations/spqr_representations/DiveHandle.h"
#include <Core/Processors/Processor.h>
#include "Utils/AgentPacket.h"


// Module definition
MODULE(DiveHandler,
{,
    REQUIRES(FrameInfo),
	REQUIRES(GameInfo),
    REQUIRES(OpponentTeamInfo),
    REQUIRES(OwnTeamInfo),
	REQUIRES(FallDownState),
	REQUIRES(RobotInfo),
    REQUIRES(BallModel),
    REQUIRES(GlobalBallEstimation),
    REQUIRES(RobotPoseSpqrFiltered),
    PROVIDES(DiveHandle),
});

// Termination conditions
#define MAX_ITER 50
#define CONVERGENCE_THRESHOLD 0.01
// PG algorithm define, if commented the module performs a GA algorithm instead
//#define PG_LEARNER

// PG parameters
#define GAMMA 0.5
#define BUFFER_DIM 10
#define REWARDS_HISTORY_SIZE 10
#define EPSILON 0.1
#define T 15

// GA parameters
#define POPULATION_SIZE 100
#define INDIVIDUAL_SIZE 11

#define SELECTION  0.1
#define CROSSOVER 0.3
#define MUTATION 0.2
#define ELITE_SIZE 0.2

// Module class declaration
class DiveHandler : public DiveHandlerBase
{
    // Learning state
    ENUM( LearningState,
        // Learning disabled
        {,
        notLearning = 1,
        // Learning paused, expecting reward
        waitReward,
        // Learning active
        learning,
     });
     
     
    // Inner base class modeling the learning agent
    class CoeffsLearner
    {
        protected:
        // Set of coefficients representing the learning objective
        std::vector<float> coeffs;
        // Set of fixed parameters defining the cost funcion
        std::map<std::string, float> params;

        // Iteration counter
        int iter_count;

        // Pointer to the DiveHandler object whose coefficients are learned
        DiveHandler* diveHandler_ptr;
		
        public:
        // Default constructor
        CoeffsLearner(int _nCoeffs, float _initValue, DiveHandler* _dhPtr):
            coeffs(_nCoeffs, _initValue), iter_count(0), diveHandler_ptr(_dhPtr) { }

        virtual ~CoeffsLearner(){}

        // Setter/getter for the coefficients
        void setCoeffs(const std::vector<float>& _coeffs);
        inline std::vector<float> getCoeffs(){ return coeffs; }

        // Setter/getter for the parameters
        void setParam(const std::string& _key, float _value);
        inline float getParam(std::string _key){ return params[_key]; }

        // Update coefficients performing a step of the learning algorithm
        virtual bool updateCoeffs() = 0;

        // Use the obtained rewards to adjust the algorithm parameters
        virtual void updateParams(const std::list<float>& rewards) = 0;

    };

    // Inner class modeling a PolicyGradient-based learning agent
    class PGLearner : public CoeffsLearner
    {
        typedef std::list< std::vector<float> > PGbuffer;

        private:

        // Current estimate for the coefficients gradient
        std::vector<float> coeffsGradient;
        // Best individual performance achieved so far
        std::vector<float> coeffsBest;

        // Current reward score
        float reward_score;
        // Current reward normalization factor
        float reward_norm;

        // Memory buffer for the PG algorithm
        PGbuffer coeffsBuffer;
        // Set of perturbations to be performed
        PGbuffer perturbationsBuffer;

        // Check for convergence of the algorithm
        bool converged();

        // Recursive perturbation generator
        void generatePerturbations(std::vector<float>* partial_perturbation, unsigned int index);

        public:

        // Default constructor
        PGLearner(DiveHandler* _dhPtr, int _nCoeffs, float _epsilon = EPSILON,
                  int _T = T, float _initValue = 1.0, bool randomize = false);

        // Generate a set of perturbations for the current policy
        void generatePerturbations();

        // Evaluate a single policy perturbation with the cost function
        float evaluatePerturbation( std::vector<float> R );

        // Update the PG parameters according to the obtained rewards
        void updateParams(const std::list<float>& rewards);

        // Update coefficients performing a step of the learning algorithm
        virtual bool updateCoeffs();

        // Update the best coefficient setting so far
        inline void updateCoeffsBest()
        {
            coeffsBest = coeffs;
        }

    };

	
	class GALearner : public CoeffsLearner
	{
	private:
		// Current reward score
		float reward_score;
		// Current reward normalization factor
		float reward_norm;

		std::list<float> fitnessBuffer;

		class Individual
		{
		public:
			float fitness;
			std::bitset<INDIVIDUAL_SIZE> hypothesis;
			Individual( std::string id): fitness(.0f), hypothesis(id){}
			Individual( float f, std::string id): fitness(f), hypothesis(id){}
			Individual( unsigned int id): fitness(.0f), hypothesis(id){}
			inline bool operator<(const Individual& right) const { return (this->fitness) <= right.fitness; }
		};

		struct cmp
		{
			bool operator()(const Individual& left, const Individual& right) const
			{
				return left < right;
			}
		};
		std::set<Individual, GALearner::cmp> population;

		float evaluate(Individual i);
		Individual rnd_mutate(Individual i);
		Individual crossover(Individual mommy, const Individual& daddy);

		// Check for convergence of the algorithm
		bool converged();

	public:
		GALearner( DiveHandler* _dhPtr, int _nCoeffs, float _initValue );

		void evolutionStep();

		// Update the GA parameters according to the obtained rewards
		void updateParams(const std::list<float>& rewards);

		// Update coefficients performing a step of the learning algorithm
		virtual bool updateCoeffs();
	};

	
private:

    // Dive type currently selected
    DiveHandle::Dive diveType;

    // Current learning state
    LearningState state;  
    // Learning agent
    CoeffsLearner* learner;
    // Obtained rewards
    std::list<float> rewardHistory;

    // Current scores
    int opponentScore;
    int ownScore;

    // Estimated time the ball needs to reach the goal
    // a.k.a. Tpapo (historical reasons)
    float tBall2Goal;
    // Estimated time needed for the current dive action to be performed
    float tDive;
    // Estimated time the goalie needs to back up to its original position
    float tBackInPose;

	// Timer
	class Timer
	{
    public:
        PTracking::Timestamp startTime, fallenTime;
        bool setTimer;

        Timer(): setTimer(false)
        {
            startTime.setToNow();
            fallenTime.setToNow();
        }

        inline void set()
		{
//			if(!setTimer)
            {
                startTime.setToNow();
				setTimer = true;
			}
		}

		inline void reset()
		{
//			if(setTimer)
				setTimer = false;
		}
	};

	Timer timer;
	Timer goalTimer;

    float tBAGO;
	float tBAGOestimate;
	float dBAGOestimate;

    // Estimated intersection between the ball projection and the goal line
    float ballProjectionIntercept;
    // Estimated distance of the ball from the own goal
    float distanceBall2Goal;
	std::list<float> sampledVelocities;

    // Computes parameters using the ball estimated position and velocity
    void estimateDiveTimes();
    void estimateBallProjection();

    // Compute the overall time the goalie needs to dive and then recover its position
    inline float computeDiveAndRecoverTime(float alpha1, float alpha2);
	
public:

    // Default constructor
    DiveHandler();
    // Destructor
    ~DiveHandler();

    // Setter for the reward list
    inline const std::list<float>& getRewardList() const
    {
        return rewardHistory;
    }

    // Update the DiveHandle for the goalie behavior
    void update(DiveHandle& diveHandle);
	
};

/**
* @file DiveHandler.cpp
*
*	This source file contains the implementation of a module working as a dive handler for the goalie.
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

#include <stdlib.h>
#include <cmath>
#include <assert.h>

#include "Tools/Enum.h"
#include "DiveHandler.h"

// Uncomment to have debug information
//#define DIVEHANDLER_DEBUG
//#define DIVEHANDLER_TRAINING_DEBUG
//#define DIVEHANDLER_TRAINING
//#define RAND_PERMUTATIONS

#define NEGATIVE_REWARD -1.0
#define POSITIVE_REWARD 1.5

// Debug messages template
#define SPQR_ERR(x) std::cerr << "\033[22;31;1m" <<"[DiveHandler] " << x << "\033[0m"<< std::endl;
#define SPQR_INFO(x) std::cerr << "\033[22;34;1m" <<"[DiveHandler] " << x << "\033[0m" << std::endl;
#define SPQR_SUCCESS(x) std::cerr << "\033[0;32;1m" <<"[DiveHandler] " << x << "\033[0m" << std::endl;
#define SPQR_FAILURE(x) std::cerr << "\033[22;31;1m" <<"[DiveHandler] " << x << "\033[0m"<< std::endl;

#define LEARNING_STATE(x) \
    if(x == 1) std::cerr << "\033[22;34;1m"<<"Learner state: disabled. "<<"\033[0m" << std::endl; \
    else if(x == 2) std::cerr << "\033[22;34;1m"<<"Learner state: paused (waiting for reward). "<<"\033[0m" << std::endl; \
    else if(x == 3) std::cerr << "\033[22;34;1m"<<"Learner state: enabled. "<<"\033[0m" << std::endl; \

using PTracking::Timestamp;

bool stamp =false;
bool tooEarly=false;
bool fallen=false;
bool estimatedTime=false;
bool goalDetected=false;
bool superCase=false;

#ifdef DIVEHANDLER_TRAINING
int n_mutation = 0;
int n_crossover = 0;
#endif

MAKE_MODULE(DiveHandler, spqr_modules)

// Shortcut to compute the magnitude of a vector
float magnitude(std::vector<float> v)
{
    float m = 0.0;
    for (unsigned int i = 0; i < v.size(); ++i)
        m += v.at(i) * v.at(i);

    return sqrt(m);
}


/** --------------------- CoeffsLearner: base class --------------------- */


/*
 * Simple setters for the learner's parameters and coefficients.
 */
void DiveHandler::CoeffsLearner::setCoeffs(const std::vector<float>& _coeffs)
{
    coeffs = _coeffs;
}

void DiveHandler::CoeffsLearner::setParam(const std::string& _key, float _value)
{
    params[_key] = _value;
}



/** --------------------- CoeffsLearner: Policy Gradient --------------------- */

/*
 * Default constructor. Initializes the algorithm parameters and coefficients.
 * Input arguments are:
 * - The number of coefficients involved in the learning process (mandatory);
 * - The step size for the policy perturbation phase;
 * - The number of perturbations to be considered simultaneously;
 * - An initial value for the learning coefficients (or an upper bound for the random initialization of those);
 * - A flag indicating whether a fixed or random initialization has to be performed.
 */
DiveHandler::PGLearner::PGLearner( DiveHandler* _dhPtr, int _nCoeffs, float _epsilon, int _T, float _initValue, bool randomize ):
    // Initialize the base class
    CoeffsLearner(_nCoeffs, _initValue, _dhPtr),
    // Initialize the gradient estimate
    coeffsGradient(_nCoeffs, 0.0)
{
    // Initializing reward scores
    reward_score = 0.0;
    reward_norm = 1.0;
    coeffsBest = coeffs;

    // Initializing coefficients
    if(randomize)
    {
        // Random initialization in [0, INIT_VALUE]
        srand(time(NULL));
        for( int i=0; i<_nCoeffs; ++i)
            coeffs.at(i) = (static_cast<float>(rand()%101)/100 ) *_initValue;
    }

    // Initializing parameters
    setParam("epsilon", _epsilon);

#ifdef RAND_PERMUTATIONS
    setParam("T", _T);
#else
    setParam("T", pow(3,coeffs.size()));
#endif
}

/* TOTEST&COMMENT */
bool DiveHandler::PGLearner::converged()
{
    // Skip convergence check if the buffer is not full
    if (coeffsBuffer.size() < BUFFER_DIM)
        return false;
    // Average every coefficients variation across the buffer
    else
    {
        // Compute variations mean
        // Delta previous to current step
        float avg_variation = (magnitude(coeffs) - magnitude(coeffsBuffer.front()))/coeffsBuffer.size() ;
        // Iterate over the whole buffer and compute deltas from step i-1 to i
        PGbuffer::const_iterator i = coeffsBuffer.begin();
        PGbuffer::const_iterator j = coeffsBuffer.begin(); ++j;
        while (j != coeffsBuffer.end())
        {
            avg_variation += ( magnitude(*i) - magnitude(*j) )/coeffsBuffer.size();
            ++i; ++j;
        }

        // Compute variations standard deviation
        // Delta previous to current step
        float std_variation = pow(magnitude(coeffs)-magnitude(coeffsBuffer.front()) - avg_variation, 2) / coeffsBuffer.size();
        // Iterate over the whole buffer and compute deltas from step i-1 to i
        PGbuffer::const_iterator k = coeffsBuffer.begin();
        PGbuffer::const_iterator t = coeffsBuffer.begin(); ++t;
        while (t != coeffsBuffer.end())
        {
            std_variation += (pow(magnitude(*k)-magnitude(*t) - avg_variation, 2)) / coeffsBuffer.size();
            ++k; ++t;
        }
        std_variation = sqrt(std_variation);

        // Check result against variation threshold
        if ((avg_variation < CONVERGENCE_THRESHOLD) && (std_variation < CONVERGENCE_THRESHOLD))
        {
#ifdef DIVEHANDLER_TRAINING
            SPQR_SUCCESS("PGLearner converged!");
            SPQR_SUCCESS("Coefficients values:");
            for (unsigned int i = 0; i < coeffs.size(); ++i)
                SPQR_SUCCESS("\t" << coeffs.at(i));
#endif
            return true;
        }
        else
            return false;
    }
}

/* TOTEST&COMMENT */
void DiveHandler::PGLearner::generatePerturbations()
{
    // Clean up the buffer
    if(! perturbationsBuffer.empty())
        perturbationsBuffer.clear();

#ifdef RAND_PERMUTATIONS
    srand(time(NULL));

    for(int i=0; i<params["T"]; ++i)
    {
        std::vector<float> perturbation(coeffs);

        for(unsigned int j=0; j<coeffs.size(); ++j)
            perturbation.at(j) += (rand()%3 -1)*params["epsilon"];

        perturbationsBuffer.push_back(perturbation);

#ifdef DIVEHANDLER_DEBUG
        SPQR_INFO("Generated perturbation: [" << perturbation.at(0) << ", " << perturbation.at(1) << "]");
#endif

    }
#else
    // Initialize a placeholder for perturbations
    std::vector<float> perturbation (coeffs.size(),0.0);

    // Generate all possible combinations recursively
    generatePerturbations(&perturbation, 0);

#ifdef DIVEHANDLER_DEBUG
    PGbuffer::const_iterator printer = perturbationsBuffer.begin();
    while(printer != perturbationsBuffer.end())
    {
        SPQR_INFO("Generated perturbation: [" << (*printer).at(0) << ", " << (*printer).at(1) << "]");
        ++printer;
    }
#endif

#endif

}

/* TOTEST&COMMENT */
void DiveHandler::PGLearner::generatePerturbations(std::vector<float>* partial_perturbation, unsigned int index)
{
    if (index == partial_perturbation->size()-1)
    {
        // Base case: generate all combinations of the last coefficient for the current partial vector
        for (int perturbation_type = -1; perturbation_type <= 1; ++perturbation_type)
        {
            // Compute last index and generate the final perturbation
            std::vector<float> perturbation (*partial_perturbation);
            perturbation.at(index) = coeffs.at(index) + perturbation_type * params["epsilon"];

            // Update the perturbations buffer
            perturbationsBuffer.push_back(perturbation);
        }
    }
    else
    {
        for (int perturbation_type = -1; perturbation_type <= 1; ++perturbation_type)
        {
            // Generate current perturbation
            partial_perturbation->at(index) = coeffs.at(index) + perturbation_type * params["epsilon"];

            // Generate all possible perturbations for the current index
            generatePerturbations(partial_perturbation, index+1);
        }
    }
}

/* TOCOMMENT */
float DiveHandler::PGLearner::evaluatePerturbation( std::vector<float> R )
{
    // Dimensions check
    assert(R.size() == coeffs.size());

    return ( std::abs(diveHandler_ptr->tBAGO - ( R.at(0)*diveHandler_ptr->tBAGOestimate)) ) ;
}


/* TOTEST&COMMENT */
void DiveHandler::PGLearner::updateParams(const std::list<float>& rewards)
{
    // Re-initialize reward scores
    reward_score = 0.0;
    if (!rewards.empty()) reward_norm = 0.0;
    int discount_exp = 0;
    int positives = 0;

    std::list<float>::const_iterator i = rewards.begin();
    while (i != rewards.end())
    {
        // Counting positives
        if (*i == POSITIVE_REWARD)
            ++positives;

        // Computing discounted rewards
        reward_score += (*i) * pow(GAMMA, discount_exp);
        reward_norm += fabs((*i) * pow(GAMMA, discount_exp));
        ++i; ++discount_exp;        
    }

#ifdef DIVEHANDLER_TRAINING
    SPQR_INFO("Positive rewards: " << positives << " out of " << rewards.size());
    SPQR_INFO("Negative rewards: " << (rewards.size() - positives) << " out of " << rewards.size());
    SPQR_INFO("Reward total score: " << reward_score);
#endif

    //Adjusting PG parameters according to the obtained score
    setParam("epsilon", exp( -reward_score / REWARDS_HISTORY_SIZE ) * getParam("epsilon"));

    // Update best performance
    if (rewards.front() == POSITIVE_REWARD)
        coeffsBest = coeffs;

#ifdef DIVEHANDLER_TRAINING
    SPQR_INFO( "Epsilon value changed to: " << getParam("epsilon") << " according to the obtained rewards. ");
#endif

#ifdef RAND_PERMUTATIONS
    setParam("T", exp( -reward_score / REWARDS_HISTORY_SIZE ) * getParam("T"));
#endif
}


/* TOTEST&COMMENT */
bool DiveHandler::PGLearner::updateCoeffs()
{

#ifdef DIVEHANDLER_TRAINING
    SPQR_INFO( "\nPG algorithm, iteration " << iter_count << "... " );
#endif

    if( iter_count == MAX_ITER || converged() )
        return false;
    else
    {
        // First generate the set of random perturbation for the current coefficients
        generatePerturbations();

        // For each perturbation, evaluate with the objective function and store the result in a temporary container
        std::vector<float> evaluatedPerturbations (perturbationsBuffer.size());
        PGbuffer::const_iterator evaluator;
        for(evaluator = perturbationsBuffer.begin(); evaluator != perturbationsBuffer.end(); ++evaluator)
            evaluatedPerturbations.push_back( evaluatePerturbation(*evaluator) );

        // Compute the average 'gradient' for the current coefficients
        std::vector<float> coeffs_avgGradient(coeffs.size());

#ifdef RAND_PERMUTATIONS
        // For each coefficient, compute the average score to determine the correspondent 'gradient' entry
        PGbuffer::const_iterator current_perturbation = perturbationsBuffer.begin();
        for( unsigned int n = 0; n < coeffs.size(); ++n )
        {
            std::vector<float> score_plus, score_minus, score_zero;

            // Keep track of the perturbation type and store each score in a container
            for( unsigned int i = 0; i < evaluatedPerturbations.size(); ++i )
            {
                if ( ((*current_perturbation).at(n) - coeffs.at(n)) > 0 )
                    score_plus.push_back(evaluatedPerturbations.at(i));
                else if ( ((*current_perturbation).at(n) - coeffs.at(n)) < 0 )
                    score_minus.push_back(evaluatedPerturbations.at(i));
                else
                    score_zero.push_back(evaluatedPerturbations.at(i));

                ++current_perturbation;
            }

            // Sum up all positive perturbation scores
            float avg_plus = 0.0;
            for (unsigned int j = 0; j < score_plus.size(); ++j)
                avg_plus += score_plus.at(j) / score_plus.size();

            // Sum up all negative perturbation scores
            float avg_minus = 0.0;
            for (unsigned int j = 0; j < score_minus.size(); ++j)
                avg_minus += score_minus.at(j) / score_minus.size();

            // Sum up all null perturbation scores
            float avg_zero = 0.0;
            for (unsigned int j = 0; j < score_zero.size(); ++j)
                avg_zero += score_zero.at(j) / score_zero.size();

            if( avg_zero <= avg_plus && avg_zero<= avg_minus )
                coeffs_avgGradient.at(n) = 0.0;
            else
                coeffs_avgGradient.at(n) = avg_plus - avg_minus;
        }
#else
        // For each coefficient, compute different averages to determine the correspondent 'gradient' entry
        for( unsigned int n = 0; n < coeffs.size(); ++n )
        {
            int avg_selector = 0;
            float avg_minus = 0.0 , avg_zero = 0.0, avg_plus = 0.0;
            for( unsigned int i = 0; i < evaluatedPerturbations.size(); i = i + pow(3,n) )
            {
                for( unsigned int k = i; k < i + pow(3,n); ++k )
                {
                    float evaluation = evaluatedPerturbations.at(k) / (evaluatedPerturbations.size()/3);

                    if( (avg_selector)%3 == 0 ) avg_minus += evaluation;
                    if( (avg_selector)%3 == 1 ) avg_zero += evaluation;
                    if( (avg_selector)%3 == 2 ) avg_plus += evaluation;
                }
                ++avg_selector;
            }
            // evaluate An
            if( avg_zero <= avg_plus && avg_zero<= avg_minus )
                coeffs_avgGradient.at(coeffs.size() - (n +1)) = 0.0;
            else
                coeffs_avgGradient.at(coeffs.size() - (n +1)) = avg_plus - avg_minus;
        }
#endif
        // Avoid 'nan' when the gradient is zeroed
        float normalization = 1.0;
        if (magnitude(coeffs_avgGradient) != 0)
            normalization = magnitude(coeffs_avgGradient);


#ifdef DIVEHANDLER_TRAINING
        SPQR_INFO("Computed policy gradient: [ " << coeffs_avgGradient.at(0)/normalization
				  /*<< ", " << coeffs_avgGradient.at(1)/normalization */<< " ]");
#endif
        // Weight new gradient estimate and previous one according to the reward score
        std::vector<float> newGradient (coeffsGradient.size());
        for( unsigned int j=0; j<newGradient.size(); ++j )
            newGradient.at(j) = coeffs_avgGradient.at(j)/normalization;

#ifdef DIVEHANDLER_TRAINING
        SPQR_INFO("New policy gradient: [ " << newGradient.at(0)
				  << /*", " << newGradient.at(1) << */" ]");
#endif

        // Update coefficients history
        coeffsBuffer.push_front(coeffs);
        // Crop buffer
        if (coeffsBuffer.size() > BUFFER_DIM)
            coeffsBuffer.resize(BUFFER_DIM);

        // Update the coefficients following the gradient direction
        for( unsigned int i=0; i<coeffs_avgGradient.size(); ++i )
        {
            // Coefficients
            coeffs.at(i) += - newGradient.at(i) * getParam("epsilon");
            // Gradient estimate
            coeffsGradient.at(i) = newGradient.at(i);

            // Crop negative coefficients
            if (coeffs.at(i) < 0) coeffs.at(i) = 0.0;
        }

#ifdef DIVEHANDLER_TRAINING
		SPQR_INFO("New coefficients: [ " << coeffs.at(0) << /*", " << coeffs.at(1) <<*/ " ]");
#endif
        ++iter_count;

        return true;
    }
}


/** --------------------- CoeffsLearner: Genetic Algorithm --------------------- */
DiveHandler::GALearner::GALearner( DiveHandler* _dhPtr, int _nCoeffs, float _initValue ):
    CoeffsLearner(_nCoeffs, _initValue, _dhPtr),
    reward_score(.0f), reward_norm(.0f)
{
    setParam("selection", SELECTION);
    setParam("crossover", CROSSOVER);
    setParam("mutation", MUTATION);

    setParam("elite", ELITE_SIZE);

    srand(time(NULL));
    for(unsigned int i=0; i< POPULATION_SIZE; ++i)
		population.insert( Individual( (rand()%600) + 600) );

#ifdef DIVEHANDLER_DEBUG
    std::set<Individual, cmp>::iterator i = population.begin();
    for(; i != population.end(); ++i)
        SPQR_INFO("Individual, encoding: " << (*i).hypothesis.to_string() << ", value: " << (((float)(*i).hypothesis.to_ulong())/1000));

#endif

}

float DiveHandler::GALearner::evaluate(Individual i)
{
    return ( std::abs(diveHandler_ptr->tBAGO - ( (((float)i.hypothesis.to_ulong())/1000)*diveHandler_ptr->tBAGOestimate)) );
}

DiveHandler::GALearner::Individual DiveHandler::GALearner::rnd_mutate(Individual i)
{
#ifdef DIVEHANDLER_TRAINING_DEBUG
    SPQR_INFO("Individual " << (((float)i.hypothesis.to_ulong())/1000) << " mutates into: ");
#endif

#ifdef DIVEHANDLER_TRAINING
	++n_mutation;
#endif

//    srand(time(NULL));
    unsigned int n_flips = rand()%3+1;
    for(unsigned int j=0; j< n_flips; ++j )
		(i.hypothesis).flip(rand()%(INDIVIDUAL_SIZE-7) + 2);

#ifdef DIVEHANDLER_TRAINING_DEBUG
    SPQR_INFO(((float)i.hypothesis.to_ulong())/1000);
#endif

    return i;
}

DiveHandler::GALearner::Individual DiveHandler::GALearner::crossover(Individual mommy, const Individual& daddy)
{
#ifdef DIVEHANDLER_TRAINING_DEBUG
    SPQR_INFO("Couple " << ((float)mommy.hypothesis.to_ulong())/1000 << " and " << ((float)daddy.hypothesis.to_ulong())/1000);
#endif

#ifdef DIVEHANDLER_TRAINING
	++n_crossover;
#endif

//    srand(time(NULL));
	int crossover_point = rand() % (INDIVIDUAL_SIZE-7) +2;

#ifdef DIVEHANDLER_TRAINING_DEBUG
    SPQR_INFO("Crossover point: " << crossover_point);
#endif

    for(unsigned int i = crossover_point; i < INDIVIDUAL_SIZE; ++i)
        mommy.hypothesis[i] = daddy.hypothesis[i];

#ifdef DIVEHANDLER_TRAINING_DEBUG
    SPQR_INFO(((float)mommy.hypothesis.to_ulong())/1000);
#endif
    return mommy;
}

bool DiveHandler::GALearner::converged()
{
    // Skip convergence check if the buffer is not full
    if (fitnessBuffer.size() < BUFFER_DIM)
        return false;
    // Average every coefficients variation across the buffer
    else
    {
        // Compute variations mean
        float avg_variation = .0f;
        // Iterate over the whole buffer and compute deltas from step i-1 to i
        std::list<float>::const_iterator i = fitnessBuffer.begin();
        std::list<float>::const_iterator j = fitnessBuffer.begin(); ++j;
        while (j != fitnessBuffer.end())
        {
            avg_variation += ( (*i) - (*j) )/fitnessBuffer.size();
            ++i; ++j;
        }

        // Compute variations standard deviation
        float std_variation = .0f;
        // Iterate over the whole buffer and compute deltas from step i-1 to i
        std::list<float>::const_iterator k = fitnessBuffer.begin();
        std::list<float>::const_iterator t = fitnessBuffer.begin(); ++t;
        while (t != fitnessBuffer.end())
        {
            std_variation += ( pow((*k)-(*t) - avg_variation, 2) ) / fitnessBuffer.size();
            ++k; ++t;
        }
        std_variation = sqrt(std_variation);

        // Check result against variation threshold
        if ((avg_variation < CONVERGENCE_THRESHOLD) && (std_variation < CONVERGENCE_THRESHOLD))
        {
#ifdef DIVEHANDLER_TRAINING
            SPQR_SUCCESS("GALearner converged!");
            SPQR_SUCCESS("Coefficients values:");
            for (unsigned int i = 0; i < coeffs.size(); ++i)
                SPQR_SUCCESS("\t" << coeffs.at(i));
#endif
            return true;
        }
        else
            return false;
    }
}

void DiveHandler::GALearner::evolutionStep()
{
#ifdef DIVEHANDLER_DEBUG
    SPQR_INFO("Population before:");
    std::set<Individual, cmp>::iterator i = population.begin();
    for(; i != population.end(); ++i)
        SPQR_INFO("Individual, value: " << (((double)(*i).hypothesis.to_ulong())/1000) << ", fitness: " << ((*i).fitness));

#endif
    std::set<Individual, cmp> previousPopulation(population);
    population.clear();

    int sel = 0;
    std::set<Individual, cmp>::iterator selector = previousPopulation.begin();
    std::set<Individual, cmp>::iterator partner = previousPopulation.end();
    for(; selector != previousPopulation.end(); ++selector, ++sel)
    {
        if(sel < round(getParam("selection")*POPULATION_SIZE))
            population.insert(Individual(evaluate(*selector), (*selector).hypothesis.to_string()));
        else
        {
            if( ((double)rand())/RAND_MAX < getParam("mutation") )
            {
                Individual mutated (rnd_mutate( *selector ));
                population.insert( Individual(evaluate(mutated), (mutated).hypothesis.to_string()) );
            }
            else if( ((double)rand())/RAND_MAX < sqrt(getParam("crossover")) )
            {
                if(partner == previousPopulation.end())
                    partner = selector;
                else
                {
                    Individual first_child (crossover( *selector, *partner ));
                    Individual second_child (crossover( *partner, *selector ));
                    population.insert(Individual(evaluate(first_child), first_child.hypothesis.to_string()));
                    population.insert(Individual(evaluate(second_child), second_child.hypothesis.to_string()));

                    partner = previousPopulation.end();
                }
            }

            population.insert(Individual(evaluate( *selector ), ( *selector ).hypothesis.to_string()));
        }
    }

    std::set<Individual, cmp>::iterator resizer = population.begin();
    for(int resizer_count = 0; ((resizer_count != POPULATION_SIZE) && (resizer != population.end())); ++resizer, ++resizer_count)
    {}
    population.erase(resizer, population.end());


#ifdef DIVEHANDLER_TRAINING
        SPQR_INFO("Population size: " << population.size());
    SPQR_INFO("Number of mutations: " << n_mutation);
    SPQR_INFO("Number of crossover: " << n_crossover);
    n_mutation = 0; n_crossover = 0;
#endif

#ifdef DIVEHANDLER_TRAINING_DEBUG

    SPQR_INFO("New population:");
    std::set<Individual, cmp>::iterator i = population.begin();
    for(; i != population.end(); ++i)
        SPQR_INFO("Individual, value: " << (((double)(*i).hypothesis.to_ulong())/1000) << ", fitness: " << ((*i).fitness));

#endif

}

void DiveHandler::GALearner::updateParams(const std::list<float>& rewards)
{
    // Re-initialize reward scores
    reward_score = 0.0;
    if (!rewards.empty()) reward_norm = 0.0;
    int discount_exp = 0;
    int positives = 0;

    std::list<float>::const_iterator i = rewards.begin();
    while (i != rewards.end())
    {
        // Counting positives
        if (*i == POSITIVE_REWARD)
            ++positives;

        // Computing discounted rewards
        reward_score += (*i) * pow(GAMMA, discount_exp);
        reward_norm += fabs((*i) * pow(GAMMA, discount_exp));
        ++i; ++discount_exp;
    }

#ifdef DIVEHANDLER_TRAINING
    SPQR_INFO("Positive rewards: " << positives << " out of " << rewards.size());
    SPQR_INFO("Negative rewards: " << (rewards.size() - positives) << " out of " << rewards.size());
    SPQR_INFO("Reward total score: " << reward_score);
#endif

    //Adjusting GA parameters according to the obtained score
    if(exp( -reward_score / (2*REWARDS_HISTORY_SIZE) ) * getParam("mutation") >= 1.0)
        setParam("mutation", 1.0);
    else
        setParam("mutation", exp( -reward_score / REWARDS_HISTORY_SIZE ) * getParam("mutation"));

    if(exp( -reward_score / (2*REWARDS_HISTORY_SIZE) ) * getParam("crossover") >= 1.0)
        setParam("crossover", 1.0);
    else
		setParam("crossover", exp( -reward_score / (REWARDS_HISTORY_SIZE) ) * getParam("crossover"));

    if(exp( -reward_score / (2*REWARDS_HISTORY_SIZE) ) * getParam("elite") >= 1.0)
        setParam("elite", 1.0);
    else
        setParam("elite", exp( -reward_score / (2*REWARDS_HISTORY_SIZE) ) * getParam("elite"));

#ifdef DIVEHANDLER_TRAINING
    SPQR_INFO( "Mutation rate value changed to: " << getParam("mutation") << " according to the obtained rewards. ");
    SPQR_INFO( "Crossover rate value changed to: " << getParam("crossover") << " according to the obtained rewards. ");
    SPQR_INFO( "Elite percentage changed to: " << getParam("elite") << " according to the obtained rewards. ");
#endif

}

bool DiveHandler::GALearner::updateCoeffs()
{
#ifdef DIVEHANDLER_TRAINING
    SPQR_INFO( "\nGA algorithm, iteration " << iter_count << "... " );
#endif

    if( iter_count == MAX_ITER || converged() )
    {
        reward_score = 0.0;
        fitnessBuffer.clear();
        iter_count = 0;
        return false;
    }
    else
    {
        evolutionStep();

        double avg_fitness=.0f;
        double avg_coeff=.0f;
        std::set<Individual, cmp>::iterator evaluator = population.begin();
        for( unsigned int sel=0; sel<round(getParam("elite")*POPULATION_SIZE); ++evaluator, ++sel)
        {
            avg_fitness += evaluator->fitness / round(getParam("elite")*POPULATION_SIZE);
            avg_coeff += ((double)evaluator->hypothesis.to_ulong()) / (1000*round(getParam("elite")*POPULATION_SIZE));
        }

        fitnessBuffer.push_front(avg_fitness);

        // Crop buffer
        if (fitnessBuffer.size() > BUFFER_DIM)
            fitnessBuffer.resize(BUFFER_DIM);

        coeffs.at(0) = avg_coeff;

#ifdef DIVEHANDLER_TRAINING
        SPQR_INFO("New coefficients: [ " << coeffs.at(0) << " ]");
#endif
        ++iter_count;

        return true;
    }
}


/** --------------------------- Dive Handler ---------------------------- */


/*
 * Default class constructor: initializes all parameters and generates the learning agent.
 */
DiveHandler::DiveHandler():
    diveType(DiveHandle::no_dive), state(static_cast<DiveHandler::LearningState>(SPQR::GOALIE_LEARNING_STATE)),
#ifdef PG_LEARNER
	learner(new PGLearner(this, 1)),
#else
	learner(new GALearner(this, 1, 1.0)),
#endif
    opponentScore(0), tBall2Goal(-1), tDive(0.0), tBackInPose(0.0), tBAGO(0), tBAGOestimate(0),
    ballProjectionIntercept(SPQR::FIELD_DIMENSION_Y), distanceBall2Goal(SPQR::FIELD_DIMENSION_X)
{
    SPQR::ConfigurationParameters();
#ifdef DIVEHANDLER_TRAINING
    SPQR_INFO("Initializing GAlearner...");
    std::vector<float> coeffs = learner->getCoeffs();
    SPQR_INFO("Coefficient alpha = " << coeffs.at(0));
    //    SPQR_INFO("Parameters: epsilon = " << learner->getParam("epsilon") << ", T = " << learner->getParam("T"));
#endif
}

/*
 * Default class destructor: destroys the learning agent and deallocates memory.
 */
DiveHandler::~DiveHandler()
{
    if(learner) delete learner;
}

/*
 * Computation of the ball projection on the goal line from its estimated position and velocity.
 * Intersecting such projection with the goal line itself yields the shift along the goalie Y-axis
 * at which the ball is expected to reach the goal.
 * Then, the diveTime and the diveType parameters are defined accordingly.
 */
void DiveHandler::estimateBallProjection()
{
    // Ball path line
    float A1 = (theBallModel.estimate.position.y() - theBallModel.estimate.velocity.y()) - theBallModel.estimate.position.y();
    float B1 = theBallModel.estimate.position.x() - (theBallModel.estimate.position.x() - theBallModel.estimate.velocity.x());
    float C1 = A1*theBallModel.estimate.position.x() + B1*theBallModel.estimate.position.y();

    // Goal line
    float A2 = SPQR::GOALIE_FAR_LIMIT_Y - -SPQR::GOALIE_FAR_LIMIT_Y;

    // Cross product/determinant
    float det = - A2*B1;

    // Y-intercept initialized with the maximum value possible
    float yIntercept = SPQR::FIELD_DIMENSION_Y;

    // Non-singular case
    if( fabs(det) > SPQR::GOALIE_EPSILON_COLLINEAR )
    {
        // Computing Y-intercept
        yIntercept = (- A2*C1) / det;

        // Devising the type of dive to be performed

        if( yIntercept > ( SPQR::GOALIE_CLOSE_LIMIT_Y/2) && yIntercept < SPQR::GOALIE_FAR_LIMIT_Y )
            // Close intercept on the left
            diveType = DiveHandle::lcloseDive;
        else if( yIntercept > SPQR::GOALIE_FAR_LIMIT_Y )
            // Far intercept on the left
            diveType = DiveHandle::lDive;
        else if( yIntercept < (-SPQR::GOALIE_CLOSE_LIMIT_Y/2) && yIntercept > -SPQR::GOALIE_FAR_LIMIT_Y )
            // Close intercept on the right
            diveType = DiveHandle::rcloseDive;
        else if( yIntercept < -SPQR::GOALIE_FAR_LIMIT_Y )
            // Far intercept on the right
            diveType = DiveHandle::rDive;

        else if( fabs(yIntercept) < SPQR::GOALIE_CLOSE_LIMIT_Y/2)
            diveType = DiveHandle::stopBall;
        else
            // Any other case: no dive at all
            diveType = DiveHandle::no_dive;
    }

    // Using the appropriate estimate for the dive time
    if (diveType == DiveHandle::lDive || diveType == DiveHandle::rDive )
        tDive = SPQR::GOALIE_DIVE_TIME;
    else if (diveType == DiveHandle::lcloseDive || diveType == DiveHandle::rcloseDive )
        tDive = SPQR::GOALIE_CLOSE_DIVE_TIME;
    else if (diveType == DiveHandle::stopBall )
        tDive = SPQR::GOALIE_STOP_BALL_TIME;
    else
        tDive = 0.0;

    // Updating the class parameters with the obtained value
    ballProjectionIntercept = yIntercept;

    // Estimated distance from the ball
    distanceBall2Goal = theBallModel.estimate.position.norm();
}

/*
 * Estimation of the time needed for the ball to reach the goal line, from its its estimated position and velocity.
 * If the ball is either too slow or completely off target, such time is set to -1.0 by default.
 * The estimated time for the goalie to recover its position is defined accordingly.
 */
void DiveHandler::estimateDiveTimes()
{
    // Check whether the ball is actually moving toward the goal
    if ( (theBallModel.estimate.velocity.norm() != 0.0) &&
         (theBallModel.estimate.velocity.x() < 0.0) )
        // Use a constant velocity approximation to the estimate the time interval
        tBall2Goal = 1000.0 * ( distanceBall2Goal / theBallModel.estimate.velocity.norm() );
    else
        // Otherwise, set the parameter to a meaningless value
        tBall2Goal = -1.0;

    // Using the appropriate estimates for recover and reposition times
    float tRecover = 0.0;
    float tReposition = 0.0;
    if( diveType == DiveHandle::rcloseDive || diveType == DiveHandle::lcloseDive )
        // Close dive: no need to back up to the original position
        tRecover = SPQR::GOALIE_CLOSE_DIVE_RECOVER_TIME;
    else if( diveType == DiveHandle::rDive || diveType == DiveHandle::lDive )
    {
        // Long dive: the robot has to stand up and reposition
        tRecover = SPQR::GOALIE_DIVE_RECOVER_TIME;
        tReposition = SPQR::GOALIE_DIVE_REPOSITION_TIME;
    }
    else if( diveType == DiveHandle::stopBall )
    {
        // stop ball: the robot has to stand up and stop the ball
        tRecover = SPQR::GOALIE_STOP_BALL_RECOVER_TIME;
    }

    // Total time needed to recover the original position
    tBackInPose = tRecover + tReposition;
}

/* TOCOMMENT */
inline float DiveHandler::computeDiveAndRecoverTime(float alpha1, float alpha2)
{
    return alpha2*( alpha1*tBall2Goal - tDive );
}

/* TOTEST&COMMENT */
/*
 * The module update function.
 * At each time step, the goalie enables the DiveHandler if the ball is close enough.
 * Using the ball percept and the robot position, all DiveHandler parameters are estimated: then, if the module
 * is in the learning state, performs a single iteration of the algorithm and updates the policy coefficients.
 * Then, the learning process is suspended until a reward is obtained (in terms of ball saved or scored).
 * Such coefficients are used to provide the goalie with a DiveHandle, containing:
 * - A timer to trigger the dive action;
 * - The type of dive to be performed;
 * - The Y-intercept of the ball projection with the goal line.
 * If the module is not in the learning state, the coefficients current values are used without any further processing.
 */
void DiveHandler::update(DiveHandle& diveHandle)
{
	if ( time(NULL) % 30 == 0 )
        srand(time(NULL));

    // Check you're actually the goalie...
    if (theRobotInfo.number == 1)
    {
        // Compute the ball projection estimate
        estimateBallProjection();
        // Update the DiveHandle
        diveHandle.ballProjectionEstimate = ballProjectionIntercept;

#ifdef DIVEHANDLER_TRAINING
        if( (Timestamp() - timer.fallenTime).getMs() > 5000 && (Timestamp() - timer.fallenTime).getMs() < 5040 && timer.fallenTime != 0)
            SPQR_SUCCESS("TooEarly time window START...");

        if( (Timestamp() - timer.fallenTime).getMs() > 9961 && (Timestamp() - timer.fallenTime).getMs() < 9999 && timer.fallenTime != 0)
            SPQR_SUCCESS("TooEarly time window END.");
#endif

        if(opponentScore != (int)theOpponentTeamInfo.score && !goalDetected)
        {
            if( (Timestamp() - timer.fallenTime).getMs() > 5000 && (Timestamp() - timer.fallenTime).getMs() < 10000 &&
                    (unsigned int) timer.fallenTime != 0)
            {
#ifdef DIVEHANDLER_TRAINING
                SPQR_FAILURE("too FAST dude!");
#endif
                tBAGO += /*300*/0;
                fallen=false;
            }
            else
            {
                //				if(goalTimer.setTimer)
                {
#ifdef DIVEHANDLER_TRAINING
                    SPQR_FAILURE("too SLOW dude!");
#endif
                    tBAGO = (float) (Timestamp() - goalTimer.startTime).getMs();
					if(tBAGO > 4000000000) tBAGO=1000;
                }
            }
            estimatedTime=true;
            goalDetected=true;
        }

        if(theGameInfo.state == STATE_SET)
        {
            tBAGOestimate=0;
            dBAGOestimate=0;
            sampledVelocities.clear();
            goalTimer.reset();
        }

        // Check whether the ball is close enough
        if( (distanceBall2Goal < SPQR::FIELD_DIMENSION_X) && (fabs(ballProjectionIntercept) < SPQR::FIELD_DIMENSION_Y) )
        {
            // Estimate all temporal parameters
            estimateDiveTimes();

            if(state != notLearning)
            {
                // if not in playing state
                if(theGameInfo.state != STATE_PLAYING)
                    timer.reset();
                else
                {
                    //					if(goalTimer.setTimer)
                    //						SPQR_INFO("time: "<< goalTimer.getTimeSince(goalTimer.start));

                    // if the ball is moving enough fast then set the timer
                    if( (theBallModel.estimate.velocity.norm() > SPQR::GOALIE_MOVING_BALL_MIN_VELOCITY &&
                         theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < 1000) )
                    {
                        sampledVelocities.push_back( theBallModel.estimate.velocity.norm() );
                        if(!timer.setTimer)
                        {
                            timer.set();
                            goalTimer.set();
                            dBAGOestimate=distanceBall2Goal;

#ifdef DIVEHANDLER_TRAINING
                            std::cerr << "\033[33;1m" <<"[DiveHandler] " << "set Timer!" << "\033[0m" << std::endl;
                            std::cerr << "\033[33;1m" <<"[DiveHandler] " << "set goal Timer!" << "\033[0m" << std::endl;
#endif
                        }
                    }
                    // else reset it...
                    if( (theBallModel.estimate.velocity.norm() < SPQR::GOALIE_MOVING_BALL_MIN_VELOCITY ||
                         theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) > 10000) )
                    {
                        if(timer.setTimer)
                        {
                            timer.reset();
#ifdef DIVEHANDLER_TRAINING
                            std::cerr << "\033[33;1m" <<"[DiveHandler] " << "reset Timer!" << "\033[0m" << std::endl;
#endif
                        }
                        if(goalTimer.setTimer)
                        {
                            goalTimer.reset();
#ifdef DIVEHANDLER_TRAINING
                            std::cerr << "\033[33;1m" <<"[DiveHandler] " << "reset goal Timer!" << "\033[0m" << std::endl;
#endif
                            tBAGOestimate=0;
                            dBAGOestimate=0;
                            sampledVelocities.clear();
                        }
                    }

                    // if the goalie succeeded
                    if(ownScore != (int)theOwnTeamInfo.score && !estimatedTime)
                    {
#ifdef DIVEHANDLER_TRAINING
                        SPQR_SUCCESS("SUPER!");
#endif
                        tBAGO -= .0f;
                        fallen=false;
                        estimatedTime=true;
                        superCase=true;
                    }

                    // if the goalie dives
                    if( (int)theFallDownState.state == (int)FallDownState::onGround && !fallen && !superCase)
                    {
                        timer.fallenTime.setToNow();
                        tBAGO = (float)((Timestamp() - goalTimer.startTime).getMs());
                        SPQR_INFO("Falling right now: " << tBAGO-1000);
                        fallen=true;
                    }
                }
            }

            if(estimatedTime && !superCase)
            {
                float velocityMean=0;
                float velocityMax=0;
                std::list<float>::const_iterator it=sampledVelocities.begin();
                for(; it != sampledVelocities.end(); ++it)
                {
                    if((*it) > velocityMax) velocityMax=(*it);
                    velocityMean += (*it) /sampledVelocities.size();
                }

				if(velocityMax != .0f)
					tBAGOestimate = 1000*(dBAGOestimate / (.75f*velocityMax));
//                SPQR_INFO("distance: " << dBAGOestimate);
//                SPQR_INFO("velocity: " << (.75f*velocityMax)/1000);
            }


#ifdef DIVEHANDLER_DEBUG
            SPQR_INFO("Ball projection: " << ballProjectionIntercept);
            SPQR_INFO("PAPO time: " << tBall2Goal);
            SPQR_INFO("Dive time: " << tDive);
            SPQR_INFO("Back-in-position time: " << tBackInPose);
            LEARNING_STATE( (int)state );
#endif

            // The module is in the learning state and a reward has been received
            if( /*clock() % 240 &&*/ state == learning )
            {
                // Perform a single iteration of the learning algorithm
                if( !learner->updateCoeffs() )
                {
                    // Change the state in 'waiting for reward'
                    state = waitReward;
                    // Flag a pending reward to the goalie behavior
                    diveHandle.rewardAck = false;
                }
#ifdef PG_LEARNER
                else
                    // The algorithm has converged: turning off learning
                    state = notLearning;
#endif

            }
            // The module is in the learning state, waiting for the next reward
			else if( state == waitReward )
            {
                // The opponent team scores: the goalie failed and gets a negative reward
                if(goalDetected && estimatedTime)
                {
                    // The learner obtains a negative reward
                    rewardHistory.push_front(NEGATIVE_REWARD);

                    // Crop the buffer
                    if (rewardHistory.size() > REWARDS_HISTORY_SIZE)
                        rewardHistory.resize(REWARDS_HISTORY_SIZE);
                    // Update opponent score
                    opponentScore = (int)theOpponentTeamInfo.score;

#ifdef DIVEHANDLER_TRAINING
                    SPQR_FAILURE("The opponent team scored! Negative reward for the learner.");
#endif
                    // A reward has been received: re-enable learning
#ifdef PG_LEARNER
                    state = learning;
#endif
                    // Clear the pending reward
                    if(!diveHandle.rewardAck)
                        diveHandle.rewardAck = true;

                    goalDetected=false;
                    estimatedTime=false;
                    stamp =true;
                }
                // The own team scores: user-guided move to provide the goalie a positive reward
                else if(ownScore != (int)theOwnTeamInfo.score /*&& estimatedTime*/)
                {
                    // The learner obtains a positive reward
                    rewardHistory.push_front(POSITIVE_REWARD);

                    // Crop the buffer
                    if (rewardHistory.size() > REWARDS_HISTORY_SIZE)
                        rewardHistory.resize(REWARDS_HISTORY_SIZE);
                    // Update own score
                    ownScore = (int)theOwnTeamInfo.score;

#ifdef DIVEHANDLER_TRAINING
                    SPQR_SUCCESS("The goalie has succeeded! Positive reward for the learner.  ");
#endif
                    // A reward has been received: re-enable learning
#ifdef PG_LEARNER
                    state = learning;
#endif
                    // Clear the pending reward
                    if(!diveHandle.rewardAck)
                        diveHandle.rewardAck = true;

                    estimatedTime=false;
                    stamp=true;
                    superCase=false;
                }
            }

            // Use the reward to adjust the algorithm parameters
#ifdef PG_LEARNER
            if( state == learning )
#else
            if( state == waitReward && diveHandle.rewardAck )
#endif
 #ifdef PG_LEARNER
                learner->updateParams(rewardHistory);
#else
            {
                learner->updateParams(rewardHistory);
                state = learning;
            }
#endif


            // Compute the dive time using the current coefficients as T = alpha2 * (alpha1*T_PAPO - T_dive)
            float diveTime = ( (learner->getCoeffs()).at(0) * tBall2Goal );

#ifdef DIVEHANDLER_TRAINING
            if(stamp)
            {
                SPQR_INFO("BAGO: " << tBAGO );
                SPQR_INFO("BAGO estimate: " << tBAGOestimate );
                SPQR_ERR("BAGO error: "<< std::abs(tBAGO - tBAGOestimate) );
                stamp = false;
            }
#endif

#ifdef DIVEHANDLER_DEBUG
            SPQR_INFO( "Estimated overall time to dive and recover position: " <<
                       computeDiveAndRecoverTime( (learner->getCoeffs()).at(0), (learner->getCoeffs()).at(1) ) );
            SPQR_INFO("Suggested dive in " << diveTime << " ms. ");
#endif

            // Update the DiveHandle
            if (diveTime > 0.0)
                diveHandle.diveTime = .70*(diveTime);
//            else
//                diveHandle.diveTime = -1.0;
        }
        // If the ball is far away or completely off target, no dive has to performed
        else
        {
            diveHandle.diveTime = -1;
            diveHandle.diveType = diveType;
            timer.reset();
        }
    }
}

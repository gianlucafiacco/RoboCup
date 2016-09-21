/**
* @file LibCodeRelease.h
*/

#define SPQR_INFO(x) std::cerr<<"\033[22;34;1m" << x <<"  \033[0m"<<std::endl
#define STATE(x) std::cerr<<"\033[22;34;1mState := "<< x <<"  \033[0m"<<std::endl

class LibCodeRelease : public LibraryBase
{
public:
  /** Constructor for initializing all members*/
  LibCodeRelease();

  class KickParameters
  {
  public:
      std::string id;
      float distance;
      float angle;
      Vector2f relKickDisplacement;
      float score;

      KickParameters(std::string _id = " ", float _d = .0f, float _a = .0f, Vector2f _rkd = Vector2f(), float _s = .0f):
          id(_id), distance(_d), angle(_a){}
  };

  std::map<std::string, KickParameters*> kickMap;

  void preProcess() override;

  void postProcess() override;

  void saveData();

  std::string kickToPerform();

  bool between(float value, float min, float max);

  float norm(float x, float y);

  int timeSinceBallWasSeen();

  bool ballIsInGame();

  bool isBallInKickAwayRange();
  
  bool isBallInCoverRange();

  bool isValueBalanced(float currentValue, float target, float bound);

  bool isGoalieInStartingPosition();

  bool isGoalieInAngle();
  
  bool isGoalieInArea();
  
  bool isBallInArea();

  bool isDribbleBallPosition();

  bool dribbleAnOpponent();

  float angleToTarget(float x, float y);

  float computeKickAngle();

  float correctionAngle(float _kickAngle);

  bool sideWhenLastTimeBallWasSeen();

  bool isTheAreaCleanFromOpponents(float radius);

  Pose2f getReadyPose(bool kickoff, ContextCoordination::SpqrRole rRole);
  Pose2f getDefenderPlayingPosition();
  Pose2f getSupporterPlayingPosition();
  Pose2f getJollyPlayingPosition();

  Pose2f getGoalieCoverPosition();
  float getGoalieCoverAngleDisplacement();
  
  Pose2f glob2Rel(float x, float y, float theta = 0);
  Pose2f rel2Glob(float x, float y);

  float goalie_displacement;

  float angleToGoal;
  float angleToMyGoal;
  float penaltyAngle;
  float kickAngle;
  float correctionKickAngle;
  bool ballOutOnLeft;
  bool diveBool;

  PTracking::Timestamp stamp;
  std::fstream dump;
  std::fstream opponent_robot_poses_dump;
  std::fstream own_robot_poses_dump;
  
  bool playData();
  bool playLocation();

  bool isCloserToBall(); //CHECK !!
  
  //
  Vector2f getTeammatePosition();
  float getAngleToTeammate();
  //
  
private:

	template <typename Word>
	std::ostream& write_word( std::ostream& outs, Word value, unsigned size = sizeof( Word ) );
	
	 void generaWav(int prima[70],int seconda[70],string s);
};

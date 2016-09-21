#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Enum.h"

STREAMABLE(OpponentModel,
{
               Pose2f estimated_pose,
               (Vector2f)(Vector2f::Zero()) estimated_velocity,

               OpponentModel() = default;
           });

STREAMABLE(SpqrDWKcombiner,
{
               ENUM(Context,
               {,
                no_context = 1,
                playing,
                search_for_ball,
                throw_in,
               });

               SpqrDWKcombiner() = default;

               Vector2f estimated_ball_global,
               (Vector2f)(Vector2f::Zero()) estimated_ball_relative,
               (Vector2f)(Vector2f::Zero()) predicted_ball_global,
               (Vector2f)(Vector2f::Zero()) predicted_ball_relative,
               (float)(600000) timeSinceWasSeen,
               (std::vector<Pose2f>)(std::vector<Pose2f>(5,Pose2f())) robots_poses,
               (std::vector<OpponentModel>) opponents,

               (std::vector<Vector2f>)(std::vector<Vector2f>(3,Vector2f::Zero())) explored_clusters_centroids,
               (std::vector<Vector2f>)(std::vector<Vector2f>(3,Vector2f::Zero())) unexplored_clusters_centroids,
               (Context)(no_context) current_context,
               (Vector2f)(Vector2f::Zero()) vel_avg,
               (std::vector<int>) robots_out_of_coordination,
           });

STREAMABLE(SpqrDWKcombinerCompressed,
{
               public:
               SpqrDWKcombinerCompressed() = default;
               SpqrDWKcombinerCompressed(const SpqrDWKcombiner& spqrDWKcombiner);
               operator SpqrDWKcombiner() const,
               (std::vector<Vector2f>) explored_clusters_centroids,
           });

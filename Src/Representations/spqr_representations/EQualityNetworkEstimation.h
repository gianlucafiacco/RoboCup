#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(EQualityNetworkEstimation,
{
               public:

               //used only in a simulated way to reproduce a reliable delay on the network communication
               unsigned long delay_vs_channel[5];

               EQualityNetworkEstimation() = default,

               (int)(-1) clock,
               (int)(100) network_quality,
               (int)(100) second_quality,
               (int)(100) convex_quality,
               (std::vector<float>)(std::vector<float>(5,100)) quality,


           });

STREAMABLE(EQualityNetworkEstimationCompressed,
{
               public:
               EQualityNetworkEstimationCompressed() = default;
               EQualityNetworkEstimationCompressed(const EQualityNetworkEstimation& EQualityNetworkEstimation);
               operator EQualityNetworkEstimation() const,
           });

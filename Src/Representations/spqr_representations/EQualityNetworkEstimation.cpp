#include "EQualityNetworkEstimation.h"

EQualityNetworkEstimationCompressed::EQualityNetworkEstimationCompressed(const EQualityNetworkEstimation& EQualityNetworkEstimation)
{

}

EQualityNetworkEstimationCompressed::operator EQualityNetworkEstimation() const
{
    EQualityNetworkEstimation eQualityNetworkEstimation;
    return eQualityNetworkEstimation;
}

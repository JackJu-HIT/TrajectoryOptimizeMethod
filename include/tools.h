#pragma once 
namespace tools
{
    inline double penaltyBoundFromBelow(const double& var, const double& a,const double& epsilon)
    {
        if (var >= a+epsilon)
        {
            return 0.;
        }
        else
        {
            return (-var + (a+epsilon));
        }
    }


    struct pathInfo
    {
        float x;
        float y;
        float theta;
    };

    struct obstacleInfo
    {
        float x;
        float y;
        float theta;
    };
}
#pragma once

// PCL includes
//#include <pcl/memory.h>
//#include <pcl/pcl_base.h>
//#include <pcl/pcl_macros.h>
//#include <pcl/search/search.h>

#include <iostream>
#include <functional>

#include "utils/RPGALExport.h"
#include "FeatureEstimation.h"

#include "core/geometry/Keypoint.h"

namespace rpgal {
namespace feature {

    class FPFHParam
    {
    public:
        FPFHParam() {}

        FPFHParam(double pointResolution) :
            kpParam(pointResolution, nullptr),
            pointResolution(pointResolution),
            featureRadius(featureRadiusScale* pointResolution) { }

        geometry::keypoint::ISSKeypointParam kpParam;
        int randomSampleCount = 500;
        double pointResolution = 0;
        double featureRadiusScale = 6;
        double featureRadius = 0;
        bool featureOriginSurface = false;
        bool appendQuadricKeyPoint = false; // adding additional Quadric features for robust
        
    };

    //template <typename PointInT, typename PointOutT>
    class RPGAL_EXPORT_TEMPLATE FeatureEstimationFPFH : public FeatureEstimation<FPFHSignature33>
    {
    public:
        
        // calc features and return keypoints
        static geometry::PointCloud::Ptr ComputeFeatures(
            const geometry::PointCloud& pointCloud, 
            OUT std::vector<FPFHSignature33>& resultFeatures, 
            const FPFHParam* param = nullptr);

    public:
        /** \brief Empty constructor. */
        FeatureEstimationFPFH() : FeatureEstimation(FeatureType::FPFH)
        {}

        virtual bool Compute(std::vector<FPFHSignature33>& featurePoints, bool isUseCPU = true);
    
    };

} // end namespace feature
} // end namespace rpgal


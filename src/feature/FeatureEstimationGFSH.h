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



    class GFSHParam
    {
    public:
        GFSHParam() {}

        GFSHParam(double pointResolution) :
            pointResolution(pointResolution) { }

        // 体素降采样 尺寸，建议用该值，不用pointResolutionVoxelScale, 
        // 推荐 max(8*pointResolution,featureRadius*0.1)
        // 通常6~8倍单帧点距, 按照640分辨率L计算，搜索半径SearchRadius = L*0.1, 体素降采样VoxelSize=SearchRadiu*0.1 ~= 6.4,故取6~8左右的单帧点距
        // sampleVoxelSize 和 pointResolutionVoxelScale 都<=0时，不进行降采样
        double sampleVoxelSize = 0.0;           // 建议直接给出这个数值,若给不出则设置0，根据pointResolution*pointResolutionVoxelScale来计算        
        double pointResolution = 0.0;           // 点密度，
        double pointResolutionVoxelScale = 5;   // 体素降采样 基于点密度的缩放系数，基于640分辨率设计，500的有效范围，5倍的点距；不推荐用该值，体素尺寸=pointResolution*缩放系数
        double featureRadiusScale = 6;          // 搜索半径 基于[采样点密度]的缩放系数，基于640分辨率设计，6倍的采样密度；不推荐用该值，搜索尺寸=pointResolution*缩放系数
        double featureRadius = 0;               // 搜索半径，推荐 包围盒对角线*0.10
        int neighborThreshold = 5;              // 特征计算时的最小邻域点数
        int threadCount = 0;                    // 并行计算线程数，0自动调整
        float filterScore = 0.0f;               // 根据分数阈值过滤 特征不明显的点，默认0不过滤，在特征匹配的地方加
        bool isSingleViewing = true;            // 按照单视角数据来计算特征(单视角重建的数据邻域法线夹角不能超过90°)
        
    };

    //template <typename PointInT, typename PointOutT>
    class RPGAL_EXPORT_TEMPLATE FeatureEstimationGFSH : public FeatureEstimation<GFSHDescriptor>
    {
    public:
        
        // calc features and return keypoints
        static geometry::PointCloud::Ptr ComputeFeatures(
            const geometry::PointCloud& pointCloud, 
            OUT std::vector<GFSHDescriptor>& resultFeatures,
            const GFSHParam* param = nullptr);

    public:
        
        FeatureEstimationGFSH() : FeatureEstimation(FeatureType::GFSH), neighborThreshold_(1), isSingleViewing_(true)
        {}

        virtual bool Compute(std::vector<GFSHDescriptor>& featurePoints, bool isUseCPU = true) override;
    
        void SetNeighborThreshold(int threshold);
        void SetSingleViewing(bool flag);
    protected:
        // neighbor count must be greater than this threshold when calc feature ,otherwise feature set -1
        int neighborThreshold_ = 1;
        bool isSingleViewing_ = true;
    };

} // end namespace feature
} // end namespace rpgal


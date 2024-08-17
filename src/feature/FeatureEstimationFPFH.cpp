#include "FeatureEstimationFPFH.h"

#include "utils/log/TLog.h"
#include "core/geometry/KDTreeFlann.h"
#include "algorithm/DownSampleEstimation.h"

using namespace rpgal::geometry;
using namespace rpgal::math;

namespace rpgal {
namespace feature {

    static Eigen::Vector4f ComputePairFeatures(const Vector3f& p1,
        const Vector3f& n1,
        const Vector3f& p2,
        const Vector3f& n2) {
        Eigen::Vector4f result;

        Vector3f dp2p1 = p2 - p1;
        result(3) = dp2p1.Norm();
        if (result(3) == 0.0) {
            return Eigen::Vector4f::Zero();
        }
        Vector3f n1_copy = n1;
        Vector3f n2_copy = n2;
        float angle1 = n1_copy.Dot(dp2p1) / result(3);
        float angle2 = n2_copy.Dot(dp2p1) / result(3);
        if (acos(fabs(angle1)) > acos(fabs(angle2))) {
            n1_copy = n2;
            n2_copy = n1;
            dp2p1 *= -1.0;
            result(2) = -angle2;
        }
        else {
            result(2) = angle1;
        }
        Vector3f v = dp2p1.Cross(n1_copy);
        float v_norm = v.Norm();
        if (v_norm == 0.0) {
            return Eigen::Vector4f::Zero();
        }
        v /= v_norm;
        Vector3f w = n1_copy.Cross(v);
        result(1) = v.Dot(n2_copy);
        result(0) = atan2(w.Dot(n2_copy), n1_copy.Dot(n2_copy));

        return result;
    }

    bool ComputePointSPFHFeature(
        const Vector3f& point,
        const Vector3f& normal,
        const std::vector<math::Vector3f>& kdPoints,
        const std::vector<math::Vector3f>& kdNormals,
        const KDTreeVector3f& kdtree,
        const KDTreeSearchParam* searchParam,
        Eigen::VectorXd& features) {

        features.resize(33);
        features.setZero();

        std::vector<int> indices;
        std::vector<float> distance2;
        if (kdtree.Search(point, *searchParam, indices, distance2) > 1) {
            // only compute SPFH feature when a point has neighbors
            double hist_incr = 100.0 / (double)(indices.size() - 1);
            for (size_t k = 1; k < indices.size(); k++) {
                // skip the point itself, compute histogram
                Eigen::Vector4f pf = ComputePairFeatures(point, normal,
                    kdPoints[indices[k]], kdNormals[indices[k]]);
                
                int h_index = (int)(std::floor(11 * (pf(0) + math::PI) / (2.0 * math::PI)));
                if (h_index < 0) h_index = 0;
                if (h_index >= 11) h_index = 10;
                features(h_index) += hist_incr;
                h_index = (int)(floor(11 * (pf(1) + 1.0) * 0.5));
                if (h_index < 0) h_index = 0;
                if (h_index >= 11) h_index = 10;
                features(h_index + 11) += hist_incr;
                h_index = (int)(floor(11 * (pf(2) + 1.0) * 0.5));
                if (h_index < 0) h_index = 0;
                if (h_index >= 11) h_index = 10;
                features(h_index + 22) += hist_incr;
            }
            return true;
        }
        return false;
    }

    bool ComputeSPFHFeature(
        const std::vector<math::Vector3f>& inputPoint,
        const std::vector<math::Vector3f>& inputNormal,
        const std::vector<math::Vector3f>& kdPoints,
        const std::vector<math::Vector3f>& kdNormals,
        const KDTreeVector3f& kdtree,
        const KDTreeSearchParam* searchParam,
        Eigen::MatrixXd& features, bool omp = false) {

        features.resize(33, (int)inputPoint.size());
        features.setZero();

#pragma omp parallel for schedule(static) if (omp)
        for (int i = 0; i < (int)inputPoint.size(); i++) {
            const Vector3f& point = inputPoint.at(i);
            const Vector3f& normal = inputNormal.at(i);
            Eigen::VectorXd pointFeature;
            ComputePointSPFHFeature(point, normal, kdPoints, kdNormals, kdtree, searchParam, pointFeature);
            features.col(i) = pointFeature;
        }
        return true;
    }

    // surface 和 input可以不一样， surface为nullptr时 input做为surface数据
    bool ComputeFPFHFeature(
        const PointCloud& input,
        const PointCloud* surface,
        const KDTreeSearchParam* searchParam,
        Eigen::MatrixXd& features,
        bool fullFeature, bool omp = false) {
        
        features.resize(33, (int)input.GetPointCount());
        features.setZero();

        if (!input.HasNormal()) {
            RPGAL_ERROR("Failed because input point cloud has no normal.");
            return false;
        }
        
        bool spfhIsAll = true;
        if (surface) {
            spfhIsAll = false;
            if (!surface->HasNormal()) {
                RPGAL_ERROR("Failed because surface point cloud has no normal.");
                return false;
            }
        }
        else {
            surface = &input;
        }
        const std::vector<math::Vector3f>* points = input.GetCoordList();
        KDTreeVector3f knnSurface;// (surface->GetCoordList());
        
        
        knnSurface.SetData(surface->GetCoordList());
        Eigen::MatrixXd spfh;
        if (!ComputeSPFHFeature(*input.GetCoordList(), *input.GetNormalList(), 
            *surface->GetCoordList(), *surface->GetNormalList(),
            knnSurface, searchParam, spfh)) {
            RPGAL_ERROR("Internal error: SPFH feature is nullptr.");
            return false;
        }
#pragma omp parallel for schedule(static) if (omp)
        for (int i = 0; i < (int)points->size(); i++) {
            const Vector3f& point = points->at(i);
            std::vector<int> indices;
            std::vector<float> distance2;
            if (knnSurface.Search(point, *searchParam, indices, distance2) > 1) {

                double sum[3] = { 0.0, 0.0, 0.0 };
                for (size_t k = 1; k < indices.size(); k++) {
                    int neighborI = indices[k];
                    double dist = distance2[k];
                    // skip the point itself
                    if (dist == 0.0) continue;
                    if (spfhIsAll) {
                        for (int j = 0; j < 33; j++) {
                            double val = spfh(j, neighborI) / dist;
                            sum[j / 11] += val;
                            features(j, i) += val;
                        }
                    } else {
                        // surface
                        const Vector3f& pointNeighbor = surface->GetPointCoord(neighborI);
                        const Vector3f& normalNeighbor = surface->GetPointNormal(neighborI);
                        Eigen::VectorXd spfhNeighbor;
                        ComputePointSPFHFeature(pointNeighbor, normalNeighbor,
                            *(surface->GetCoordList()), *(surface->GetNormalList()), 
                            knnSurface, searchParam, spfhNeighbor);
                        
                        for (int j = 0; j < 33; j++) {
                            double val = spfhNeighbor(j) / dist;
                            sum[j / 11] += val;
                            features(j, i) += val;
                        }
                    }
                }
                for (int j = 0; j < 3; j++)
                    if (sum[j] != 0.0) sum[j] = 100.0 / sum[j];
                for (int j = 0; j < 33; j++) {
                    features(j, i) *= sum[j / 11];

                    if (fullFeature) {
                        // The commented line is the fpfh function in the paper.
                        // But according to PCL implementation, it is skipped.
                        // Our initial test shows that the full fpfh function in the
                        // paper seems to be better than PCL implementation. Further
                        // test required.
                        features(j, i) += spfh(j, i);
                    }
                }
            }
        }
        return true;
    }


    geometry::PointCloud::Ptr FeatureEstimationFPFH::ComputeFeatures(
        const geometry::PointCloud& pointCloud, 
        OUT std::vector<FPFHSignature33>& resultFeatures, 
        const FPFHParam* param)
    {
        utils::ScopeTime st("ComputeFeatures");

        FPFHParam fpfhParam;
        if (param) {
            fpfhParam = *param;
        }

        double pointResolution = fpfhParam.pointResolution;

        geometry::KDTreeVector3f* nnQuery = nullptr;
        KDTreeVector3f nnQueryLocal;// (&points, 10);
        if (fpfhParam.kpParam.nnQuery_ == nullptr) {
            utils::ScopeTime st("ComputeFeatures KDTree");
            nnQueryLocal.SetData(pointCloud.GetCoordList(), 10);
            nnQuery = &nnQueryLocal;
        }
        else {
            nnQuery = fpfhParam.kpParam.nnQuery_;
        }
        if (pointResolution < FLOAT_MIN) {
            utils::ScopeTime st("ComputeFeatures CalcResolution");
            pointResolution = Geometry3D::CalcResolution(*pointCloud.GetCoordList(), 4, 100, nnQuery);
        }
        
        PointCloud::Ptr keypoints;
        if (param && param->randomSampleCount == 0) {
            geometry::keypoint::ISSKeypointParam kpParam = param ? fpfhParam.kpParam : geometry::keypoint::ISSKeypointParam(pointResolution, nnQuery);
            keypoints = geometry::keypoint::ComputeISSKeypoints(pointCloud, &kpParam);

            if ((param && param->appendQuadricKeyPoint))
            { // 额外添加凸点
                std::vector<rpgal::feature::QuadricSignature7> featuresConvex;
                PointCloud::Ptr keypointsConvex = geometry::keypoint::ComputeConvexKeypoints(
                    pointCloud,
                    featuresConvex,
                    kpParam.salientRadius_,
                    kpParam.nonMaxRadius_,
                    kpParam.minNeighbors_);

                KDTreeVector3f knn;
                knn.SetData(keypoints->GetCoordList());
                KDTreeSearchParamKNN knnParam(1);
                int addConvex = 0;
                for (int pid = 0; pid < keypointsConvex->GetPointCount(); pid++) {
                    const Vector3f& point = keypointsConvex->GetPointCoord(pid);
                    std::vector<int> indices;
                    std::vector<float> distance2;
                    if (knn.Search(point, knnParam, indices, distance2) > 0) {
                        if (distance2[0] > 0.1) {
                            keypoints->InsertPoint(point, keypointsConvex->GetPointNormal(pid));
                            addConvex++;
                        }
                    }
                }
                RPGAL_WARN("ComputeFeature add convex:%d, total:%d\n", addConvex, keypointsConvex->GetPointCount());
            }

        } else {
            utils::ScopeTime st("ComputeFeatures Sample");
            std::vector<size_t> indices;
            int sampleCount = int(pointCloud.GetPointCount()*0.5);
            if (fpfhParam.randomSampleCount < sampleCount) {
                sampleCount = fpfhParam.randomSampleCount;
            }
            RPGAL_INFO("feature sample count:%d \n", sampleCount);
            keypoints = algo::DownSampleEstimation::RandomDownSample(pointCloud, sampleCount, indices);
        }
        
        
        {
            utils::ScopeTime st("ComputeFeatures Feature");
            float resolutionKP = 0; 
            {
                utils::ScopeTime st("ComputeFeatures CalcResolution KeyPoint");
                resolutionKP = Geometry3D::CalcResolution(*keypoints->GetCoordList(), 4, 100);
            }
            
            FeatureEstimationFPFH featureEstimation;
            featureEstimation.SetInputCloud(keypoints.get());
            if (fpfhParam.featureOriginSurface) {
                featureEstimation.SetSearchSurface(&pointCloud);
                featureEstimation.SetRadiusSearch(fpfhParam.featureRadius);
            }
            else {
                featureEstimation.SetRadiusSearch(resolutionKP * fpfhParam.featureRadiusScale);
            }
            featureEstimation.Compute(resultFeatures);

        }

        
        return keypoints;
    }

    bool FeatureEstimationFPFH::Compute(std::vector<FPFHSignature33> &featurePoints, bool isUseCPU /*=true*/)
    {
        if (input_ == nullptr) {
            RPGAL_ERROR("FeatureEstimationFPFH miss input");
            return false;
        }
        KDTreeSearchParam* searchParam = nullptr;
        if (searchRadius_ > 0 && k_ > 0) {
            searchParam = new KDTreeSearchParamHybrid(searchRadius_, k_);
        }
        else if (searchRadius_ > 0) {
            searchParam = new KDTreeSearchParamRadius(searchRadius_);
        }
        else if (k_ > 0) {
            searchParam = new KDTreeSearchParamKNN(k_);
        }
        if (searchParam == nullptr) {
            RPGAL_ERROR("FeatureEstimationFPFH miss SearchParam");
            return false;
        }
        featurePoints.clear();
        featurePoints.resize(input_->GetPointCount());
        Eigen::MatrixXd features;
        bool fullFeature = true; // open3d true, pcl false
        if (ComputeFPFHFeature(*input_, surface_, searchParam, features, fullFeature)) {
            size_t dim = features.rows();
            for (int pid = 0; pid < input_->GetPointCount(); pid++)
            {
                featurePoints[pid].point_ = input_->GetPointCoord(pid);
                featurePoints[pid].normal_ = input_->GetPointNormal(pid);
                //featurePoints[pid].data_.Resize(dim);
                for (size_t i = 0; i < dim; i++)
                {
                    featurePoints[pid].data_[i] = features(i, pid);
                }
            }
        }
        return false;
    }

} // end namespace feature
} // end namespace rpgal


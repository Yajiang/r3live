#pragma once

// PCL includes
//#include <pcl/memory.h>
//#include <pcl/pcl_base.h>
//#include <pcl/pcl_macros.h>
//#include <pcl/search/search.h>

#include <iostream>
#include <functional>

#include "utils/RPGALExport.h"
#include "core/geometry/PointCloud.h"

#include "FeatureType.h"


namespace rpgal {
namespace feature {
#if 1       
    template <typename FeatureOut>
    class FeatureEstimation
    {
    public:


        using Ptr = std::shared_ptr< FeatureEstimation >;
        using ConstPtr = std::shared_ptr< const FeatureEstimation >;

        //using PointCloud = rpgal::geometry::PointCloud;
        //using PointCloudPtr = typename PointCloud::Ptr;
        //using PointCloudConstPtr = typename PointCloud::ConstPtr;

    public:
        /** \brief Empty constructor. */
        FeatureEstimation(FeatureType type) :
            type_(type),
            input_(nullptr),
            surface_(nullptr),
            searchParameter_(0), searchRadius_(0), k_(0), threadCount_(0)
        {}

        /** \brief Empty destructor */
        virtual ~FeatureEstimation() {}

        /** \brief Provide a pointer to the input dataset
          * \param[in] cloud the const boost shared pointer to a PointCloud message
          */
        virtual void SetInputCloud(const rpgal::geometry::PointCloud* cloud) {
            input_ = cloud;
            surface_ = cloud;// default
        };

        /** \brief Get a pointer to the input point cloud dataset. */
        const rpgal::geometry::PointCloud* GetInputCloud() const 
        { return (input_); }

        /** \brief Provide a pointer to a dataset to add additional information
            * to estimate the features for every point in the input dataset.  This
            * is optional, if this is not set, it will only use the data in the
            * input cloud to estimate the features.  This is useful when you only
            * need to compute the features for a downsampled cloud.
            * \param[in] cloud a pointer to a PointCloud message
            */
        void SetSearchSurface(const rpgal::geometry::PointCloud* cloud)
        { surface_ = cloud; }

        /** \brief Get a pointer to the surface point cloud dataset. */
        const rpgal::geometry::PointCloud* GetSearchSurface() const
        { return (surface_); }

        /** \brief Get the internal search parameter. */
        double GetSearchParameter() const { return (searchParameter_); }

        /** \brief Set the number of k nearest neighbors to use for the feature estimation.
            * \param[in] k the number of k-nearest neighbors
            */
        void SetKSearch(int k) { k_ = k; }

        /** \brief get the number of k nearest neighbors used for the feature estimation. */
        int GetKSearch() const { return (k_); }

        void SetThreadCount(int count) { threadCount_ = count; }

        int GetThreadCount() const { return (threadCount_); }

        /** \brief Set the sphere radius that is to be used for determining the nearest neighbors used for the feature
            * estimation.
            * \param[in] radius the sphere radius used as the maximum distance to consider a point a neighbor
            */
        void SetRadiusSearch(double radius) { searchRadius_ = radius; }

        /** \brief Get the sphere radius used for determining the neighbors. */
        double GetRadiusSearch() const { return (searchRadius_); }

        /** \brief Base method for feature estimation for all points given in
            * <setInputCloud (), setIndices ()> using the surface in setSearchSurface ()
            * and the spatial locator in setSearchMethod ()
            * \param[out] output the resultant point cloud model dataset containing the estimated features
            */
        virtual bool Compute(std::vector<FeatureOut>& featurePoints, bool isUseCPU = true) = 0;

        /** \brief The search method template for points. */
        //virtual int search(const PointCloud& cloud, std::size_t index, double , pcl::Indices&, std::vector<float>& )

        virtual void Reset() {
            input_ = nullptr;
            surface_ = nullptr;
            searchParameter_ = 0;
            searchRadius_ = 0;
            k_ = 0;
            threadCount_ = 0;
        }
    protected:
        /** \brief The feature type. */
        FeatureType type_;

        /** \brief The input point cloud dataset. */
        const rpgal::geometry::PointCloud* input_;

        /** \brief An input point cloud describing the surface that is to be used
            * for nearest neighbors estimation.
            */
        const rpgal::geometry::PointCloud* surface_;

        /** \brief The actual search parameter (from either \a search_radius_ or \a k_). */
        double searchParameter_;

        /** \brief The nearest neighbors search radius for each point. */
        double searchRadius_;

        /** \brief The number of K nearest neighbors to use for each point. */
        int k_;

        /** \brief The number of thread use parallel computing. */
        int threadCount_;
    };
#endif

} // end namespace feature
} // end namespace rpgal


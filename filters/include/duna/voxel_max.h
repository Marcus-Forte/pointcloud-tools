

/*
 *  Copyright (c) 2023, Marcus Forte
 *  All rights reserved.
 *
 */

#pragma once

#include <pcl/filters/filter.h>
#include <unordered_map>

namespace duna
{
    /** \brief VoxelMax ...
     *
     * VoxelMax ...
     *
     * \author Marcus Forte
     * \ingroup filters
     */
    template <typename PointT>
    class VoxelMax : public pcl::Filter<PointT>
    {
        using PointCloud = typename pcl::Filter<PointT>::PointCloud;

        using pcl::Filter<PointT>::filter_name_;
        using pcl::Filter<PointT>::input_;
        using pcl::Filter<PointT>::indices_;
        using pcl::Filter<PointT>::removed_indices_;
        using pcl::Filter<PointT>::extract_removed_indices_;
        using pcl::Filter<PointT>::getClassName;

    public:
        using Ptr = pcl::shared_ptr<VoxelMax<PointT>>;
        using ConstPtr = pcl::shared_ptr<const VoxelMax<PointT>>;

        PCL_MAKE_ALIGNED_OPERATOR_NEW

        VoxelMax(bool extract_removed_indices = false) : 
        pcl::Filter<PointT>(extract_removed_indices),
        leaves_ (),
        leaf_size_ (Eigen::Vector4f::Zero ()),
        inverse_leaf_size_ (Eigen::Vector4f::Zero ()),
        min_b_ (Eigen::Vector4i::Zero ()),
        max_b_ (Eigen::Vector4i::Zero ()),
        div_b_ (Eigen::Vector4i::Zero ()),
        divb_mul_ (Eigen::Vector4i::Zero ()),
        search_radius_ (0),
        max_points_per_voxel_(1)
        {
            filter_name_ = "VoxelMax";
        }

        virtual ~VoxelMax() 
        {
            leaves_.clear();
        }

        inline void setMaximumPointsPerVoxel(int max_points_per_voxel)
        {
            max_points_per_voxel_ = max_points_per_voxel;
        }

        inline int getMaximumPointsPerVoxel() const
        {
            return max_points_per_voxel_;
        }

        virtual inline void
        setRadiusSearch(double radius)
        {
            leaf_size_[0] = leaf_size_[1] = leaf_size_[2] = static_cast<float>(radius);
            // Avoid division errors
            if (leaf_size_[3] == 0)
                leaf_size_[3] = 1;
            // Use multiplications instead of divisions
            inverse_leaf_size_ = Eigen::Array4f::Ones() / leaf_size_.array();
            search_radius_ = radius;
        }

    protected:
        /** \brief Simple structure to hold an nD centroid and the number of points in a leaf. */
        struct Leaf
        {
            Leaf() 
            {}
            pcl::Indices indices;
            
        };

        /** \brief The 3D grid leaves. */
        std::unordered_map<std::size_t, Leaf> leaves_;

        /** \brief The size of a leaf. */
        Eigen::Vector4f leaf_size_;

        /** \brief Internal leaf sizes stored as 1/leaf_size_ for efficiency reasons. */
        Eigen::Array4f inverse_leaf_size_;

        /** \brief The minimum and maximum bin coordinates, the number of divisions, and the division multiplier. */
        Eigen::Vector4i min_b_, max_b_, div_b_, divb_mul_;

        /** \brief The nearest neighbors search radius for each point. */
        double search_radius_;

        /** \brief The maximum allowed points per voxel. */
        int max_points_per_voxel_;

        /** \brief Downsample a Point Cloud using a voxelized grid approach
         * \param[out] output the resultant point cloud message
         */
        void
        applyFilter(PointCloud &output) override;
    };

} // namespace
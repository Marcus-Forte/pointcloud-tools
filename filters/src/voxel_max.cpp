#include "duna/voxel_max.h"

#include <pcl/common/common.h>

namespace duna
{
    template <typename PointT>
    void VoxelMax<PointT>::applyFilter(PointCloud &output)
    {
        if (!input_)
        {
            PCL_WARN("[pcl::%s::detectKeypoints] No input dataset given!\n", getClassName().c_str());
            output.width = output.height = 0;
            output.clear();
            return;
        }

        output.height = 1;      // downsampling breaks the organized structure
        output.is_dense = true; // we filter out invalid points

        Eigen::Vector4f min_p, max_p;
        // Get the minimum and maximum dimensions
        pcl::getMinMax3D<PointT>(*input_, min_p, max_p);

        // Compute the minimum and maximum bounding box values
        min_b_[0] = static_cast<int>(std::floor(min_p[0] * inverse_leaf_size_[0]));
        max_b_[0] = static_cast<int>(std::floor(max_p[0] * inverse_leaf_size_[0]));
        min_b_[1] = static_cast<int>(std::floor(min_p[1] * inverse_leaf_size_[1]));
        max_b_[1] = static_cast<int>(std::floor(max_p[1] * inverse_leaf_size_[1]));
        min_b_[2] = static_cast<int>(std::floor(min_p[2] * inverse_leaf_size_[2]));
        max_b_[2] = static_cast<int>(std::floor(max_p[2] * inverse_leaf_size_[2]));

        // Compute the number of divisions needed along all axis
        div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones();
        div_b_[3] = 0;

        // Clear the leaves
        leaves_.clear();

        // Set up the division multiplier (x + width*y + width*height*z)
        divb_mul_ = Eigen::Vector4i(1, div_b_[0], div_b_[0] * div_b_[1], 0);

        removed_indices_->clear();
        std::size_t output_point_count = 0;
        // First pass: build a set of leaves with the point index closest to the leaf center
        for (std::size_t cp = 0; cp < indices_->size(); ++cp)
        {
            if (!input_->is_dense)
            {
                // Check if the point is invalid
                if (!std::isfinite((*input_)[(*indices_)[cp]].x) ||
                    !std::isfinite((*input_)[(*indices_)[cp]].y) ||
                    !std::isfinite((*input_)[(*indices_)[cp]].z))
                {
                    if (extract_removed_indices_)
                        removed_indices_->push_back((*indices_)[cp]);
                    continue;
                }
            }

            Eigen::Vector4i ijk = Eigen::Vector4i::Zero();
            ijk[0] = static_cast<int>(std::floor((*input_)[(*indices_)[cp]].x * inverse_leaf_size_[0]));
            ijk[1] = static_cast<int>(std::floor((*input_)[(*indices_)[cp]].y * inverse_leaf_size_[1]));
            ijk[2] = static_cast<int>(std::floor((*input_)[(*indices_)[cp]].z * inverse_leaf_size_[2]));

            // Compute the leaf index
            int idx = (ijk - min_b_).dot(divb_mul_);
            Leaf &leaf = leaves_[idx];
            // First time we initialize the index
            if (leaf.indices.size() < max_points_per_voxel_)
            {
                leaf.indices.push_back((*indices_)[cp]);
                output_point_count++;
                continue;
            }
            else
            {
                if (extract_removed_indices_)
                    removed_indices_->push_back((*indices_)[cp]);
            }
        }

        // Second pass: go over all leaves and copy data
        output.resize(output_point_count);
        int cp = 0;

        for (const auto &leaf : leaves_)
        {
            for (const auto& index : leaf.second.indices)
            {
            output[cp++] = (*input_)[index];
            }
            // output[cp++] = (*input_)[leaf.second.idx];
        }
        output.width = output.size();
    }

    template class VoxelMax<pcl::PointXYZINormal>;
    template class VoxelMax<pcl::PointXYZ>;
    template class VoxelMax<pcl::PointXYZRGB>;
}
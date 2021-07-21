/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
#ifndef SEGMENTERS_INCLUDE_SEGMENTERS_GROUND_PROGRESSIVE_MORPHOLOGICAL_FILTER_HPP_
#define SEGMENTERS_INCLUDE_SEGMENTERS_GROUND_PROGRESSIVE_MORPHOLOGICAL_FILTER_HPP_

#include <pcl/segmentation/approximate_progressive_morphological_filter.h>  // pcl::ProgressiveMorphologicalFilter
#include <string>
#include <vector>

#include "common/types/type.h"
#include "segmenters/base_segmenter.hpp"

namespace autosense {
namespace segmenter {

class GroundProgressiveMorphologicalFilter : public BaseSegmenter {
 public:
    GroundProgressiveMorphologicalFilter();

    explicit GroundProgressiveMorphologicalFilter(const SegmenterParams &params);

    ~GroundProgressiveMorphologicalFilter();

    /// @brief Segment the point cloud.
    virtual void segment(
        const PointICloud &cloud_in,
        std::vector<PointICloudPtr> &cloud_clusters);  // NOLINT

    virtual std::string name() const { return "GroundProgressiveMorphologicalFilter"; }

 private:
    SegmenterParams params_;

    pcl::ApproximateProgressiveMorphologicalFilter<PointI> pmf_estimator_;
};  // class GroundProgressiveMorphologicalFilter

}  // namespace segmenter
}  // namespace autosense

#endif  // SEGMENTERS_INCLUDE_SEGMENTERS_GROUND_RPROGRESSIVE_MORPHOLOGICAL_FILTER_HPP_

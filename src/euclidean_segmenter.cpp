/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
#include "segmenters/euclidean_segmenter.hpp"

#include <pcl/filters/extract_indices.h>  // pcl::ExtractIndices
#include <ros/ros.h>

#include "common/time.hpp"  // common::Clock

namespace autosense {
namespace segmenter {

EuclideanSegmenter::EuclideanSegmenter() {}

EuclideanSegmenter::EuclideanSegmenter(const SegmenterParams& params)
    : params_(params), kd_tree_(new pcl::search::KdTree<PointI>) {
    euclidean_cluster_extractor_.setSearchMethod(kd_tree_);
    euclidean_cluster_extractor_.setMinClusterSize(params_.ec_min_cluster_size);
    euclidean_cluster_extractor_.setMaxClusterSize(params_.ec_max_cluster_size);
    euclidean_cluster_extractor_.setClusterTolerance(params_.ec_tolerance);
}

EuclideanSegmenter::~EuclideanSegmenter() {}

void EuclideanSegmenter::segment(const PointICloud &cloud_in,
                                 std::vector<PointICloudPtr> &cloud_clusters) {
    if (cloud_in.empty()) {
        ROS_WARN_STREAM("Empty non-ground for segmentation, do nonthing.");
        return;
    }

    // Segment
    std::vector<pcl::PointIndices> clusters_indices;
    EuclideanSegmenter::segment(cloud_in, clusters_indices);

    // Clear segments.
    cloud_clusters.clear();

    PointICloudPtr cloud(new PointICloud);
    *cloud = cloud_in;

    // extract clusters
    if (clusters_indices.size() > 0) {
        for (size_t cluster_idx = 0u; cluster_idx < clusters_indices.size();
             ++cluster_idx) {
            PointICloudPtr cluster_cloud(new PointICloud);
            pcl::copyPointCloud(*cloud, clusters_indices[cluster_idx],
                                *cluster_cloud);
            cloud_clusters.push_back(cluster_cloud);
        }
    }

}

void EuclideanSegmenter::segment(const PointICloud &cloud_in,
                                 std::vector<pcl::PointIndices> &clusters_indices) {
    if (cloud_in.empty()) {
        ROS_WARN_STREAM("Empty non-ground for segmentation, do nonthing.");
        return;
    }

    common::Clock clock;
    ROS_DEBUG("Starting Euclidean segmentation.");

    PointICloudPtr cloud(new PointICloud);
    *cloud = cloud_in;

    // extract clusters
    euclidean_cluster_extractor_.setInputCloud(cloud);
    euclidean_cluster_extractor_.extract(clusters_indices);

    ROS_DEBUG_STREAM("Segmentation complete. Took " << clock.takeRealTime()
                                                   << "ms.");
}

}  // namespace segmenter
}  // namespace autosense

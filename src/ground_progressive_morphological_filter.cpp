/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
#include "segmenters/ground_progressive_morphological_filter.hpp"
#include <pcl/filters/statistical_outlier_removal.h>

#include <ros/ros.h>

#include <pcl/ModelCoefficients.h>              // pcl::ModelCoefficients
#include <pcl/filters/extract_indices.h>        // pcl::ExtractIndices
#include <pcl/io/io.h>                          // pcl::copyPointCloud

#include "common/time.hpp"  // common::Clock

namespace autosense {
namespace segmenter {

GroundProgressiveMorphologicalFilter::GroundProgressiveMorphologicalFilter() {}

GroundProgressiveMorphologicalFilter::GroundProgressiveMorphologicalFilter(const SegmenterParams& params)
    : params_(params) {
    // Optional

    // Create the segmenting object
    pmf_estimator_.setCellSize(params_.pmf_cell_size);
    pmf_estimator_.setExponential(true);
    pmf_estimator_.setMaxWindowSize(params_.pmf_max_window_size);
    pmf_estimator_.setSlope (params_.pmf_slope);
    pmf_estimator_.setInitialDistance (params_.pmf_initial_distance);
    pmf_estimator_.setMaxDistance (params_.pmf_max_distance);
}

GroundProgressiveMorphologicalFilter::~GroundProgressiveMorphologicalFilter() {}

/**
 * @breif Ground Removal based-on ProgressiveMorphologicalFilter
 */

void GroundProgressiveMorphologicalFilter::segment(
    const PointICloud &cloud_in, std::vector<PointICloudPtr> &cloud_clusters) {
    if (cloud_in.empty()) {
        ROS_WARN_STREAM("Empty ground for segmentation, do nothing.");
        return;
    }

    // Segment Ground
    std::vector<pcl::PointIndices> clusters_indices;
    GroundProgressiveMorphologicalFilter::segment(cloud_in, clusters_indices);

    // Clear segments.
    cloud_clusters.clear();

    PointICloudPtr cloud(new PointICloud);
    *cloud = cloud_in;

    // Create Segmentation objects
    PointICloudPtr cloud_ground(new PointICloud);
    PointICloudPtr cloud_nonground(new PointICloud);
    
    pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices ());
    *ground_indices = clusters_indices[0];
    if (ground_indices->indices.size() > 0) {
        pcl::ExtractIndices<PointI> indiceExtractor;
        indiceExtractor.setInputCloud(cloud);
        indiceExtractor.setIndices(ground_indices);

        // extract ground points
        indiceExtractor.setNegative(false);
        indiceExtractor.filter(*cloud_ground);

        // extract non-ground points
        indiceExtractor.setNegative(true);
        indiceExtractor.filter(*cloud_nonground);
    }

    cloud_clusters.push_back(cloud_ground);
    cloud_clusters.push_back(cloud_nonground);
}

void GroundProgressiveMorphologicalFilter::segment(
    const PointICloud &cloud_in, std::vector<pcl::PointIndices> &clusters_indices) {
    if (cloud_in.empty()) {
        ROS_WARN_STREAM("Empty ground for segmentation, do nothing.");
        return;
    }

    common::Clock clock;
    ROS_DEBUG("Starting GroundProgressiveMorphologicalFilter.");

    PointICloudPtr cloud(new PointICloud);
    *cloud = cloud_in;

    // Create the filtering object
    pcl::PointCloud<PointI>::Ptr cloud_filtered (new pcl::PointCloud<PointI>);
    pcl::PointIndices::Ptr indices_filtered (new pcl::PointIndices);
    pcl::StatisticalOutlierRemoval<PointI> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(params_.pmf_meanK);
    sor.setStddevMulThresh(params_.pmf_std);
    sor.filter(indices_filtered->indices);

    // Create Segmentation objects
    PointICloudPtr cloud_ground(new PointICloud);
    PointICloudPtr cloud_nonground(new PointICloud);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices ground_indices;
    pmf_estimator_.setInputCloud(cloud);
    pmf_estimator_.setIndices(indices_filtered);
    pmf_estimator_.extract(ground_indices.indices);

    clusters_indices.push_back(ground_indices);

    ROS_DEBUG_STREAM("Segmentation complete. Took " << clock.takeRealTime()
                                                   << "ms.");
}

}  // namespace segmenter
}  // namespace autosense

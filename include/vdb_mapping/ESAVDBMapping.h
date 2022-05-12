// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2021 FZI Forschungszentrum Informatik
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Marvin Große Besselmann grosse@fzi.de
 * \author  Lennart Puck puck@fzi.de
 * \date    2021-04-29
 *
 */
//----------------------------------------------------------------------
#ifndef VDB_MAPPING_ESA_VDB_MAPPING_H_INCLUDED
#define VDB_MAPPING_ESA_VDB_MAPPING_H_INCLUDED

#define PCL_NO_PRECOMPILE
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


#include "vdb_mapping/DataNode.h"
#include "vdb_mapping/VDBMapping.h"

// https://pointclouds.org/documentation/tutorials/adding_custom_ptype.html
struct ESADataPoint
{
  float x;
  float y;
  float z;
  int stone_type;
  float light_gradient;
  PCL_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;                // enforce SSE padding for correct memory alignment

// needs to be outside of namespace
// https://github.com/PointCloudLibrary/pcl/issues/1152
POINT_CLOUD_REGISTER_POINT_STRUCT(
  ESADataPoint,
  (float, x, x)(float, y, y)(float, z, z)(int, stone_type, stone_type)(float,
                                                                       light_gradient,
                                                                       light_gradient))
namespace vdb_mapping {

struct ESADataNode
{
  float occupancy;
  int stone_type;       // dummy
  float light_gradient; // dummy
};

/*!
 * \brief Accumulation of configuration parameters
 */
struct Config : BaseConfig
{
  double prob_hit;
  double prob_miss;
  double prob_thres_min;
  double prob_thres_max;
};

class ESAVDBMapping : public VDBMapping<DataNode<ESADataNode>, Config>
{
public:
  using DataCloudT = pcl::PointCloud<ESADataPoint>;

  ESAVDBMapping(const double resolution)
    : VDBMapping<DataNode<ESADataNode>, Config>(resolution)
  {
  }

  /*!
   * \brief Handles changing the mapping config
   *
   * \param config Configuration structure
   */
  void setConfig(const Config& config) override;

  /*!
   * \brief Handles the integration of new data into the VDB data structure.
   *
   * \param cloud Input cloud in map coordinates
   * \param origin Sensor position in map coordinates
   *
   * \returns Was the insertion of the new cloud successful
   */
  bool insertDataCloud(const DataCloudT::ConstPtr& cloud,
                       const Eigen::Matrix<double, 3, 1>& origin);

protected:
  bool updateFreeNode(DataNode<ESADataNode>& voxel_value, bool& active) override;
  bool updateOccupiedNode(DataNode<ESADataNode>& voxel_value, bool& active) override;

  /*!
   * \brief Probability update value for passing an obstacle
   */
  double m_logodds_hit;
  /*!
   * \brief Probability update value for passing free space
   */
  double m_logodds_miss;
  /*!
   * \brief Upper occupancy probability threshold
   */
  double m_logodds_thres_min;
  /*!
   * \brief Lower occupancy probability threshold
   */
  double m_logodds_thres_max;
};

} // namespace vdb_mapping

#endif /* VDB_MAPPING_ESA_VDB_MAPPING_H_INCLUDED */

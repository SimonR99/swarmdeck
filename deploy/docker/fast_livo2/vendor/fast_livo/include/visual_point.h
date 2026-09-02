/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIVO_POINT_H_
#define LIVO_POINT_H_

#include <boost/noncopyable.hpp>
#include "common_lib.h"
#include "frame.h"

class Feature;

/// A visual map point on the surface of the scene.
class VisualPoint : boost::noncopyable
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Vector3d pos_;                //!< 3d pos of the point in the world coordinate frame.
  Vector3d normal_;             //!< Surface normal at point.
  Matrix3d normal_information_; //!< Inverse covariance matrix of normal estimation.
  Vector3d previous_normal_;    //!< Last updated normal vector.
  list<Feature *> obs_;         //!< Reference patches which observe the point.
  Eigen::Matrix3d covariance_;  //!< Covariance of the point.
  bool is_converged_;           //!< True if the point is converged.
  bool is_normal_initialized_;  //!< True if the normal is initialized.
  bool has_ref_patch_;          //!< True if the point has a reference patch.
  Feature *ref_patch;           //!< Reference patch of the point.
  int last_seen_id_;            //!< Frame id that last MATCHED this point.
                                //!< obs_.front()->id_ only advances when a new
                                //!< Feature is added, which needs parallax, so a
                                //!< stationary platform never refreshes it.

  VisualPoint(const Vector3d &pos);
  ~VisualPoint();
  void findMinScoreFeature(const Vector3d &framepos, Feature *&ftr) const;
  void deleteNonRefPatchFeatures();
  void deleteFeatureRef(Feature *ftr);
  void addFrameRef(Feature *ftr);
  bool getCloseViewObs(const Vector3d &pos, Feature *&obs, const Vector2d &cur_px) const;
};

#endif // LIVO_POINT_H_

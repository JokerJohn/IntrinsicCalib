// Kalibr AprilGrid corner extraction, a line-for-line port of
// GridCalibrationTargetAprilgrid::computeObservation (kalibr master,
// commit 1f60227, aslam_cv/aslam_cameras_april) with the Kalibr default
// options, exposed through a C interface for ctypes.  The tag detector is
// Kalibr's own ethz_apriltag2 (third_party/), unmodified apart from the
// OpenCV 4 include paths.  Debug-video drawing is omitted.
#include <algorithm>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "apriltags/TagDetector.h"
#include "apriltags/Tag36h11.h"

extern "C" {

// Return codes: 1 success, 0 not enough tags, -1 bad input,
// -2 duplicate tag ids (Kalibr aborts here: a tag not belonging to the board).
int kalibr_aprilgrid_detect(const std::uint8_t* gray, int width, int height,
                            int tag_rows, int tag_cols,
                            double max_subpix_displacement2,  // 1.5
                            int min_tags_for_valid_obs,       // 4
                            double min_border_distance,       // 4.0
                            int black_tag_border,             // 2
                            int do_subpix_refinement,         // 1
                            double* out_points,               // size()*2
                            std::uint8_t* out_observed,       // size()
                            int* out_num_tags) {
  if (gray == nullptr || width <= 0 || height <= 0 || tag_rows <= 0 ||
      tag_cols <= 0)
    return -1;
  const int rows = 2 * tag_rows;  // grid corners, as in Kalibr's ctor
  const int cols = 2 * tag_cols;
  const int size = rows * cols;

  cv::Mat image(height, width, CV_8UC1, const_cast<std::uint8_t*>(gray));
  AprilTags::TagDetector detector(AprilTags::tagCodes36h11,
                                  static_cast<size_t>(black_tag_border));
  std::vector<AprilTags::TagDetection> detections = detector.extractTags(image);

  // min. distance [px] of tag corners from image border, bad tags, ids
  // out of range for this grid
  for (auto it = detections.begin(); it != detections.end();) {
    bool remove = false;
    for (int j = 0; j < 4; j++) {
      remove |= it->p[j].first < min_border_distance;
      remove |= it->p[j].first > (float)(image.cols) - min_border_distance;
      remove |= it->p[j].second < min_border_distance;
      remove |= it->p[j].second > (float)(image.rows) - min_border_distance;
    }
    if (it->good != 1) remove = true;
    if (it->id >= size / 4) remove = true;
    if (remove)
      it = detections.erase(it);
    else
      ++it;
  }

  for (int i = 0; i < size; i++) {
    out_observed[i] = 0;
    out_points[2 * i] = 0.0;
    out_points[2 * i + 1] = 0.0;
  }
  *out_num_tags = static_cast<int>(detections.size());
  if ((int)detections.size() < min_tags_for_valid_obs) return 0;

  std::sort(detections.begin(), detections.end(),
            AprilTags::TagDetection::sortByIdCompare);
  for (size_t i = 0; i + 1 < detections.size(); i++)
    if (detections[i].id == detections[i + 1].id) return -2;

  cv::Mat tagCorners(4 * detections.size(), 2, CV_32F);
  for (size_t i = 0; i < detections.size(); i++)
    for (int j = 0; j < 4; j++) {
      tagCorners.at<float>(4 * i + j, 0) = detections[i].p[j].first;
      tagCorners.at<float>(4 * i + j, 1) = detections[i].p[j].second;
    }
  cv::Mat tagCornersRaw = tagCorners.clone();

  if (do_subpix_refinement)
    cv::cornerSubPix(
        image, tagCorners, cv::Size(2, 2), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::Type::EPS +
                             cv::TermCriteria::Type::MAX_ITER,
                         30, 0.1));

  for (size_t i = 0; i < detections.size(); i++) {
    const unsigned int tagId = detections[i].id;
    const unsigned int baseId = (int)(tagId / (cols / 2)) * cols * 2 +
                                (tagId % (cols / 2)) * 2;
    const unsigned int pIdx[] = {baseId, baseId + 1,
                                 baseId + (unsigned int)cols + 1,
                                 baseId + (unsigned int)cols};
    for (int j = 0; j < 4; j++) {
      const double cx = tagCorners.at<float>(4 * i + j, 0);
      const double cy = tagCorners.at<float>(4 * i + j, 1);
      const double rx = tagCornersRaw.at<float>(4 * i + j, 0);
      const double ry = tagCornersRaw.at<float>(4 * i + j, 1);
      const double d2 = (cx - rx) * (cx - rx) + (cy - ry) * (cy - ry);
      out_points[2 * pIdx[j]] = cx;
      out_points[2 * pIdx[j] + 1] = cy;
      out_observed[pIdx[j]] = d2 <= max_subpix_displacement2 ? 1 : 0;
    }
  }
  return 1;
}

}  // extern "C"

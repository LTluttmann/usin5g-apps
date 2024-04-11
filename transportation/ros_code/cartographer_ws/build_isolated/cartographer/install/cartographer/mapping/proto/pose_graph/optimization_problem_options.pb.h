// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping/proto/pose_graph/optimization_problem_options.proto

#ifndef PROTOBUF_cartographer_2fmapping_2fproto_2fpose_5fgraph_2foptimization_5fproblem_5foptions_2eproto__INCLUDED
#define PROTOBUF_cartographer_2fmapping_2fproto_2fpose_5fgraph_2foptimization_5fproblem_5foptions_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3000000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3000000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/unknown_field_set.h>
#include "cartographer/common/proto/ceres_solver_options.pb.h"
// @@protoc_insertion_point(includes)

namespace cartographer {
namespace mapping {
namespace optimization {
namespace proto {

// Internal implementation detail -- do not call these.
void protobuf_AddDesc_cartographer_2fmapping_2fproto_2fpose_5fgraph_2foptimization_5fproblem_5foptions_2eproto();
void protobuf_AssignDesc_cartographer_2fmapping_2fproto_2fpose_5fgraph_2foptimization_5fproblem_5foptions_2eproto();
void protobuf_ShutdownFile_cartographer_2fmapping_2fproto_2fpose_5fgraph_2foptimization_5fproblem_5foptions_2eproto();

class OptimizationProblemOptions;

// ===================================================================

class OptimizationProblemOptions : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:cartographer.mapping.optimization.proto.OptimizationProblemOptions) */ {
 public:
  OptimizationProblemOptions();
  virtual ~OptimizationProblemOptions();

  OptimizationProblemOptions(const OptimizationProblemOptions& from);

  inline OptimizationProblemOptions& operator=(const OptimizationProblemOptions& from) {
    CopyFrom(from);
    return *this;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const OptimizationProblemOptions& default_instance();

  void Swap(OptimizationProblemOptions* other);

  // implements Message ----------------------------------------------

  inline OptimizationProblemOptions* New() const { return New(NULL); }

  OptimizationProblemOptions* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const OptimizationProblemOptions& from);
  void MergeFrom(const OptimizationProblemOptions& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const {
    return InternalSerializeWithCachedSizesToArray(false, output);
  }
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  void InternalSwap(OptimizationProblemOptions* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return _internal_metadata_.arena();
  }
  inline void* MaybeArenaPtr() const {
    return _internal_metadata_.raw_arena_ptr();
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional double huber_scale = 1;
  void clear_huber_scale();
  static const int kHuberScaleFieldNumber = 1;
  double huber_scale() const;
  void set_huber_scale(double value);

  // optional double acceleration_weight = 8;
  void clear_acceleration_weight();
  static const int kAccelerationWeightFieldNumber = 8;
  double acceleration_weight() const;
  void set_acceleration_weight(double value);

  // optional double rotation_weight = 9;
  void clear_rotation_weight();
  static const int kRotationWeightFieldNumber = 9;
  double rotation_weight() const;
  void set_rotation_weight(double value);

  // optional double local_slam_pose_translation_weight = 14;
  void clear_local_slam_pose_translation_weight();
  static const int kLocalSlamPoseTranslationWeightFieldNumber = 14;
  double local_slam_pose_translation_weight() const;
  void set_local_slam_pose_translation_weight(double value);

  // optional double local_slam_pose_rotation_weight = 15;
  void clear_local_slam_pose_rotation_weight();
  static const int kLocalSlamPoseRotationWeightFieldNumber = 15;
  double local_slam_pose_rotation_weight() const;
  void set_local_slam_pose_rotation_weight(double value);

  // optional double odometry_translation_weight = 16;
  void clear_odometry_translation_weight();
  static const int kOdometryTranslationWeightFieldNumber = 16;
  double odometry_translation_weight() const;
  void set_odometry_translation_weight(double value);

  // optional double odometry_rotation_weight = 17;
  void clear_odometry_rotation_weight();
  static const int kOdometryRotationWeightFieldNumber = 17;
  double odometry_rotation_weight() const;
  void set_odometry_rotation_weight(double value);

  // optional double fixed_frame_pose_translation_weight = 11;
  void clear_fixed_frame_pose_translation_weight();
  static const int kFixedFramePoseTranslationWeightFieldNumber = 11;
  double fixed_frame_pose_translation_weight() const;
  void set_fixed_frame_pose_translation_weight(double value);

  // optional double fixed_frame_pose_rotation_weight = 12;
  void clear_fixed_frame_pose_rotation_weight();
  static const int kFixedFramePoseRotationWeightFieldNumber = 12;
  double fixed_frame_pose_rotation_weight() const;
  void set_fixed_frame_pose_rotation_weight(double value);

  // optional bool fixed_frame_pose_use_tolerant_loss = 23;
  void clear_fixed_frame_pose_use_tolerant_loss();
  static const int kFixedFramePoseUseTolerantLossFieldNumber = 23;
  bool fixed_frame_pose_use_tolerant_loss() const;
  void set_fixed_frame_pose_use_tolerant_loss(bool value);

  // optional double fixed_frame_pose_tolerant_loss_param_a = 24;
  void clear_fixed_frame_pose_tolerant_loss_param_a();
  static const int kFixedFramePoseTolerantLossParamAFieldNumber = 24;
  double fixed_frame_pose_tolerant_loss_param_a() const;
  void set_fixed_frame_pose_tolerant_loss_param_a(double value);

  // optional double fixed_frame_pose_tolerant_loss_param_b = 25;
  void clear_fixed_frame_pose_tolerant_loss_param_b();
  static const int kFixedFramePoseTolerantLossParamBFieldNumber = 25;
  double fixed_frame_pose_tolerant_loss_param_b() const;
  void set_fixed_frame_pose_tolerant_loss_param_b(double value);

  // optional bool fix_z_in_3d = 13;
  void clear_fix_z_in_3d();
  static const int kFixZIn3DFieldNumber = 13;
  bool fix_z_in_3d() const;
  void set_fix_z_in_3d(bool value);

  // optional bool use_online_imu_extrinsics_in_3d = 18;
  void clear_use_online_imu_extrinsics_in_3d();
  static const int kUseOnlineImuExtrinsicsIn3DFieldNumber = 18;
  bool use_online_imu_extrinsics_in_3d() const;
  void set_use_online_imu_extrinsics_in_3d(bool value);

  // optional bool log_solver_summary = 5;
  void clear_log_solver_summary();
  static const int kLogSolverSummaryFieldNumber = 5;
  bool log_solver_summary() const;
  void set_log_solver_summary(bool value);

  // optional .cartographer.common.proto.CeresSolverOptions ceres_solver_options = 7;
  bool has_ceres_solver_options() const;
  void clear_ceres_solver_options();
  static const int kCeresSolverOptionsFieldNumber = 7;
  const ::cartographer::common::proto::CeresSolverOptions& ceres_solver_options() const;
  ::cartographer::common::proto::CeresSolverOptions* mutable_ceres_solver_options();
  ::cartographer::common::proto::CeresSolverOptions* release_ceres_solver_options();
  void set_allocated_ceres_solver_options(::cartographer::common::proto::CeresSolverOptions* ceres_solver_options);

  // @@protoc_insertion_point(class_scope:cartographer.mapping.optimization.proto.OptimizationProblemOptions)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  bool _is_default_instance_;
  double huber_scale_;
  double acceleration_weight_;
  double rotation_weight_;
  double local_slam_pose_translation_weight_;
  double local_slam_pose_rotation_weight_;
  double odometry_translation_weight_;
  double odometry_rotation_weight_;
  double fixed_frame_pose_translation_weight_;
  double fixed_frame_pose_rotation_weight_;
  double fixed_frame_pose_tolerant_loss_param_a_;
  double fixed_frame_pose_tolerant_loss_param_b_;
  ::cartographer::common::proto::CeresSolverOptions* ceres_solver_options_;
  bool fixed_frame_pose_use_tolerant_loss_;
  bool fix_z_in_3d_;
  bool use_online_imu_extrinsics_in_3d_;
  bool log_solver_summary_;
  mutable int _cached_size_;
  friend void  protobuf_AddDesc_cartographer_2fmapping_2fproto_2fpose_5fgraph_2foptimization_5fproblem_5foptions_2eproto();
  friend void protobuf_AssignDesc_cartographer_2fmapping_2fproto_2fpose_5fgraph_2foptimization_5fproblem_5foptions_2eproto();
  friend void protobuf_ShutdownFile_cartographer_2fmapping_2fproto_2fpose_5fgraph_2foptimization_5fproblem_5foptions_2eproto();

  void InitAsDefaultInstance();
  static OptimizationProblemOptions* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// OptimizationProblemOptions

// optional double huber_scale = 1;
inline void OptimizationProblemOptions::clear_huber_scale() {
  huber_scale_ = 0;
}
inline double OptimizationProblemOptions::huber_scale() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.optimization.proto.OptimizationProblemOptions.huber_scale)
  return huber_scale_;
}
inline void OptimizationProblemOptions::set_huber_scale(double value) {
  
  huber_scale_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.optimization.proto.OptimizationProblemOptions.huber_scale)
}

// optional double acceleration_weight = 8;
inline void OptimizationProblemOptions::clear_acceleration_weight() {
  acceleration_weight_ = 0;
}
inline double OptimizationProblemOptions::acceleration_weight() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.optimization.proto.OptimizationProblemOptions.acceleration_weight)
  return acceleration_weight_;
}
inline void OptimizationProblemOptions::set_acceleration_weight(double value) {
  
  acceleration_weight_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.optimization.proto.OptimizationProblemOptions.acceleration_weight)
}

// optional double rotation_weight = 9;
inline void OptimizationProblemOptions::clear_rotation_weight() {
  rotation_weight_ = 0;
}
inline double OptimizationProblemOptions::rotation_weight() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.optimization.proto.OptimizationProblemOptions.rotation_weight)
  return rotation_weight_;
}
inline void OptimizationProblemOptions::set_rotation_weight(double value) {
  
  rotation_weight_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.optimization.proto.OptimizationProblemOptions.rotation_weight)
}

// optional double local_slam_pose_translation_weight = 14;
inline void OptimizationProblemOptions::clear_local_slam_pose_translation_weight() {
  local_slam_pose_translation_weight_ = 0;
}
inline double OptimizationProblemOptions::local_slam_pose_translation_weight() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.optimization.proto.OptimizationProblemOptions.local_slam_pose_translation_weight)
  return local_slam_pose_translation_weight_;
}
inline void OptimizationProblemOptions::set_local_slam_pose_translation_weight(double value) {
  
  local_slam_pose_translation_weight_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.optimization.proto.OptimizationProblemOptions.local_slam_pose_translation_weight)
}

// optional double local_slam_pose_rotation_weight = 15;
inline void OptimizationProblemOptions::clear_local_slam_pose_rotation_weight() {
  local_slam_pose_rotation_weight_ = 0;
}
inline double OptimizationProblemOptions::local_slam_pose_rotation_weight() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.optimization.proto.OptimizationProblemOptions.local_slam_pose_rotation_weight)
  return local_slam_pose_rotation_weight_;
}
inline void OptimizationProblemOptions::set_local_slam_pose_rotation_weight(double value) {
  
  local_slam_pose_rotation_weight_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.optimization.proto.OptimizationProblemOptions.local_slam_pose_rotation_weight)
}

// optional double odometry_translation_weight = 16;
inline void OptimizationProblemOptions::clear_odometry_translation_weight() {
  odometry_translation_weight_ = 0;
}
inline double OptimizationProblemOptions::odometry_translation_weight() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.optimization.proto.OptimizationProblemOptions.odometry_translation_weight)
  return odometry_translation_weight_;
}
inline void OptimizationProblemOptions::set_odometry_translation_weight(double value) {
  
  odometry_translation_weight_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.optimization.proto.OptimizationProblemOptions.odometry_translation_weight)
}

// optional double odometry_rotation_weight = 17;
inline void OptimizationProblemOptions::clear_odometry_rotation_weight() {
  odometry_rotation_weight_ = 0;
}
inline double OptimizationProblemOptions::odometry_rotation_weight() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.optimization.proto.OptimizationProblemOptions.odometry_rotation_weight)
  return odometry_rotation_weight_;
}
inline void OptimizationProblemOptions::set_odometry_rotation_weight(double value) {
  
  odometry_rotation_weight_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.optimization.proto.OptimizationProblemOptions.odometry_rotation_weight)
}

// optional double fixed_frame_pose_translation_weight = 11;
inline void OptimizationProblemOptions::clear_fixed_frame_pose_translation_weight() {
  fixed_frame_pose_translation_weight_ = 0;
}
inline double OptimizationProblemOptions::fixed_frame_pose_translation_weight() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.optimization.proto.OptimizationProblemOptions.fixed_frame_pose_translation_weight)
  return fixed_frame_pose_translation_weight_;
}
inline void OptimizationProblemOptions::set_fixed_frame_pose_translation_weight(double value) {
  
  fixed_frame_pose_translation_weight_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.optimization.proto.OptimizationProblemOptions.fixed_frame_pose_translation_weight)
}

// optional double fixed_frame_pose_rotation_weight = 12;
inline void OptimizationProblemOptions::clear_fixed_frame_pose_rotation_weight() {
  fixed_frame_pose_rotation_weight_ = 0;
}
inline double OptimizationProblemOptions::fixed_frame_pose_rotation_weight() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.optimization.proto.OptimizationProblemOptions.fixed_frame_pose_rotation_weight)
  return fixed_frame_pose_rotation_weight_;
}
inline void OptimizationProblemOptions::set_fixed_frame_pose_rotation_weight(double value) {
  
  fixed_frame_pose_rotation_weight_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.optimization.proto.OptimizationProblemOptions.fixed_frame_pose_rotation_weight)
}

// optional bool fixed_frame_pose_use_tolerant_loss = 23;
inline void OptimizationProblemOptions::clear_fixed_frame_pose_use_tolerant_loss() {
  fixed_frame_pose_use_tolerant_loss_ = false;
}
inline bool OptimizationProblemOptions::fixed_frame_pose_use_tolerant_loss() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.optimization.proto.OptimizationProblemOptions.fixed_frame_pose_use_tolerant_loss)
  return fixed_frame_pose_use_tolerant_loss_;
}
inline void OptimizationProblemOptions::set_fixed_frame_pose_use_tolerant_loss(bool value) {
  
  fixed_frame_pose_use_tolerant_loss_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.optimization.proto.OptimizationProblemOptions.fixed_frame_pose_use_tolerant_loss)
}

// optional double fixed_frame_pose_tolerant_loss_param_a = 24;
inline void OptimizationProblemOptions::clear_fixed_frame_pose_tolerant_loss_param_a() {
  fixed_frame_pose_tolerant_loss_param_a_ = 0;
}
inline double OptimizationProblemOptions::fixed_frame_pose_tolerant_loss_param_a() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.optimization.proto.OptimizationProblemOptions.fixed_frame_pose_tolerant_loss_param_a)
  return fixed_frame_pose_tolerant_loss_param_a_;
}
inline void OptimizationProblemOptions::set_fixed_frame_pose_tolerant_loss_param_a(double value) {
  
  fixed_frame_pose_tolerant_loss_param_a_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.optimization.proto.OptimizationProblemOptions.fixed_frame_pose_tolerant_loss_param_a)
}

// optional double fixed_frame_pose_tolerant_loss_param_b = 25;
inline void OptimizationProblemOptions::clear_fixed_frame_pose_tolerant_loss_param_b() {
  fixed_frame_pose_tolerant_loss_param_b_ = 0;
}
inline double OptimizationProblemOptions::fixed_frame_pose_tolerant_loss_param_b() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.optimization.proto.OptimizationProblemOptions.fixed_frame_pose_tolerant_loss_param_b)
  return fixed_frame_pose_tolerant_loss_param_b_;
}
inline void OptimizationProblemOptions::set_fixed_frame_pose_tolerant_loss_param_b(double value) {
  
  fixed_frame_pose_tolerant_loss_param_b_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.optimization.proto.OptimizationProblemOptions.fixed_frame_pose_tolerant_loss_param_b)
}

// optional bool fix_z_in_3d = 13;
inline void OptimizationProblemOptions::clear_fix_z_in_3d() {
  fix_z_in_3d_ = false;
}
inline bool OptimizationProblemOptions::fix_z_in_3d() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.optimization.proto.OptimizationProblemOptions.fix_z_in_3d)
  return fix_z_in_3d_;
}
inline void OptimizationProblemOptions::set_fix_z_in_3d(bool value) {
  
  fix_z_in_3d_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.optimization.proto.OptimizationProblemOptions.fix_z_in_3d)
}

// optional bool use_online_imu_extrinsics_in_3d = 18;
inline void OptimizationProblemOptions::clear_use_online_imu_extrinsics_in_3d() {
  use_online_imu_extrinsics_in_3d_ = false;
}
inline bool OptimizationProblemOptions::use_online_imu_extrinsics_in_3d() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.optimization.proto.OptimizationProblemOptions.use_online_imu_extrinsics_in_3d)
  return use_online_imu_extrinsics_in_3d_;
}
inline void OptimizationProblemOptions::set_use_online_imu_extrinsics_in_3d(bool value) {
  
  use_online_imu_extrinsics_in_3d_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.optimization.proto.OptimizationProblemOptions.use_online_imu_extrinsics_in_3d)
}

// optional bool log_solver_summary = 5;
inline void OptimizationProblemOptions::clear_log_solver_summary() {
  log_solver_summary_ = false;
}
inline bool OptimizationProblemOptions::log_solver_summary() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.optimization.proto.OptimizationProblemOptions.log_solver_summary)
  return log_solver_summary_;
}
inline void OptimizationProblemOptions::set_log_solver_summary(bool value) {
  
  log_solver_summary_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.optimization.proto.OptimizationProblemOptions.log_solver_summary)
}

// optional .cartographer.common.proto.CeresSolverOptions ceres_solver_options = 7;
inline bool OptimizationProblemOptions::has_ceres_solver_options() const {
  return !_is_default_instance_ && ceres_solver_options_ != NULL;
}
inline void OptimizationProblemOptions::clear_ceres_solver_options() {
  if (GetArenaNoVirtual() == NULL && ceres_solver_options_ != NULL) delete ceres_solver_options_;
  ceres_solver_options_ = NULL;
}
inline const ::cartographer::common::proto::CeresSolverOptions& OptimizationProblemOptions::ceres_solver_options() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.optimization.proto.OptimizationProblemOptions.ceres_solver_options)
  return ceres_solver_options_ != NULL ? *ceres_solver_options_ : *default_instance_->ceres_solver_options_;
}
inline ::cartographer::common::proto::CeresSolverOptions* OptimizationProblemOptions::mutable_ceres_solver_options() {
  
  if (ceres_solver_options_ == NULL) {
    ceres_solver_options_ = new ::cartographer::common::proto::CeresSolverOptions;
  }
  // @@protoc_insertion_point(field_mutable:cartographer.mapping.optimization.proto.OptimizationProblemOptions.ceres_solver_options)
  return ceres_solver_options_;
}
inline ::cartographer::common::proto::CeresSolverOptions* OptimizationProblemOptions::release_ceres_solver_options() {
  // @@protoc_insertion_point(field_release:cartographer.mapping.optimization.proto.OptimizationProblemOptions.ceres_solver_options)
  
  ::cartographer::common::proto::CeresSolverOptions* temp = ceres_solver_options_;
  ceres_solver_options_ = NULL;
  return temp;
}
inline void OptimizationProblemOptions::set_allocated_ceres_solver_options(::cartographer::common::proto::CeresSolverOptions* ceres_solver_options) {
  delete ceres_solver_options_;
  ceres_solver_options_ = ceres_solver_options;
  if (ceres_solver_options) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_set_allocated:cartographer.mapping.optimization.proto.OptimizationProblemOptions.ceres_solver_options)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_cartographer_2fmapping_2fproto_2fpose_5fgraph_2foptimization_5fproblem_5foptions_2eproto__INCLUDED
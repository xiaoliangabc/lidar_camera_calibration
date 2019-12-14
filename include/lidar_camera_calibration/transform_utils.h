#ifndef LIDAR_CAMERA_CALIBRATION_TRANSFORM_UTILS_H_
#define LIDAR_CAMERA_CALIBRATION_TRANSFORM_UTILS_H_

#include <Eigen/Dense>

// Part 1: Roatation vector
// Rotation vector to rotation matrix
inline Eigen::Matrix3d RotationVectorToRotationMatrix(
    const Eigen::Vector3d& rotation_vector) {
  double norm = rotation_vector.norm();
  Eigen::AngleAxisd angle_axis(norm, rotation_vector / norm);
  return angle_axis.matrix();
}

// Rotation vector to euler angles
inline Eigen::Vector3d RotationVectorToEulerAngles(
    const Eigen::Vector3d& rotation_vector) {
  double norm = rotation_vector.norm();
  Eigen::AngleAxisd angle_axis(norm, rotation_vector / norm);
  return angle_axis.matrix().eulerAngles(2, 1, 0);
}

// Rotation vector quaternion
inline Eigen::Quaterniond RotationVectorToQuaternion(
    const Eigen::Vector3d& rotation_vector) {
  double norm = rotation_vector.norm();
  Eigen::AngleAxisd angle_axis(norm, rotation_vector / norm);
  Eigen::Quaterniond quaternion;
  quaternion = angle_axis;
  return quaternion;
}

// Part 2: Roatation matrix
// Rotation matrix to rotation vector
inline Eigen::Vector3d RotationMatrixToRotationVector(
    const Eigen::Matrix3d& rotation_matrix) {
  Eigen::AngleAxisd angle_axis(rotation_matrix);
  return angle_axis.angle() * angle_axis.axis();
}

// Rotation matrix to euler angles
inline Eigen::Vector3d RotationMatrixToEulerAngles(
    const Eigen::Matrix3d& rotation_matrix) {
  return rotation_matrix.eulerAngles(2, 1, 0);
}

// Rotation matrix to quaternion
inline Eigen::Quaterniond RotationMatrixToQuaternion(
    const Eigen::Matrix3d& rotation_matrix) {
  Eigen::Quaterniond quaternion;
  quaternion = rotation_matrix;
  return quaternion;
}

// Part 3: Euler angles
// Euler angles to rotation vector
inline Eigen::Vector3d EulerAnglesToRotationVector(
    const Eigen::Vector3d& euler_angles) {
  Eigen::AngleAxisd roll_angle(
      Eigen::AngleAxisd(euler_angles(2), Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitch_angle(
      Eigen::AngleAxisd(euler_angles(1), Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yaw_angle(
      Eigen::AngleAxisd(euler_angles(0), Eigen::Vector3d::UnitZ()));
  Eigen::AngleAxisd angle_axis;
  angle_axis = yaw_angle * pitch_angle * roll_angle;
  return angle_axis.angle() * angle_axis.axis();
}

// Euler angles to rotation matrix
inline Eigen::Matrix3d EulerAnglesToRotationMatrix(
    const Eigen::Vector3d& euler_angles) {
  Eigen::AngleAxisd roll_angle(
      Eigen::AngleAxisd(euler_angles(2), Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitch_angle(
      Eigen::AngleAxisd(euler_angles(1), Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yaw_angle(
      Eigen::AngleAxisd(euler_angles(0), Eigen::Vector3d::UnitZ()));
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix = yaw_angle * pitch_angle * roll_angle;
  return rotation_matrix;
}

// Euler angles to quaternion
inline Eigen::Quaterniond EulerAnglesToQuaternion(
    const Eigen::Vector3d& euler_angles) {
  Eigen::AngleAxisd roll_angle(
      Eigen::AngleAxisd(euler_angles(2), Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitch_angle(
      Eigen::AngleAxisd(euler_angles(1), Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yaw_angle(
      Eigen::AngleAxisd(euler_angles(0), Eigen::Vector3d::UnitZ()));
  Eigen::Quaterniond quaternion;
  quaternion = yaw_angle * pitch_angle * roll_angle;
  return quaternion;
}

// Part 4: Quaternion
// Quaternion to rotation vector
inline Eigen::Vector3d QuaternionToRotationVector(
    const Eigen::Quaterniond& quaternion) {
  Eigen::AngleAxisd angle_axis(quaternion);
  return angle_axis.angle() * angle_axis.axis();
}

// Quaternion to rotation matrix
inline Eigen::Matrix3d QuaternionToRotationMatrix(
    const Eigen::Quaterniond& quaternion) {
  return quaternion.matrix();
}

// Quaternion to euler angles
inline Eigen::Vector3d QuaternionToEulerAngles(
    const Eigen::Quaterniond& quaternion) {
  return quaternion.matrix().eulerAngles(2, 1, 0);
}

// Part 5: To Transform affine
// Rotation vector and translation vector to transform affine
inline Eigen::Affine3d RotationVectorToTransformAffine(
    const Eigen::Vector3d& rotation_vector,
    const Eigen::Vector3d& translation_vector) {
  Eigen::Affine3d transform_affine = Eigen::Affine3d::Identity();
  transform_affine.linear() = RotationVectorToRotationMatrix(rotation_vector);
  transform_affine.translation() = translation_vector;
  return transform_affine;
}

// Rotation matrix and translation vector to transform affine
inline Eigen::Affine3d RotationMatrixToTransformAffine(
    const Eigen::Matrix3d& rotation_matrix,
    const Eigen::Vector3d& translation_vector) {
  Eigen::Affine3d transform_affine = Eigen::Affine3d::Identity();
  transform_affine.linear() = rotation_matrix;
  transform_affine.translation() = translation_vector;
  return transform_affine;
}

// Euler angles and translation vector to transform affine
inline Eigen::Affine3d EulerAnglesToTransformAffine(
    const Eigen::Vector3d& euler_angles,
    const Eigen::Vector3d& translation_vector) {
  Eigen::Affine3d transform_affine = Eigen::Affine3d::Identity();
  transform_affine.linear() = EulerAnglesToRotationMatrix(euler_angles);
  transform_affine.translation() = translation_vector;
  return transform_affine;
}

// Quaternion and translation vector to transform affine
inline Eigen::Affine3d QuaternionToTransformAffine(
    const Eigen::Quaterniond& quaternion,
    const Eigen::Vector3d& translation_vector) {
  Eigen::Affine3d transform_affine = Eigen::Affine3d::Identity();
  transform_affine.linear() = QuaternionToRotationMatrix(quaternion);
  transform_affine.translation() = translation_vector;
  return transform_affine;
}

// Part 6: From Transform matrix
// Transform matrix to rotation vector and translation vector
inline void TransformMatrixToRotationVector(
    const Eigen::Matrix4d& transform_matrix, Eigen::Vector3d& rotation_vector,
    Eigen::Vector3d& translation_vector) {
  Eigen::Affine3d transform_affine;
  transform_affine.matrix() = transform_matrix;
  rotation_vector = RotationMatrixToRotationVector(transform_affine.linear());
  translation_vector = transform_affine.translation();
}

// Transform matrix to rotation matrix and translation vector
inline void TransformMatrixToRotationMatrix(
    const Eigen::Matrix4d& transform_matrix, Eigen::Matrix3d& rotation_matrix,
    Eigen::Vector3d& translation_vector) {
  Eigen::Affine3d transform_affine;
  transform_affine.matrix() = transform_matrix;
  rotation_matrix = transform_affine.linear();
  translation_vector = transform_affine.translation();
}

// Transform matrix to euler angles and translation vector
inline void TransformMatrixToEulerAngles(
    const Eigen::Matrix4d& transform_matrix, Eigen::Vector3d& euler_angles,
    Eigen::Vector3d& translation_vector) {
  Eigen::Affine3d transform_affine;
  transform_affine.matrix() = transform_matrix;
  euler_angles = RotationMatrixToEulerAngles(transform_affine.linear());
  translation_vector = transform_affine.translation();
}

// Transform matrix to quaternion and translation vector
inline void TransformMatrixToQuaternion(const Eigen::Matrix4d& transform_matrix,
                                        Eigen::Quaterniond& quaternion,
                                        Eigen::Vector3d& translation_vector) {
  Eigen::Affine3d transform_affine;
  transform_affine.matrix() = transform_matrix;
  quaternion = RotationMatrixToQuaternion(transform_affine.linear());
  translation_vector = transform_affine.translation();
}

#endif  // LIDAR_CAMERA_CALIBRATION_TRANSFORM_UTILS_H_

//
// Created by nico on 23.07.19.
//

#include <algorithm>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>
#include <aslam/common/memory.h>
#include <aslam/common/sensor.h>
#include <aslam/common/unique-id.h>
#include <aslam/common/yaml-file-serialization.h>
#include <glog/logging.h>

#include "empty_rosnode_cplusplus/some_dummy_class.h"

constexpr char kYamlFieldNameSensors[] = "sensors";
constexpr char kYamlFieldNameExtrinsics[] = "extrinsics";
constexpr char kYamlFieldNameSensorId[] = "sensor_id";
constexpr char kYamlFieldNameBaseSensorId[] = "base_sensor_id";
constexpr char kYamlFieldNameT_B_S[] = "T_B_S";
constexpr char kYamlFieldNameT_S_B[] = "T_S_B";

void some_dummy_class::transformKindrToValidRotationMatrix(const std::string& filename, const std::string& out_file_name)
{
  const YAML::Node& yaml_node = YAML::LoadFile(filename.c_str());
  YAML::Node corrections_node;

  CHECK(yaml_node.IsDefined());
  CHECK(yaml_node.IsMap());

  const YAML::Node sensors_node = yaml_node[static_cast<std::string>(kYamlFieldNameSensors)];
  for (const YAML::Node& sensor_node : sensors_node) {
      CHECK(!sensor_node.IsNull());

      std::string sensor_type_as_string;
      if (YAML::safeGet(sensor_node,
              static_cast<std::string>(aslam::kYamlFieldNameSensorType),
              &sensor_type_as_string) && sensor_type_as_string == "NCAMERA") {
            // Parse the cameras.
            LOG(INFO) << "Parsing cameras...";
            const YAML::Node& cameras_node = sensor_node["cameras"];
            if (cameras_node.IsDefined() && !cameras_node.IsNull()
            && cameras_node.IsSequence()) {
              size_t num_cameras = cameras_node.size();
              for (size_t camera_index = 0; camera_index < num_cameras; ++camera_index) {
                LOG(INFO) << "cam " << camera_index;
                // Decode the camera
                const YAML::Node& camera_node = cameras_node[camera_index];
                if (!camera_node) {
                  LOG(ERROR) << "Unable to get camera node for camera " << camera_index;
                }

                if (!camera_node.IsMap()) {
                  LOG(ERROR) << "Camera node for camera " << camera_index
                             << " is not a map.";
                }

                YAML::Node intrinsics = camera_node["camera"];
                std::string id_as_string;
                if (!YAML::safeGet(intrinsics, "id", &id_as_string)) {
                  LOG(ERROR) << "Unable to find sensor id for camera intrinsics. ";
                }

                aslam::SensorId sensor_id;
                CHECK(sensor_id.fromHexString(id_as_string));

                // Get the transformation matrix T_B_C (takes points from the frame C to
                // frame B).
                Eigen::Matrix4d input_matrix;
                aslam::Transformation T_B_C;
                if (camera_node["T_B_C"]) {
                  CHECK(YAML::safeGet(camera_node, "T_B_C", &input_matrix));

                  Eigen::Matrix3d R_B_C = input_matrix.topLeftCorner<3, 3>().eval();
                  if (!aslam::Quaternion::isValidRotationMatrix(R_B_C))
                  {
                    input_matrix.topLeftCorner<3, 3>() =
                        aslam::Quaternion::fromApproximateRotationMatrix(R_B_C)
                            .getRotationMatrix();

                    LOG(WARNING) << "The extrinsics for sensor " << sensor_id
                    << " do not contain a valid rotation matrix!\n"
                    << "Even when relaxing the conditions."
                    << "Approximating with nearest orthonormal matrix! \nT_B_C: "
                    << input_matrix;

                    YAML::Node camera_T_B_C_node;
                    camera_T_B_C_node["id"] = sensor_id.hexString();
                    camera_T_B_C_node[static_cast<std::string>("T_B_C")] =
                        YAML::convert<Eigen::Matrix4d>::encode(input_matrix);
                    corrections_node.push_back(camera_T_B_C_node);

                  }
                  else {
                    LOG(INFO) << "Cam: " << camera_index << " is doing ok";
                  }
                  T_B_C = aslam::Transformation(input_matrix);
                } else if (camera_node["T_C_B"]) {
                  CHECK(YAML::safeGet(camera_node, "T_C_B", &input_matrix));
                  Eigen::Matrix3d R_C_B = input_matrix.topLeftCorner<3, 3>().eval();
                  if (!aslam::Quaternion::isValidRotationMatrix(R_C_B))
                  {
                    input_matrix.topLeftCorner<3, 3>() =
                        aslam::Quaternion::fromApproximateRotationMatrix(R_C_B)
                            .getRotationMatrix();

                    LOG(WARNING) << "The extrinsics for sensor " << sensor_id
                    << " do not contain a valid rotation matrix!\n"
                    << "Even when relaxing the conditions."
                    << "Approximating with nearest orthonormal matrix! \nT_C_B: "
                    << input_matrix;

                    YAML::Node camera_T_C_B_node;
                    camera_T_C_B_node["id"] = sensor_id.hexString();
                    camera_T_C_B_node[static_cast<std::string>("T_C_B")] =
                        YAML::convert<Eigen::Matrix4d>::encode(input_matrix);
                    corrections_node.push_back(camera_T_C_B_node);
                  }
                  T_B_C = aslam::Transformation(input_matrix).inverse();

                } else {
                  LOG(ERROR)
                      << "Unable to get extrinsic transformation T_B_C or T_C_B for camera "
                      << camera_index;
                }
                LOG(INFO) << "Cam " << camera_index << "loaded successfully";
              }
            }
      }
  }

  aslam::SensorIdSet base_sensor_ids;
  AlignedUnorderedMap<aslam::SensorId, aslam::SensorId> base_sensor_id_map;
  AlignedUnorderedMap<aslam::SensorId, aslam::Transformation> T_B_S_map;

  const YAML::Node extrinsics_node =
      yaml_node[static_cast<std::string>(kYamlFieldNameExtrinsics)];
  if (extrinsics_node.IsDefined() && !extrinsics_node.IsNull()) {
    CHECK(extrinsics_node.IsSequence());
    for (const YAML::Node& extrinsic_node : extrinsics_node) {
      CHECK(!extrinsic_node.IsNull());

      std::string id_as_string;
      if (!YAML::safeGet(
              extrinsic_node, static_cast<std::string>(kYamlFieldNameSensorId),
              &id_as_string)) {
        LOG(ERROR) << "Unable to find sensor id for extrinsics. ";
      }
      aslam::SensorId sensor_id;
      CHECK(sensor_id.fromHexString(id_as_string));

      if (!YAML::safeGet(
              extrinsic_node,
              static_cast<std::string>(kYamlFieldNameBaseSensorId),
              &id_as_string)) {
        LOG(ERROR) << "Unable to find base sensor id for extrinsics.";
      }
      aslam::SensorId base_sensor_id;
      CHECK(base_sensor_id.fromHexString(id_as_string));

      aslam::Transformation T_B_S;
      Eigen::Matrix4d input_matrix;
      if (extrinsic_node[kYamlFieldNameT_B_S]) {
        CHECK(YAML::safeGet(
            extrinsic_node, static_cast<std::string>(kYamlFieldNameT_B_S),
            &input_matrix));
        Eigen::Matrix3d R_B_S = input_matrix.topLeftCorner<3, 3>().eval();
        if (!aslam::Quaternion::isValidRotationMatrix(R_B_S)) {
          input_matrix.topLeftCorner<3, 3>() =
              aslam::Quaternion::fromApproximateRotationMatrix(R_B_S)
                  .getRotationMatrix();
          if(!aslam::Quaternion::isValidRotationMatrix(R_B_S, 1e-4))
          {
            LOG(WARNING) << "The extrinsics for sensor " << sensor_id
            << " do not contain a valid rotation matrix!\n"
            << "Even when relaxing the conditions."
            << "Approximating with nearest orthonormal matrix! \nT_B_S: "
            << input_matrix;
          }
          else {
            LOG(WARNING) << "The extrinsics for sensor " << sensor_id
                         << " do not contain a valid rotation matrix,"
                         << " approximating with nearest orthonormal matrix!"
                         << "\nT_B_S: " << input_matrix;
          }
          // save to corrections file
          YAML::Node extrinsic_node;
          extrinsic_node[static_cast<std::string>(kYamlFieldNameSensorId)] =
              sensor_id.hexString();
          extrinsic_node[static_cast<std::string>(kYamlFieldNameBaseSensorId)] =
              base_sensor_id.hexString();
          extrinsic_node[static_cast<std::string>(kYamlFieldNameT_B_S)] =
              YAML::convert<Eigen::Matrix4d>::encode(input_matrix);

          corrections_node.push_back(extrinsic_node);
        }

        T_B_S = aslam::Transformation(input_matrix);
      } else if (extrinsic_node[kYamlFieldNameT_S_B]) {
        CHECK(YAML::safeGet(
            extrinsic_node, static_cast<std::string>(kYamlFieldNameT_S_B),
            &input_matrix));
        Eigen::Matrix3d R_S_B = input_matrix.topLeftCorner<3, 3>().eval();
        if (!aslam::Quaternion::isValidRotationMatrix(R_S_B)) {
          input_matrix.topLeftCorner<3, 3>() =
            aslam::Quaternion::fromApproximateRotationMatrix(R_S_B)
                .getRotationMatrix();
          if(!aslam::Quaternion::isValidRotationMatrix(R_S_B, 1e-4))
          {
            LOG(WARNING) << "The extrinsics for sensor " << sensor_id
            << " do not contain a valid rotation matrix!\n"
            << "Even when relaxing the conditions."
            << "Approximating with nearest orthonormal matrix! \nT_S_B: "
            << input_matrix;
          } else {
            LOG(WARNING)
                << "The extrinsics for sensor " << sensor_id
                << " do not contain a valid rotation matrix, approximating with "
                << "nearest orthonormal matrix! \nT_S_B: " << input_matrix;
          }
          // save to corrections file
          YAML::Node extrinsic_node;
          extrinsic_node[static_cast<std::string>(kYamlFieldNameSensorId)] =
              sensor_id.hexString();
          extrinsic_node[static_cast<std::string>(kYamlFieldNameBaseSensorId)] =
              base_sensor_id.hexString();
          extrinsic_node[static_cast<std::string>(kYamlFieldNameT_S_B)] =
              YAML::convert<Eigen::Matrix4d>::encode(input_matrix);

          corrections_node.push_back(extrinsic_node);
        }
        T_B_S = aslam::Transformation(input_matrix).inverse();
      } else {
        LOG(ERROR) << "Unable to find T_B_S or T_S_B for extrinsics.";
      }

      if (base_sensor_id == sensor_id) {
        // Check that for a base sensor the calibration matrix is identity
        if ((T_B_S.getTransformationMatrix() - Eigen::Matrix4d::Identity())
                .cwiseAbs()
                .maxCoeff() > aslam::common::macros::kEpsilon) {
          LOG(FATAL) << "Sensor " << sensor_id << " of type "
                     << " is a base sensor but the extrinsics associated with "
                     << "it (T_B_S) are not idenity! T_B_S: "
                     << T_B_S.getTransformationMatrix();
        }
        base_sensor_ids.emplace(sensor_id);
      } else {
        base_sensor_id_map.emplace(sensor_id, base_sensor_id);
        T_B_S_map.emplace(sensor_id, T_B_S);
      }
    }
  }

  // write corrections to file
  CHECK(corrections_node.IsDefined());
  if (corrections_node.size() > 0 )
  {
    CHECK(!out_file_name.empty()) << "Need to define an out_file_name if you wish to save corrections.";
    std::ofstream output_file_stream(out_file_name);
    CHECK(output_file_stream.is_open())
        << "Failed to open file " << out_file_name << " for writing.";
    output_file_stream << corrections_node;
    output_file_stream.close();
    LOG(INFO) << "Saved corrections to: " << out_file_name;
  }
  else {
    LOG(INFO) << "Everything seems to be in order.";
  }
}

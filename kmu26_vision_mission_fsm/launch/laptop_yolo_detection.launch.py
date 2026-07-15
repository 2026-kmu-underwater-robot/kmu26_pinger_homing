from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/camera/color/image_raw/compressed",
                description="Compressed camera topic received from the AUV NUC.",
            ),
            DeclareLaunchArgument(
                "bbox_topic",
                default_value="/vision/buoy_bbox",
                description="BBox topic published back to the AUV NUC.",
            ),
            DeclareLaunchArgument("observation_topic", default_value="/vision/buoy_observation"),
            DeclareLaunchArgument(
                "annotated_topic", default_value="/vision/buoy/image_annotated/compressed"
            ),
            DeclareLaunchArgument("status_topic", default_value="/vision/buoy/status"),
            DeclareLaunchArgument(
                "model_path",
                default_value="",
                description="Optional YOLO .pt path. Empty uses the packaged models/best.pt.",
            ),
            DeclareLaunchArgument(
                "target_class_name",
                default_value="buoy",
                description="Target class name. Empty accepts every detected class.",
            ),
            DeclareLaunchArgument(
                "target_class_id",
                default_value="-1",
                description="Target class id. Overrides target_class_name when >= 0.",
            ),
            DeclareLaunchArgument("confidence_threshold", default_value="0.35"),
            DeclareLaunchArgument("iou_threshold", default_value="0.45"),
            DeclareLaunchArgument(
                "device",
                default_value="auto",
                description="Inference device: auto, cpu, cuda:0, etc.",
            ),
            DeclareLaunchArgument("cpu_threads", default_value="2"),
            DeclareLaunchArgument("cpu_affinity_cores", default_value="3"),
            DeclareLaunchArgument("limit_cpu_affinity", default_value="false"),
            DeclareLaunchArgument("imgsz", default_value="640"),
            DeclareLaunchArgument("inference_hz", default_value="3.0"),
            DeclareLaunchArgument(
                "publish_per_class",
                default_value="true",
                description="Publish the best buoy and stick bbox separately for the visual controller.",
            ),
            DeclareLaunchArgument("selection_policy", default_value="largest_mask"),
            DeclareLaunchArgument("min_vertical_aspect", default_value="0.50"),
            DeclareLaunchArgument("min_mask_area_ratio", default_value="0.0001"),
            DeclareLaunchArgument("max_mask_area_ratio", default_value="0.20"),
            DeclareLaunchArgument("track_hold_seconds", default_value="1.60"),
            DeclareLaunchArgument("max_inference_result_age_s", default_value="2.00"),
            DeclareLaunchArgument("preprocess_enabled", default_value="true"),
            DeclareLaunchArgument("white_balance", default_value="true"),
            DeclareLaunchArgument("clahe_clip", default_value="2.0"),
            DeclareLaunchArgument("clahe_grid", default_value="8"),
            DeclareLaunchArgument("jpeg_quality", default_value="88"),
            DeclareLaunchArgument(
                "show_preview",
                default_value="false",
                description="Show a local OpenCV window. Keep false on a headless NUC.",
            ),
            DeclareLaunchArgument(
                "preview_window_name",
                default_value="YOLO Buoy Detection",
                description="OpenCV window title for the preview UI.",
            ),
            Node(
                package="kmu26_vision_mission_fsm",
                executable="run_yolo_buoy_detector",
                name="yolo_buoy_detector",
                output="screen",
                parameters=[
                    {
                        "image_topic": LaunchConfiguration("image_topic"),
                        "bbox_topic": LaunchConfiguration("bbox_topic"),
                        "observation_topic": LaunchConfiguration("observation_topic"),
                        "annotated_topic": LaunchConfiguration("annotated_topic"),
                        "status_topic": LaunchConfiguration("status_topic"),
                        "model_path": LaunchConfiguration("model_path"),
                        "target_class_name": LaunchConfiguration("target_class_name"),
                        "target_class_id": ParameterValue(LaunchConfiguration("target_class_id"), value_type=int),
                        "confidence_threshold": ParameterValue(
                            LaunchConfiguration("confidence_threshold"),
                            value_type=float,
                        ),
                        "iou_threshold": ParameterValue(
                            LaunchConfiguration("iou_threshold"), value_type=float
                        ),
                        "device": LaunchConfiguration("device"),
                        "cpu_threads": ParameterValue(
                            LaunchConfiguration("cpu_threads"), value_type=int
                        ),
                        "cpu_affinity_cores": ParameterValue(
                            LaunchConfiguration("cpu_affinity_cores"), value_type=int
                        ),
                        "limit_cpu_affinity": ParameterValue(
                            LaunchConfiguration("limit_cpu_affinity"), value_type=bool
                        ),
                        "imgsz": ParameterValue(LaunchConfiguration("imgsz"), value_type=int),
                        "inference_hz": ParameterValue(
                            LaunchConfiguration("inference_hz"), value_type=float
                        ),
                        "publish_per_class": ParameterValue(
                            LaunchConfiguration("publish_per_class"), value_type=bool
                        ),
                        "selection_policy": LaunchConfiguration("selection_policy"),
                        "min_vertical_aspect": ParameterValue(
                            LaunchConfiguration("min_vertical_aspect"), value_type=float
                        ),
                        "min_mask_area_ratio": ParameterValue(
                            LaunchConfiguration("min_mask_area_ratio"), value_type=float
                        ),
                        "max_mask_area_ratio": ParameterValue(
                            LaunchConfiguration("max_mask_area_ratio"), value_type=float
                        ),
                        "track_hold_seconds": ParameterValue(
                            LaunchConfiguration("track_hold_seconds"), value_type=float
                        ),
                        "max_inference_result_age_s": ParameterValue(
                            LaunchConfiguration("max_inference_result_age_s"), value_type=float
                        ),
                        "preprocess_enabled": ParameterValue(
                            LaunchConfiguration("preprocess_enabled"), value_type=bool
                        ),
                        "white_balance": ParameterValue(
                            LaunchConfiguration("white_balance"), value_type=bool
                        ),
                        "clahe_clip": ParameterValue(
                            LaunchConfiguration("clahe_clip"), value_type=float
                        ),
                        "clahe_grid": ParameterValue(
                            LaunchConfiguration("clahe_grid"), value_type=int
                        ),
                        "jpeg_quality": ParameterValue(
                            LaunchConfiguration("jpeg_quality"), value_type=int
                        ),
                        "show_preview": ParameterValue(LaunchConfiguration("show_preview"), value_type=bool),
                        "preview_window_name": LaunchConfiguration("preview_window_name"),
                    }
                ],
                additional_env={
                    "OMP_NUM_THREADS": "2",
                    "MKL_NUM_THREADS": "2",
                    "OPENBLAS_NUM_THREADS": "2",
                    "NUMEXPR_NUM_THREADS": "2",
                },
            ),
        ]
    )

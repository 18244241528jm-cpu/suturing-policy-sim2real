from glob import glob
from setuptools import find_packages, setup

package_name = "suturing_runtime"

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="SurgicAI Project 34",
    maintainer_email="noreply@example.com",
    description="Topic-first, guarded SurgicAI Reach runtime for AMBF and dVRK.",
    license="MIT",
    entry_points={"console_scripts": [
        "dvrk_topic_adapter = suturing_runtime.dvrk_topic_adapter:main",
        "pipeline_supervisor = suturing_runtime.pipeline_supervisor:main",
        "approach_goal_builder = suturing_runtime.approach_goal_builder:main",
        "guarded_pose_executor = suturing_runtime.guarded_pose_executor:main",
        "topic_preflight = suturing_runtime.topic_preflight:main",
        "mock_topic_source = suturing_runtime.mock_topic_source:main",
        "initialization_snapshot = suturing_runtime.snapshot_node:main",
        "perception_input_adapter = suturing_runtime.perception_input_adapter:main",
        "fp_bundle_join = suturing_runtime.bundle_join_node:main",
        "fp_candidate_adapter = suturing_runtime.fp_candidate_adapter:main",
        "needle_flat_gate = suturing_runtime.needle_flat_gate:main",
        "psm_camera_bridge = suturing_runtime.psm_camera_bridge:main",
        "metric_da_depth = suturing_runtime.metric_da_depth:main",
    ]},
)

import os
from glob import glob

from setuptools import find_packages, setup


package_name = "fr3_vision_sorting"


setup(
    name=package_name,
    version="0.0.0",

    packages=find_packages(
        exclude=["test"],
    ),

    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [
                "resource/" + package_name,
            ],
        ),

        (
            os.path.join(
                "share",
                package_name,
            ),
            [
                "package.xml",
            ],
        ),
        # Additional data files for config and launch due to ROS2 package structure

        # Which is (1)config/fixed_grasp/*.yaml and 
        #          (2) config/fixed_pick_place/*.yaml

        # Stage 3 fixed-grasp poses
        (
            os.path.join(
                "share",
                package_name,
                "config",
                "fixed_grasp",
            ),
            glob(
                "config/fixed_grasp/*.yaml"
            ),
        ),

        # Stage 4 fixed pick-and-place poses
        (
            os.path.join(
                "share",
                package_name,
                "config",
                "fixed_pick_place",
            ),
            glob(
                "config/fixed_pick_place/*.yaml"
            ),
        ),


        # ROS 2 launch files
        (
            os.path.join(
                "share",
                package_name,
                "launch",
            ),
            glob(
                "launch/*.launch.py"
            ),
        ),

    ],

    install_requires=[
        "setuptools",
    ],

    zip_safe=True,

    maintainer="Yujie Tang",
    maintainer_email="ytang19@ualberta.ca",

    description="FR3 vision-guided sorting cell",

    license="Apache-2.0",

    tests_require=[
        "pytest",
    ],

    entry_points={
        "console_scripts": [
            "fixed_pose_demo = "
            "fr3_vision_sorting.fixed_pose_demo:main",

            "gripper_control = "
            "fr3_vision_sorting.gripper_control:main",

            "fixed_grasp_demo = "
            "fr3_vision_sorting.fixed_grasp_demo:main",

            "fixed_pick_place_demo = "
            "fr3_vision_sorting.fixed_pick_place_demo:main",

            "automatic_pick_place_demo = "
            "fr3_vision_sorting.automatic_pick_place_demo:main",

            "vision_pick_place_demo="
            "fr3_vision_sorting.vision_pick_place_demo:main",

            "camera_object_localizer = "
            "fr3_vision_sorting.camera_object_localizer:main",
            
            "calibration_board_detector = "
            "fr3_vision_sorting.calibration_board_detector:main",

            'collect_calibration = '
            'fr3_vision_sorting.collect_calibration:main',

            'solve_calibration = '
            'fr3_vision_sorting.solve_calibration:main',


        ],
    },
)

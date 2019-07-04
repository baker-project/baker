#!/usr/bin/env python

CLEAN_PATTERN_STR = '/vacuum_cleaning_module_interface/clean_pattern'
ROOM_EXPLORATION_SERVICE_STR = '/room_exploration/room_exploration_server'
MOVE_BASE_PATH_SERVICE_STR = '/move_base_path'
MOVE_BASE_WALL_FOLLOW_SERVICE_STR = '/move_base_wall_follow'
MOVE_BASE_SERVICE_STR = 'move_base'
START_CLEANING_SERVICE_STR = '/brush_cleaning_module_interface/start_brush_cleaner'
STOP_CLEANING_SERVICE_STR = '/brush_cleaning_module_interface/stop_brush_cleaner'
COVERAGE_MONITOR_DYNAMIC_RECONFIGURE_SERVICE_STR = '/room_exploration/coverage_monitor_server'
STOP_COVERAGE_MONITORING_SERVICE_STR = "/room_exploration/coverage_monitor_server/stop_coverage_monitoring"
START_COVERAGE_MONITORING_SERVICE_STR = "/room_exploration/coverage_monitor_server/start_coverage_monitoring"
RESET_COVERAGE_MONITORING_SERVICE_STR = "/room_exploration/coverage_monitor_server/reset_coverage_monitoring"
RECEIVE_COVERAGE_IMAGE_SERVICE_STR = "/room_exploration/coverage_monitor_server/get_coverage_image"
START_DIRT_DETECTOR_SERVICE_STR = "/dirt_detection_server_preprocessing/activate_detection"
STOP_DIRT_DETECTOR_SERVICE_STR = "/dirt_detection_server_preprocessing/deactivate_detection"
START_TRASH_DETECTOR_SERVICE_STR = "/trash_detector/activate_detection"
STOP_TRASH_DETECTOR_SERVICE_STR = "/trash_detector/deactivate_detection"
TROLLEY_MOVEMENT_SERVICE_STR = ""
TOOL_CHANGING_SERVICE_STR = ""
MAP_ACCESSIBILITY_SERVICE_STR = "map_accessibility_analysis/map_accessibility_analysis/map_perimeter_accessibility_check"

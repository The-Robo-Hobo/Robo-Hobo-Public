from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    '''format:
    <name_node> = Node(
        package="<package_name>",
        executable="<name_node>",
    )'''

    # robo_detection package
    v4l2_camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
    )

    save_image_node = Node(
        package="robo_detection",
        executable="save_image_node",
    )

    evidence_recog_node = Node(
        package="robo_detection",
        executable="evidence_recog_node",
    )

    thermal_publisher_node = Node(
        package="robo_detection",
        executable="thermal_publisher_node",
    )

    facial_recog_node = Node(
        package="robo_detection",
        executable="facial_recog_node",
    )

    flash_node = Node(
        package="robo_detection",
        executable="flash_node",
    )

    # robo_deterrent package
    siren_node = Node(
        package="robo_deterrent",
        executable="siren_node",
    )

    # robo_navigation package
    arduino_comms_node = Node(
        package="robo_navigation",
        executable="arduino_comms_node",
    )

    # robo_notification package
    web_server_node = Node(
        package="robo_notification",
        executable="web_server_node",
    )

    stat_collector_node = Node(
        package="robo_notification",
        executable="stat_collector_node",
    )

    email_notif_node = Node(
        package="robo_notification",
        executable="email_notif_node",
    )

    '''format: 
    ld.add_action(<name_node>)
    '''

    # robo_detection package
    ld.add_action(v4l2_camera_node)
    ld.add_action(save_image_node)
    ld.add_action(evidence_recog_node)
    ld.add_action(thermal_publisher_node)
    ld.add_action(facial_recog_node)
    ld.add_action(flash_node)

    # robo_deterrent package
    ld.add_action(siren_node)

    # robo_navigation package
    ld.add_action(arduino_comms_node)

    # robo_notification package
    ld.add_action(web_server_node)
    ld.add_action(stat_collector_node)
    ld.add_action(email_notif_node)


    return ld

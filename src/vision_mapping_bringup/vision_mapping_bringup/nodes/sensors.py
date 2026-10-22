"""Camera drivers, the F9R IMU and static sensor transforms (launch group: sensors)."""
from vision_mapping_bringup.camera import FRONT_FRAME, REAR_FRAME, monoscale_rear
from vision_mapping_bringup.rig import quaternion_xyzw


def static_tf(ctx, name, xyz, q, child):
    keys = ('--x', '--y', '--z', '--qx', '--qy', '--qz', '--qw')
    arguments = [part for key, value in zip(keys, [*xyz, *q]) for part in (key, str(float(value)))]
    return ctx.make_node(package='tf2_ros', executable='static_transform_publisher', name=name,
                         arguments=arguments + ['--frame-id', 'base_link', '--child-frame-id', child],
                         parameters=[{'use_sim_time': ctx.sim_time}])


def _gscam(ctx, name, device, frame, camera_name, image_topic, info_topic, jpeg_topic):
    """camera_output raw (default): the camera's MJPEG decoded to rgb8 on image_topic.
    camera_output jpeg: the camera's MJPEG frames passed on untouched as CompressedImage on
    jpeg_topic (no decode in the driver; consumers take image_input compressed*)."""
    s = ctx.setup
    caps = f'v4l2src device={device} ! image/jpeg,width={s.width},height={s.height},framerate={s.fps}/1'
    jpeg = camera_output(ctx) == 'jpeg'
    return ctx.make_node(
        package='gscam', executable='gscam_node', name=name, output='screen',
        parameters=[{
            'gscam_config': caps if jpeg else caps + ' ! jpegdec ! videoconvert ! video/x-raw,format=RGB',
            'image_encoding': 'jpeg' if jpeg else 'rgb8', 'frame_id': frame,
            'camera_name': camera_name, 'use_sensor_data_qos': True,
            'sync_sink': False, 'use_sim_time': False,
        }],
        remappings=[('/camera/image_raw', image_topic), ('/camera/image_raw/compressed', jpeg_topic),
                    ('/camera/camera_info', info_topic)])


def camera_output(ctx):
    value = ctx.args.arg('camera_output').strip().lower() or 'raw'
    if value not in ('raw', 'jpeg'):
        raise ValueError('camera_output must be raw or jpeg')
    return value


def drivers(ctx):
    """Front camera and F9R drivers, when launched."""
    nodes = []
    args = ctx.args
    if ctx.launch_camera:
        device = args.device_path('device', 'launch_camera:=true', '/dev/v4l/by-id/usb-<front-camera>-video-index0')
        nodes.append(_gscam(ctx, 'stage1_color_camera', device, FRONT_FRAME, 'front_fisheye',
                            ctx.topics.image, '/sensing/camera/front/fisheye/camera_info',
                            '/measurement/front/image_raw/compressed'))
    if ctx.launch_f9r:
        nodes.append(ctx.make_node(package='monoscale_ublox', executable='f9r_imu',
                                   name='f9r_imu', output='screen', parameters=[{
                                       'device': args.device_path('imu_device', 'launch_f9r:=true',
                                                                  '/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00'),
                                       'frame_id': 'imu_link',
                                       'gyro_yaw_fallback': True, 'gyro_bias_calibration_sec': 2.0,
                                       'gyro_bias_stationary_threshold_rad_s': 0.035,
                                       'gyro_yaw_rate_tau_sec': float(args.arg('imu_yaw_rate_tau_sec')),
                                   }], remappings=[('/sensing/imu/imu_data', ctx.topics.imu)]))
    return nodes


def rear(ctx):
    """Rear camera mount transform and driver (ctx.setup.rear already loaded)."""
    args = ctx.args
    rear_camera = ctx.setup.rear
    nodes = [static_tf(ctx, 'rear_camera_tf', rear_camera.translation,
                       quaternion_xyzw(rear_camera.rotation), REAR_FRAME)]
    if ctx.launch_camera:
        rear_device = args.device_path('rear_device', 'rear_enabled:=true and launch_camera:=true',
                                       '/dev/v4l/by-id/usb-<rear-camera>-video-index0')
        if args.drivers and rear_device == args.arg('device').strip():
            raise ValueError('rear_device must differ from device')
        nodes.append(_gscam(ctx, 'rear_color_camera', rear_device, REAR_FRAME, 'rear_fisheye',
                            '/sensing/camera/rear/fisheye/image_raw',
                            '/sensing/camera/rear/fisheye/camera_info',
                            '/measurement/rear/image_raw/compressed'))
    return nodes


def mount_transforms(ctx):
    s = ctx.setup
    return [static_tf(ctx, 'stage1_camera_tf', s.front.translation, s.quaternion, FRONT_FRAME),
            static_tf(ctx, 'stage1_imu_tf', s.imu_position, [0., 0., 0., 1.], 'imu_link')]


def rear_parameters(ctx):
    return monoscale_rear(ctx.setup.rear)

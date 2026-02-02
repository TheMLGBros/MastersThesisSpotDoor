
class Gaze(SpotServiceBehaviour):
    def __init__(self, name: str, robot, x, y, z) -> None:
        super().__init__(name, robot=robot, client_type=RobotCommandClient)
        self.x = x
        self.y = y
        self.z = z

    def initialise(self) -> None:
        # robot_state = self._robot_state_client.get_robot_state()
        # odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
        #                                  ODOM_FRAME_NAME, BODY_FRAME_NAME)

        # # Look at a point 3 meters in front and 4 meters to the left.
        # # We are not specifying a hand location, the robot will pick one.
        # gaze_target_in_odom = odom_T_flat_body.transform_point(x=1, y=0, z=-0.85)


        # gaze_command = RobotCommandBuilder.arm_gaze_command(gaze_target_in_odom[0],
        #                                                     gaze_target_in_odom[1],
        #                                                     gaze_target_in_odom[2], ODOM_FRAME_NAME)

        gaze_command = RobotCommandBuilder.arm_gaze_command(self.x, self.y, self.z, BODY_FRAME_NAME)
        # Make the open gripper RobotCommand
        # gripper_command = RobotCommandBuilder.claw_gripper_open_command()

        # Combine the arm and gripper commands into one RobotCommand
        synchro_command = RobotCommandBuilder.build_synchro_command(gaze_command)

        # Send the request
        self.robot.logger.info('Requesting gaze.')
        self.gaze_command_id = self.client.robot_command(synchro_command)
    
    def update(self):
        gaze_response = self.client.robot_command_feedback(self.gaze_command_id)
        status = gaze_response.feedback.synchronized_feedback.arm_command_feedback.arm_gaze_feedback.status
        if (status == arm_command_pb2.GazeCommand.Feedback.STATUS_TRAJECTORY_COMPLETE):
            return py_trees.common.Status.SUCCESS 
        elif (status == arm_command_pb2.GazeCommand.Feedback.STATUS_IN_PROGRESS):
            return py_trees.common.Status.RUNNING
        # NOTE: THIS IS NOT A FAILURE, ONLY A STALL. IF THE BODY IS MOVED THEN IT WILL TRY AGAIN.
        return py_trees.common.Status.FAILURE


class WalkToObject(SpotServiceBehaviour):
    def __init__(self, name: str, robot) -> None:
        super().__init__(name, robot=robot, client_type=ManipulationApiClient)
        self.img_client = self.robot.ensure_client(ImageClient.default_service_name)

    def initialise(self):
        def cv_mouse_callback(event, x, y, flags, param):
            global g_image_click, g_image_display
            clone = g_image_display.copy()
            if event == cv2.EVENT_LBUTTONUP:
                g_image_click = (x, y)
            else:
                # Draw some lines on the image.
                #print('mouse', x, y)
                color = (30, 30, 30)
                thickness = 2
                image_title = 'Click to walk up to something'
                height = clone.shape[0]
                width = clone.shape[1]
                cv2.line(clone, (0, y), (width, y), color, thickness)
                cv2.line(clone, (x, 0), (x, height), color, thickness)
                cv2.imshow(image_title, clone)
        # Take a picture with a camera
        image_responses = self.img_client.get_image_from_sources(["frontright_fisheye_image"])

        if len(image_responses) != 1:
            print(f'Got invalid number of images: {len(image_responses)}')
            print(image_responses)
            assert False

        image = image_responses[0]
        if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
            dtype = np.uint16
        else:
            dtype = np.uint8
        img = np.fromstring(image.shot.image.data, dtype=dtype)
        if image.shot.image.format == image_pb2.Image.FORMAT_RAW:
            img = img.reshape(image.shot.image.rows, image.shot.image.cols)
        else:
            img = cv2.imdecode(img, -1)

        # Show the image to the user and wait for them to click on a pixel
        image_title = 'Click to walk up to something'
        cv2.namedWindow(image_title)
        cv2.setMouseCallback(image_title, cv_mouse_callback)

        global g_image_click, g_image_display
        g_image_display = img
        cv2.imshow(image_title, g_image_display)
        while g_image_click is None:
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == ord('Q'):
                # Quit
                print('"q" pressed, exiting.')
                exit(0)

        walk_vec = geometry_pb2.Vec2(x=g_image_click[0], y=g_image_click[1])

        # Build the proto
        walk_to = manipulation_api_pb2.WalkToObjectInImage(
            pixel_xy=walk_vec, transforms_snapshot_for_camera=image.shot.transforms_snapshot,
            frame_name_image_sensor=image.shot.frame_name_image_sensor,
            camera_model=image.source.pinhole)

        # Ask the robot to pick up the object
        walk_to_request = manipulation_api_pb2.ManipulationApiRequest(
            walk_to_object_in_image=walk_to)

        # Send the request
        self.cmd_id = self.client.manipulation_api_command(
            manipulation_api_request=walk_to_request).manipulation_cmd_id
    
    def update(self):
        feedback_request = manipulation_api_pb2.ManipulationApiFeedbackRequest(manipulation_cmd_id=self.cmd_id)

        # Send the request
        response = self.client.manipulation_api_feedback_command(
            manipulation_api_feedback_request=feedback_request)

        print(
            f'Current state: {manipulation_api_pb2.ManipulationFeedbackState.Name(response.current_state)}'
        )

        if response.current_state == manipulation_api_pb2.MANIP_STATE_DONE:
            return py_trees.common.Status.SUCCESS 
        return py_trees.common.Status.RUNNING


class CheckBatteryLevel(SpotServiceBehaviour):
    def __init__(self, name: str, robot, threshold: float = 20) -> None:
        super().__init__(name, robot=robot, client_type=RobotStateClient)
        self.threshold = threshold

    def update(self): # Ska denna vara en update eller en initialise egentligen?
        state = self.client.get_robot_state()
        battery_states = state.battery_states
        if not battery_states:
            self.logger.warning("No battery state reported.")
            return py_trees.common.Status.FAILURE

        battery_level = battery_states[0].charge_percentage.value

        if battery_level < self.threshold:
            self.logger.warning(f"Battery low: {battery_level:.1f}%")
            return py_trees.common.Status.FAILURE
        print(f"Current battery level: {battery_level:.1f}%")

        return py_trees.common.Status.SUCCESS
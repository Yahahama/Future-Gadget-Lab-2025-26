package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.vision.pipelines.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@TeleOp(name="Basic Drive", group="Linear OpMode")
public class drive extends LinearOpMode {
    /*
     * Driving (1):
     *    FLIP                - Right Stick
     * Intake (BOTH):
     *    COLLECT             - A
     *    OUT                 - B
     *    FAST                - X
     *    STOP                - Y
     * Launcher (2):
     *    FAST                - Left Trigger
     *    SLOW                - Right Trigger
     *    BACK                - Left Stick
     * Load (2):
     *    RAISE               - Dpad Up
     *    LOWER               - Dpad Down
     *    RECYCLE             - Dpad Left
     * Bunt (2):
     *    RESET               - Left Trigger
     *    LAUNCH              - Right Trigger
     * Macros (1):
     *    NONE
     * Macros (2):
     *    NONE
     */

    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();

        // Initialize and configure drive motor variables
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "lfMtr"); // CH 2
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "lbMtr"); // CH 3
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "rfMtr"); // CH 0
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "rbMtr"); // CH 1
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize and configure intake motor
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize and configure launch motor
        DcMotorEx launch1 = hardwareMap.get(DcMotorEx.class, "launch1");
        DcMotorEx launch2 = hardwareMap.get(DcMotorEx.class, "launch2");
        launch1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch1.setDirection(DcMotorEx.Direction.FORWARD);
        launch2.setDirection(DcMotorEx.Direction.REVERSE);
        launch1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        launch2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Initialize servos
        Servo load = hardwareMap.get(Servo.class, "load");
        Servo bunt = hardwareMap.get(Servo.class, "bunt");

        // Initialize IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu = hardwareMap.get(IMU.class, "imu");

        // Initialize localizer and robot position variables. Get position constants
        MecanumDrive drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        MecanumDrive.Params parameters = new MecanumDrive.Params();
        double robotAngle = 0;
        YawPitchRollAngles robotOrientation;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewID", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "FGLs Webcam 2025!"), cameraMonitorViewId);
        AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(parameters.TAGSIZE_METERS, parameters.WEBCAM_FOCAL_X, parameters.WEBCAM_FOCAL_Y, parameters.WEBCAM_PRINCIPAL_POINT_X, parameters.WEBCAM_PRINCIPAL_POINT_Y);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        int numFramesWithoutDetection = 0;
        ArrayList<AprilTagDetection> detections = new ArrayList<>();

        // Initialize control parameters
        int intakeDirection = parameters.INTAKE_DIRECTION_START;
        float intakeSpeed = 0f;
        boolean isIntakeCentric = true;
        boolean isLoadUp = false;
        boolean previousDown = false;

        // Robot is ready to start! Display message to screen
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.setMsTransmissionInterval(50);

        //Initialization Actions
        load.setPosition(parameters.LOAD_INIT);
        bunt.setPosition(parameters.BUNT_RESET);

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Get Pose
            drive.updatePoseEstimate();
            Pose2d myPose = drive.pose;

            // Get IMU data
            robotOrientation = imu.getRobotYawPitchRollAngles();

            // What are these used for?
            // Now use these simple methods to extract each angle
            // (Java type double) from the object you just created:
            double Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
            double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
            double Roll = robotOrientation.getRoll(AngleUnit.DEGREES);

            if (myPose != null) robotAngle = myPose.heading.toDouble(); // TODO: Change to right one

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial_target = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral_target = gamepad1.left_stick_x * 1.1;
            double yaw = -gamepad1.right_stick_x;

            double theta = isIntakeCentric ? 180 : Math.toRadians(0); // Change this to use robot-centric soon
            double cosine = Math.cos(theta);
            double sine = Math.sin(theta);

            double lateral_real = lateral_target * cosine - axial_target * sine;
            double axial_real = lateral_target * sine + axial_target * cosine;

            double right_trigger = 1 + gamepad1.right_trigger;
            double left_trigger = 1 - gamepad1.left_trigger;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = ((axial_real + lateral_real + yaw) / 2) * right_trigger * left_trigger;
            double rightFrontPower = ((axial_real - lateral_real - yaw) / 2) * right_trigger * left_trigger;
            double leftBackPower = ((axial_real - lateral_real + yaw) / 2) * right_trigger * left_trigger;
            double rightBackPower = ((axial_real + lateral_real - yaw) / 2) * right_trigger * left_trigger;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            double max = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                    Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Intake
            if (gamepad1.y || gamepad2.y) {
                intakeDirection = 0;
            } else if (gamepad1.a || gamepad2.a) {
                intakeDirection = 1;
                intakeSpeed = parameters.INTAKE_SPEED_IN;
            } else if (gamepad1.b || gamepad2.b) {
                intakeDirection = -1;
                intakeSpeed = parameters.INTAKE_SPEED_OUT;
            } else if (gamepad1.x || gamepad2.x) {
                intakeDirection = 1;
                intakeSpeed = parameters.INTAKE_SPEED_LOAD;
            }

            // Power Intake
            if (intakeDirection == 1) {
                intake.setPower(intakeSpeed);
            } else if (intakeDirection == -1) {
                intake.setPower(intakeSpeed);
            } else {
                intake.setPower(0f);
            }

            // Power Launch
            if (gamepad2.right_trigger > 0.5f) {
                launch1.setVelocity(isLoadUp ? parameters.LAUNCH_SPEED_CLOSE_LOW : parameters.LAUNCH_SPEED_CLOSE_HIGH, AngleUnit.RADIANS);
                launch2.setVelocity(isLoadUp ? parameters.LAUNCH_SPEED_CLOSE_LOW : parameters.LAUNCH_SPEED_CLOSE_HIGH, AngleUnit.RADIANS);
            } else if (gamepad2.left_trigger > 0.5f) {
                launch1.setVelocity(isLoadUp ? parameters.LAUNCH_SPEED_FAR_LOW : parameters.LAUNCH_SPEED_FAR_HIGH, AngleUnit.RADIANS);
                launch2.setVelocity(isLoadUp ? parameters.LAUNCH_SPEED_FAR_LOW : parameters.LAUNCH_SPEED_FAR_HIGH, AngleUnit.RADIANS);
            } else if (gamepad2.left_stick_button) {
                launch1.setVelocity(parameters.LAUNCH_SPEED_DROP, AngleUnit.RADIANS);
                launch2.setVelocity(parameters.LAUNCH_SPEED_DROP, AngleUnit.RADIANS);
            } else {
                launch1.setVelocity(0, AngleUnit.RADIANS);
                launch2.setVelocity(0, AngleUnit.RADIANS);
            }

            // Loader Servo
            if (gamepad2.dpad_up) {
                load.setPosition(parameters.LOAD_LOAD);
                isLoadUp = true;
            } else if (gamepad2.dpad_down) {
                load.setPosition(parameters.LOAD_RESET);
                isLoadUp = false;
            } else if (gamepad2.dpad_right) {
                load.setPosition(parameters.LOAD_RELOAD);
                isLoadUp = false;
            } else if (gamepad2.dpad_left) {
                load.setPosition(parameters.LOAD_FALL);
                isLoadUp = false;
            }

            if (gamepad2.left_bumper) {
                bunt.setPosition(parameters.BUNT_RESET);
            } else if (gamepad2.right_bumper) {
                bunt.setPosition(parameters.BUNT_LAUNCH);
            } else if (gamepad2.left_stick_y > 0.5f) {
                bunt.setPosition(parameters.BUNT_LOAD);
            } else if (gamepad2.left_stick_y < -0.5f) {
                bunt.setPosition(parameters.BUNT_RESET);
            }

            ArrayList<AprilTagDetection> newDetections = aprilTagDetectionPipeline.getDetectionsUpdate();

            if (newDetections != null) {
                if (newDetections.isEmpty()) {
                    numFramesWithoutDetection++;

                    if (numFramesWithoutDetection >= parameters.THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                        aprilTagDetectionPipeline.setDecimation(parameters.DECIMATION_LOW);
                    }
                } else {
                    numFramesWithoutDetection = 0;
                    detections = newDetections;

                    if (newDetections.get(0).pose.z < parameters.THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                        aprilTagDetectionPipeline.setDecimation(parameters.DECIMATION_HIGH);
                    }

                    for (AprilTagDetection detection : newDetections) {
                        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                    }
                }
            }

            // Power Wheels
            if (gamepad1.right_bumper && !detections.isEmpty()) {
                AprilTagDetection detection = detections.get(0);
                Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                double bearing = Math.toDegrees(Math.atan2(detection.pose.x, detection.pose.z));
                if (bearing < 1) {
                    leftFrontDrive.setPower(-0.2f);
                    rightFrontDrive.setPower(0.2f);
                    leftBackDrive.setPower(-0.2f);
                    rightBackDrive.setPower(0.2f);
                } else {
                    leftFrontDrive.setPower(0.2f);
                    rightFrontDrive.setPower(-0.2f);
                    leftBackDrive.setPower(0.2f);
                    rightBackDrive.setPower(-0.2f);
                }
            } else {
                leftFrontDrive.setPower(-leftFrontPower);
                rightFrontDrive.setPower(-rightFrontPower);
                leftBackDrive.setPower(-leftBackPower);
                rightBackDrive.setPower(-rightBackPower);
            }

            if (gamepad1.dpad_down && !previousDown) {
                isIntakeCentric = !isIntakeCentric;
            }
             previousDown = gamepad1.dpad_down;

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime);
            if (myPose != null) {
                telemetry.addData("Position", "x: " + myPose.position.x + "y: " + myPose.position.y);
                telemetry.addData("Heading", "Angle: " + myPose.heading.toDouble());
            }
            telemetry.addData("Intake", "Direction: " + intakeDirection);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Load Up", isLoadUp);
            if (newDetections != null) {
                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
            } else {
                telemetry.addLine("No new frame");
            }
            for (AprilTagDetection detection : detections) {
                Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                telemetry.addData("Detected Tag ID", detection.id);
                telemetry.addData("Translation X: ", detection.pose.x*parameters.FEET_PER_METER);
                telemetry.addData("Translation Y: ", detection.pose.y*parameters.FEET_PER_METER);
                telemetry.addData("Translation Z: ", detection.pose.z*parameters.FEET_PER_METER);
                telemetry.addData("Rotation Yaw: ", rot.firstAngle);
                telemetry.addData("Rotation Pitch: ", rot.secondAngle);
                telemetry.addData("Rotation Roll: ", rot.thirdAngle);
                telemetry.addData("Bearing ", Math.toDegrees(Math.atan2(detection.pose.x, detection.pose.z)));
            }
            telemetry.update();
        }
    }
}
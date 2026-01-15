package org.firstinspires.ftc.teamcode.drive;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.pipelines.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class AimAssist {
    private static final double RED_BEARING = -4.76;
    private static final double BLUE_BEARING = 1;


    private static final MecanumDrive.Params PARAMS = new MecanumDrive.Params();
    private final PIDFController PIDF = new PIDFController(PARAMS.DRIVE_kP, PARAMS.DRIVE_kI, PARAMS.DRIVE_kD, PARAMS.DRIVE_kF, 0, 0);

    private final AprilTagDetectionPipeline aprilTagDetectionPipeline;
    private int numFramesWithoutDetection = 0;

    public AimAssist(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewID", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "FGLs Webcam 2025!"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(PARAMS.TAG_SIZE_METERS, PARAMS.WEBCAM_FOCAL_X, PARAMS.WEBCAM_FOCAL_Y, PARAMS.WEBCAM_PRINCIPAL_POINT_X, PARAMS.WEBCAM_PRINCIPAL_POINT_Y);
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
    }

    public void detect(double heading) {
        ArrayList<AprilTagDetection> newDetections = aprilTagDetectionPipeline.getDetectionsUpdate();

        if (newDetections != null) {
            if (newDetections.isEmpty()) {
                numFramesWithoutDetection++;

                if (numFramesWithoutDetection >= PARAMS.THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                    aprilTagDetectionPipeline.setDecimation(PARAMS.DECIMATION_LOW);
                }
            } else {
                numFramesWithoutDetection = 0;
                processDetections(newDetections, heading);

                if (newDetections.get(0).pose.z < PARAMS.THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                    aprilTagDetectionPipeline.setDecimation(PARAMS.DECIMATION_HIGH);
                }
            }
        }
    }

    public void processDetections(ArrayList<AprilTagDetection> detections, double heading) {
        for (int i = 0; i < detections.size(); i++) {
            AprilTagDetection detection = detections.get(i);
            if (detection.id != 20 && detection.id != 24) {
                continue;
            }
            double bearing = Math.toDegrees(Math.atan2(detection.pose.x, detection.pose.z));
            double threshold = detection.id == 24 ? RED_BEARING : BLUE_BEARING;
            double target = heading + threshold - bearing;
            PIDF.setSetPoint(target);
        }
    }

    public double calculate(double heading) {
        double power = Math.min(PIDF.calculate(heading) * 0.2, 0.5);
        return PIDF.atSetPoint() ? 0 : power;
    }
}

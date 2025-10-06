package org.firstinspires.ftc.teamcode.vision;

import java.util.*;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class Vision extends LinearOpMode
{
    @Override
    public void runOpMode() {
        AprilTagLibrary.Builder myAprilTagLibraryBuilder;
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        AprilTagLibrary myAprilTagLibrary;
        AprilTagProcessor myAprilTagProcessor;

        myAprilTagLibraryBuilder = new AprilTagLibrary.Builder();

        // Add all the tags from the given AprilTagLibrary to the AprilTagLibrary.Builder.
        // Get the AprilTagLibrary for the current season.
        myAprilTagLibraryBuilder.addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());

        // Add a tag, without pose information, to the AprilTagLibrary.Builder.
        myAprilTagLibraryBuilder.addTag(6, "A page with tag 6 in our book!", 8.5, DistanceUnit.INCH);

        myAprilTagLibrary = myAprilTagLibraryBuilder.build();

        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();

        myAprilTagProcessorBuilder.setTagLibrary(myAprilTagLibrary);

        // Build the AprilTag processor and assign it to a variable.
        myAprilTagProcessor = myAprilTagProcessorBuilder.build();

        VisionPortal myVisionPortal;

        // Create a VisionPortal, with the specified camera and AprilTag processor, and assign it to a variable.
        myVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "FGLs Webcam 2025!"))
                .addProcessor(myAprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
//                .enableCameraMonitoring(true)  // this method isn't recognized for some reason
                .setAutoStopLiveView(true)
                .build();

        List<AprilTagDetection> myAprilTagDetections;

        String pattern = "";
        int redGoalID = 24;
        int blueGoalID = 20;

        waitForStart();

        while (opModeIsActive()) {
            myAprilTagDetections = myAprilTagProcessor.getDetections();
            int myAprilTagIdCode;
            for (AprilTagDetection myAprilTagDetection : myAprilTagDetections) {
                myAprilTagIdCode = myAprilTagDetection.id;

                if (myAprilTagIdCode == redGoalID || myAprilTagIdCode == blueGoalID) {
                    // Now take action based on this tag's ID code, or store info for later action.
                    double myTagPoseX = myAprilTagDetection.ftcPose.x;
                    double myTagPoseY = myAprilTagDetection.ftcPose.y;
                    double myTagPoseZ = myAprilTagDetection.ftcPose.z;
                    double myTagPosePitch = myAprilTagDetection.ftcPose.pitch;
                    double myTagPoseRoll = myAprilTagDetection.ftcPose.roll;
                    double myTagPoseYaw = myAprilTagDetection.ftcPose.yaw;

                    double myTagPoseRange = myAprilTagDetection.ftcPose.range;
                    double myTagPoseBearing = myAprilTagDetection.ftcPose.bearing;
                    double myTagPoseElevation = myAprilTagDetection.ftcPose.elevation;

                } else if (pattern.isEmpty()) {  // The tag spotted is an OBELISK tag and we don't know the pattern yet
                    switch (myAprilTagIdCode) {
                        case 21:
                            pattern = "GPP";
                            break;
                        case 22:
                            pattern = "PGP";
                            break;
                        case 23:
                            pattern = "PPG";
                            break;
                        default:
                            break; /// What the helly? We've seen a tag that isn't a GOAL or OBELISK
                    }
                }
            }
        }
    }
}
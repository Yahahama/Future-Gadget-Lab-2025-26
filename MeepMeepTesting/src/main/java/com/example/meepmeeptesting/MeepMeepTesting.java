package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {

    // This "Positions" class is copied from Autonomous.java and should be kept up to date
    // by copy-pasting. If you can figure out how to just import Positions from Autonomous,
    // please do that.

    //IMPORTANT: READ THE ANNOTATIONS FOR THE ENUMS
    public static class Positions {

        //Note: These are APPROXIMATE POSITIONS

        enum GOAL {
            RED(new Pose2d(-64, 57, Math.toRadians(135))),
            BLUE(new Pose2d(-64, -57, Math.toRadians(225)));

            private Pose2d pose2d;

            GOAL(Pose2d _pose2d) {
                this.pose2d = _pose2d;
            }

            public Pose2d getPose() {
                return pose2d;
            }
        }

        enum START {
            RED(new Pose2d(48, -48, Math.toRadians(135))),
            BLUE(new Pose2d(-48, -48, Math.toRadians(225)));

            private Pose2d pose2d;

            START(Pose2d _pose2d) {
                this.pose2d = _pose2d;
            }

            public Pose2d getPose() {
                return pose2d;
            }
        }

        enum START {

            //RED_CLOSE means the starting position CLOSE TO THE GOAL

            RED_CLOSE(new Pose2d(-49.3f, 50.3f, Math.toRadians(315))),
            RED_FAR(new Pose2d(62, 16, Math.toRadians(180))), //y pos + 12???
            BLUE_CLOSE(new Pose2d(-49.3f, -50.3f, Math.toRadians(45))),
            BLUE_FAR(new Pose2d(62, -16, Math.toRadians(180))); //y pos - 12???

            private Pose2d pose2d;

            START(Pose2d _pose2d) {
                this.pose2d = _pose2d;
            }

            public Pose2d getPose() {
                return pose2d;
            }
        }
        enum ARTIFACT {

            //+-5 for the other artifacts near the middle one

            RED_A(new Pose2d(-11.5f, 46.5f, Math.toRadians(0))),
            RED_B(new Pose2d(12.3f, 46.5f, Math.toRadians(0))),
            RED_C(new Pose2d(36, 46.5f, Math.toRadians(0))),
            BLUE_A(new Pose2d(-11.5f, -46.5f, Math.toRadians(0))),
            BLUE_B(new Pose2d(12.3f, -46.5f, Math.toRadians(0))),
            BLUE_C(new Pose2d(36, -46.5f, Math.toRadians(0)));

            private Pose2d pose2d;

            ARTIFACT(Pose2d _pose2d) {
                this.pose2d = _pose2d;
            }

            public Pose2d getPose() {
                return pose2d;
            }
        }
    }
    enum PARKING {

        //+-5 for the other artifacts near the middle one

        PARK_RED(new Pose2d(38.25f, 32, Math.toRadians(0))),
        PARK_BLUE(new Pose2d(38.25f, -32, Math.toRadians(0)));

        private Pose2d pose2d;

        PARKING(Pose2d _pose2d) {
            this.pose2d = _pose2d;
        }

        public Pose2d getPose() {
            return pose2d;
        }
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        double secondsToWait = 1;

        RoadRunnerBotEntity robot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .build();

        // Blue close
//        Action blueBucket = robot.getDrive().actionBuilder(new Pose2d(35, 62, 0))
//                .setTangent(Math.toRadians(0))
//                .splineToLinearHeading(Positions.BUCKET_BLUE, Math.toRadians(315))
//                .setTangent(Math.toRadians(225))
//                .splineToLinearHeading(Positions.SAMPLE_NEUTRAL_BLUE_FAR, Math.toRadians(-80))
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(Positions.BUCKET_BLUE, Math.toRadians(45))
//                .setTangent(Math.toRadians(225))
//                .splineToLinearHeading(Positions.SAMPLE_NEUTRAL_BLUE_MIDDLE, Math.toRadians(-90))
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(Positions.BUCKET_BLUE, Math.toRadians(45))
//                .setTangent(Math.toRadians(225))
//                .splineToLinearHeading(Positions.SAMPLE_NEUTRAL_BLUE_CLOSE, Math.toRadians(-60))
//                .setTangent(Math.toRadians(120))
//                .splineToLinearHeading(Positions.BUCKET_BLUE, Math.toRadians(45))
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(26, 10, Math.toRadians(180)), Math.toRadians(270))
//                .build();
//
//        robot.runAction(blueBucket);

//        Action toRedBucketToBlueBucket = robot.getDrive().actionBuilder(new Pose2d(33, 38, 0))
//            .splineToLinearHeading(Positions.GOAL.RED, Math.toRadians(45)))
//            .build();

        Image img = null;
        try { img = ImageIO.read(new File("/Users/abibolov27/Documents/Images/Robotics/DecodeMeepMeepBackground.png")); }
        catch (IOException e) {}

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(robot)
                .start();
    }
}
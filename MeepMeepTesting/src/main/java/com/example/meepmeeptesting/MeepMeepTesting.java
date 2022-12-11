
package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static double location = 0;
    public static double startPoseX = 34;
    public static double startPoseY = -61;
    public static double depositX = 34;
    public static double depositY = -13;
    public static double startAngle = Math.toRadians(0);
    public static double depositAngle = Math.toRadians(0);
    public static double loadAngle = Math.toRadians(0);
    public static double parkAngle = Math.toRadians(0);
    public static double loadX = 55.5;
    public static double loadY = -12;
    public static double parkX = 34 + 24 * (location - 1);
    public static double parkY = depositY;
    public static void main(String[] args) throws InterruptedException {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(58.3191798, 50, 4.6055, 4.6055, 12)
                .setDimensions(14,17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(startPoseX, startPoseY, startAngle))
                                .lineToLinearHeading(new Pose2d(depositX, depositY, depositAngle))
                                .lineToLinearHeading(new Pose2d(loadX, loadY, loadAngle))
                                .lineToLinearHeading(new Pose2d(depositX, depositY, depositAngle))
                                .lineToLinearHeading(new Pose2d(loadX, loadY, loadAngle))
                                .lineToLinearHeading(new Pose2d(depositX, depositY, depositAngle))
                                .lineToLinearHeading(new Pose2d(loadX, loadY, loadAngle))
                                .lineToLinearHeading(new Pose2d(depositX, depositY, depositAngle))
                                .lineToLinearHeading(new Pose2d(loadX, loadY, loadAngle))
                                .lineToLinearHeading(new Pose2d(depositX, depositY, depositAngle))
                                .lineToLinearHeading(new Pose2d(loadX, loadY, loadAngle))
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(parkX, parkY, depositAngle))
                                .waitSeconds(5)
                                .build()





                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
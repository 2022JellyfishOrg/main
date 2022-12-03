
package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    static int signalPosUpdate = -1;
    public static int startPoseX = 34;
    public static int startPoseY = -61;
    public static int depositX = 28;
    public static int depositY = -12;
    public static int depositAngle = -45;
    public static int loadAngle = 0;
    public static int loadX = 56;
    public static int loadY = -12;
    public static int parkX = 34 + 24 * signalPosUpdate;
    public static int parkY = depositY;
    public static void main(String[] args) throws InterruptedException {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(58.3191798, 50, 4.6055, 4.6055, 12)
                .setDimensions(14,17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(startPoseX, startPoseY, 0))
                                .lineTo(new Vector2d(startPoseX + 1, depositY - 5))
                                .lineToLinearHeading(new Pose2d(depositX, depositY, depositAngle))
                                .waitSeconds(3)
                                .lineToLinearHeading(new Pose2d(loadX, loadY, loadAngle))
                                .waitSeconds(1.5)
                                .lineToLinearHeading(new Pose2d(depositX, depositY, depositAngle))
                                .waitSeconds(3)
                                .lineToLinearHeading(new Pose2d(loadX, loadY, loadAngle))
                                .waitSeconds(1.5)
                                .lineToLinearHeading(new Pose2d(depositX, depositY, depositAngle))
                                .waitSeconds(3)
                                .lineToLinearHeading(new Pose2d(36, parkY, 0))
                                .lineTo(new Vector2d(parkX, parkY))
                                .build()


                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
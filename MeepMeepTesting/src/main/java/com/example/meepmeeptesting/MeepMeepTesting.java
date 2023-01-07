
package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static double startPoseX = 34;
    public static double startPoseY = -61;
    public static double depositX = 33;
    public static double depositY = -14;
    public static double startAngle = Math.toRadians(0);
    public static double depositAngle = Math.toRadians(-45);
    public static double loadAngle = Math.toRadians(0);
    public static double parkAngle = Math.toRadians(0);
    public static double loadX = 57;
    public static double loadY = -11.5;
    public static double parkX = 33;
    public static double parkY = -14;

    public static void main(String[] args) throws InterruptedException {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(58.3191798, 20, 4.6055, 2, 12)
                .setDimensions(14.5,18)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35.5, -61, 0))
                                //PARKING: ALL ARE FROM DEPOSIT

                                //Parking Spot 1
                                .lineToSplineHeading(new Pose2d(34, -10, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(12, -12), Math.toRadians(200))

                                //Parking Spot 2
                                .lineToSplineHeading(new Pose2d(36, -12, Math.toRadians(0)))

                                //Parking Spot 3, although I think it also decelerates
                                .lineToSplineHeading(new Pose2d(40, -12, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(59,-12), Math.toRadians(0))
                                .build()





                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
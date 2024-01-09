package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.5, 52.5, 142.9, Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(15 - 54, 60+3, Math.toRadians(270)))
                                .splineToLinearHeading(new Pose2d(16 - 54, 36, Math.toRadians(0)), Math.toRadians(225)) // used to be 225
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                })
                                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                                })
                                .waitSeconds(1)
                                .back(6)
                                .strafeLeft(20)
                                .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(180)), Math.toRadians(0))
                                .setTangent(0)
                                .splineToLinearHeading(new Pose2d(50, 40, Math.toRadians(180)), Math.toRadians(-45))
                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setAxesInterval(10)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
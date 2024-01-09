package com.example.meepmeeptesting.LeftBlue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class CenterTapeLB {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10.5)

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(15, 61, Math.toRadians(90)))
                                //Moving onto center tape
                                .lineToConstantHeading(new Vector2d(11, 35.5))
                                .waitSeconds(2)
                                //Moving away from center tape
                                .lineToConstantHeading(new Vector2d(11, 45))
                                .waitSeconds(.5)
                                //going to backboard
                                .lineToLinearHeading(new Pose2d(54, 35, Math.toRadians(180)))
                                .waitSeconds(.5)
                                //Moving away from backboard
                                .lineToConstantHeading(new Vector2d(40, 35))
                                .waitSeconds(1)
                                //Moving towards park position
                                .lineToLinearHeading(new Pose2d(40, 57,Math.toRadians(270)))
                                .waitSeconds(.5)
                                //Parking
                                .lineToConstantHeading(new Vector2d(49, 57))

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
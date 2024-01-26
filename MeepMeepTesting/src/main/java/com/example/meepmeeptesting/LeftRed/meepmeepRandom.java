package com.example.meepmeeptesting.LeftRed;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


    public class meepmeepRandom {
        public static void main(String[] args) {
            MeepMeep meepMeep = new MeepMeep(700);

            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10.5)

                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                                    .addDisplacementMarker(() -> {

                                    })

                                    //Move away from wall
                                    .lineToConstantHeading(new Vector2d(-36, 55))
                                    //Push to tape
                                    .lineToConstantHeading(new Vector2d(-36, 31))

                                    //Move away from tape
                                    .lineToConstantHeading(new Vector2d(-36,35))

                                    //Moving to gate
                                    .lineToConstantHeading(new Vector2d(-44,35))
                                    .lineToLinearHeading(new Pose2d(-49, 14, Math.toRadians(180)))

                                    //Passing through gate
                                    .lineToConstantHeading(new Vector2d(39, 14))
                                    //Set slides, arm, and wrist to outtake position
                                    .addDisplacementMarker( () -> {

                                    })

                                    //Lining up in front of back board
                                    .splineToConstantHeading(new Vector2d(43, 39.5), Math.toRadians(300))
                                    .lineToConstantHeading(new Vector2d(52, 39.5))
                                    //Score
                                    .addDisplacementMarker( () -> {

                                    })


                                    .lineToConstantHeading(new Vector2d(51.8, 39.5))
                                    //Move slides to score
                                    .addDisplacementMarker(() -> {

                                    })


                                    //Move behind in front of board
                                    .splineToConstantHeading(new Vector2d(35, 10), Math.toRadians(180))

                                    //Set to initialization position
                                    .addDisplacementMarker(() -> {


                                    })
                                    .lineToConstantHeading(new Vector2d(-55,12 ))
                                    .addDisplacementMarker(() -> {

                                    })

                                    .lineToLinearHeading(new Pose2d(-60, 9 ,Math.toRadians(225)))
                                    .lineToLinearHeading(new Pose2d(-60, 12 ,Math.toRadians(180)))

                                    .build()
                    );

            meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        }
    }


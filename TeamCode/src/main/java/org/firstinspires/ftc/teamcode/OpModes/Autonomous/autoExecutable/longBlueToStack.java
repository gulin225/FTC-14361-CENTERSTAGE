package org.firstinspires.ftc.teamcode.OpModes.Autonomous.autoExecutable;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.OpModes.Autonomous.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.drive.SampleMecanumDrive;

import java.util.Arrays;

@Autonomous(name = "longBlueToStack", group = "Auto")
public class longBlueToStack extends LinearOpMode {
    OpenCvCamera camera;
    HSVBlueDetection blueDetection;
    String webcamName;
    Robot bot;


    public void runOpMode() {
        bot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-31, 61, Math.toRadians(90));

        initCam();
        drive.setPoseEstimate(startPose);

        // ---------------------------- Left ---------------------------- //

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                //Initialization
                .addDisplacementMarker(() -> {
                    bot.setLidPosition(lidState.close);
                })

                //Move away from wall
                .lineToConstantHeading(new Vector2d(-34, 55))


                //Move to tape
                .lineToLinearHeading(new Pose2d(-32,40,Math.toRadians(150)))
                //Push onto tape
                .lineToLinearHeading(new Pose2d(-34,42,Math.toRadians(150)))
                //Move away from tape
                .lineToLinearHeading(new Pose2d(-45,36,Math.toRadians(90)))
                //Move behind gate
                .lineToLinearHeading(new Pose2d(-45,14,Math.toRadians(180)))
                //Pass through gate
                .lineToConstantHeading(new Vector2d(39, 14))

                //Set slides, arm, and wrist to outtake position
                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                    bot.setOuttakeSlidePosition(outtakeSlidesState.LOWOUT, extensionState.extending);
                })

                //Lining up in front of back board
                .lineToConstantHeading(new Vector2d(40, 43))
                //Line up with back board
                .lineToConstantHeading(new Vector2d(52, 43))



                //Score pixel
                .addDisplacementMarker(() -> {
                    bot.setLidPosition(lidState.open);
                })

                .lineToConstantHeading(new Vector2d(51.8, 43))

                //Move slides to score
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.MEDIUMOUT, extensionState.extending);
                })

                //Move behind in front of board
                .lineToConstantHeading(new Vector2d(40,43))

                //Set to initialization position
                .addDisplacementMarker(() -> {
                    bot.setWristPosition(wristState.intaking);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                })

                //Move closer to park position
                .lineToConstantHeading(new Vector2d(40,39))
                //Rotate to parking position
                .lineToLinearHeading(new Pose2d(44 ,12, Math.toRadians(270)))
                //Move in front of parking position
                .lineToConstantHeading(new Vector2d(45, 12))
                //Park
                .lineToConstantHeading(new Vector2d(52,12))

                .build();

        // ---------------------------- Center ---------------------------- //

        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
                //Initialization
                .addDisplacementMarker(() -> {
                    bot.setLidPosition(lidState.close);
                })

                //Move away from wall
                .lineToConstantHeading(new Vector2d(-36, 55))
                //Push to tape
                .lineToConstantHeading(new Vector2d(-36, 31))
                //Move away from tape
                .lineToConstantHeading(new Vector2d(-36,39))
                //Move left of tape
                .lineToConstantHeading(new Vector2d(-40,42))
                //Moving to gate
                .lineToConstantHeading(new Vector2d(-49,42))
                //Lining up with gate
                .lineToLinearHeading(new Pose2d(-49,14,Math.toRadians(180)))
                //Passing through gate
                .lineToConstantHeading(new Vector2d(39, 14))

                //Set slides, arm, and wrist to outtake position
                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                    bot.setOuttakeSlidePosition(outtakeSlidesState.LOWOUT, extensionState.extending);
                })

                //Lining up in front of back board
                .lineToConstantHeading(new Vector2d(40, 39.5))
                //Lining up with back board
                .lineToConstantHeading(new Vector2d(52, 39.5))

                //Score
                .addDisplacementMarker( () -> {
                    bot.setLidPosition(lidState.open);
                })


                .lineToConstantHeading(new Vector2d(51.8, 39.5))
                //Move slides to score
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.MEDIUMOUT, extensionState.extending);
                })

                //Move behind in front of board
                .splineToConstantHeading(new Vector2d(35, 10), Math.toRadians(0))

                //Set to initialization position
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);

                })
                .lineToConstantHeading(new Vector2d(-55,12 ))
                .addDisplacementMarker(() -> {
                    bot.linkage.setLinkageCustomPosition(.88);
                    bot.activeIntake.setActiveIntakePower(.4);
                })

                .lineToLinearHeading(new Pose2d(-60, 9 ,Math.toRadians(225)))
                .lineToLinearHeading(new Pose2d(-60, 12 ,Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    bot.linkage.setLinkageCustomPosition(.96);
                    bot.setActiveIntakePosition(activeIntakeState.active);
                })
                .waitSeconds(.5)
                .lineToConstantHeading(new Vector2d(-55,12 ))
                .addDisplacementMarker(() -> {
                    bot.setActiveIntakePosition(activeIntakeState.activeReverse);
                })
                .waitSeconds(.25)
                .lineToConstantHeading(new Vector2d(-54.99,12))
                .addDisplacementMarker(() -> {
                    bot.setActiveIntakePosition(activeIntakeState.inactive);

                })
                .lineToConstantHeading(new Vector2d(48,12))
                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                    bot.setOuttakeSlidePosition(outtakeSlidesState.LOWOUT, extensionState.extending);
                })
                .lineToConstantHeading(new Vector2d(52, 39.5))
                //Score
                .addDisplacementMarker( () -> {
                    bot.setLidPosition(lidState.open);
                })


                .lineToConstantHeading(new Vector2d(51.8, 39.5))
                //Move slides to score
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.MEDIUMOUT, extensionState.extending);
                })
                .lineToConstantHeading(new Vector2d(50, 39.5))
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);

                })
                .build();
        TrajectorySequence centerSplines = drive.trajectorySequenceBuilder(startPose)
                //Initialization
                .addDisplacementMarker(() -> {
                    bot.setLidPosition(lidState.close);
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
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                    bot.setOuttakeSlidePosition(outtakeSlidesState.LOWOUT, extensionState.extending);
                })

                //Lining up in front of back board
                .splineToConstantHeading(new Vector2d(43, 39.5), Math.toRadians(300))
                //Score

                .lineToConstantHeading(new Vector2d(52, 39.5))
                .addDisplacementMarker( () -> {
                    bot.setLidPosition(lidState.open);
                })

                .lineToConstantHeading(new Vector2d(51.8, 39.5))
                //Move slides to score
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.MEDIUMOUT, extensionState.extending);
                })
                .waitSeconds(.25)
                //Move behind in front of board
                .splineToConstantHeading(new Vector2d(35, 10), Math.toRadians(180))

                //Set to initialization position
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);

                })
                .lineToConstantHeading(new Vector2d(-55,12 ))
                .addDisplacementMarker(() -> {
                    bot.linkage.setLinkageCustomPosition(.88);
                    bot.activeIntake.setActiveIntakePower(.4);
                })

                .lineToLinearHeading(new Pose2d(-60, 9 ,Math.toRadians(225)))
                .lineToLinearHeading(new Pose2d(-60, 12 ,Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    bot.linkage.setLinkageCustomPosition(.96);
                    bot.setActiveIntakePosition(activeIntakeState.active);
                })
                .waitSeconds(.5)
                .lineToConstantHeading(new Vector2d(-55,12 ))
                .addDisplacementMarker(() -> {
                    bot.setActiveIntakePosition(activeIntakeState.activeReverse);
                })
                .waitSeconds(.25)
                .lineToConstantHeading(new Vector2d(-54.99,12))
                .addDisplacementMarker(() -> {
                    bot.setActiveIntakePosition(activeIntakeState.inactive);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setWristPosition(wristState.init);

                })
                .lineToConstantHeading(new Vector2d(48,12))
                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                    bot.setOuttakeSlidePosition(outtakeSlidesState.LOWOUT, extensionState.extending);
                })
                .lineToConstantHeading(new Vector2d(52, 39.5))
                //Score
                .addDisplacementMarker( () -> {
                    bot.setLidPosition(lidState.open);
                })


                .lineToConstantHeading(new Vector2d(51.8, 39.5))
                //Move slides to score
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.MEDIUMOUT, extensionState.extending);
                })
                .lineToConstantHeading(new Vector2d(50, 39.5))
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);

                })
                .build();




                //Rotate to park position
//                .lineToLinearHeading(new Pose2d(44 ,15, Math.toRadians(270)))
//                //Move in front of parking position
//                .lineToConstantHeading(new Vector2d(45, 12))
//                //Park
//                .lineToConstantHeading(new Vector2d(52,12))
//
//                .build();

        // ---------------------------- Right ---------------------------- //

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                //Initialization
                .addDisplacementMarker(() -> {
                    bot.setLidPosition(lidState.close);
                })

                //Move to tape
                .lineToConstantHeading(new Vector2d(-45,52))
                //Push to tape
                .lineToConstantHeading(new Vector2d(-45, 42))
                //Move away from tape
                .lineToConstantHeading(new Vector2d(-45, 50))
                //Move to center
                .lineToConstantHeading(new Vector2d(-32,50))
                //Align with gate
                .lineToLinearHeading(new Pose2d(-32,12, Math.toRadians(180)))
                //Move through gate
                .lineToConstantHeading(new Vector2d(40,12))
                //Lining up in front of back board
                .lineToConstantHeading(new Vector2d(35,34))

                //Set slides, arm, and wrist to outtake position
                .addDisplacementMarker(() -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                    bot.setOuttakeSlidePosition(outtakeSlidesState.LOWOUT, extensionState.extending);
                })


                //Line up with back board
                .lineToConstantHeading(new Vector2d(54,34))

                //Score pixel
                .addDisplacementMarker(() -> {
                    bot.setLidPosition(lidState.open);
                })

                .lineToConstantHeading(new Vector2d(53.8,34))

                //Move slides to score
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.MEDIUMOUT,extensionState.extending);
                })

                //Move behind in front of board
                .lineToConstantHeading(new Vector2d(48,33.5))

                //Set back to initialization position
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })

                //Move back more
                .lineToConstantHeading(new Vector2d(45, 30))
                //Rotate to park position
                .lineToLinearHeading(new Pose2d(46 ,12, Math.toRadians(270)))
                //Move in front of park position
                .lineToConstantHeading(new Vector2d(45, 12))
                //Park
                .lineToConstantHeading(new Vector2d(52,12))

                .build();

        // ---------------------------- Camera Initialization ---------------------------- //

        waitForStart();
        camera.stopStreaming();
        if (isStopRequested()) return;

        switch (blueDetection.getLocation())
        {
            case LEFT:
                drive.setPoseEstimate(startPose);
                bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                bot.setOuttakeSlideState(outtakeSlidesState.STATION);
                bot.setArmState(armState.intaking);
                bot.setArmPosition(armState.intaking, armExtensionState.extending);
                bot.setWristPosition(wristState.intaking);
                bot.setWristState(wristState.intaking);

                drive.followTrajectorySequence(left);

                break;
            case RIGHT:
                drive.setPoseEstimate(startPose);
                bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                bot.setOuttakeSlideState(outtakeSlidesState.STATION);
                bot.setArmState(armState.intaking);
                bot.setArmPosition(armState.intaking, armExtensionState.extending);
                bot.setWristPosition(wristState.intaking);
                bot.setWristState(wristState.intaking);

                drive.followTrajectorySequence(right);

                break;
            case MIDDLE:
                drive.setPoseEstimate(startPose);
                bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                bot.setOuttakeSlideState(outtakeSlidesState.STATION);
                bot.setArmState(armState.intaking);
                bot.setArmPosition(armState.intaking, armExtensionState.extending);
                bot.setWristPosition(wristState.intaking);
                bot.setWristState(wristState.intaking);

                drive.followTrajectorySequence(centerSplines);

                break;
        }
    }

    private void initCam() {

        //This line retrieves the resource identifier for the camera monitor view. The camera monitor view is typically used to display the camera feed
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcamName = "Webcam 1";

        // This line creates a webcam instance using the OpenCvCameraFactor with the webcam name (webcamName) and the camera monitor view ID.
        // The camera instance is stored in the camera variable that we can use later
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);

        // initializing our Detection class (details on how it works at the top)
        blueDetection = new HSVBlueDetection(telemetry);

        // yeah what this does is it gets the thing which uses the thing so we can get the thing
        /*
        (fr tho idk what pipeline does, but from what I gathered,
         we basically passthrough our detection into the camera
         and we feed the streaming camera frames into our Detection algorithm)
         */
        camera.setPipeline(blueDetection);

        /*
        this starts the camera streaming, with 2 possible combinations
        it starts streaming at a chosen res, or if something goes wrong it throws an error
         */
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.showFpsMeterOnViewport(true);
                camera.startStreaming(320, 240, OpenCvCameraRotation.SENSOR_NATIVE);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Unspecified Error Occurred; Camera Opening");
            }
        });
    }
}





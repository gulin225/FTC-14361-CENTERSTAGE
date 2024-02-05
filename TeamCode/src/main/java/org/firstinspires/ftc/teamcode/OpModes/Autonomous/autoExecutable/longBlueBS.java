package org.firstinspires.ftc.teamcode.OpModes.Autonomous.autoExecutable;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Commands.activeIntakeState;
import org.firstinspires.ftc.teamcode.Commands.armExtensionState;
import org.firstinspires.ftc.teamcode.Commands.armState;
import org.firstinspires.ftc.teamcode.Commands.extensionState;
import org.firstinspires.ftc.teamcode.Commands.lidState;
import org.firstinspires.ftc.teamcode.Commands.outtakeSlidesState;
import org.firstinspires.ftc.teamcode.Commands.wristState;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.autoExecutable.HSVRedDetection;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Autonomous(name = "longBlueBS", group = "goobTest")
public class longBlueBS extends LinearOpMode {
    OpenCvCamera camera;
    HSVBlueDetection blueDetection;
    String webcamName;
    Robot bot;
    double xOffset = 3;
    double yOffset = 3;


    public void runOpMode() {
        bot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-33, 61, Math.toRadians(90));

        drive.setPoseEstimate(startPose);
        //left ------------------------------------------------------------------
//        TrajectorySequence toTapeDropOff = drive.trajectorySequenceBuilder(startPose)
//
//                //Push to tape
//                //Move away from tape

//
//                .build();
        TrajectorySequence centerUnderTruss = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-41, 55))
                .addDisplacementMarker(3, () -> {
                    bot.setLidPosition(lidState.close);
                })
                .lineToConstantHeading(new Vector2d(-41, 34))
                .lineToConstantHeading(new Vector2d(-41, 38))
                .waitSeconds(1)
                .turn(Math.toRadians(90))
                .addDisplacementMarker(55, () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                })

                .lineToConstantHeading(new Vector2d(40, 37))

                .build();
        TrajectorySequence toCenterBackboard = drive.trajectorySequenceBuilder(centerUnderTruss.end())

                .lineToConstantHeading(new Vector2d(51.75, 37))
                .addDisplacementMarker(3, () -> {
                    bot.outtakeSlide.setPosition(700);
                })
                .addDisplacementMarker(() -> {
                    bot.setLidPosition(lidState.open);
                    bot.outtakeSlide.setPosition(800);
                })
                .waitSeconds(.1)
                .lineToConstantHeading(new Vector2d(45, 37))
                .addDisplacementMarker(25, () -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                })
                .lineToLinearHeading(new Pose2d(47, 55, Math.toRadians(270)))


                .build();
        TrajectorySequence toCenterStack = drive.trajectorySequenceBuilder(toCenterBackboard.end())
                .lineToLinearHeading(new Pose2d(-57, 43, Math.toRadians(230)))
                .addDisplacementMarker(10, () -> {

                    bot.linkage.setLinkageCustomPosition(.96);
                    bot.setActiveIntakePosition(activeIntakeState.active);
                })
                .forward(5.5)

                .turn(Math.toRadians(-93.5))
                .waitSeconds(.1)
                .forward(8)
                .lineToLinearHeading(new Pose2d(-41, 39, Math.toRadians(180)))

                .build();
        TrajectorySequence toBackboardFromStack = drive.trajectorySequenceBuilder(toCenterStack.end())
                .lineToConstantHeading(new Vector2d(40, 39))
                .addDisplacementMarker(5, () -> {
                    bot.setActiveIntakePosition(activeIntakeState.inactive);
                })
                .addDisplacementMarker(20, () -> {

                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setWristPosition(wristState.init);
                    bot.setLidPosition(lidState.close);

                })
                .addDisplacementMarker(55, () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                })
                .lineToConstantHeading(new Vector2d(45, 42))
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.MEDIUMOUT, extensionState.extending);
                })
                .lineToConstantHeading(new Vector2d(52, 42))
                .addDisplacementMarker(() -> {
                    bot.setLidPosition(lidState.open);
                })
                .lineToConstantHeading(new Vector2d(46, 42))
                .addDisplacementMarker(() -> {
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToLinearHeading(new Pose2d(46, 57, Math.toRadians(270)))

                .lineToConstantHeading(new Vector2d(52, 57))

                .build();
        TrajectorySequence leftUnderTruss = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(1.5, () -> {
                    bot.setLidPosition(lidState.close);
                })

                //Move away from wall
                .lineToConstantHeading(new Vector2d(-39, 55))


                //Move to tape
                .lineToLinearHeading(new Pose2d(-39, 35, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(-34.5, 35))
                .lineToConstantHeading(new Vector2d(-39, 35))
                .lineToConstantHeading(new Vector2d(-39, 61))
                .addDisplacementMarker(55, () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                })
                .lineToConstantHeading(new Vector2d(40, 61))
                .build();
        TrajectorySequence toLeftBackboard = drive.trajectorySequenceBuilder(leftUnderTruss.end())
                .lineToConstantHeading(new Vector2d(52, 45))

                .addDisplacementMarker(2, () -> {
                    bot.outtakeSlide.setPosition(650);
                })
                .addDisplacementMarker(() -> {
                    bot.setLidPosition(lidState.open);
                    bot.setOuttakeSlidePosition(outtakeSlidesState.MEDIUMOUT, extensionState.extending);

                })
                .lineToConstantHeading(new Vector2d(45, 45))
                .addDisplacementMarker(() -> {
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToLinearHeading(new Pose2d(45, 55, Math.toRadians(270)))
                .build();
        TrajectorySequence rightUnderTruss = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    bot.setLidPosition(lidState.close);
                })

                //Move to tape
                .lineToConstantHeading(new Vector2d(-45,50))
                //Push to tape
                .lineToConstantHeading(new Vector2d(-46, 42))
                //Move away from tape
                .lineToConstantHeading(new Vector2d(-45, 47.5))
                .addDisplacementMarker( 55, () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                })
                .lineToLinearHeading(new Pose2d(-39,61,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(35, 61))
                .build();

        TrajectorySequence toRightBackboard = drive.trajectorySequenceBuilder(rightUnderTruss.end())
                .lineToConstantHeading(new Vector2d(45, 43.5))

                //Set slides, arm, and wrist to outtake position
                .addDisplacementMarker(3,() -> {
                    bot.outtakeSlide.setPosition(650);
                })


                //Line up with back board
                .lineToConstantHeading(new Vector2d(54,27))
                //Score pixel
                .addDisplacementMarker(() -> {
                    bot.setLidPosition(lidState.open);
                })

                .lineToConstantHeading(new Vector2d(53.8,27))

                //Move slides to score
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.MEDIUMOUT,extensionState.extending);
                })

                //Move behind in front of board
                .lineToConstantHeading(new Vector2d(48,27))

                //Set back to initialization position
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToConstantHeading(new Vector2d(48,60))
                .lineToConstantHeading(new Vector2d(52,55))
                .lineToLinearHeading(new Pose2d(52,61,Math.toRadians(270)))


                .build();

//        TrajectorySequence toCenterBackboard = drive.trajectorySequenceBuilder(centerUnderTruss.end())
//
//                .build();
//        TrajectorySequence toLeftBackboard = drive.trajectorySequenceBuilder(leftUnderTruss.end())
//
//                .build();

//
//        TrajectorySequence toLeftStack = drive.trajectorySequenceBuilder(toCenterBackboard.end())
//
        //         .build();
//        TrajectorySequence toCenterStack = drive.trajectorySequenceBuilder(toCenterBackboard.end())
//
//
//                .build();
//        TrajectorySequence rightStack = drive.trajectorySequenceBuilder(toCenterBackboard.end())

        //     .build();
//        TrajectorySequence toBackboardFromTruss = drive.trajectorySequenceBuilder(toStack.end())
//
//
//
//
//
//                .build();


        initCam();
        waitForStart();
        camera.stopStreaming();
        if (isStopRequested()) return;

        switch (blueDetection.getLocation()) {
            case LEFT:
                drive.setPoseEstimate(startPose);


                drive.followTrajectorySequence(leftUnderTruss);
                drive.followTrajectorySequence(toLeftBackboard);
                //  drive.followTrajectorySequence(toCenterStack);
                //   drive.followTrajectorySequence(toBackboardFromStack);
                break;
            case RIGHT:
                drive.setPoseEstimate(startPose);
                drive.followTrajectorySequence(rightUnderTruss);
                drive.followTrajectorySequence(toRightBackboard);
//                drive.followTrajectorySequence(rightStack);

                // drive.followTrajectorySequence(toBackboardFromTruss);


                break;
            case MIDDLE:
                drive.setPoseEstimate(startPose);


                drive.followTrajectorySequence(centerUnderTruss);
               drive.followTrajectorySequence(toCenterBackboard);
//                drive.followTrajectorySequence(toCenterStack);
                // drive.followTrajectorySequence(toBackboardFromTruss);
                break;
        }

        //     drive.followTrajectorySequence(center);
    }

    private void initCam() {

        //This line retrieves the resource identifier for the camera monitor view. The camera monitor view is typically used to display the camera feed
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcamName = "Webcam 1";

        // This line creates a webcam instance using the OpenCvCameraFactor with the webcam name (webcamName) and the camera monitor view ID.
        // The camera instance is stored in the camera variable that we can use later
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);

        // initializing our Detection class (details on how it works at the top)
        blueDetection = new HSVBlueDetection((telemetry));

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


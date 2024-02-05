package org.firstinspires.ftc.teamcode.OpModes.Autonomous.autoExecutable;

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

@Autonomous(name = "longBlueObjectDetect", group = "Auto")
public class longBlue extends LinearOpMode {
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
                .lineToConstantHeading(new Vector2d(-36, 55))


                //Move to tape
                .lineToLinearHeading(new Pose2d(-37,32,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(-33, 32))
                .lineToConstantHeading(new Vector2d(-37, 32))
                .lineToConstantHeading(new Vector2d(-37, 14))
                .lineToConstantHeading(new Vector2d(39, 14))

                //Set slides, arm, and wrist to outtake position
                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                    bot.outtakeSlide.setPosition(800);
                })

                //Lining up in front of back board
                .lineToConstantHeading(new Vector2d(40, 44))
                //Line up with back board
                .lineToConstantHeading(new Vector2d(52, 44))



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
                .lineToConstantHeading(new Vector2d(-36, 32))
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
                    bot.outtakeSlide.setPosition(700);
                })

                //Lining up in front of back board
                .lineToConstantHeading(new Vector2d(40, 36))
                //Lining up with back board
                .lineToConstantHeading(new Vector2d(52, 36))

                //Score
                .addDisplacementMarker( () -> {
                    bot.setLidPosition(lidState.open);
                })


                .lineToConstantHeading(new Vector2d(51.8, 36.5))
                //Move slides to score
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.MEDIUMOUT, extensionState.extending);
                })

                //Move behind in front of board
                .lineToConstantHeading(new Vector2d(40, 36.5))

                //Set to initialization position
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })

                //Move closer to park position
                .lineToConstantHeading(new Vector2d(44,41))
                //Rotate to park position
                .lineToLinearHeading(new Pose2d(44 ,15, Math.toRadians(270)))
                //Move in front of parking position
                .lineToConstantHeading(new Vector2d(45, 14))
                //Park
                .lineToConstantHeading(new Vector2d(52,14))

                .build();

        // ---------------------------- Right ---------------------------- //

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                //Initialization
                .addDisplacementMarker(() -> {
                    bot.setLidPosition(lidState.close);
                })

                //Move to tape
                .lineToConstantHeading(new Vector2d(-45,50))
                //Push to tape
                .lineToConstantHeading(new Vector2d(-45, 42))
                //Move away from tape
                .lineToConstantHeading(new Vector2d(-45, 47.5))
                //Move to center
                .lineToConstantHeading(new Vector2d(-35,47.5))
                //Align with gate
                .lineToConstantHeading(new Vector2d(-35,12))
                //Move through gate
                .lineToConstantHeading(new Vector2d(40,12))
                //Lining up in front of back board
                .lineToLinearHeading(new Pose2d(35,43.5, Math.toRadians(180)))


                //Set slides, arm, and wrist to outtake position
                .addDisplacementMarker(() -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                    bot.outtakeSlide.setPosition(700);
                })


                //Line up with back board
                .lineToConstantHeading(new Vector2d(54,34))

                //Score pixel
                .addDisplacementMarker(() -> {
                    bot.setLidPosition(lidState.open);
                })

                .lineToConstantHeading(new Vector2d(53.8,43.5))

                //Move slides to score
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.MEDIUMOUT,extensionState.extending);
                })

                //Move behind in front of board
                .lineToConstantHeading(new Vector2d(48,43.5))

                //Set back to initialization position
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })

                //Move back more
                .lineToConstantHeading(new Vector2d(45, 30))
                //Rotate to park position
                .lineToLinearHeading(new Pose2d(46 ,14, Math.toRadians(270)))
                //Move in front of park position
                .lineToConstantHeading(new Vector2d(45, 14))
                //Park
                .lineToConstantHeading(new Vector2d(52,14))

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

                drive.followTrajectorySequence(center);

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





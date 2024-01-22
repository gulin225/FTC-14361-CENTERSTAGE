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

@Autonomous(name = "longRedObjectDetect", group = "Auto")
public class longRed extends LinearOpMode {
    OpenCvCamera camera;


    HSVRedDetection redDetection;
    String webcamName;
    Robot bot;

    public void runOpMode() {
        bot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-31, -61, Math.toRadians(270));
        //idk if this will cause issues!!!
        double angleOffset = 2;
        //i dont know if these two lines below need to be here ESP initCAM
        //initCam(); try adding this in if doesnt work
        //line 32 may have actually been the issue pls
        drive.setPoseEstimate(startPose);
        //left ------------------------------------------------------------------
        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-37,-52))

                .addDisplacementMarker(() -> {

                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setWristPosition(wristState.init);
                    bot.setLidPosition(lidState.close);

                })

                .lineToConstantHeading(new Vector2d(-37, -44))


                .lineToConstantHeading(new Vector2d(-34,-50))

                .lineToLinearHeading(new Pose2d(-34,-55,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(45,-55))
                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                   bot.setWristPosition(wristState.outtaking);
                    bot.setOuttakeSlidePosition(outtakeSlidesState.LOWOUT, extensionState.extending);
                })

                .lineToConstantHeading(new Vector2d(44, -23))
                .waitSeconds(.25)
                .lineToConstantHeading(new Vector2d(61, -23))
                .addDisplacementMarker(() -> {
                bot.setLidState(lidState.open);
                })
                .waitSeconds(.25)
                .lineToConstantHeading(new Vector2d(50,-22))

                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setWristPosition(wristState.init);
                })

                .lineToLinearHeading(new Pose2d(50 ,-15, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(55,-15))
                .build();


        //waitForStart();
        // if (isStopRequested()) return;
        //drive.followTrajectorySequence(left);

        //center ------------------------------------------------------------------
        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)

                // going forward & little left
                .lineToConstantHeading(new Vector2d(-26,-52))

                .addDisplacementMarker(() -> {
                // init all transfer stuff
                    bot.setWristState(wristState.init);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setLidState(lidState.close);
                })

                // to center tape to push pixel
                .lineToConstantHeading(new Vector2d(-26, -36))

                // Backing up and aligning with backboard
                .lineToConstantHeading(new Vector2d(-26  ,-42))

                // turning to face backboard & moving towards backboard
                .lineToLinearHeading(new Pose2d(-34,-42, Math.toRadians(180)))

                // going to level with gate
                .lineToConstantHeading(new Vector2d(-34 , -12))

                // stopping after going under gate
                .lineToConstantHeading(new Vector2d(45,-12))

                // all transfer set to outtake
                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setOuttakeSlidePosition(outtakeSlidesState.LOWOUT, extensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                })
                // wait a second so it finishes outtake setup
                .waitSeconds(.25)

                // going to backboard
                .lineToConstantHeading(new Vector2d(61,-29))

                // open up the box
                .addDisplacementMarker( () -> {
                 bot.setLidState(lidState.open);
                })

                // back up from backboard
                .lineToConstantHeading(new Vector2d(50,-31))

                // prepare states for tele op
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })

                // parking
                .lineToLinearHeading(new Pose2d(50 ,-15, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(55,-15))

                .build();
        

        //right ------------------------------------------------------------------
        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-36,-55))

                .addDisplacementMarker(() -> {
                    bot.setActiveIntakePosition(activeIntakeState.active);
                    bot.setActiveIntakeState(activeIntakeState.active);
                    bot.setWristPosition(wristState.init);
                    bot.setWristState(wristState.init);
                    bot.setArmState(armState.init);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                })
                .waitSeconds(3)
                .lineToConstantHeading(new Vector2d(-29, -55))
                .addDisplacementMarker( () -> {
                    bot.setWristPosition(wristState.outtaking);
                    bot.setWristState(wristState.outtaking);
                    bot.setActiveIntakePosition(activeIntakeState.inactive);
                    bot.setActiveIntakeState(activeIntakeState.inactive);
                })
                .lineToConstantHeading(new Vector2d(-29, -55.01))
                .waitSeconds(.25)
                .addDisplacementMarker(() -> {
                    bot.setArmPosition(armState.autoDrop, armExtensionState.extending);
                    bot.setArmState(armState.autoDrop);

                })
                .lineToConstantHeading(new Vector2d(-31,-31))
                .lineToLinearHeading(new Pose2d(-31,-31.01,Math.toRadians(180)))
                .waitSeconds(.5)
                .lineToConstantHeading(new Vector2d(-23,-29))
                .lineToConstantHeading(new Vector2d(-24,-29))
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    ;

                })

                .lineToConstantHeading(new Vector2d(-34,-57))



                .lineToConstantHeading(new Vector2d(45,-57))
                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setArmState(armState.outtaking);

                    bot.outtakeSlide.setPosition(575);
                })


                .lineToConstantHeading(new Vector2d(62, -35.5))
                .addDisplacementMarker(() -> {


                })
                .lineToConstantHeading(new Vector2d(50,-38))

                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToConstantHeading(new Vector2d(55, -33))
                .lineToLinearHeading(new Pose2d(50 ,-15, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(55,-15))

                .build();


        //waitForStart();
        //if (isStopRequested())
        //drive.followTrajectorySequence(right);

        //camera initialization -----------------------------------------------------
        initCam();
        waitForStart();
        camera.stopStreaming();
        if (isStopRequested()) return;

        switch (redDetection.getLocation())
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
        redDetection = new HSVRedDetection(telemetry);

        // yeah what this does is it gets the thing which uses the thing so we can get the thing
        /*
        (fr tho idk what pipeline does, but from what I gathered,
         we basically passthrough our detection into the camera
         and we feed the streaming camera frames into our Detection algorithm)
         */
        camera.setPipeline(redDetection);

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


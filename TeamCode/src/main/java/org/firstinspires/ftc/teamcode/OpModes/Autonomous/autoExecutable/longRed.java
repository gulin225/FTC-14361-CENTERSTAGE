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

        drive.setPoseEstimate(startPose);
        initCam();

        // ---------------------------- Left ---------------------------- //

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                //Moving
                .lineToConstantHeading(new Vector2d(-47,-52))

                //Close lid
                .addDisplacementMarker(() -> {
                    bot.setLidPosition(lidState.close);
                })

                //Push to tape
                .lineToConstantHeading(new Vector2d(-47, -44))
                //Move away from tape
                .lineToConstantHeading(new Vector2d(-47, -50))
                //Move to center
                .lineToConstantHeading(new Vector2d(-33.5,-50))
                //Align with gate
                .lineToLinearHeading(new Pose2d(-33.5,-12, Math.toRadians(180)))
                //Pass through gate
                .lineToConstantHeading(new Vector2d(40,-12))
                //Lining up in front of back board
                .lineToConstantHeading(new Vector2d(35,-30))

                //Set slides, arm, and wrist to outtake position
                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                    bot.setOuttakeSlidePosition(outtakeSlidesState.LOWOUT, extensionState.extending);
                })

                .waitSeconds(.25)

                //Lining up with back board
                .lineToConstantHeading(new Vector2d(53, -30))

                //Score
                .addDisplacementMarker(() -> {
                    bot.setLidPosition(lidState.open);
                })

                .waitSeconds(.5)

                //Move slides to score
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.MEDIUMOUT,extensionState.extending);
                })

                //Move in front of back board
                .lineToConstantHeading(new Vector2d(48,-30))

                //Set to initialization position
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })

                //Move closer to parking position
                .lineToConstantHeading(new Vector2d(45, -30))
                //Rotate to parking position
                .lineToLinearHeading(new Pose2d(46 ,-15, Math.toRadians(90)))
                //Move in front of parking position
                .lineToConstantHeading(new Vector2d(45, -15))
                //Park
                .lineToConstantHeading(new Vector2d(52,-16))

                .build();

        // ---------------------------- Center ---------------------------- //

        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
                //Initialization
                .addDisplacementMarker(() -> {
                    bot.setLidPosition(lidState.close);
                })

                //Move away from wall
                .lineToConstantHeading(new Vector2d(-34, -55))
                .waitSeconds(.25)
                //Push to tape
                .lineToConstantHeading(new Vector2d(-34, -31))
                //Move away from tape
                .lineToConstantHeading(new Vector2d(-34,-39))
                //Move left of tape
                .lineToConstantHeading(new Vector2d(-38,-42))
                //Moving to gate
                .lineToConstantHeading(new Vector2d(-47,-42))
                //Lining up with gate
                .lineToLinearHeading(new Pose2d(-48,-12,Math.toRadians(180)))
                //Pass through gate
                .lineToConstantHeading(new Vector2d(39, -12))

                //Set slides, arm, and wrist to outtake position
                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                    bot.setOuttakeSlidePosition(outtakeSlidesState.LOWOUT, extensionState.extending);
                })

                //Lining up in front of back board
                .lineToConstantHeading(new Vector2d(40, -29))
                //Lining up with back board
                .lineToConstantHeading(new Vector2d(52, -29))

                .waitSeconds(.5)

                //Score
                .addDisplacementMarker( () -> {
                    bot.setLidPosition(lidState.open);
                })

                .waitSeconds(1)

                //Move slides to score
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.MEDIUMOUT, extensionState.extending);
                })

                //Move in front of back board
                .lineToConstantHeading(new Vector2d(40, -29))

                //Set to initialization position
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })

                .lineToConstantHeading(new Vector2d(44,-39))
                //Move closer to park position
                .lineToLinearHeading(new Pose2d(44 ,-15, Math.toRadians(90)))
                //Rotate to park position
                .lineToConstantHeading(new Vector2d(45, -16))
                //Park
                .lineToConstantHeading(new Vector2d(52,-16))

                .build();

        // ---------------------------- Right ---------------------------- //

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                //Initialization
                .addDisplacementMarker(() -> {
                    bot.setLidPosition(lidState.close);
                })

                //Move away from wall
                .lineToConstantHeading(new Vector2d(-34, -55))
                .waitSeconds(.25)
                //Push to tape
                .lineToLinearHeading(new Pose2d(-32,-37,Math.toRadians(210)))
                //Move away from tape
                .lineToLinearHeading(new Pose2d(-34,-39,Math.toRadians(210)))
                //Move left of tape
                .lineToLinearHeading(new Pose2d(-45,-33,Math.toRadians(270)))
                //Move behind gate
                .lineToLinearHeading(new Pose2d(-45,-14,Math.toRadians(180)))
                //Pass through gate
                .lineToConstantHeading(new Vector2d(39, -14))

                //Set slides, arm, and wrist to outtake position
                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                    bot.setOuttakeSlidePosition(outtakeSlidesState.LOWOUT, extensionState.extending);
                })

                //Lining up in front of back board
                .lineToConstantHeading(new Vector2d(40, -37))
                //Lining up with back board
                .lineToConstantHeading(new Vector2d(52, -37))
                .waitSeconds(.25)

                //Score
                .addDisplacementMarker(() -> {
                    bot.setLidPosition(lidState.open);
                })

                .waitSeconds(.5)

                //Move slides to score
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.MEDIUMOUT, extensionState.extending);
                })

                //Move in front of back board
                .lineToConstantHeading(new Vector2d(40,-43))

                //Set to initialization position
                .addDisplacementMarker(() -> {
                    bot.setWristPosition(wristState.intaking);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                })

                //Move closer to park position
                .lineToConstantHeading(new Vector2d(40,-39))
                //Rotate to park position
                .lineToLinearHeading(new Pose2d(44 ,-12, Math.toRadians(90)))
                //Move in front of park position
                .lineToConstantHeading(new Vector2d(45, -12))
                //Park
                .lineToConstantHeading(new Vector2d(52,-12))

                .build();

        // ---------------------------- Camera Initialization ---------------------------- //

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

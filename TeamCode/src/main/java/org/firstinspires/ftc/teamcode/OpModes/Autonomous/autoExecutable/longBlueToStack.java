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
    HSVRedDetection redDetection;
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
                .addDisplacementMarker(  3,() -> {
                    bot.setLidPosition(lidState.close);
                })
                .lineToConstantHeading(new Vector2d(-41, 34))
                .lineToConstantHeading(new Vector2d(-41, 39))
                .lineToConstantHeading(new Vector2d(-50, 39))
                .addDisplacementMarker( 65, () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                })

                .lineToConstantHeading(new Vector2d(-50, 9))
                .lineToConstantHeading(new Vector2d(40, 9))
                .build();
        TrajectorySequence toCenterBackboard = drive.trajectorySequenceBuilder(centerUnderTruss.end())

                .lineToConstantHeading(new Vector2d(52, 39))
                .addDisplacementMarker( 3, () -> {
                    bot.outtakeSlide.setPosition(700);
                })
                .addDisplacementMarker(  () -> {
                    bot.setLidPosition(lidState.open);
                })
                .waitSeconds(.1)
                .lineToConstantHeading(new Vector2d(40, 39))
                .addDisplacementMarker( 25, () -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                })
                .lineToConstantHeading(new Vector2d(-41, 39))


                .build();
        TrajectorySequence toCenterStack = drive.trajectorySequenceBuilder(toCenterBackboard.end())
                .lineToLinearHeading(new Pose2d(-57,43,Math.toRadians(230)))
                .addDisplacementMarker(10,  () -> {

                    bot.linkage.setLinkageCustomPosition(.96);
                    bot.setActiveIntakePosition(activeIntakeState.active);
                })
                .forward(5.5)

                .turn(Math.toRadians(-93.5))
                .waitSeconds(.1)
                .forward(8)
                .lineToLinearHeading(new Pose2d(-41,39,Math.toRadians(180)))

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
                .addDisplacementMarker( 55, () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                })
                .lineToConstantHeading(new Vector2d(45, 42))
                .addDisplacementMarker(  () -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.MEDIUMOUT, extensionState.extending);
                })
                .lineToConstantHeading(new Vector2d(52, 42))
                .addDisplacementMarker(  () -> {
                    bot.setLidPosition(lidState.open);
                })
                .lineToConstantHeading(new Vector2d(46, 42))
                .addDisplacementMarker(  () -> {
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToLinearHeading(new Pose2d(46,57,Math.toRadians(270)))

                .lineToConstantHeading(new Vector2d(52, 57))

                .build();
        TrajectorySequence leftUnderTruss = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(1.5,() -> {
                    bot.setLidPosition(lidState.close);
                })

                //Move away from wall
                .lineToConstantHeading(new Vector2d(-36, 55))


                //Move to tape
                .lineToLinearHeading(new Pose2d(-37,32,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(-33, 32))
                .lineToConstantHeading(new Vector2d(-37, 32))
                .lineToConstantHeading(new Vector2d(-37, 55))
                .addDisplacementMarker( 55, () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                })
                .lineToConstantHeading(new Vector2d(40, 55))
                .build();
        TrajectorySequence toLeftBackboard = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(52, 45))

                .addDisplacementMarker( 2, () -> {
                    bot.outtakeSlide.setPosition(700);
                })
                .addDisplacementMarker(  () -> {
                    bot.setLidPosition(lidState.open);
                    bot.setOuttakeSlidePosition(outtakeSlidesState.MEDIUMOUT, extensionState.extending);

                })
                .lineToConstantHeading(new Vector2d(45, 45))
                .lineToConstantHeading(new Vector2d(30, 55))
                .build();
//        TrajectorySequence rightUnderTruss = drive.trajectorySequenceBuilder(startPose)
//
//                .build();
//        TrajectorySequence toCenterBackboard = drive.trajectorySequenceBuilder(centerUnderTruss.end())
//
//                .build();
//        TrajectorySequence toLeftBackboard = drive.trajectorySequenceBuilder(leftUnderTruss.end())
//
//                .build();
//        TrajectorySequence toRightBackboard = drive.trajectorySequenceBuilder(rightUnderTruss.end())
//

        //        .build();
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

        switch (redDetection.getLocation())
        {
            case LEFT:
                drive.setPoseEstimate(startPose);


                drive.followTrajectorySequence(leftUnderTruss);
                drive.followTrajectorySequence(toLeftBackboard);
                //  drive.followTrajectorySequence(toCenterStack);
                //   drive.followTrajectorySequence(toBackboardFromStack);
                break;
            case RIGHT:
                drive.setPoseEstimate(startPose);
                drive.followTrajectorySequence(centerUnderTruss);
//                drive.followTrajectorySequence(toRightBackboard);
//                drive.followTrajectorySequence(rightStack);

                // drive.followTrajectorySequence(toBackboardFromTruss);


                break;
            case MIDDLE:
                drive.setPoseEstimate(startPose);


                drive.followTrajectorySequence(centerUnderTruss);
//                drive.followTrajectorySequence(toCenterBackboard);
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
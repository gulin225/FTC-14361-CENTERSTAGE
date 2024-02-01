package org.firstinspires.ftc.teamcode.OpModes.Autonomous.RedPaths;

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
import org.firstinspires.ftc.teamcode.Subsystems.HSVRedDetection;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Autonomous(name = "RedLongStackNormal", group = "goobTest")
public class LongRedStackNormal extends LinearOpMode {
    OpenCvCamera camera;
    HSVRedDetection redDetection;
    String webcamName;
    Robot bot;

    public void runOpMode() {
        bot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-40.5, -61, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        // ---------------------------- Left ---------------------------- //

//        TrajectorySequence toTapeDropOff = drive.trajectorySequenceBuilder(startPose)
//                //Push to tape
//                //Move away from tape
//                .build();

        TrajectorySequence underTruss = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-47,-52))
                .addDisplacementMarker(() -> {
                    bot.setLidPosition(lidState.close);
                })

                //Push to tape
                .lineToConstantHeading(new Vector2d(-48.25, -44))
                //Move away from tape
                .lineToConstantHeading(new Vector2d(-48.25, -50))
                //Move to center
                .lineToConstantHeading(new Vector2d(-38.5,-50))
                .lineToConstantHeading(new Vector2d(-38.5, -10))
                .turn(Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(40,-10, Math.toRadians(180)))
                .build();

        TrajectorySequence toBackboard = drive.trajectorySequenceBuilder(underTruss.end())
                .lineToConstantHeading(new Vector2d(35,-22))
                //Set slides, arm, and wrist to outtake position
                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                    bot.outtakeSlide.setPosition(800);
                })
                //Lining up with back board
                .lineToConstantHeading(new Vector2d(51.3, -22))
                //Score
                .addDisplacementMarker(() -> {
                    bot.setLidPosition(lidState.open);
                })
                .waitSeconds(.25)
                .lineToConstantHeading(new Vector2d(51.2, -22))
                //Move slides to score
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.MEDIUMOUT,extensionState.extending);
                })
                .build();

        TrajectorySequence toStack = drive.trajectorySequenceBuilder(toBackboard.end())
//                .splineTo(new Vector2d(-54.5,-33))

                .addDisplacementMarker(20,() -> {
                    bot.linkage.setLinkageCustomPosition(.96);
                    bot.setActiveIntakePosition(activeIntakeState.active);
                })

                .turn(Math.toRadians(70))

                .forward(3)

                .lineToLinearHeading(new Pose2d(-15,-56, Math.toRadians(180)))
                .addDisplacementMarker(50, () -> {
                    bot.setActiveIntakePosition(activeIntakeState.activeReverse);
                })
                .addDisplacementMarker(75, () -> {
                    bot.setActiveIntakePosition(activeIntakeState.inactive);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setWristPosition(wristState.init);
                })
                .addDisplacementMarker(80, () -> {
                    bot.setLidPosition(lidState.close);
                })
                .lineToConstantHeading(new Vector2d(40,-55))
                .build();

        TrajectorySequence toBackboardFromTruss = drive.trajectorySequenceBuilder(toStack.end())
                .lineToConstantHeading(new Vector2d(52,-37))
                .addDisplacementMarker(3,() ->{
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                    bot.outtakeSlide.setPosition(800);
                })
                .addDisplacementMarker( () -> {
                    bot.setLidPosition(lidState.open);
                    bot.setOuttakeSlidePosition(outtakeSlidesState.MEDIUMOUT, extensionState.extending);
                })
                .lineToConstantHeading(new Vector2d(50,-37))

                .build();

        initCam();
        waitForStart();
        camera.stopStreaming();
        if (isStopRequested()) return;

        drive.followTrajectorySequence(underTruss);
        drive.followTrajectorySequence(toBackboard);
        drive.followTrajectorySequence(toStack);
        drive.followTrajectorySequence(toBackboardFromTruss);
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

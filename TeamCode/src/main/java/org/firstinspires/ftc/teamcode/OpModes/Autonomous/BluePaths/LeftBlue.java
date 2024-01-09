package org.firstinspires.ftc.teamcode.OpModes.Autonomous.BluePaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Commands.extensionState;
//import org.firstinspires.ftc.teamcode.Commands.intakeSlidesState;
import org.firstinspires.ftc.teamcode.Commands.armExtensionState;
import org.firstinspires.ftc.teamcode.Subsystems.HSVBlueDetection;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "LeftBlue")

public class LeftBlue extends LinearOpMode {
    Robot bot;
    OpenCvCamera camera;
    HSVBlueDetection blueDetection;
    String webcamName;

    @Override
    public void runOpMode()
    {
        bot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(15, 61, Math.toRadians(90));
        //bot.setInBrake();

        initCam();

        drive.setPoseEstimate(startPose);

        // ---------------------------- toLeftTape ---------------------------- //

        TrajectorySequence toLeftTape = drive.trajectorySequenceBuilder(startPose)
                //Moving away from wall
                .lineToConstantHeading(new Vector2d(15, 55))
                .waitSeconds(1)
                //Moving behind the left tape
                .lineToConstantHeading(new Vector2d(22, 55))
                .waitSeconds(1)
                //Moving onto the left tape
                .lineToConstantHeading(new Vector2d(22, 43))
                .waitSeconds(2)
                //Moving back behind the left tape
                .lineToConstantHeading(new Vector2d(22, 55))
                .waitSeconds(1)
                //Moving towards backboard zone
                .lineToConstantHeading(new Vector2d(36, 55))
                .waitSeconds(1)
                //Moving to backboard
                .lineToLinearHeading(new Pose2d(51, 40, Math.toRadians(180)))
                .waitSeconds(.5)
                //Moving away from backboard
                .lineToConstantHeading(new Vector2d(48.5, 40))
                .waitSeconds(1)
                //Moving towards park position
                .lineToConstantHeading(new Vector2d(40, 40))
                .waitSeconds(1)
                //Line up to park position
                .lineToLinearHeading(new Pose2d(40, 57, Math.toRadians(270)))
                .waitSeconds(1)
                //Parking
                .lineToConstantHeading(new Vector2d(46, 57))

                .build();

        // ---------------------------- toCenterTape ---------------------------- //

        TrajectorySequence toCenterTape = drive.trajectorySequenceBuilder(startPose)
                //Moving onto center tape
                .lineToConstantHeading(new Vector2d(11, 36))
                .waitSeconds(1.5)
                //Moving away from center tape
                .lineToConstantHeading(new Vector2d(11, 45))
                .waitSeconds(1)
                //going to backboard
                .lineToLinearHeading(new Pose2d(51, 35, Math.toRadians(180)))
                .waitSeconds(1)
                //Moving away from backboard
                .lineToConstantHeading(new Vector2d(40, 35))
                .waitSeconds(1)
                //Moving towards park position
                .lineToConstantHeading(new Vector2d(40, 57))
                .waitSeconds(1)
                //Parking
                .lineToLinearHeading(new Pose2d(46, 57, Math.toRadians(270)))

                .build();

        // ---------------------------- toRightTape ---------------------------- //

        TrajectorySequence toRightTape = drive.trajectorySequenceBuilder(startPose)
                //going to right tape
                .lineToLinearHeading(new Pose2d(11, 31, Math.toRadians(0)))
                .waitSeconds(2)
                //going to backboard
                .lineToLinearHeading(new Pose2d(51, 31, Math.toRadians(180)))
                .waitSeconds(1)
                //Moving away from backboard
                .lineToConstantHeading(new Vector2d(40, 31))
                .waitSeconds(1)
                //Moving towards park position
                .lineToLinearHeading(new Pose2d(40, 57, Math.toRadians(270)))
                .waitSeconds(1)
                //Parking
                .lineToConstantHeading(new Vector2d(46, 57))

                .build();

        // ---------------------------- Camera ---------------------------- //

        waitForStart();

        camera.stopStreaming();

        if (isStopRequested()) return;


        switch (blueDetection.getLocation())
        {
            case LEFT:
                drive.setPoseEstimate(startPose);

                drive.followTrajectorySequence(toLeftTape);

                break;
            case RIGHT:
                drive.setPoseEstimate(startPose);

                drive.followTrajectorySequence(toRightTape);

                break;
            case MIDDLE:
                drive.setPoseEstimate(startPose);

                drive.followTrajectorySequence(toCenterTape);

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
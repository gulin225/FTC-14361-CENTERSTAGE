package org.firstinspires.ftc.teamcode.OpModes.Autonomous.RedPaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.armExtensionState;
import org.firstinspires.ftc.teamcode.Commands.armState;
import org.firstinspires.ftc.teamcode.Commands.clawState;
import org.firstinspires.ftc.teamcode.Commands.extensionState;
import org.firstinspires.ftc.teamcode.Commands.outtakeSlidesState;
import org.firstinspires.ftc.teamcode.Commands.wristState;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.HSVRedDetection;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "CloseRedMid")
public class CloseRedMid extends LinearOpMode {
    Robot bot;
    OpenCvCamera camera;
    HSVRedDetection redDetection;
    String webcamName;

    public void runOpMode() {

        bot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d start = new Pose2d(9, -61, Math.toRadians(270));
        drive.setPoseEstimate(start);
        //initCam();

        TrajectorySequence everything = drive.trajectorySequenceBuilder(start)
                .waitSeconds(1)

//                .addDisplacementMarker(0.05, 0, () -> {
//                    // This example will run 50% of the way through the path, plus 2.1 inches
//                    // The offset can be left at zero but is useful for making slight adjustments to the timing
//                })
                .addDisplacementMarker(() -> {
                    bot.setWristPosition(wristState.init);
                    bot.setWristState(wristState.init);
                    bot.setArmState(armState.init);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                })
                .lineToConstantHeading(new Vector2d(15, -61))

//                .addDisplacementMarker(5, () -> {
//                    bot.setVirtualFourBarState(virtualFourBarState.init);
//                    bot.setVirtualFourBarPosition(virtualFourBarState.init, virtualFourBarExtensionState.extending);
//                })
                .addDisplacementMarker( () -> {
                    bot.setWristPosition(wristState.outtaking);
                    bot.setWristState(wristState.outtaking);
                })
                .addDisplacementMarker(15, () -> {
                    bot.setArmPosition(armState.autoDrop, armExtensionState.extending);
                    bot.setArmState(armState.autoDrop);

                })

                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(15, -35.5))
                .addDisplacementMarker(() -> {
                    bot.setClawPosition(clawState.leftClose);
                    bot.setClawState(clawState.leftClose);
                })
                .lineToConstantHeading(new Vector2d(15, -40))

                .waitSeconds(.5)

                .lineToLinearHeading(new Pose2d(59 ,-32, Math.toRadians(180)))
                .addDisplacementMarker(45, () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setArmState(armState.outtaking);

                    bot.outtakeSlide.setPosition(265);
                })
                .addDisplacementMarker(() -> {
                    bot.setClawPosition(clawState.rightClose);
                    bot.setClawState(clawState.rightClose);
                })
                .waitSeconds(.5)
                .lineToConstantHeading(new Vector2d(48, -32))
//                .waitSeconds(.5)
//                .lineToConstantHeading(new Vector2d(48, -58))
//                .waitSeconds(.5)
//                .lineToConstantHeading(new Vector2d(65, -58))

                .build();
        waitForStart();
        if(isStopRequested()) return;

        //     bot.intakeSlide.setPosition(50);
        bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
        bot.setOuttakeSlideState(outtakeSlidesState.STATION);
        bot.setArmState(armState.intaking);
        bot.setArmPosition(armState.intaking, armExtensionState.extending);
        bot.setWristPosition(wristState.intaking);
        bot.setWristState(wristState.intaking);
        bot.setClawPosition(clawState.open);
        bot.setClawState(clawState.open);


        drive.followTrajectorySequence(everything);


    }
}






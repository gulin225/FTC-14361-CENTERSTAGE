package org.firstinspires.ftc.teamcode.OpModes.Autonomous.BluePaths;

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

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@Autonomous(name = "CloseBlueRight")
public class CloseBlueRight extends LinearOpMode
{
    Robot bot;

    public void runOpMode()
    {
        bot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d start = new Pose2d(15, 61, Math.toRadians(90));
        drive.setPoseEstimate(start);

        TrajectorySequence toRightTape = drive.trajectorySequenceBuilder(start)
                .waitSeconds(1)

                .addDisplacementMarker(() -> {
                    bot.setWristPosition(wristState.init);
                    bot.setWristState(wristState.init);
                    bot.setArmState(armState.init);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                })

                .lineToConstantHeading(new Vector2d(11, 60))

                .waitSeconds(.25)

                .addDisplacementMarker( () -> {
                    bot.setWristPosition(wristState.outtaking);
                    bot.setWristState(wristState.outtaking);
                })

                .lineToConstantHeading(new Vector2d(11,60.01))

                .waitSeconds(.25)

                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.autoDrop, armExtensionState.extending);
                    bot.setArmState(armState.autoDrop);
                })

                .lineToLinearHeading(new Pose2d(10,36,Math.toRadians(30)))

                .addDisplacementMarker( () -> {
                    bot.setClawPosition(clawState.leftClose);
                    bot.setClawState(clawState.leftClose);
                })

                .lineToLinearHeading(new Pose2d(20,36, Math.toRadians(90)))

                .lineToLinearHeading(new Pose2d(30,32,Math.toRadians(180)))

                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setArmState(armState.outtaking);
                    bot.outtakeSlide.setPosition(450);
                })

                .lineToConstantHeading(new Vector2d(53,31))

                .addDisplacementMarker(() -> {
                    bot.setClawPosition(clawState.rightClose);
                    bot.setClawState(clawState.rightClose);
                })

                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })

                .lineToConstantHeading(new Vector2d(47, 32))

                .lineToLinearHeading(new Pose2d(47 ,58, Math.toRadians(270)))

                .build();

        waitForStart();
        if(isStopRequested()) return;

        bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
        bot.setOuttakeSlideState(outtakeSlidesState.STATION);
        bot.setArmState(armState.intaking);
        bot.setArmPosition(armState.intaking, armExtensionState.extending);
        bot.setWristPosition(wristState.intaking);
        bot.setWristState(wristState.intaking);
        bot.setClawPosition(clawState.open);
        bot.setClawState(clawState.open);

        drive.followTrajectorySequence(toRightTape);
    }
}

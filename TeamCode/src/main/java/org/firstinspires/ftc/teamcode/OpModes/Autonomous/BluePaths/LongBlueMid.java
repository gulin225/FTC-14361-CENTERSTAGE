package org.firstinspires.ftc.teamcode.OpModes.Autonomous.BluePaths;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.clawState;
import org.firstinspires.ftc.teamcode.Commands.armExtensionState;
import org.firstinspires.ftc.teamcode.Commands.armState;
import org.firstinspires.ftc.teamcode.Commands.extensionState;
import org.firstinspires.ftc.teamcode.Commands.outtakeSlidesState;
import org.firstinspires.ftc.teamcode.Commands.wristState;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.drive.SampleMecanumDrive;

@Autonomous(name = "LongBlueMid")
public class LongBlueMid extends LinearOpMode
{

    Robot bot;
    Pose2d startPose = new Pose2d(-31, 61, Math.toRadians(90));
    private double angleOffset = 2;

    @Override
    public void runOpMode() throws InterruptedException
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        bot = new Robot(hardwareMap, telemetry);
        //bot.setInBrake();

        drive.setPoseEstimate(startPose);

        TrajectorySequence toLeftTape = drive.trajectorySequenceBuilder(startPose)

                .addDisplacementMarker(() -> {
                    bot.setWristPosition(wristState.init);
                    bot.setWristState(wristState.init);
                    bot.setArmState(armState.init);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                })
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(-34, 61))
                .addDisplacementMarker( () -> {
                    bot.setWristPosition(wristState.outtaking);
                    bot.setWristState(wristState.outtaking);
                })
                .lineToConstantHeading(new Vector2d(-34, 61.01))
                .waitSeconds(.25)
                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.autoDrop, armExtensionState.extending);
                    bot.setArmState(armState.autoDrop);

                })

                .lineToConstantHeading(new Vector2d(-34, 34))
                .addDisplacementMarker(() -> {
                    bot.setClawPosition(clawState.leftClose);
                    bot.setClawState(clawState.leftClose);
                })

                .lineToConstantHeading(new Vector2d(-34,42))
                .lineToLinearHeading(new Pose2d(-47,42,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-47,13,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(39, 7))


                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setArmState(armState.outtaking);

                    bot.outtakeSlide.setPosition(450);
                })

                .lineToConstantHeading(new Vector2d(50, 44))
                .waitSeconds(.5)
                .addDisplacementMarker( () -> {
                    bot.setClawPosition(clawState.rightClose);
                    bot.setClawState(clawState.rightClose);
                })
                .waitSeconds(.5)
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToConstantHeading(new Vector2d(44,39))
                .lineToLinearHeading(new Pose2d(44 ,15, Math.toRadians(270)))
                .lineToConstantHeading(new Vector2d(45, 16))
                .lineToConstantHeading(new Vector2d(52,16))


                .build();
        waitForStart();


        if (isStopRequested()) return;
        bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
        bot.setOuttakeSlideState(outtakeSlidesState.STATION);
        bot.setArmState(armState.intaking);
        bot.setArmPosition(armState.intaking, armExtensionState.extending);
        bot.setWristPosition(wristState.intaking);
        bot.setWristState(wristState.intaking);
        bot.setClawPosition(clawState.open);
        bot.setClawState(clawState.open);

        drive.followTrajectorySequence(toLeftTape);
    }
}

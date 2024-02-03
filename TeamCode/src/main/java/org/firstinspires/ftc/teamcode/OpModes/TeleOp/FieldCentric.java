package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.activeIntakeState;
import org.firstinspires.ftc.teamcode.Commands.armExtensionState;
import org.firstinspires.ftc.teamcode.Commands.clawState;
import org.firstinspires.ftc.teamcode.Commands.extensionState;
import org.firstinspires.ftc.teamcode.Commands.lidState;
import org.firstinspires.ftc.teamcode.Commands.linkageState;
import org.firstinspires.ftc.teamcode.Commands.mecanumState;
import org.firstinspires.ftc.teamcode.Commands.outtakeSlidesState;
import org.firstinspires.ftc.teamcode.Commands.slowDownState;
import org.firstinspires.ftc.teamcode.Commands.armExtensionState;
import org.firstinspires.ftc.teamcode.Commands.armState;
import org.firstinspires.ftc.teamcode.Commands.wristState;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.robotConstants;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp

public class FieldCentric extends OpMode {
    private ElapsedTime runTime;
    private GamepadEx driver, operator;
    private Robot bot;
    public int outtakeSlideCount = 0;

    @Override
    public void init() {
        runTime = new ElapsedTime();
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        bot = new Robot(hardwareMap, telemetry);

        telemetry.addLine("It's goobin time");
        telemetry.addLine("Time taken: " + getRuntime() + " seconds.");

        telemetry.update();

        bot.setArmPosition(armState.intaking, armExtensionState.extending);



        bot.setWristPosition(wristState.intaking);
        bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);

        bot.setLinkagePosition(linkageState.LOW);
        bot.setLidPosition(lidState.open);

        // bot.setIntakeSlidePosition(intakeSlidesState.STATION, extensionState.extending);
        // bot.setIntakeSlideState(intakeSlidesState.STATION);

//        
//        
//
//        bot.setLeftClawState(clawState.leftClose);
//        bot.setRightClawState(clawState.leftClose);

        // bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);

        bot.setDrone();

        bot.setSlowDownState(slowDownState.FULL);
    }

    // ---------------------------- LOOPING ---------------------------- //

    @Override
    public void loop() {
        telemetry.addLine("Total Runtime: " + getRuntime() + " seconds.");
        telemetry.addLine("Left Slide Position: " + bot.getOuttakeLeftSlidePosition() + " ticks");
        telemetry.addLine("Right Slide Position: " + bot.getOuttakeRightSlidePosition() + " ticks");
//      telemetry.addLine("Intake Slide Position" + bot.getIntakeSlidePosition());
//      telemetry.addLine("Intake Slide Count " + intakeSlideCountAdd);
//      telemetry.addLine("Intake Slide Subtract Count " + intakeSlideCountSubstract);

        telemetry.addLine("Wrist Position: " + bot.wrist.getWristPosition());
        telemetry.addLine("State of V4B: init / " + bot.arm.getArmExtensionState());
//        telemetry.addLine("Right Claw Position: " + bot.claw.getRightClawPosition());
//        telemetry.addLine("Left Claw Position: " + bot.claw.getLeftClawPosition());
        telemetry.addLine("Right Arm Position: " + bot.arm.getRightArmPosition() + " ticks.");
        telemetry.addLine("Right Arm Decimal Position: " + (1 - bot.arm.getRightArmPosition() / 360) + " decimal.");
        telemetry.addLine("Left Arm Position: " + bot.arm.getLeftArmPosition() + " ticks.");
        telemetry.addLine("Left Arm Decimal Position: " + (1 - bot.arm.getLeftArmPosition() / 360) + " decimal.");
//        telemetry.addLine("DIstance in CM" + bot.getDistanceSensor());
//      telemetry.addLine("Intake Slide Encoder Tick Count " + intakeSlideCountSubstract);
        telemetry.update();

        driver.readButtons();
        operator.readButtons();


        bot.driveTrain.setMotorPower();

        // ---------------------------- DRIVER CODE ---------------------------- //

        if (driver.wasJustPressed(GamepadKeys.Button.START)) {
            bot.driveTrain.resetIMU();
        }

//        if (driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1) {
//            bot.driveTrain.angleLock(driver, 270);
//        }
//        else {
//            bot.driveTrain.drive(driver, mecanumState.NORMAL);
//        }
//
//        if (driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {
//            bot.driveTrain.angleLock(driver, 90);
//        }
//        else {
//            bot.driveTrain.drive(driver, mecanumState.NORMAL);
//        }

        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            if (bot.getActiveIntakeState() != null && (bot.getActiveIntakeState().equals(activeIntakeState.active))) {
                bot.setActiveIntakePosition(activeIntakeState.inactive);
                bot.setActiveIntakeState(activeIntakeState.inactive);
            } else if ((bot.getOuttakeState().equals(outtakeSlidesState.STATION) && (bot.getArmState()).equals(armState.intaking) && (bot.getWristState()).equals(wristState.intaking))) {
                bot.setActiveIntakePosition(activeIntakeState.active);
                bot.setActiveIntakeState(activeIntakeState.active);
            }
        }

        if (driver.wasJustPressed(GamepadKeys.Button.Y)) {
            if (bot.getActiveIntakeState() != null && bot.getActiveIntakeState().equals(activeIntakeState.activeReverse)) {
                bot.setActiveIntakePosition(activeIntakeState.inactive);
                bot.setActiveIntakeState(activeIntakeState.inactive);
            } else {
                bot.setActiveIntakePosition(activeIntakeState.activeReverse);
                bot.setActiveIntakeState(activeIntakeState.activeReverse);
            }
        }

        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                if (bot.getLidState() != null && bot.getLidState().equals(lidState.close)) {
                    bot.setLidPosition(lidState.open);
                }
                else
                {
                    bot.setLidPosition(lidState.close);
                }
        }

        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            bot.launchDrone();
        }
        if (driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            if (bot.getLinkageState() != null && bot.getLinkageState().equals(linkageState.HIGH)) {
                bot.setLinkagePosition(linkageState.LOW);
            } else if (bot.getLinkageState() != null && bot.getLinkageState().equals(linkageState.LOW)) {
                bot.setLinkagePosition(linkageState.HIGH);
            } else {
                bot.setLinkagePosition(linkageState.LOW);
            }

        }
        if(driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
            bot.setSlowDownState(slowDownState.FULL);
            bot.driveTrain.setFullPower();

        }
        if(driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
            bot.setSlowDownState(slowDownState.SLOW);
            bot.driveTrain.setSlowDownMotorPower();

        }
//        if(driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
//            bot.driveTrain.drive(driver, mecanumState.NORMAL);
//        }

        // --------------------------- OPERATOR CODE --------------------------- //


        if (operator.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
            if (bot.getWristState() != null && bot.getWristState().equals(wristState.intaking)) {
                bot.setWristPosition(wristState.outtaking);
                bot.setWristState(wristState.outtaking);
            } else if (bot.getWristState() != null && bot.getWristState().equals(wristState.init)) {
                bot.setWristPosition(wristState.outtaking);
                bot.setWristState(wristState.outtaking);
            } else {
                bot.setWristPosition(wristState.intaking);
                bot.setWristState(wristState.intaking);
            }
            // bot.wrist.setWristCustomPosition(.95);
        }
        if (operator.wasJustPressed(GamepadKeys.Button.BACK)) {
            bot.setOuttakeSlidePosition(outtakeSlidesState.HIGHOUT, extensionState.extending);
            bot.setOuttakeSlideState(outtakeSlidesState.HIGHOUT);
        }

        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            bot.setOuttakeSlidePosition(outtakeSlidesState.MEDIUMOUT, extensionState.extending);
            bot.setOuttakeSlideState(outtakeSlidesState.MEDIUMOUT);


        }
if(operator.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
    bot.setLidPosition(lidState.close);
}
        if(operator.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
            bot.setWristPosition(wristState.init);
        }
        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            bot.setOuttakeSlidePosition(outtakeSlidesState.LOWMED, extensionState.extending);
            bot.setOuttakeSlideState(outtakeSlidesState.LOWMED);

        }

        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            bot.setOuttakeSlidePosition(outtakeSlidesState.LOWOUT, extensionState.extending);
            bot.setOuttakeSlideState(outtakeSlidesState.LOWOUT);
        }

        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
            bot.setOuttakeSlideState(outtakeSlidesState.STATION);
        }

//        if (operator.wasJustPressed(GamepadKeys.Button.B)) {
//            intakeSlideCountAdd += 5;
//            bot.intakeSlide.setPosition(intakeSlide.retracted + intakeSlideCountAdd);
//        }

        if (operator.wasJustPressed(GamepadKeys.Button.Y)) {
            bot.setWristState(wristState.outtaking);
            bot.setWristPosition(wristState.outtaking);

            bot.setArmPosition(armState.outtaking, armExtensionState.extending);
            bot.setArmState(armState.outtaking);
        }

        if (operator.wasJustPressed(GamepadKeys.Button.A)) {


            bot.setWristState(wristState.intaking);
            bot.setWristPosition(wristState.intaking);
            bot.setLidPosition(lidState.open);

            bot.setArmPosition(armState.intaking, armExtensionState.extending);
            bot.setArmState(armState.intaking);
        }

        if (operator.wasJustPressed(GamepadKeys.Button.X)) {

            if (bot.getArmState() != null && bot.getArmState().equals(armState.outtaking)) {
                bot.setWristPosition(wristState.intaking);
            }
            if (bot.getArmState() != null && bot.getArmState().equals(armState.intaking)) {
                bot.setArmPosition(armState.init, armExtensionState.extending);
                bot.setArmPosition(armState.init, armExtensionState.extending);
                bot.setLidPosition(lidState.open);

            }
            bot.setArmPosition(armState.init, armExtensionState.extending);
            bot.setArmState(armState.init);
        }

        if (operator.getRightY() > .1) {

            bot.outtakeSlide.setLeftOuttakeSlidePosition((int) bot.outtakeSlide.getLeftOuttakeSlideMotorPosition() - (int) (operator.getRightY() * 10));
            bot.outtakeSlide.setRightouttakeSlidePosition((int) bot.outtakeSlide.getRightOuttakeSlideMotorPosition() - (int) (operator.getRightY() * 10));
        }
        if (operator.getRightY() < -.1) {

            bot.outtakeSlide.setLeftOuttakeSlidePosition((int) bot.outtakeSlide.getLeftOuttakeSlideMotorPosition() - (int) (operator.getRightY() * 10));
            bot.outtakeSlide.setRightouttakeSlidePosition((int) bot.outtakeSlide.getRightOuttakeSlideMotorPosition() - (int) (operator.getRightY() * 10));
        }



    }
}
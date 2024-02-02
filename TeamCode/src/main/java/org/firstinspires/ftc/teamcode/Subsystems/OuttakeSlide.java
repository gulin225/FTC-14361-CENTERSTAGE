package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Commands.extensionState;
import org.firstinspires.ftc.teamcode.Commands.outtakeSlidesState;
import org.firstinspires.ftc.teamcode.util.robotConstants;

public class OuttakeSlide
{
    DcMotorEx rightouttakeSlide, leftouttakeSlide;
    double leftPower, rightPower, basePower, f, ffL, ffR, leftSlidePosition, rightSlidePosition;

    private final double ticks_in_degree = 384.539792388;

    public OuttakeSlide(HardwareMap hardwareMap) {
        rightouttakeSlide = hardwareMap.get(DcMotorEx.class, "rightOuttakeSlide");
        leftouttakeSlide = hardwareMap.get(DcMotorEx.class, "leftOuttakeSlide");

        rightouttakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftouttakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightouttakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftouttakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftouttakeSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftPower = 0;
        rightPower = 0;
        basePower = .9;

        rightouttakeSlide.setTargetPositionTolerance(5);
        leftouttakeSlide.setTargetPositionTolerance(5);
    }

    public void setOuttakeSlidePosition(extensionState extensionState, outtakeSlidesState outtakeSlidesState, double leftPidVal, double rightPidVal)
    {
        switch(extensionState)
        {
            case retracted:
                break;
            case extending:
            {
                switch (outtakeSlidesState)
                {
                    case MOSTHIGHOUT:
                        leftSlidePosition = getLeftOuttakeSlideMotorPosition();
                        rightSlidePosition = getRightOuttakeSlideMotorPosition();

                        ffL = Math.cos(Math.toRadians(robotConstants.outtakeSlide.MOSTHIGHLEFT/ticks_in_degree)) * f;
                        ffR = Math.cos(Math.toRadians(robotConstants.outtakeSlide.MOSTHIGHRIGHT/ticks_in_degree)) * f;

                        leftPower = leftPidVal + ffL;
                        rightPower = rightPidVal + ffR;

                        leftouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.MOSTHIGHLEFT);
                        rightouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.MOSTHIGHRIGHT);

                        leftouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        leftouttakeSlide.setPower(leftPower);
                        rightouttakeSlide.setPower(rightPower);
                        break;
                    case HIGHOUT:
                        leftSlidePosition = getLeftOuttakeSlideMotorPosition();
                        rightSlidePosition = getRightOuttakeSlideMotorPosition();

                        ffL = Math.cos(Math.toRadians(robotConstants.outtakeSlide.HIGHLEFT/ticks_in_degree)) * f;
                        ffR = Math.cos(Math.toRadians(robotConstants.outtakeSlide.HIGHRIGHT/ticks_in_degree)) * f;

                        leftPower = leftPidVal + ffL;
                        rightPower = rightPidVal + ffR;

                        leftouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.HIGHLEFT);
                        rightouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.HIGHRIGHT);

                        leftouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        leftouttakeSlide.setPower(leftPower);
                        rightouttakeSlide.setPower(rightPower);
                        break;
                    case MEDIUMOUT:
                        leftSlidePosition = getLeftOuttakeSlideMotorPosition();
                        rightSlidePosition = getRightOuttakeSlideMotorPosition();

                        ffL = Math.cos(Math.toRadians(robotConstants.outtakeSlide.MEDIUMLEFT/ticks_in_degree)) * f;
                        ffR = Math.cos(Math.toRadians(robotConstants.outtakeSlide.MEDIUMRIGHT/ticks_in_degree)) * f;

                        leftPower = leftPidVal + ffL;
                        rightPower = rightPidVal + ffR;

                        leftouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.MEDIUMLEFT);
                        rightouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.MEDIUMRIGHT);

                        leftouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        leftouttakeSlide.setPower(leftPower);
                        rightouttakeSlide.setPower(rightPower);
                        break;
                    case LOWMED:
                        leftSlidePosition = getLeftOuttakeSlideMotorPosition();
                        rightSlidePosition = getRightOuttakeSlideMotorPosition();

                        ffL = Math.cos(Math.toRadians(robotConstants.outtakeSlide.LOWMEDLEFT/ticks_in_degree)) * f;
                        ffR = Math.cos(Math.toRadians(robotConstants.outtakeSlide.LOWMEDRIGHT/ticks_in_degree)) * f;

                        leftPower = leftPidVal + ffL;
                        rightPower = rightPidVal + ffR;

                        leftouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.LOWMEDLEFT);
                        rightouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.LOWMEDRIGHT);

                        leftouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        leftouttakeSlide.setPower(leftPower);
                        rightouttakeSlide.setPower(rightPower);
                        break;
                    case LOWOUT:
                        leftSlidePosition = getLeftOuttakeSlideMotorPosition();
                        rightSlidePosition = getRightOuttakeSlideMotorPosition();

                        ffL = Math.cos(Math.toRadians(robotConstants.outtakeSlide.LOWLEFT/ticks_in_degree)) * f;
                        ffR = Math.cos(Math.toRadians(robotConstants.outtakeSlide.LOWRIGHT/ticks_in_degree)) * f;

                        leftPower = leftPidVal + ffL;
                        rightPower = rightPidVal + ffR;

                        leftouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.LOWLEFT);
                        rightouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.LOWRIGHT);

                        leftouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        leftouttakeSlide.setPower(leftPower);
                        rightouttakeSlide.setPower(rightPower);
                        break;
                    case AUTOLOWOUT:
                        leftSlidePosition = getLeftOuttakeSlideMotorPosition();
                        rightSlidePosition = getRightOuttakeSlideMotorPosition();

                        ffL = Math.cos(Math.toRadians(robotConstants.outtakeSlide.AUTOLOWLEFT/ticks_in_degree)) * f;
                        ffR = Math.cos(Math.toRadians(robotConstants.outtakeSlide.AUTOLOWRIGHT/ticks_in_degree)) * f;

                        leftPower = leftPidVal + ffL;
                        rightPower = rightPidVal + ffR;

                        leftouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.AUTOLOWLEFT);
                        rightouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.AUTOLOWRIGHT);

                        leftouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        leftouttakeSlide.setPower(leftPower);
                        rightouttakeSlide.setPower(rightPower);
                        break;
                    case STATION:
                        leftSlidePosition = getLeftOuttakeSlideMotorPosition();
                        rightSlidePosition = getRightOuttakeSlideMotorPosition();

                        ffL = Math.cos(Math.toRadians(robotConstants.outtakeSlide.GROUNDLEFT/ticks_in_degree)) * f;
                        ffR = Math.cos(Math.toRadians(robotConstants.outtakeSlide.GROUNDRIGHT/ticks_in_degree)) * f;

                        leftPower = leftPidVal + ffL;
                        rightPower = rightPidVal + ffR;

                        leftouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.GROUNDLEFT);
                        rightouttakeSlide.setTargetPosition(robotConstants.outtakeSlide.GROUNDRIGHT);

                        leftouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        leftouttakeSlide.setPower(leftPower);
                        rightouttakeSlide.setPower(rightPower);
                        break;
                }
            }

            case extended:
                break;
        }
    }
    public double getLeftOuttakeSlideMotorPosition()
    {
        return leftouttakeSlide.getCurrentPosition();
    }

    public double getRightOuttakeSlideMotorPosition()
    {
        return rightouttakeSlide.getCurrentPosition();
    }
    public void setPosition(int pos, double leftPidVal, double rightPidVal) {
        leftSlidePosition = getLeftOuttakeSlideMotorPosition();
        rightSlidePosition = getRightOuttakeSlideMotorPosition();

        ffL = Math.cos(Math.toRadians(pos/ticks_in_degree)) * f;
        ffR = Math.cos(Math.toRadians(pos/ticks_in_degree)) * f;

        leftPower = leftPidVal + ffL;
        rightPower = rightPidVal + ffR;

        leftouttakeSlide.setTargetPosition(pos);
        rightouttakeSlide.setTargetPosition(pos);

        leftouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftouttakeSlide.setPower(leftPower);
        rightouttakeSlide.setPower(rightPower);
    }
    public void setLeftOuttakeSlidePosition(int pos) {
        leftouttakeSlide.setTargetPosition(pos);

        leftouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftouttakeSlide.setPower(basePower);

    }
    public void setRightouttakeSlidePosition(int pos) {

        rightouttakeSlide.setTargetPosition(pos);


        rightouttakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        rightouttakeSlide.setPower(basePower);
    }
}


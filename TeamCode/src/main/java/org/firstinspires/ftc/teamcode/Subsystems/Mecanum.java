package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.mecanumState;

import java.text.DecimalFormat;

public class Mecanum
{
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private double leftFrontPower, leftRearPower, rightFrontPower,rightRearPower, rotY, rotX, rx, x, y, denominator;
    private double offset = 1.1;
    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    double integralSum = 0;

    double Kp = 0.02;
    double Ki = 0;
    double Kd = 1;
    double output, botHeading, error;


    DecimalFormat df = new DecimalFormat("#.##");
    // This rounds to two decimal places
    BNO055IMU imu;
    BNO055IMU.Parameters parameters;

    public Mecanum(HardwareMap hardwareMap)
    {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        // this is making a new object called 'parameters' that we use to hold the angle the imu is at
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    public void drive(GamepadEx gamepad1)
    {
        y = gamepad1.getLeftY();
        x = gamepad1.getLeftX();
        rx = gamepad1.getRightX();

        double botHeading = -imu.getAngularOrientation().firstAngle;

        rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        leftFrontPower = (rotY + rotX + rx) / denominator;
        leftRearPower = (rotY - rotX + rx) / denominator;
        rightFrontPower = (rotY - rotX - rx) / denominator;
        rightRearPower = (rotY + rotX - rx) / denominator;

    }

    public void driveAngleLock(mecanumState mecanumState, GamepadEx gamepad1){
        switch (mecanumState){
            case NORMAL:
                y = gamepad1.getLeftY();
                x = gamepad1.getLeftX();
                rx = gamepad1.getRightX();

                botHeading = -imu.getAngularOrientation().firstAngle;

                rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
                rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

                denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                leftFrontPower = (rotY + rotX + rx) / denominator;
                leftRearPower = (rotY - rotX + rx) / denominator;
                rightFrontPower = (rotY - rotX - rx) / denominator;
                rightRearPower = (rotY + rotX - rx) / denominator;
                break;
            case TOREDBACKBOARD:
                y = gamepad1.getLeftY();
                x = gamepad1.getLeftX();
                // double error = angleWrap(Math.toRadians(90) - imu.getAngularOrientation().firstAngle);
                // rx = .1*(Math.toRadians(90)-imu.getAngularOrientation().firstAngle);
                // rx = gamepad1.getRightX(); // 0.01 * (des_angle - curr_angle)
                error = smallestAngleDifference(90, imu.getAngularOrientation().firstAngle * (180/Math.PI));


                timer.reset();
                output = (error * -Kp) + (imu.getAngularVelocity().zRotationRate * Kd) + (integralSum * Ki);
                rx = output;

                botHeading = -imu.getAngularOrientation().firstAngle;



                rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
                rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

                denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                leftFrontPower = (rotY + rotX + rx) / denominator;
                leftRearPower = (rotY - rotX + rx) / denominator;
                rightFrontPower = (rotY - rotX - rx) / denominator;
                rightRearPower = (rotY + rotX - rx) / denominator;
                break;
            case TOBLUEBACKBOARD:
                y = gamepad1.getLeftY();
                x = gamepad1.getLeftX();
                // double error = angleWrap(Math.toRadians(90) - imu.getAngularOrientation().firstAngle);
                // rx = .1*(Math.toRadians(90)-imu.getAngularOrientation().firstAngle);
                // rx = gamepad1.getRightX(); // 0.01 * (des_angle - curr_angle)
                error = smallestAngleDifference(270, imu.getAngularOrientation().firstAngle * (180/Math.PI));


                timer.reset();
                output = (error * -Kp) + (imu.getAngularVelocity().zRotationRate * Kd) + (integralSum * Ki);
                rx = output;

                botHeading = -imu.getAngularOrientation().firstAngle;


                rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
                rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

                denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                leftFrontPower = (rotY + rotX + rx) / denominator;
                leftRearPower = (rotY - rotX + rx) / denominator;
                rightFrontPower = (rotY - rotX - rx) / denominator;
                rightRearPower = (rotY + rotX - rx) / denominator;
break;

        }
    }

    public void setMotorPower()
    {
        leftFront.setPower(leftFrontPower * offset);
        leftRear.setPower(leftRearPower * offset );
        rightFront.setPower(rightFrontPower * offset);
        rightRear.setPower(rightRearPower * offset );
    }
    public void setSlowDownMotorPower()
    {
        offset = 1.1 * 0.65;
    }

    public void setFullPower()
    {
        offset = 1.1;
    }

    public void resetIMU()
    {
        imu.initialize(parameters);
    }

    public double smallestAngleDifference(double current, double desired){
        current = angleWrap360(current);
        desired = angleWrap360(desired);

        double difference = current - desired;
        if(difference > 180.0){
            difference = -(360.0 - difference);
        }
        else if(difference < -180.0){
            difference = 360.0 + difference;
        }
        return difference;
    }
    public double angleWrap360(double angle){
        angle = angle % 360;
        if(angle < 0.0){
            angle += 360;
        }
        return angle;

    }

    public double angleWrap180(double angle){
       angle = angleWrap360(angle);
        if(angle > 180.0){
            angle-=360;
        } else if(angle < -180.0) {
            angle+=360;
        }
        return angle;
    }
}

package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.text.DecimalFormat;

public class Mecanum
{
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private double leftFrontPower, leftRearPower, rightFrontPower,rightRearPower, rotY, rotX, rx, x, y, denominator;
    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    double integralSum = 0;

    double Kp = 0.025;
    double Ki = 0;
    double Kd = .1;
    private double offset = 1.1;
    Telemetry telemetry;

    DecimalFormat df = new DecimalFormat("#.##");
    // This rounds to two decimal places
    BNO055IMU imu;
    BNO055IMU.Parameters parameters;

    public Mecanum(HardwareMap hardwareMap, Telemetry telemetry)
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
        this.telemetry = telemetry;

    }

    public void drive(GamepadEx gamepad1)
    {


        y = gamepad1.getLeftY();
        x = gamepad1.getLeftX();
       // double error = angleWrap(Math.toRadians(90) - imu.getAngularOrientation().firstAngle);
       // rx = .1*(Math.toRadians(90)-imu.getAngularOrientation().firstAngle);
       // rx = gamepad1.getRightX(); // 0.01 * (des_angle - curr_angle)
        double error = angleWrap(Math.toRadians(90) - imu.getAngularOrientation().firstAngle);

        integralSum += error * timer.seconds();
        double derivative = -(angleWrap(error - lastError)) / (timer.seconds());
        lastError = error;
        telemetry.addData("Error: ", error);

        timer.reset();
        double output = (error * Kp) + (imu.getAngularVelocity().zRotationRate * Kd) + (integralSum * Ki);
        double rx = output;

        double botHeading = -imu.getAngularOrientation().firstAngle;



        rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        leftFrontPower = (rotY + rotX + rx) / denominator;
        leftRearPower = (rotY - rotX + rx) / denominator;
        rightFrontPower = (rotY - rotX - rx) / denominator;
        rightRearPower = (rotY + rotX - rx) / denominator;
        telemetry.addData("Target IMU Angle", 90);
        telemetry.addData("Target IMU Angle", 90);
        telemetry.addData("Derivative", imu.getAngularVelocity().zRotationRate);
        telemetry.addData("Derivative", error);


        telemetry.update();

    }

    public void setMotorPower()
    {
        leftFront.setPower(leftFrontPower* 1);
        leftRear.setPower(leftRearPower * 1 );
        rightFront.setPower(rightFrontPower * 1);
        rightRear.setPower(rightRearPower * 1 );
    }
    public void setSlowDownMotorPower()
    {
        offset = 1.1 * 0.65;
    }

    public void setFullPower()
    {
        offset = 1.1;
    }
    public double angleWrap(double radians){
        while(radians > Math.PI){
            radians-=2*Math.PI;
        }
        while(radians < -Math.PI){
            radians+=2*Math.PI;
        }
        return radians;

    }
    public double PIDControl(double refrence, double state) {
        double error = angleWrap(refrence - state);
        telemetry.addData("Error: ", error);

        integralSum += error * timer.seconds();
        double derivative = -(error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }

    public void resetIMU()
    {
        imu.initialize(parameters);
    }
}

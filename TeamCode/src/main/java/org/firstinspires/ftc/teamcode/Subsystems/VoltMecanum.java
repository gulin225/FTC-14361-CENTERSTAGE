package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.text.DecimalFormat;

public class VoltMecanum
{
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private double leftFrontPower, leftRearPower, rightFrontPower,rightRearPower, rotY, rotX, rx, x, y, denominator;
    private double voltMult = 0;

    private HardwareMap voltMap;

    DecimalFormat df = new DecimalFormat("#.##");
    // This rounds to two decimal places
    BNO055IMU imu;
    BNO055IMU.Parameters parameters;

    public VoltMecanum(HardwareMap hardwareMap)
    {
        voltMap = hardwareMap;
        leftFront = voltMap.get(DcMotorEx.class, "leftFront");
        leftRear = voltMap.get(DcMotorEx.class, "leftRear");
        rightFront = voltMap.get(DcMotorEx.class, "rightFront");
        rightRear = voltMap.get(DcMotorEx.class, "rightRear");

        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);

        imu = voltMap.get(BNO055IMU.class, "imu");
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

    public void setMotorPower()
    {
        adjustVoltMult();
        leftFront.setPower(leftFrontPower * voltMult);
        leftRear.setPower(leftRearPower * voltMult);
        rightFront.setPower(rightFrontPower * voltMult);
        rightRear.setPower(rightRearPower * voltMult);
    }

    public double getBatteryVoltage()
    {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : voltMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    public double getBatteryMult(){
        adjustVoltMult();
        return voltMult * 100;
    }

    public double getFLSpeed(){
        return leftFront.getVelocity();
    }

    public double getBLSpeed(){
        return leftRear.getVelocity();
    }

    public double getFRSpeed(){
        return rightFront.getVelocity();
    }

    public double getBRSpeed(){
        return rightRear.getVelocity();
    }
    public void adjustVoltMult(){

        if(getBatteryVoltage() >= 14 )
            voltMult = 0.65;
        else if (14 > getBatteryVoltage() && getBatteryVoltage() >= 13.75 )
            voltMult = .70;
        else if (13.75 > getBatteryVoltage() && getBatteryVoltage() >= 13.50 )
            voltMult = .75;
        else if (13.50 > getBatteryVoltage() && getBatteryVoltage() >= 13.25 )
            voltMult = .80;
        else if (13.25 > getBatteryVoltage() && getBatteryVoltage() >= 12.75 )
            voltMult = .90;
        else if (12.75 > getBatteryVoltage() && getBatteryVoltage() >= 12 )
            voltMult = 1;
    }


    public void resetIMU()
    {
        imu.initialize(parameters);
    }
}

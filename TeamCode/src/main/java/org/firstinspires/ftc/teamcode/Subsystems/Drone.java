package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drone
{
    private ServoEx droneServo;
    public Drone(HardwareMap hardwareMap)
    {
        droneServo = new SimpleServo(hardwareMap, "droneServo",0, 360, AngleUnit.DEGREES);
    }

    public void launch()
    {
        droneServo.setPosition(.5);
    }

    public void resetDrone(){
        droneServo.setPosition(1);
    }

    public double getInfo(){
      return  droneServo.getPosition();
    }
}


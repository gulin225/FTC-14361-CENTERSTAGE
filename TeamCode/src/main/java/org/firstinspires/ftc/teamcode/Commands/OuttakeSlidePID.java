//package org.firstinspires.ftc.teamcode.Commands;
//
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import java.util.ArrayList;
//
//public class OuttakeSlidePID {
//    private double kP, kI, kD, target, totalError, lastError, currentError, currentTime, lastTime;
//    private ArrayList<Double> error;
//    private ElapsedTime timer;
//
//    public OuttakeSlidePID(double kP, double kI, double kD, double target)
//    {
//        kP = kP;
//        kI = kI;
//        kD = kD;
//        target = target;
//        error = new ArrayList<Double>();
//        timer = new ElapsedTime();
//    }
//
//    public double getCorrection(double error, double time)
//    {
//        lastTime = time;
//
//        double output = getP(error) + getI(error) + getD(error);
//
//        totalError += error;
//        lastError = error;
//
//        return output;
//    }
//    public double getP(double error)
//    {
//        return kP * error;
//    }
//
//    public double getI(double error ) {
//        return kI * totalError;
//    }
//
//    public double getD(double error, double time)
//    {
//        currentTime = time;
//        currentError = error;
//
//        return kD * (currentError - lastError)/(currentTime/lastTime);
//    }
//}

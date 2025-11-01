package org.firstinspires.ftc.teamcode.mechanisms;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import kotlin.math.UMathKt;

public class MecDrive {
    private DcMotor FL,BL,FR,BR;

    private IMU imu;

    private VoltageSensor batteryVoltageSensor;  // ← Now private and initialized

    private static final double FULLYCHARGEDBATTERYVOLTAGE = 14.0;

    public void init(HardwareMap hwMap) {
        FL = hwMap.get(DcMotor.class, "FL");
        BL = hwMap.get(DcMotor.class, "BL");
        FR = hwMap.get(DcMotor.class, "FR");
        BR = hwMap.get(DcMotor.class, "BR");

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        batteryVoltageSensor = hwMap.get(VoltageSensor.class, "Control Hub");


        imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrinentation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        );
        imu.initialize(new IMU.Parameters(RevOrinentation));
    }

    public void drive(double forward, double strafe, double rotate,double maxPower) {
        double FLPower = forward + strafe + rotate;
        double BLPower = forward - strafe + rotate;
        double FRPower = forward - strafe - rotate;
        double BRPower = forward + strafe - rotate;

        maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower,Math.abs(FLPower));
        maxPower = Math.max(maxPower,Math.abs(BLPower));
        maxPower = Math.max(maxPower,Math.abs(FRPower));
        maxPower = Math.max(maxPower,Math.abs(BLPower));

        FL.setPower((maxSpeed * (FLPower/maxPower)));
        BL.setPower((maxSpeed * (BLPower/maxPower)));
        FR.setPower((maxSpeed * (FRPower/maxPower)));
        BR.setPower((maxSpeed * (BRPower/maxPower)));
    }

//    public void driveFieldRelative(double foward, double strafe, double rotate) {
//        double theta = Math.atan2(strafe,foward);
//        double r = Math.hypot(strafe, foward);
//
//        theta = AngleUnit.normalizeRadians(theta -
//                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
//
//        double newForward = r * Math.sin(theta);
//        double newStrafe = r * Math.cos(theta);
//
//        this.drive(newForward,newStrafe,rotate);
//    }

    public double getCompensatedPower(double basePower) {
        double referenceVoltage = FULLYCHARGEDBATTERYVOLTAGE;
        double currentVoltage = batteryVoltageSensor.getVoltage();
        double compensatedPower = basePower * (referenceVoltage / currentVoltage);
        return Math.min(1.0, compensatedPower); // Ensure power doesn't exceed max limit
    }
}

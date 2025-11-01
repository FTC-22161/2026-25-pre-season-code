package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Launcher {
    private DcMotor LL, LR;
    private Servo gate;
    private VoltageSensor batteryVoltageSensor;  // ← Now private and initialized

    private static final double FULLYCHARGEDBATTERYVOLTAGE = 14.0;

    public void init(HardwareMap hwMap) {
        // Initialize motors and servo
        LL = hwMap.get(DcMotor.class, "LL");
        LR = hwMap.get(DcMotor.class, "LR");
        gate = hwMap.get(Servo.class, "gate");

        LL.setDirection(DcMotor.Direction.REVERSE);
        gate.setDirection(Servo.Direction.REVERSE);

        // FIX: Initialize the voltage sensor!
        batteryVoltageSensor = hwMap.get(VoltageSensor.class, "Control Hub");// ← CHANGE "battery" to your config name!

        // Optional: Add telemetry in OpMode to verify it works
    }

    public double getCompensatedPower(double basePower) {
        double referenceVoltage = FULLYCHARGEDBATTERYVOLTAGE;
        double currentVoltage = batteryVoltageSensor.getVoltage();
        double compensatedPower = basePower * (referenceVoltage / currentVoltage);
        return Math.min(1.0, compensatedPower); // Ensure power doesn't exceed max limit
    }

        public void go(double speed) {
        double power = getCompensatedPower(speed);
        LL.setPower(power);
        LR.setPower(power);
    }

    public void pos(double pos) {
        gate.setPosition(pos);
    }
}

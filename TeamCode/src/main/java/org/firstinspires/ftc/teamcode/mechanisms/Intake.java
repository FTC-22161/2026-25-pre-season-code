package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Intake {
    private DcMotor Intake;

    private VoltageSensor batteryVoltageSensor;

    private static final double FULLYCHARGEDBATTERYVOLTAGE = 14.0;

    public void init(HardwareMap hwMap) {

        Intake = hwMap.get(DcMotor.class, "Intake");
    }

    public void go(double speed) {
        Intake.setPower(getCompensatedPower(speed));
    }
    public double getCompensatedPower(double basePower) {
        double referenceVoltage = FULLYCHARGEDBATTERYVOLTAGE;
        double currentVoltage = batteryVoltageSensor.getVoltage();
        double compensatedPower = basePower * (referenceVoltage / currentVoltage);
        return Math.min(1.0, compensatedPower); // Ensure power doesn't exceed max limit
    }
}

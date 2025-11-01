package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Intake {
    private DcMotor Intake;

    private VoltageSensor batteryVoltageSensor;
    double FULLYCHARGEDBATTERYVOLTAGE = 14;

    public void init(HardwareMap hwMap) {

        Intake = hwMap.get(DcMotor.class, "Intake");
        batteryVoltageSensor = hwMap.get(VoltageSensor.class, "Control Hub");
    }
    public double getCompensatedPower(double basePower) {
        double referenceVoltage = FULLYCHARGEDBATTERYVOLTAGE;
        double currentVoltage = batteryVoltageSensor.getVoltage();
        double compensatedPower = basePower * (referenceVoltage / currentVoltage);
        return Math.min(1.0, compensatedPower); // Ensure power doesn't exceed max limit
    }

    public void go(double speed) {
        speed = getCompensatedPower(speed);
        Intake.setPower(getCompensatedPower(speed));
    }

}

package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Launcher {
    private DcMotor LL,LR;

    private Servo gate;

    private VoltageSensor batteryVoltageSensor;

    private static final double FULLYCHARGEDBATTERYVOLTAGE = 14.0;

    public void init(HardwareMap hwMap) {
        LL = hwMap.get(DcMotor.class, "LL");
        LR = hwMap.get(DcMotor.class, "LR");
        gate = hwMap.get(Servo.class, "gate");

        LL.setDirection(DcMotor.Direction.REVERSE);
        gate.setDirection(Servo.Direction.REVERSE);
    }

    public void go(double speed) {
        LL.setPower(getCompensatedPower(speed));
        LR.setPower(getCompensatedPower(speed));

    }
    public void pos(double pos) {
        gate.setPosition(pos);
    }
    public double getCompensatedPower(double basePower) {
        double referenceVoltage = FULLYCHARGEDBATTERYVOLTAGE;
        double currentVoltage = batteryVoltageSensor.getVoltage();
        double compensatedPower = basePower * (referenceVoltage / currentVoltage);
        return Math.min(1.0, compensatedPower); // Ensure power doesn't exceed max limit
    }
}

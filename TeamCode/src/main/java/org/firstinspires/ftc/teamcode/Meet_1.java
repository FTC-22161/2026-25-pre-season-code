package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.ApirlTagWebcam;


@TeleOp(group = "Meets")
public class Meet_1 extends LinearOpMode {
    MecDrive drive = new MecDrive();

    Launcher launch = new Launcher();

    Intake intake = new Intake();
    double forward,strafe,rotate;

    VoltageSensor batteryVoltageSensor;

    private static final double FULLYCHARGEDBATTERYVOLTAGE = 14.0;

    @Override
    public void runOpMode() {

        drive.init(hardwareMap);

        launch.init(hardwareMap);

        intake.init(hardwareMap);


        waitForStart();
        if (opModeIsActive()) {

            while (opModeIsActive()) {
                drive();

                if (gamepad2.right_bumper) {
                    launch.go(0);
                }else {
                    launch.go(.6);}

                if (gamepad2.x) {
                    launch.pos(.6);
                }else {
                    launch.pos(.07);}

                if (gamepad2.right_trigger > 0) {
                    intake.go(1);
                }else if (gamepad2.left_trigger > 0 ) {
                    intake.go(-1);
                }else {
                    intake.go(0);
                }
            }
        }
    }
    public void drive() {
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        drive.driveFieldRelative(forward,strafe,rotate);
    }    public double getCompensatedPower(double basePower) {
        double referenceVoltage = FULLYCHARGEDBATTERYVOLTAGE;
        double currentVoltage = batteryVoltageSensor.getVoltage();
        double compensatedPower = basePower * (referenceVoltage / currentVoltage);
        return Math.min(1.0, compensatedPower); // Ensure power doesn't exceed max limit
    }
}

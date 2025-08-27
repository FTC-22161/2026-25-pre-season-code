package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Test")
public class Test extends LinearOpMode {
    DcMotor FL,BL,BR,FR;

    double DriveSpeed;

    @Override
    public void runOpMode() throws InterruptedException {
        FL = hardwareMap.get(DcMotor.class , "FL");
        BL = hardwareMap.get(DcMotor.class , "BL");
        BR = hardwareMap.get(DcMotor.class , "BR");
        FR = hardwareMap.get(DcMotor.class , "FR");


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            FL.setPower(DriveSpeed * ((-gamepad1.left_stick_x) - (-gamepad1.left_stick_y) - (gamepad1.right_stick_x)));
            BL.setPower(DriveSpeed * ((gamepad1.left_stick_x) - (-gamepad1.left_stick_y) - (gamepad1.right_stick_x)));
            FR.setPower(DriveSpeed * ((-gamepad1.left_stick_x) - (gamepad1.left_stick_y) - (gamepad1.right_stick_x)));
            BR.setPower(DriveSpeed * ((gamepad1.left_stick_x) - (gamepad1.left_stick_y) - (gamepad1.right_stick_x)));


            if (gamepad1.right_bumper) {
                DriveSpeed = 1;
            }
            else {
                DriveSpeed = 0.4;
        }
    }
    }
}
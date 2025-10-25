package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Meet_0")
public class Meet_1 extends LinearOpMode {

    private DcMotor FL,FR,BL,BR,LL,LR,Intake;

    private CRServo thing;

    private Servo gate;

    public double DriveSpeed;
    @Override
    public void runOpMode() {

        thing = hardwareMap.get(CRServo.class, "hi");
        FL = hardwareMap.get(DcMotor.class, "FL");

        FR = hardwareMap.get(DcMotor.class, "FR");

        BL = hardwareMap.get(DcMotor.class, "BL");

        BR = hardwareMap.get(DcMotor.class, "BR");

        LL = hardwareMap.get(DcMotor.class, "LL");

        LR = hardwareMap.get(DcMotor.class, "LR");

        Intake = hardwareMap.get(DcMotor.class, "Intake");

        gate = hardwareMap.get(Servo.class, "gate");
        double fire = 0;
        double speed = .525;
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // OpMode loop
                drive();
                launch();
                intake();



                if (gamepad2.right_bumper) {
                    LL.setPower(0);
                    LR.setPower(0);
                }else {
                    LL.setPower(-speed);
                    LR.setPower(speed);
                }
            }
        }
    }
    public void intake() {
        if (gamepad2.right_trigger > 0) {
            Intake.setPower(1);
            thing.setPower(1);
        }else if (gamepad2.left_trigger > 0) {
            Intake.setPower(-1);
            thing.setPower(-1);
        }else {
            Intake.setPower(0);
        }
    }
    public void launch() {
        if (gamepad2.x) {
            gate.setPosition(0);
        }
//        double fire = 0;
//        double speed = .5;
//        if (gamepad2.right_bumper) {
//            LL.setPower(0);
//            LR.setPower(0);
//        }
//        LL.setPower(-speed);
//        LR.setPower(speed);
//
//        if (gamepad2.dpad_up) {
//             speed = .5;
//        }else if (gamepad2.dpad_down) {
//          speed = .75;
//       }
    }
    public void drive() {
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

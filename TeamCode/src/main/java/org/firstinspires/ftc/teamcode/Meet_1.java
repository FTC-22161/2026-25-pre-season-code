package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.ApirlTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@TeleOp(name = "MEET 1",group = "Meets")
public class Meet_1 extends LinearOpMode {
    MecDrive drive = new MecDrive();

    DcMotor LL,LR;

    Launcher launch = new Launcher();

    Intake intake = new Intake();


    double forward,strafe,rotate;

    @Override
    public void runOpMode() {
        LL = hardwareMap.get(DcMotor.class, "LL");
        LR = hardwareMap.get(DcMotor.class, "LR");
        LL.setDirection(DcMotor.Direction.REVERSE);

        drive.init(hardwareMap);

        launch.init(hardwareMap);

        intake.init(hardwareMap);


        double speed = .4;
        boolean mode = true;


        waitForStart();
        if (opModeIsActive()) {

            while (opModeIsActive()) {

                forward = gamepad1.left_stick_y;
                strafe = gamepad1.left_stick_x;
                rotate = gamepad1.right_stick_x;

                drive.drive(forward,strafe,rotate,1);
                if (gamepad1.a) {
                    mode = !mode;
                }
                if (mode == true) {
                    drive.drive(forward,strafe,rotate,1);
                }else if (mode == false )  {
                    drive.drive(forward,strafe,rotate,.5);
                }

                if (gamepad2.dpad_up && !(speed > .6) && gamepad2.dpadUpWasPressed()) {
                    speed += .05;
                }else if (gamepad2.dpad_down && !(speed < .3) && gamepad2.dpadDownWasPressed()) {
                    speed -= .05;
                }

                telemetry.addData("Speed", speed);
                telemetry.update();



                if (gamepad2.right_bumper) {
                    launch.go(0);
//                    LL.setPower(0);
//                    LR.setPower(0);
                }else {
                  launch.go(speed);
//                    LL.setPower(getCompensatedPower(speed));
//                    LR.setPower(getCompensatedPower(speed));
                   }//speeds .4 to .49 from up close and .6? from far

                if (gamepad2.x) {
                    launch.pos(.6);
                }else {
                    launch.pos(.01);}

                if (gamepad2.right_trigger > 0) {
                    intake.go((1));
                }else if (gamepad2.left_trigger > 0 ) {
                    intake.go(-1);
                }else {
                    intake.go(0);
                }
            }
        }
    }
}

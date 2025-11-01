package org.firstinspires.ftc.teamcode.Auto;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.CharArrayWriter;

import javax.sql.StatementEvent;


@Autonomous(name = "Meet_1_auto")
public class Meet_1 extends LinearOpMode {

    DcMotor FL,BL,BR,FR,LL,LR,Intake;
    Servo gate;

    static final double COUNTS_PER_MOTOR_REV2 = 145.1;

    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;

    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double COUNTS_PER_INCH2 = (COUNTS_PER_MOTOR_REV2 * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    VoltageSensor batteryVoltageSensor;
    double FULLYCHARGEDBATTERYVOLTAGE = 14;
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {


        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        LL = hardwareMap.get(DcMotor.class, "LL");
        LR = hardwareMap.get(DcMotor.class, "LR");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        gate = hardwareMap.get(Servo.class, "gate");
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LL.setDirection(DcMotor.Direction.REVERSE);
        batteryVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");


        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                launcher(.58);
                sleep(1000);
                intake(1, 2);
//                sleep(100);
//                Gate(.01,2);
//                intake(1,2);
//                Gate(.6,2);
//                intake(1,2);'
                sleep(1000);
                encoderDrive(1,10,-10,-10,10,2);




            }
        }
    }
    public void launcher(double speed) {
        LL.setPower(getCompensatedPower(speed));
        LR.setPower(getCompensatedPower(speed));
    }
    public void Gate(double gatepos, double timeoutS) {
        while (opModeIsActive() && (runtime.seconds() < timeoutS)) {
            gate.setPosition(gatepos);
        }
    }
    public void intake(double speed, double timeoutS) {
        while (opModeIsActive() && (runtime.seconds() < timeoutS)) {
            Intake.setPower(speed);
        }
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double rightBinches, double leftBinches,
                             double timeoutS) {
        int newLeftTarget;
        int newLeftBTarget;
        int newRightBTarget;
        int newRightTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = FL.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = FR.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newLeftBTarget = BL.getCurrentPosition() + (int) (leftBinches * COUNTS_PER_INCH);
            newRightBTarget = BR.getCurrentPosition() + (int) (rightBinches * COUNTS_PER_INCH);

            FL.setTargetPosition(newLeftTarget);
            FR.setTargetPosition(newRightTarget);
            BL.setTargetPosition(newLeftBTarget);
            BR.setTargetPosition(newRightBTarget);

            // Turn On RUN_TO_POSITION
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            FL.setPower(Math.abs(speed));
            BL.setPower(Math.abs(speed));
            FR.setPower(Math.abs(speed));
            BR.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (FL.isBusy() && BL.isBusy() && FR.isBusy() && BR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget, newRightBTarget, newLeftBTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        FL.getCurrentPosition(),
                        BL.getCurrentPosition());
                BR.getCurrentPosition();
                FR.getCurrentPosition();
                telemetry.update();
            }

            FL.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);


            // Turn off RUN_TO_POSITION
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public double getCompensatedPower(double basePower) {
        double referenceVoltage = FULLYCHARGEDBATTERYVOLTAGE;
        double currentVoltage = batteryVoltageSensor.getVoltage();
        double compensatedPower = basePower * (referenceVoltage / currentVoltage);
        return Math.min(1.0, compensatedPower); // Ensure power doesn't exceed max limit
    }


}

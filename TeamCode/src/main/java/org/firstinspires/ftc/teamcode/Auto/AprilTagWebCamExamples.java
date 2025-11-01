package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.mechanisms.ApirlTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@Autonomous(name = "AprilTagWebCamExamples", group = "tests")
public class AprilTagWebCamExamples extends OpMode {
    ApirlTagWebcam apirlTagWebcam = new ApirlTagWebcam();

    @Override
    public void init() {
        apirlTagWebcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        apirlTagWebcam.update();
        AprilTagDetection id21 = apirlTagWebcam.getTagBySpecificID(21);

    }
}

package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.support.SRSHub;

@TeleOp(name = "SRS Test")
public class SRSTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // --- Configure what’s plugged into the SRS Hub ---
        SRSHub.Config config = new SRSHub.Config();

        // Flywheel encoder on port E1 → channel 1, Quadrature type
        config.setEncoder(1, SRSHub.Encoder.QUADRATURE);

        // goBILDA Pinpoint on I2C port 1
        config.addI2CDevice(
                1,
                new SRSHub.GoBildaPinpoint(
                        -50,      // X offset mm (adjust for your robot)
                        -75,      // Y offset mm
                        19.89f,   // wheel diameter mm
                        SRSHub.GoBildaPinpoint.EncoderDirection.FORWARD,
                        SRSHub.GoBildaPinpoint.EncoderDirection.FORWARD
                )
        );

        // --- Get the SRS Hub from the hardware map ---
        SRSHub hub = hardwareMap.get(SRSHub.class, "srsHub");

        // Initialize hub with configuration
        hub.init(config);

        // Wait until hub reports ready
        while (!hub.ready() && !isStopRequested()) { idle(); }

        waitForStart();

        while (opModeIsActive()) {
            hub.update();  // refresh readings

            telemetry.addData("Encoder E1 Pos", hub.readEncoder(1).position);
            telemetry.addData("Encoder E1 Vel", hub.readEncoder(1).velocity);

            SRSHub.GoBildaPinpoint pinpoint = hub.getI2CDevice(
                    1, SRSHub.GoBildaPinpoint.class
            );

            if (!pinpoint.disconnected && pinpoint.deviceStatus == 1) {
                telemetry.addData("X (mm)", pinpoint.xPosition);
                telemetry.addData("Y (mm)", pinpoint.yPosition);
                telemetry.addData("Heading (rad)", pinpoint.hOrientation);
            }

            telemetry.update();
        }
    }
}

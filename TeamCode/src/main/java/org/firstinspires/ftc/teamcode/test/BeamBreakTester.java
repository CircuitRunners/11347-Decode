package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.BeamBreakHelper;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StaticShooter;

@Config
@TeleOp(name = "BeamBreak Tester", group = "Tuning")
public class BeamBreakTester extends LinearOpMode {

    private BeamBreakHelper intakeBeam, outtakeBeam;
    private StaticShooter shooter;
    private IntakeSubsystem intake;

    public static double TARGET = 0;
    public static boolean TRANSFER = false;

    @Override
    public void runOpMode() throws InterruptedException {

        intakeBeam = new BeamBreakHelper(hardwareMap, "intakeBeamBreak", 3);
        outtakeBeam = new BeamBreakHelper(hardwareMap, "outtakeBeamBreak", 0);

        shooter = new StaticShooter(hardwareMap, telemetry);
        shooter.setTargetRPM(0);

        intake = new IntakeSubsystem(hardwareMap);

        // preload 3 for testing
        // your new subsystem supports manual preload by directly setting internal count
        // so we add a helper here:
        for (int i = 0; i < 3; i++) intakeBeam.update();

        telemetry.addLine("BeamBreak Tester Initialized");
        telemetry.addLine("Press PLAY to begin");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            intakeBeam.update();
            outtakeBeam.update();
            shooter.update();

            shooter.setTargetRPM(TARGET);

            if (TRANSFER) intake.transfer();
            else intake.stop();

            if (outtakeBeam.getBallCount() >= 3) {
                intakeBeam.resetBallCount();
                outtakeBeam.resetBallCount();
            }

            telemetry.addLine("=== INTAKE BEAM ===");
            telemetry.addData("Raw", intakeBeam.isBeamBroken() ? "BROKEN" : "CLEAR");
            telemetry.addData("Stable Broken", intakeBeam.isBeamStable());
            telemetry.addData("Balls Passed", intakeBeam.getBallCount());

            telemetry.addLine("");

            telemetry.addLine("=== OUTTAKE BEAM ===");
            telemetry.addData("Raw", outtakeBeam.isBeamBroken() ? "BROKEN" : "CLEAR");
            telemetry.addData("Stable Broken", outtakeBeam.isBeamStable());
            telemetry.addData("Balls Passed", outtakeBeam.getBallCount());

            telemetry.addLine("");

            telemetry.addLine("=== BALL INVENTORY ===");
            telemetry.addData("Balls In Robot",
                    intakeBeam.getBallCount() - outtakeBeam.getBallCount());

            telemetry.update();
        }
    }
}

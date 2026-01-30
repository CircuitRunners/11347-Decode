package org.firstinspires.ftc.teamcode.auto.AutoPaths.BlueAutos;

import com.bylazar.configurables.annotations.Configurable;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Configurable
@Autonomous(name = "Final Blue Autos", group = "Blue Autos", preselectTeleOp="MainTeleOp")
public class BlueAutos extends SelectableOpMode {

    public BlueAutos() {
        super("Select an Auto", s -> {
            s.add("Blue Far Auto 6", BlueSideAutoFar6::new);
            s.add("Blue Far Auto 9 (Loading Zone)", BlueSideAutoFar9::new);
            s.add("Blue Far Auto 9 (Middle)", BlueSideAlt2::new);
            s.add("Blue Close Auto 9", BlueSideClose9::new);
            s.add("Blue Close Auto 12", BlueSideClose12::new);
        });
    }}



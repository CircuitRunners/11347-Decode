package org.firstinspires.ftc.teamcode.auto.AutoPaths;

import com.bylazar.configurables.annotations.Configurable;

import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Configurable
@Autonomous(name = "Final Red Autos", group = "Red Autos", preselectTeleOp="MainTeleOp")
public class RedAutos extends SelectableOpMode {

    public RedAutos() {
        super("Select an Auto", s -> {
            s.add("Red Far Auto", RedSideAutoFar::new);
            s.add("Red Close Auto", RedSideClose::new);
        });
        }}

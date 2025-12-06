package org.firstinspires.ftc.teamcode.auto.AutoPaths;

import com.bylazar.configurables.annotations.Configurable;

import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Configurable
@Autonomous(name = "Final Red Autos", group = "Red Autos", preselectTeleOp="MainTeleOp")
public class RedAutos extends SelectableOpMode {

    public RedAutos() {
        super("Select an Auto", s -> {
            s.add("Red Far Auto 9 (Observation Zone)", RedSideAutoFar9::new);
            s.add("Red Far Auto 9 (Middle)", RedSideAlt2::new);
            s.add("Red Close Auto 9", RedSideClose9::new);
            s.add("Red Close Auto 12", RedSideClose12::new);
        });
    }
}

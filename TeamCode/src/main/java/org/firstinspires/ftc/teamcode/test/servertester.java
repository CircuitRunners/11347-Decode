package org.firstinspires.ftc.teamcode.test;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="servo test hi~")
@Config
public class servertester extends OpMode {
    private Servo testServo;
    public static double position = 0.5;
    public static double INCREMENT = 0.01; // Amount to increment servo position
    public static double MAX_POSITION = 0.35;  //right = 0.32
    private final double MIN_POSITION = 0.0;
    public static boolean reversed = false;

    @Override
    public void init() {
        // Initialize the servo (name it "testServo" in your hardware configuration)
        testServo = hardwareMap.get(Servo.class, "testServo");
        //testServo.setDirection(Servo.Direction.REVERSE);

        testServo.setPosition(position);

        telemetry.addData("ServoTester", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Increment the servo position when the "A" button is pressed
        if (gamepad1.a && position < MAX_POSITION) {
            position += INCREMENT;
            if (position > MAX_POSITION) {
                position = MAX_POSITION;
            }
        }

        // Decrement the servo position when the "B" button is pressed
        if (gamepad1.b && position > MIN_POSITION) {
            position -= INCREMENT;
            if (position < MIN_POSITION) {
                position = MIN_POSITION;
            }
        }

        if (reversed) {
            testServo.setDirection(Servo.Direction.REVERSE);
        } else {
            testServo.setDirection(Servo.Direction.FORWARD);
        }

        // Update the servo position
        testServo.setPosition(position);

        // Display the current servo position
        telemetry.addData("Servo Position", "%.2f", position);
        telemetry.update();
    }


}

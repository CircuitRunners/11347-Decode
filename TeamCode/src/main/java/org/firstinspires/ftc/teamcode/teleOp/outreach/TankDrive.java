//package org.firstinspires.ftc.teamcode.teleOp.outreach;
//
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//public class TankDrive extends SubsystemBase {
//    public DcMotorEx leftMotor1;
//    public DcMotorEx leftMotor2;
//
//    public DcMotorEx rightMotor1;
//    public DcMotorEx rightMotor2;
//
//    public void init(HardwareMap hardwareMap) {
//        leftMotor1 = hardwareMap.get(DcMotorEx.class, "lm");
//        rightMotor1 = hardwareMap.get(DcMotorEx.class, "rm");
//
//        leftMotor2 = hardwareMap.get(DcMotorEx.class, "lm2");
//        rightMotor2 = hardwareMap.get(DcMotorEx.class, "lm2");
//
//        DcMotorEx[] motors = new DcMotorEx[]{
//                leftMotor1,rightMotor2,leftMotor2,rightMotor2
//
//};

package org.firstinspires.ftc.teamcode.support;

import com.acmerobotics.dashboard.config.Config;


@Config
public class Constants {

    //Pedro tuning old from last year TODO: actually measure if bad
    public static double mass = 15.5;

    public static double xMovement = 68.31575;
    public static double yMovement = 50.671225;

    public static double forwardZeroPowerAcceleration = -44.435875;
    public static double lateralZeroPowerAcceleration = -82.086;

    public static double rot_ticks_in_degree = 0.0417;
    public static double ext_ticks_in_degree = 0.0521;

    public static double pinpointXOffset = 1.41732; // MM = -36.0
    public static double pinpointYOffset = -3.18208661; // MM = -80.835 // 3.18208661
}

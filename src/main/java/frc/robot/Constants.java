// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.generated.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
    public static final int CANDLE_LEDS_PER_ANIMATION = 30;
    public static final double CANDLE_BRIGHTNESS = 1.0;


    public static class AutoConstants {
        public static final double kPThetaController = 16;
        public static final double kPXController = 0.5;
        public static final double kPYController = 0.5;
    }

    public static class OperatorConstants
    {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }

    public static class Drivetrain {
        public static final double MAX_TELEOP_SPEED = TunerConstants.kSpeedAt12VoltsMps * 0.75;
        public static final double MAX_AUTO_SPEED = TunerConstants.kSpeedAt12VoltsMps * 0.8;
        public static final double MAX_TELEOP_ANGULAR_RATE = 1.5 * Math.PI;
    }

    public static class Ports {
        public static final String CTRE_CANBUS = "moe";
        public static final int CANDLE = 0;

    }
}

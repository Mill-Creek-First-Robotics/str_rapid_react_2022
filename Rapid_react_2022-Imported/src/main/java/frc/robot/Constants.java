// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    // the ports of each motor
    public static final int FRONT_LEFT_MOTOR = 3;
    public static final int BACK_LEFT_MOTOR = 7;
    public static final int FRONT_RIGHT_MOTOR = 4;
    public static final int BACK_RIGHT_MOTOR = 8;
    public static final int ROLLER_MOTOR = 5;

    // the ports of other stuff
    public static final int ENCODER = 6;
    public static final int GYROSCOPE = 7;

    // ports of controllers
    public static final int CONTROLLER = 0;
}

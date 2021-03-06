/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class XboxController {
    
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
        public static final double DEAD_BAND = 0.1; //0.2
        
        public static final int B_BUTTON = 2;
        public static final int LEFT_TRIGGER = 2;
        public static final int RIGHT_TRIGGER = 3;
        
      };
      
      public static class Drive{
        public static final int LEFT_ENCODER_A_PORT = 0;
        public static final int LEFT_ENCODER_B_PORT = 1;
        public static final int RIGHT_ENCODER_A_PORT = 2;
        public static final int RIGHT_ENCODER_B_PORT = 3;
        public static final double DISTANCE_PER_ENCODER_PULSE = (double) (4.0 * Math.PI / 256.0);
        public static final double DRIVE_MAX_VELOCITY = 11.0;
        public static final double PID_DRIVE_DISTANCE_TOLERANCE = .2;
        
        public static final int LEFT_MOTOR_A_PORT = 0;
        public static final int LEFT_MOTOR_B_PORT = 1;
        public static final int LEFT_MOTOR_C_PORT = 2;
        public static final int RIGHT_MOTOR_A_PORT = 3;
        public static final int RIGHT_MOTOR_B_PORT = 4;
        public static final int RIGHT_MOTOR_C_PORT = 6;
      };
    
      public static class Solenoids{
        public static final int LEFT_PTO_A = 0;
        public static final int LEFT_PTO_B = 1;
        public static final int CLIMBER_RELEASE = 2;
        public static final int INTAKE = 3;
        public static final int RIGHT_POPPER_A = 4;
        public static final int RIGHT_POPPER_B = 5;
        public static final int STATUS_LIGHT = 7;
        
      }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    public static final int NUMBER_OF_WHEELS = 4;
    public final class ModuleConst {


        public static final int FRONT_LEFT_MOVE_MOTOR_ID = 7;
        public static final int FRONT_LEFT_TURN_MOTOR_ID = 8;
        public static final int FRONT_LEFT_CANCODER_ID = 11;
        public static final boolean FRONT_LEFT_SET_INVERT_TYPE = false;


        public static final int FRONT_RIGHT_MOVE_MOTOR_ID = 5;
        public static final int FRONT_RIGHT_TURN_MOTOR_ID = 6;
        public static final int FRONT_RIGHT_CANCODER_ID = 13;
        public static final boolean FRONT_RIGHT_SET_INVERT_TYPE = false;

        public static final int BACK_LEFT_MOVE_MOTOR_ID = 1;
        public static final int BACK_LEFT_TURN_MOTOR_ID = 2;
        public static final int BACK_LEFT_CANCODER_ID = 10;
        public static final boolean BACK_LEFT_SET_INVERT_TYPE = false;

        public static final int BACK_RIGHT_MOVE_MOTOR_ID = 3;
        public static final int BACK_RIGHT_TURN_MOTOR_ID = 4;
        public static final int BACK_RIGHT_CANCODER_ID = 12;
        public static final boolean BACK_RIGHT_SET_INVERT_TYPE = false;

        public static final double mVel_Kp = 0.0058;
        public static final double mVel_Ki = 0.0;
        public static final double mVel_Kd = 0.0006;

        public static final double mAngle_Kp = 0.12;
        public static final double mAngle_Ki = 0.00 ;
        public static final double mAngle_Kd = 0.001;
        public static final double mAngle_Ks = 0.035;

        public static final double TOLERANCE_STEER = 0.5;


        public static final double PPR_FALCON = 2048;

        public static final double WHEEL_PEREMITER = 0.1016*Math.PI;
        public static final double GEAR_RATIO_VEL = 8.14;
        public static final double PULSE_PER_METER = PPR_FALCON*GEAR_RATIO_VEL/WHEEL_PEREMITER;

        public static final double GEAR_RATIO_ANGLE = 12.8;
        public static final double PULSE_PER_ANGLE = GEAR_RATIO_ANGLE*PPR_FALCON/360;

        public static final double MOVE_MOTOR_Ks = 0.0479;
        public static final double MOVE_MOTOR_Kv = 0.22185;

    }

    public final class Offsets{
        public static final double FRONT_LEFT_OFFSET = 31.46484375;
        public static final double FRONT_RIGHT_OFFSET = 119.267578125;
        public static final double BACK_LEFT_OFFSEST = 45.703125;
        public static final double BACK_RIGHT_OFFSEST = 287.314453125;

    }
    public final class Buttons {
        public static final int JOYSTICK_XY_PORT_NUM = 0;
        public static final int JOYSTICK_DIRECTION_PORT_NUM = 1;

        public static final int XBOX_PORT_NUM = 2;
        public static final double CONTROLLER_RANGE = 0.15;

    }

    public final static class ChassisConst {
        public static final int GYRO_PORT_NUM = 14;
        public final static Translation2d[] wheelsMeters = new Translation2d[] {};

        public static final double ANGLE_2RADPERSEC_Kp = 3.5;
        public static final double ANGLE_2RADPERSEC_Ki = 0;
        public static final double ANGLE_2RADPERSEC_Kd = 0;

        public static final double ERRORX_2VX_Kp = 0.9;
        public static final double ERRORX_2VX_Ki = 0;
        public static final double ERRORX_2VX_Kd = 0;

        public static final double ERRORY_2VY_Kp = 0.9;
        public static final double ERRORY_2VY_Ki = 0;
        public static final double ERRORY_2VY_Kd = 0;
        //pid param for x and y should be the saOSITIONme, but because the robot is a rectangle I thought it's
        //best to seperate.

        public static final double MAX_VELOCITY_XY = 2;
        public static final double MAX_RADPERSEC = 1*Math.PI;
        public static final double DEADBAND_AUTONOMOUS_XY = 0.02;
        public static final double DEADBAND_AUTONOMOUS_RAD = 0.02*Math.PI;
        public static final double AUTONOMOUS_VELOCITY = 1;
    }

    public final static class visionConsts {
        public static final double IMAGE_CAPTURE_LATENCY_MS = 11;
        public static final double COMUNICATION_LATENCY_MS = 10;

        public static final double LIMELIGHT_X_DISTANCE = 0.3;
        public static final double LIMELIGHT_Y_DISTANCE = 0;

    }

    
    public final static class Kinematics {
        // Locations for the swerve drive modules relative to the robot center.
        public static final double FRONT_RIGHT_LOCATION_X = 0.26515;
        public static final double FRONT_RIGHT_LOCATION_Y = -0.2215;
        
        public static final double FRONT_LEFT_LOCATION_X = 0.26515;
        public static final double FRONT_LEFT_LOCATION_Y = 0.2215;
        
        public static final double BACK_LEFT_LOCATION_X = -0.26515;
        public static final double BACK_LEFT_LOCATION_Y = 0.2215;
        
        public static final double BACK_RIGHT_LOCATION_X = -0.26515;
        public static final double BACK_RIGHT_LOCATION_Y = -0.2215;
        
        // define modules location
        public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(FRONT_LEFT_LOCATION_X,
        FRONT_LEFT_LOCATION_Y);
        public static final Translation2d BACK_LEFT_LOCATION = new Translation2d(BACK_LEFT_LOCATION_X,
        BACK_LEFT_LOCATION_Y);
        public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(FRONT_RIGHT_LOCATION_X,
        FRONT_RIGHT_LOCATION_Y);
        public static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(BACK_RIGHT_LOCATION_X,
        BACK_RIGHT_LOCATION_Y);
        
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            BACK_RIGHT_LOCATION,
            FRONT_LEFT_LOCATION,
            FRONT_RIGHT_LOCATION,
            BACK_LEFT_LOCATION
            );
        }



        
}

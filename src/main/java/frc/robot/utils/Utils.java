package frc.robot.utils;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/**
 * contains functions aiding with calculations, retrieving values from controllers and other utilities
 */

public class Utils {

    private static PIDController pidRad = new 
    PIDController(Constants.ChassisConst.ANGLE_2RADPERSEC_Kp, Constants.ChassisConst.ANGLE_2RADPERSEC_Ki,
    Constants.ChassisConst.ANGLE_2RADPERSEC_Kd);

    private static PIDController pidX = new PIDController(Constants.ChassisConst.ERRORX_2VX_Kp,
    Constants.ChassisConst.ERRORX_2VX_Ki, Constants.ChassisConst.ERRORX_2VX_Kd);
    
    private static PIDController pidY = new PIDController(Constants.ChassisConst.ERRORY_2VY_Kp,
    Constants.ChassisConst.ERRORY_2VY_Ki, Constants.ChassisConst.ERRORY_2VY_Kd);

    //functions for controllers:

    /**
     * Get the x value of the joystick. contains deadband and normalization of value.
     * @param joystick the joystick 
     * @return x value of joystick
     */
    public static double getJoystickX(Joystick joystick) {
        double x = -joystick.getX();
        double val;
        if (!isJoystickInRange(x)) {
            val = 0.0;
        } else {
            val = normalizeControlVal(x);
        }
        return val;
    }

    /**
     * Get the y value of the joystick. contains deadband and normalization of value.
     * @param joystick the joystick 
     * @return y value of joystick
     */
    public static double getJoystickY(Joystick j) {
        double y = -j.getY();
        double val;
        if (!isJoystickInRange(y)) {
            val = 0.0;
        } else {
            val = normalizeControlVal(y);
        }
        return val;
    }

    /**
     * Get the angle of the joystick in radians. contains deadband.
     * @param joystick the joystick 
     * @return angle of joystick in radians
     */
    public static double getJoystickAngle(Joystick joystick) {
        double y = joystick.getY();
        if (!isJoystickInRange(y)) {
            y = 0.0;}
        double x = joystick.getX();
        if (!isJoystickInRange(x)) {
            x = 0.0; }
        double angle = Math.atan2(y, x);
        return angle;

    }

    /**
     * Checks if button in joystick is pressed.
     * @param button the joystick button
     * @return is the joystick button pressed
     */
    public static boolean isButtoonPressed(JoystickButton button) {
        return button.getAsBoolean();
    }

    /**
     * Get the angle of a specified joystick in xbox controller. contains deadband.
     * @param xboxController the xbox controller
     * @param isLeft is the specified joystick in the controller is left
     * @return the angle of the specified joystick in radians
     */
    public static double getXBoxControllerAngle(XboxController xboxController, boolean isLeft) {
        double y, x;
        if (isLeft) {
            y = xboxController.getLeftY(); 
            x = xboxController.getLeftX();
        }
        else {
            y = xboxController.getRightY(); 
            x = xboxController.getRightX();
        }
            
        if (!isJoystickInRange(y)) {
            y = 0.0;}
        if (!isJoystickInRange(x)) {
            x = 0.0; }
        x = Math.abs(x);
        y = Math.abs(y);
        double angle = Math.atan2(y, x);
        return angle;

    }

    /**
     * Get the x value of the left joystick in xbox controller. contains deadband and normalization.
     * @param xboxController the xbox controller
     * @return the x value of the left joystick
     */
    public static double getXboxControllerX(XboxController xboxController) {
        double x = xboxController.getLeftX();
        System.out.println("x: " + x);
        System.out.println(x);
        double val;
        if (!isJoystickInRange(x)) {
            val = 0.0;
        } else {
            val = normalizeControlVal(x);
        }
        return val;
    }

    /**
     * Get the y value of the left joystick in xbox controller. contains deadband and normalization.
     * @param xboxController the xbox controller
     * @return the y value of the left joystick
     */
    public static double getXboxControllerY(XboxController xboxController) {
        double y = xboxController.getLeftY();
        System.out.println(y);
        double val;
        if (!isJoystickInRange(y)) {
            val = 0.0;
        } else {
            val = normalizeControlVal(y);
        }
        return val;
    }

    /**
     * Get the value of left trigger in xbox controller. contains deadband and normalization.
     * @param xboxController the xbox controller
     * @return value of left trigger
     */
    public static double getXBoxControllerTriggerLeft(XboxController xboxController) {
        double val = xboxController.getLeftTriggerAxis();
        SmartDashboard.putNumber("trigger val bf", val);
        if (!isJoystickInRange(val)) {
            val = 0.0;
        } else {
            val = normalizeControlVal(val);
        }
        SmartDashboard.putNumber("trigger value", val);
        return val;
    }

    /**
     * Get the value of right trigger in xbox controller. contains deadband and normalization.
     * @param xboxController the xbox controller
     * @return value of right trigger
     */
    public static double getXBoxControllerTriggerRight(XboxController xboxController) {
        double val = xboxController.getRightTriggerAxis();
        if (!isJoystickInRange(val)) {
            val = 0.0;
        } else {
            val = normalizeControlVal(val);
        }
        return val;
    }
    
    /**
     * Checks if left bumper in xbox controller pressed.
     * @param xboxController xbox controller
     * @return is left bumper in xbox controller pressed
     */
    public static boolean isLeftBumperXboxPressed(XboxController xboxController) {
        return xboxController.getLeftBumper();
    }

    /**
     * Checks if right bumper in xbox controller pressed.
     * @param xboxController xbox controller
     * @return is right bumper in xbox controller pressed
     */
    public static boolean isRightBumperXboxPressed(XboxController xboxController) {
        return xboxController.getRightBumper();
    }

    /**
     * Normalizes value to be parabolic between -1 to 1.
     * @param value the value before normalization
     * @return the normalized value
     */
    public static double normalizeControlVal(double value) {
        return Math.signum(value) * ((1 / 1.1) * Math.pow(value, 2) + (1 - 1 / 1.1));
    }

    /**
     * Checks if joystick value is above a specific value.
     * @param value the value
     * @return true if in range, false if not
     */
    public static boolean isJoystickInRange(double value) {
        return Math.abs(value) > Constants.Buttons.JOYSTICK_RANGE;

    }

    //helpful calculations

    /**
     * Multiply value by max velocity.
     * @param value the value
     * @return the value multiplied by max velocity
     */
    public static double timesMaxVelocity(double value) {
        return value*Constants.ChassisConst.MAX_VELOCITY_XY;
    }

    /**
     * Normalizes angle to be between 0 to 360 degrees
     * @param angle the angle
     * @return the normalized angle
     */
    public static double normalizeAngle(double angle) {
        return ((angle%360)+360)%360;
    }

    /**
     * Optimize difference in radians (displaying between -PI to PI).
     * @param difference the difference between desired angle and current angle
     * @return the difference optimized in radians
     */
    public static double optimizeRadDemacia(double difference) {
        if (difference > Math.PI)
            return difference - 2*Math.PI;
        if (difference < -Math.PI) 
            return difference + 2*Math.PI;
        return difference;
    }

     /**
     * Optimize difference (displaying between -180 degrees to 180 degrees).
     * @param difference the difference between desired angle and current angle
     * @return the optimized difference
     */
    public static double optimizeAngleDemacia(double difference) {
        if (difference > 180)
            return difference - 360;
        if (difference < -180) 
            return difference + 360;
        return difference;
    }

    /**
     * Get Rotation2d of an angle.
     * @param angle angle in degrees
     * @return angle represented as Rotation2d
     */
    public static Rotation2d getRotation2d(double angle) {
        return Rotation2d.fromDegrees(angle);
    }

    /**
     * Convert angle from degrees to radians.
     * @param degree angle in degrees
     * @return angle in radians
     */
    public static double radianFromDegrees(double degree) {
        degree = normalizeAngle(degree);
        return degree*Math.PI/180;
    }

    /**
     * Get the angle of a vector.
     * @param x x component of vector
     * @param y y component of vector
     * @return angle of vector in degrees
     */
    public static double calcAngleinTriangle(double x, double y) {
        x = Math.abs(x);
        y = Math.abs(y);
        return Math.atan2(y, x);
    }

    //gyro position
    /**
     * Get gyro position.
     * @param gyro the gyro
     * @return the gyro position
     */
    public static double getGyroPosition(PigeonIMU gyro) {
        return normalizeAngle(gyro.getFusedHeading());
    }


    //calculations of swerve module states

    /**
     * Get swerve module states given the following parameters:
     * @param vx x component of velocity
     * @param vy y component of velocity
     * @param desiredAngle desired heading of robot (degrees)
     * @return swerve module states of modules in robot
     */
    public static SwerveModuleState[] getSwerveStates(double vx, double vy, double desiredAngle) {
        double dif = radianFromDegrees(desiredAngle) - radianFromDegrees(getGyroPosition(RobotContainer.gyro));
        dif = optimizeRadDemacia(dif);
        SmartDashboard.putNumber("differenceAngle (IN PI)", dif/Math.PI);
        double radPerSec = pidRad.calculate(dif);
        //double radPerSec = Math.PI*Math.signum(dif);
        SmartDashboard.putNumber("radPerSec (IN PI)", radPerSec/Math.PI);
        Rotation2d currentAngle = Rotation2d.fromDegrees(getGyroPosition(RobotContainer.gyro)); 
        return getModuleStates(vx, vy, radPerSec, currentAngle);
    }

    /**
     * Get swerve module states given the following parameters:
     * @param vx x component of velocity
     * @param vy y component of velocity
     * @param radPerSec angular velocity
     * @param currentAngle current heading of robot (in Rotation2d)
     * @return swerve module states of modules in robot
     */
    public static SwerveModuleState[] getModuleStates(double vx, double vy, double radPerSec, Rotation2d currentAngle) {
        ChassisSpeeds cspeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, radPerSec, currentAngle);
        SwerveModuleState[] sModuleStates = Constants.Kinematics.SWERVE_KINEMATICS.toSwerveModuleStates(cspeeds);

        return sModuleStates;
    }

    /**
     * Get swerve module states given the following parameters:
     * @param vx x component of velocity
     * @param vy y component of velocity
     * @param isPressed is left button pressed in controller
     * @param currentAngle current heading of robot (in Rotation2d)
     * @return swerve module states of modules in robot
     */
    public static SwerveModuleState[] getModuleStatesLeft(double vx, double vy, boolean isPressed, Rotation2d currentAngle) {
        double rps = 0;
        if(isPressed) rps = Math.PI;
        ChassisSpeeds cspeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rps, currentAngle);
        SwerveModuleState[] sModuleStates = Constants.Kinematics.SWERVE_KINEMATICS.toSwerveModuleStates(cspeeds);

        return sModuleStates;
    }

    /**
     * Get swerve module states given the following parameters:
     * @param vx x component of velocity
     * @param vy y component of velocity
     * @param isPressed is right button pressed in controller
     * @param currentAngle current heading of robot (in Rotation2d)
     * @return swerve module states of modules in robot
     */
    public static SwerveModuleState[] getModuleStatesRight(double vx, double vy, boolean isPressed, Rotation2d currentAngle) {
        double rps = 0;
        if(isPressed) rps = -Math.PI;
        ChassisSpeeds cspeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rps, currentAngle);
        SwerveModuleState[] sModuleStates = Constants.Kinematics.SWERVE_KINEMATICS.toSwerveModuleStates(cspeeds);

        return sModuleStates;
    }

    /**
     * Get swerve module states given the following parameters:
     * @param vx x component of velocity
     * @param vy y component of velocity
     * @param triggerVal value of controller trigger
     * @param currentAngle current heading of robot (in Rotation2d)
     * @return swerve module states of modules in robot
     */
    public static SwerveModuleState[] getModuleStatesTriggerVal(double vx, double vy, double triggerVal, Rotation2d currentAngle) {
        double rps = triggerVal*Constants.ChassisConst.MAX_RADPERSEC;
        ChassisSpeeds cspeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rps, currentAngle);
        SwerveModuleState[] sModuleStates = Constants.Kinematics.SWERVE_KINEMATICS.toSwerveModuleStates(cspeeds);

        return sModuleStates;
    }

    //functions for autonomic drive

    /**
     * Holonomic drive.
     * @param currentPose current robot pose
     * @param targetPose target robot pose
     * @return swerve module states required to get to target pose
     */
    public static SwerveModuleState[] driveToHolonimic(Pose2d currentPose, Pose2d targetPose) {
        double errorX = targetPose.getX() - currentPose.getX();
        double errorY = targetPose.getY() - currentPose.getY();
        double errorRad = targetPose.getRotation().getRadians() - 
        radianFromDegrees(getGyroPosition(RobotContainer.gyro));
        errorRad = optimizeRadDemacia(errorRad);

        double vx = pidX.calculate(errorX);
        double vy = pidY.calculate(errorY);
        double radPerSec = pidRad.calculate(errorRad);
        Rotation2d currentAngle = Rotation2d.fromDegrees(getGyroPosition(RobotContainer.gyro));

        return getModuleStates(vx, vy, radPerSec, currentAngle);

    }

    /**
     * Checks if robot in target pose.
     * @param currentPose current robot pose
     * @param targetPose target rbot pose
     * @return is robot in target pose
     */
    public static boolean isInPose(Pose2d currentPose, Pose2d targetPose) {
        double errorX = targetPose.getX() - currentPose.getX();
        System.out.println("targetPosex= " + targetPose.getX());
        double errorY = targetPose.getY() - currentPose.getY();
        double errorRad = targetPose.getRotation().getRadians() - 
        radianFromDegrees(getGyroPosition(RobotContainer.gyro));
        errorRad = optimizeRadDemacia(errorRad);

        if (Math.abs(errorX) < Constants.ChassisConst.DEADBAND_AUTONOMOUS 
        && Math.abs(errorY) < Constants.ChassisConst.DEADBAND_AUTONOMOUS && Math.abs(errorRad) 
        < Constants.ChassisConst.DEADBAND_AUTONOMOUS_RAD) {
            return true;
        }

        return false;
    }

    /**
     * Simple drive.
     * @param wantedVelocity desired robot velocity
     * @param currentPose current robot pose
     * @param targetPose target robot pose
     * @return swerve module states required to get to target pose
     */
    public static SwerveModuleState[] driveToSimple(double wantedVelocity, Pose2d currentPose, Pose2d targetPose) {
        double errorX = targetPose.getX() - currentPose.getX();
        double errorY = targetPose.getY() - currentPose.getY();
        double errorRad = targetPose.getRotation().getRadians() - 
        radianFromDegrees(getGyroPosition(RobotContainer.gyro));
        errorRad = optimizeRadDemacia(errorRad);

        double angle = calcAngleinTriangle(errorX, errorY);
        double vx = wantedVelocity*Math.cos(angle);
        double vy = wantedVelocity*Math.sin(angle);
        double radPerSec = pidRad.calculate(errorRad);
        System.out.println("angle=" + angle+ " vx=" + vx + " vy=" + vy + " radpersec=" + radPerSec);
        System.out.println(" errorX" + errorX + " errorY=" + errorY + " errorRad" + errorRad);
        Rotation2d currentAngle = Rotation2d.fromDegrees(getGyroPosition(RobotContainer.gyro));

        return getModuleStates(vx, vy, radPerSec, currentAngle);


    }

}

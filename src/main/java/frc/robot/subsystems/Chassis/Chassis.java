package frc.robot.subsystems.Chassis;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive2simple;
import frc.robot.utils.SwerveModule;
import frc.robot.utils.Utils;

/**
 * Subsystem representing the swerve chassis.
 */
public class Chassis extends SubsystemBase {
    private final Field2d field;
    private final SwerveModule[] swerveModules;
    private final SwerveModule front_right, back_right, back_left, front_left;
    private SwerveDrivePoseEstimator poseEstimator;
    private NetworkTable limeLightTable;
    private NetworkTableEntry limeLightPoseEntry;
    private NetworkTableEntry limeLightTVEntry; 
    private NetworkTableEntry limelightTLEntry;  

    /**
     * Constructs a chassis.
     */
    public Chassis() {

        SmartDashboard.putNumber("target pose x", 0);
        SmartDashboard.putNumber("target pose y", 0);

        this.field = new Field2d();

        swerveModules = new SwerveModule[Constants.NUMBER_OF_WHEELS];

        front_right = new SwerveModule(Constants.Offsets.FRONT_RIGHT_OFFSET,
                Constants.ModuleConst.FRONT_RIGHT_MOVE_MOTOR_ID,
                Constants.ModuleConst.FRONT_RIGHT_TURN_MOTOR_ID,
                Constants.ModuleConst.FRONT_RIGHT_CANCODER_ID,
                Constants.ModuleConst.FRONT_RIGHT_SET_INVERT_TYPE);
        back_right = new SwerveModule(Constants.Offsets.BACK_RIGHT_OFFSEST,
                Constants.ModuleConst.BACK_RIGHT_MOVE_MOTOR_ID,
                Constants.ModuleConst.BACK_RIGHT_TURN_MOTOR_ID,
                Constants.ModuleConst.BACK_RIGHT_CANCODER_ID,
                Constants.ModuleConst.BACK_RIGHT_SET_INVERT_TYPE);
        front_left = new SwerveModule(Constants.Offsets.FRONT_LEFT_OFFSET,
                Constants.ModuleConst.FRONT_LEFT_MOVE_MOTOR_ID,
                Constants.ModuleConst.FRONT_LEFT_TURN_MOTOR_ID,
                Constants.ModuleConst.FRONT_LEFT_CANCODER_ID,
                Constants.ModuleConst.FRONT_LEFT_SET_INVERT_TYPE);
        back_left = new SwerveModule(Constants.Offsets.BACK_LEFT_OFFSEST,
                Constants.ModuleConst.BACK_LEFT_MOVE_MOTOR_ID,
                Constants.ModuleConst.BACK_LEFT_TURN_MOTOR_ID,
                Constants.ModuleConst.BACK_LEFT_CANCODER_ID,
                Constants.ModuleConst.BACK_LEFT_SET_INVERT_TYPE);

        swerveModules[0] = back_right;
        swerveModules[1] = front_left;
        swerveModules[2] = front_right;
        swerveModules[3] = back_left;

        Pose2d zeroPose = new Pose2d(0, 0, getRotation2dGyroPosition());
        poseEstimator = new SwerveDrivePoseEstimator(Constants.Kinematics.SWERVE_KINEMATICS,
        getRotation2dGyroPosition(), getcurrentModulesPosition(), zeroPose);
        

        //setDefaultCommand(new XBoxControlwTrigger(this));
        calibrateGyro(0);

        SmartDashboard.putData("Field", getField());

        // SmartDashboard.putData("reset gyro", new InstantCommand(() ->
        // calibrateGyro(SmartDashboard.getNumber("gyro calibrate 2 angle", 0))));
        // SmartDashboard.putData("Calibrate All", new InstantCommand(() ->
        // calibrateAll(), this).ignoringDisable(true));

        //SmartDashboard.putNumber("wanted", 0);

        //SmartDashboard.putData("setRobotAngle", new Get2Angle(this, 90));
        Pose2d targetPose2d = new Pose2d(1, 1, Rotation2d.fromDegrees(90));
        SmartDashboard.putData("driveTo command", new Drive2simple(this, targetPose2d));


        limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
        limeLightPoseEntry = limeLightTable.getEntry("botpose");
        limeLightTVEntry = limeLightTable.getEntry("tv");
        limelightTLEntry = limeLightTable.getEntry("tl");


    }

    //get swerve module states

    /**
     * Gets current swerve module states.
     * @return current swerve module states
     */
    public SwerveModuleState[] getCurrentModuleStates() {
        SwerveModuleState[] sModuleStates = new SwerveModuleState[Constants.NUMBER_OF_WHEELS];
        for (int i = 0; i < sModuleStates.length; i++) {
            sModuleStates[i] = swerveModules[i].getState();
        }
        
        return sModuleStates;

    }

    /**
     * Gets current swerve module positions.
     * @return current swerve module positions
     */
    public SwerveModulePosition[] getcurrentModulesPosition() {
        SwerveModulePosition[] sModulesPosition = new SwerveModulePosition[Constants.NUMBER_OF_WHEELS];
        for (int i = 0; i < sModulesPosition.length; i++) {
            sModulesPosition[i] = swerveModules[i].getPosition();
        }
        
        return sModulesPosition;
        }

    /**
     * Optimizes module states.
     * @param currentPose current robot pose
     * @param targetPose target robot pose
     * @return optimized swerve module states.
     */
    public SwerveModuleState[] getModulesOptimizedAutonomous(Pose2d currentPose, Pose2d targetPose) {
        SwerveModuleState[] sModuleStates = Utils.driveToHolonimic(currentPose, targetPose);
        SwerveModuleState[] sModuleStatesOptimized = new SwerveModuleState[sModuleStates.length];

        for (int i = 0; i < sModuleStates.length; i++) {
            double angle = Utils.normalizeAngle(swerveModules[i].getAngle());
            sModuleStatesOptimized[i] = SwerveModuleState.optimize(sModuleStates[i],
                    Rotation2d.fromDegrees(angle));
            Rotation2d angleR2D = sModuleStatesOptimized[i].angle;
            Double newAngle = Utils.normalizeAngle(angleR2D.getDegrees());
            sModuleStatesOptimized[i] = new SwerveModuleState(sModuleStatesOptimized[i].
            speedMetersPerSecond, Rotation2d.fromDegrees(newAngle));
        }
        return sModuleStatesOptimized;
    }

    /**
     * Optimizes swerve module states.
     * @param sModuleStates swerve module states pre-optimize
     * @return optimized swerve module states
     */
    public SwerveModuleState[] getModulesOptimize(SwerveModuleState[] sModuleStates) {
        SwerveModuleState[] sModuleStatesOptimized = new SwerveModuleState[sModuleStates.length];
        for (int i = 0; i < sModuleStates.length; i++) {
            double angle = Utils.normalizeAngle(swerveModules[i].getAngle());
            sModuleStatesOptimized[i] = SwerveModuleState.optimize(sModuleStates[i],
                Rotation2d.fromDegrees(angle));
            Rotation2d angleR2D = sModuleStatesOptimized[i].angle;
            Double newAngle = Utils.normalizeAngle(angleR2D.getDegrees());
            sModuleStatesOptimized[i] = new SwerveModuleState(sModuleStatesOptimized[i].
            speedMetersPerSecond, Rotation2d.fromDegrees(newAngle));
        }
        return sModuleStatesOptimized;
    }

    //gyro

    /**
     * Gets gyro position.
     * @return the gyro position
     */
    public double getGyroPosition() {
        return RobotContainer.gyro.getFusedHeading();
    }

    /**
     * Gets gyro position in Rotation2d.
     * @return the gyro position in Rotation2d
     */
    public Rotation2d getRotation2dGyroPosition() {
        return Rotation2d.fromDegrees(getGyroPosition());
    }

    //set functions

    /**
     * Sets modules to desired states.
     * @param swerveModulesState desired module states
     */
    public void setModules(SwerveModuleState[] swerveModulesState) {
        for (int i = 0; i < swerveModulesState.length; i++) {
            swerveModules[i].setAngle(swerveModulesState[i].angle.getDegrees());
            swerveModules[i].setVel(swerveModulesState[i].speedMetersPerSecond);
            
        }
    }

    /**
     * Sets neutral mode of steer motor
     * @param isBrake is the desired neutral mode brake
     */
    public void setNeutralModeSteer(boolean isBrake) {
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setNeutraleModeSteerMotor(isBrake);
        }
    }

    /**
     * Sets neutral mode of move motor
     * @param isBrake is the desired neutral mode brake
     */
    public void setNeutralModeMove(boolean isBrake) {
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setNeutraleModeMoveMotor(isBrake);
        }
    }

    /**
     * Sets power of all steer motors.
     * @param power desired power
     */
    public void setPowerSteer(double power) {
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setPowerSteerMotor(power);
        }
    }

        /**
     * Sets power of all steer motors.
     * @param power desired power
     */
    public void setPowerVelocity(double power) {
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setPowerVelocity(power);
        }
    }

    /**
     * Sets all wheels to a certain angle.
     * @param angle the desired wheel angle
     */
    public void setAngle(double angle) {
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setAngle(angle);
        }
    }

    //odometry & field

    /**
     * Gets current robot pose.
     * @return current robot pose
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets initial robot pose.
     * @param pose initial robot pose
     */
    public void resetPoseEstimator(Pose2d pose) { 
        poseEstimator.resetPosition(getRotation2dGyroPosition(), getcurrentModulesPosition(), pose);
    }

    /**
     * Updates robot pose by module states.
     */
    public void poseEstimatorUpdate() {
        poseEstimator.update(getRotation2dGyroPosition(), getcurrentModulesPosition());

    }

    /**
     * Updates robot pose by vision data.
     * @param visionPose pose received by vision
     */
    public void poseEstimatorUpdateByVision(Pose2d visionPose) {
        poseEstimator.addVisionMeasurement(visionPose, limelightTLEntry.getDouble(0) + 
        Constants.visionConsts.IMAGE_CAPTURE_LATENCY_MS + Constants.visionConsts.COMUNICATION_LATENCY_MS);
    }

    /**
     * Checks if vision is receiving data.
     * @return is data received through vision
     */
    public boolean isVision() {
        return limeLightTVEntry.getInteger(0)==1;
     }

     /**
      * Gets robot pose from vision.
      * @return robot pose from vision
      */
     public Pose2d getPoseFromVision() {
        //byte[] poseArr = limeLightPoseEntry.getValue().getRaw();
        try {
            double[] arr = new double[1];
            double[] poseArr = limeLightPoseEntry.getDoubleArray(arr);
            return new Pose2d(poseArr[0] - Constants.visionConsts.LIMELIGHT_X_DISTANCE,
            poseArr[1] - Constants.visionConsts.LIMELIGHT_Y_DISTANCE, new Rotation2d(poseArr[5]));
        } catch (Exception e) {
            return new Pose2d();
        }
        
     }

    /**
     * Sets field by robot pose.
     * @param pose current robot pose
     */
    public void setField(Pose2d pose) {
        field.setRobotPose(pose);
    }

    /**
     * Gets field.
     * @return field
     */
    public Field2d getField() {
        return field;
    }

    //calibration

    /**
     * Calibrates all modules - sets zero point.
     */
    public void calibrateAll() {
        for (int i=0; i<swerveModules.length; i++) {
            swerveModules[i].calibrate();
        }
    }

    /**
     * Calibrates gyro.
     * @param value value of gyro
     */
    public void calibrateGyro(double value) {
        SmartDashboard.putNumber("is gyro calibrated", value);
        RobotContainer.gyro.setFusedHeading(value);
    }

    //trajectory!!!

    /**
     * Creates a command that follows a trajectory.
     * @param traj the trajectory
     * @param isFirstPath is this command the first command
     * @return command that follows a trajectory
     */
    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {

                if(isFirstPath){
                    this.resetPoseEstimator(traj.getInitialHolonomicPose());
                }
              }),
              new PPSwerveControllerCommand(
            traj, 
            this::getPose,
            Constants.Kinematics.SWERVE_KINEMATICS,
            new PIDController(0, 0, 0), 
            new PIDController(0, 0, 0), 
            new PIDController(0, 0, 0),
            this::setModules,
            true, 
            this
        )
         );
     }


    @Override
    public void initSendable(SendableBuilder builder) {

        
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("module 0 angle", swerveModules[0].getAngle());
        SmartDashboard.putNumber("module 0 speed", swerveModules[0].getVelocity());

        SmartDashboard.putNumber("module 1 angle", swerveModules[1].getAngle());
        SmartDashboard.putNumber("module 1 speed", swerveModules[1].getVelocity());

        SmartDashboard.putNumber("module 2 angle", swerveModules[2].getAngle());
        SmartDashboard.putNumber("module 2 speed", swerveModules[2].getVelocity());

        SmartDashboard.putNumber("module 3 angle", swerveModules[3].getAngle());
        SmartDashboard.putNumber("module 3 speed", swerveModules[3].getVelocity());

        SmartDashboard.putNumber("Gyro Angle", getGyroPosition());

        SmartDashboard.putNumber("offset back right", swerveModules[0].getOffset());
        SmartDashboard.putNumber("offset front left", swerveModules[1].getOffset());
        SmartDashboard.putNumber("offset front right", swerveModules[2].getOffset());
        SmartDashboard.putNumber("offset back left", swerveModules[3].getOffset());

        SmartDashboard.putNumber("current Pose2d x", getPose().getX());
        SmartDashboard.putNumber("current Pose2d y", getPose().getY());
        SmartDashboard.putNumber("current Pose2d rad i pi", getPose().getRotation().getRadians()/Math.PI);
        

        if (isVision()) {
            SmartDashboard.putNumber("is vision", 0);
            poseEstimatorUpdateByVision(getPoseFromVision());
        }

        poseEstimatorUpdate();
        setField(getPose());
        //fieldEntry.setValue(getField());        

    }

}

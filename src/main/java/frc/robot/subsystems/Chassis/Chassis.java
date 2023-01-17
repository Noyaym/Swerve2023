package frc.robot.subsystems.Chassis;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Get2Angle;
import frc.robot.utils.SwerveModule;
import frc.robot.utils.Utils;

public class Chassis extends SubsystemBase {
    private final Field2d field;
    private final SwerveModule[] swerveModules;
    private final SwerveModule front_right, back_right, back_left, front_left;
    private SwerveDrivePoseEstimator poseEstimator;
    
    //private ShuffleboardTab tab = Shuffleboard.getTab("chassis data");
    //private NetworkTableEntry fieldEntry = tab.add("Field", 0).getEntry();

    public Chassis(PigeonIMU gyro) {
        this.field = new Field2d();

        swerveModules = new SwerveModule[Constants.NUMBER_OF_WHEELS];

        front_right = new SwerveModule(Constants.Offsets.FRONT_RIGHT_OFFSET, Constants.ModuleConst.FRONT_RIGHT_MOVE_MOTOR_ID,
                Constants.ModuleConst.FRONT_RIGHT_TURN_MOTOR_ID,
                Constants.ModuleConst.FRONT_RIGHT_CANCODER_ID, Constants.ModuleConst.FRONT_RIGHT_SET_INVERT_TYPE);
        back_right = new SwerveModule(Constants.Offsets.BACK_RIGHT_OFFSEST, Constants.ModuleConst.BACK_RIGHT_MOVE_MOTOR_ID,
                Constants.ModuleConst.BACK_RIGHT_TURN_MOTOR_ID,
                Constants.ModuleConst.BACK_RIGHT_CANCODER_ID, Constants.ModuleConst.BACK_RIGHT_SET_INVERT_TYPE);
        front_left = new SwerveModule(Constants.Offsets.FRONT_LEFT_OFFSET,
        Constants.ModuleConst.FRONT_LEFT_MOVE_MOTOR_ID,
        Constants.ModuleConst.FRONT_LEFT_TURN_MOTOR_ID,
        Constants.ModuleConst.FRONT_LEFT_CANCODER_ID, Constants.ModuleConst.FRONT_LEFT_SET_INVERT_TYPE);
        back_left = new SwerveModule(Constants.Offsets.BACK_LEFT_OFFSEST, Constants.ModuleConst.BACK_LEFT_MOVE_MOTOR_ID,
                Constants.ModuleConst.BACK_LEFT_TURN_MOTOR_ID,
                Constants.ModuleConst.BACK_LEFT_CANCODER_ID, Constants.ModuleConst.BACK_LEFT_SET_INVERT_TYPE);

        swerveModules[0] = back_right;
        swerveModules[1] = front_left;
        swerveModules[2] = front_right;
        swerveModules[3] = back_left;

        Pose2d zeroPose = new Pose2d(0, 0, getRotation2dGyroPosition());
        poseEstimator = new SwerveDrivePoseEstimator(Constants.Kinematics.SWERVE_KINEMATICS,
        getRotation2dGyroPosition(), getcurrentModulesPosition(), zeroPose);
        

       // setDefaultCommand(new XBoxControlwTrigger(this));
        calibrateGyro(0);

        SmartDashboard.putData("Field", getField());

        // SmartDashboard.putData("reset gyro", new InstantCommand(() ->
        // calibrateGyro(SmartDashboard.getNumber("gyro calibrate 2 angle", 0))));
        // SmartDashboard.putData("Calibrate All", new InstantCommand(() ->
        // calibrateAll(), this).ignoringDisable(true));
        SmartDashboard.putNumber("wanted", 0);
        SmartDashboard.putData("setRobotAngle", new Get2Angle(this, 90));



    }

    public SwerveModuleState[] getCurrentModuleStates() {
        SwerveModuleState[] sModuleStates = new SwerveModuleState[Constants.NUMBER_OF_WHEELS];
        for (int i = 0; i < sModuleStates.length; i++) {
            sModuleStates[i] = swerveModules[i].getState();
        }
        
        return sModuleStates;

    }

    public SwerveModulePosition[] getcurrentModulesPosition() {
        SwerveModulePosition[] sModulesPosition = new SwerveModulePosition[Constants.NUMBER_OF_WHEELS];
        for (int i = 0; i < sModulesPosition.length; i++) {
            sModulesPosition[i] = swerveModules[i].getPosition();
        }
        
        return sModulesPosition;
        }

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
            

            // System.out.println("i=" + i + " optimize " + sModuleStatesOptimized[i].angle.getDegrees()
            //  + " before opt=" + sModuleStates[i].angle.getDegrees() + " current angle = " 
            //  + angle);

            //  System.out.println("i=" + i + " optimize " + sModuleStatesOptimized[i].speedMetersPerSecond
            //  + " before opt=" + sModuleStates[i].speedMetersPerSecond + " current angle = " 
            //  + swerveModules[i].getVel());



        }
        // System.out.println("--------");


        return sModuleStatesOptimized;

    }

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

            System.out.println("i=" + i + " optimize " + sModuleStatesOptimized[i].angle.getDegrees()
             + " before opt=" + sModuleStates[i].angle.getDegrees() + " current angle = " 
             + angle);

        }


        return sModuleStatesOptimized;
    }

    public SwerveModule[] getThisSwerveModules() {
        return swerveModules;
    }

    public double getGyroPosition() {
        return RobotContainer.gyro.getFusedHeading();
    }

    public Rotation2d getRotation2dGyroPosition() {
        return Rotation2d.fromDegrees(getGyroPosition());
    }

    public void setModules(SwerveModuleState[] swerveModulesState) {
        for (int i = 0; i < swerveModulesState.length; i++) {
            swerveModules[i].setAngle(swerveModulesState[i].angle.getDegrees());
            swerveModules[i].setVel(swerveModulesState[i].speedMetersPerSecond);
            
        }
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPoseEstimator(Pose2d pose) { 
        poseEstimator.resetPosition(getRotation2dGyroPosition(), getcurrentModulesPosition(), pose);
    }

    public void poseEstimatorUpdate() {
        poseEstimator.update(getRotation2dGyroPosition(), getcurrentModulesPosition());

    }
    public void poseEstimatorUpdateByVision(Pose2d visionPose) {
        poseEstimator.addVisionMeasurement(visionPose, Timer.getFPGATimestamp());
    }

    public void setField(Pose2d pose) {
        field.setRobotPose(pose);
    }

    public void calibrateAll() {
        for (int i=0; i<swerveModules.length; i++) {
            swerveModules[i].calibrate();
        }
    }

    public Field2d getField() {
        return field;
    }

    public void setNeutralModeAngle(boolean isBrake) {
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setNeutraleModeSteerMotor(isBrake);
        }
    }

    public void setNeutralModeVelocity(boolean isBrake) {
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setNeutraleModeMoveMotor(isBrake);
        }
    }

    public void setPowerAngle(double power) {
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setPowerAngle(power);
        }
    }

    public void setAngle(double angle) {
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setAngle(angle);
        }
    }

    public void setPowerVelocity(double power) {
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setPowerVelocity(power);
        }
    }

    public void calibrateGyro(double value) {
        SmartDashboard.putNumber("is gyro calibrated", value);
        RobotContainer.gyro.setFusedHeading(value);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        SmartDashboard.putNumber("gyro calibrate 2 angle", 0);
        
    }


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
    public void periodic() {
        SmartDashboard.putNumber("module 0 angle", swerveModules[0].getAngle());
        SmartDashboard.putNumber("module 0 speed", swerveModules[0].getVel());

        SmartDashboard.putNumber("module 1 angle", swerveModules[1].getAngle());
        SmartDashboard.putNumber("module 1 speed", swerveModules[1].getVel());

        SmartDashboard.putNumber("module 2 angle", swerveModules[2].getAngle());
        SmartDashboard.putNumber("module 2 speed", swerveModules[2].getVel());

        SmartDashboard.putNumber("module 3 angle", swerveModules[3].getAngle());
        SmartDashboard.putNumber("module 3 speed", swerveModules[3].getVel());

        SmartDashboard.putNumber("Gyro Angle", getGyroPosition());

        SmartDashboard.putNumber("offset back right", swerveModules[0].getOffset());
        SmartDashboard.putNumber("offset front left", swerveModules[1].getOffset());
        SmartDashboard.putNumber("offset front right", swerveModules[2].getOffset());
        SmartDashboard.putNumber("offset back left", swerveModules[3].getOffset());

        SmartDashboard.putNumber("current Pose2d x", getPose().getX());
        SmartDashboard.putNumber("current Pose2d y", getPose().getY());

        //System.out.println("distance meters= " + getcurrentModulesPosition()[0].distanceMeters);
        // try {
        //     Sendable poseVision = SmartDashboard.getData("botpose");
        //     poseEstimatorUpdateByVision((Pose2d) poseVision);
        // } finally {

        // }

        // if (NetworkTableInstance.getDefault()
        // .getTable("limelight").getEntry("tv").getNumber(0)==1) {
        //     poseEstimatorUpdateByVision(NetworkTableInstance.getDefault()
        // .getTable("limelight").getEntry("botpose"));

        // }

        poseEstimatorUpdate();
        setField(getPose());
        //fieldEntry.setValue(getField());        

    }

}

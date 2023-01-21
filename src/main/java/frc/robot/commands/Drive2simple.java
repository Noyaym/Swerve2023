package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis.Chassis;
import frc.robot.utils.Utils;

public class Drive2simple extends CommandBase{

    private Chassis chassis;
    private Pose2d targetPose;
    private SwerveModuleState[] swerveModulesStates;

    public Drive2simple(Chassis ch, Pose2d targetPose) {
        this.chassis = ch;
        addRequirements(chassis);
        this.targetPose = targetPose;
    }
    @Override
    public void initialize() {
        chassis.setNeutralModeSteer(true);
        chassis.setNeutralModeMove(true);
    }

    @Override
    public void execute() {
        swerveModulesStates = Utils.driveToHolonimic(chassis.getPose(), targetPose);

        swerveModulesStates = chassis.getModulesOptimize(swerveModulesStates);
        chassis.setModules(swerveModulesStates);
        
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("is in pose", Utils.isInPose(chassis.getPose(), targetPose));
        return Utils.isInPose(chassis.getPose(), targetPose);
    }

    @Override
    public void end(boolean interrupted) {
        chassis.setPowerSteer(0);
        chassis.setPowerVelocity(0);
        chassis.setNeutralModeSteer(false);
        chassis.setNeutralModeMove(false);
    }
    
}

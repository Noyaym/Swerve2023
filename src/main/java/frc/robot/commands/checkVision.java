package frc.robot.commands;

import com.ctre.phoenix.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis.Chassis;
import frc.robot.utils.Utils;

public class checkVision extends CommandBase {

    private Chassis chassis;
    private SwerveModuleState[] swerveModuleStates;

    
    public checkVision(Chassis chassis) {
        this.chassis = chassis;
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        chassis.setNeutralModeMove(true);
        chassis.setNeutralModeSteer(true);
    }

    @Override
    public void execute() {
        if (chassis.isVision()) {
            swerveModuleStates = Utils.getModuleStates(-0.5, 0, 0, 
            Rotation2d.fromDegrees(Utils.getGyroPosition(RobotContainer.gyro)));

            swerveModuleStates = chassis.getModulesOptimize(swerveModuleStates);
            chassis.setModules(swerveModuleStates);

        }
        else {
            chassis.setPowerVelocity(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        chassis.setNeutralModeMove(false);
        chassis.setNeutralModeSteer(false);  
    }
    
}

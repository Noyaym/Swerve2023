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
    }

    @Override
    public void execute() {
        if (chassis.isVision()) {
            swerveModuleStates = Utils.getModuleStatesRight(0, 0, true, 
            Rotation2d.fromDegrees(Utils.getGyroPosition(RobotContainer.gyro)));

            swerveModuleStates = chassis.getModulesOptimize(swerveModuleStates);
            chassis.setModules(swerveModuleStates);

        }
    }
    
}

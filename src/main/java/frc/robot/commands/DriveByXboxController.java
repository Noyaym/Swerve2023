package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis.Chassis;
import frc.robot.utils.Utils;

public class DriveByXboxController extends CommandBase{

    private Chassis chassis;
    private SwerveModuleState[] sms;

    public DriveByXboxController(Chassis ch) {
        this.chassis = ch;
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        chassis.setNeutralModeAngle(true);
        chassis.setNeutralModeVelocity(true);
    }

    @Override
    public void execute() {
        double vx = Utils.timesMaxVelocity(Utils.getXboxControllerY(RobotContainer.xBoxController));
        System.out.println("vx= " + vx);
        double vy = Utils.timesMaxVelocity(Utils.getXboxControllerX(RobotContainer.xBoxController));
        System.out.println("vy= " + vy);
        boolean isPressedLeft = Utils.isLeftBumperXboxPressed(RobotContainer.xBoxController);
        boolean isPressedRight = Utils.isRightBumperXboxPressed(RobotContainer.xBoxController);
        if(isPressedRight) {
            sms = Utils.getModuleStatesRight(vx, vy, isPressedRight, 
        Rotation2d.fromDegrees(Utils.getGyroPosition(RobotContainer.gyro)));
        }
        else {
            sms = Utils.getModuleStatesLeft(vx, vy, isPressedLeft, 
        Rotation2d.fromDegrees(Utils.getGyroPosition(RobotContainer.gyro)));
        }

        sms = chassis.getModulesOptimize(sms);
        if (vx == 0 && vy == 0 && !isPressedLeft && !isPressedRight) {
            chassis.setPowerAngle(0);
            chassis.setPowerVelocity(0);
        }
        else
            chassis.setModules(sms);
            
    }

    @Override
    public void end(boolean interrupted) {
        chassis.setNeutralModeVelocity(false);
        chassis.setNeutralModeAngle(false);
    }
    
}

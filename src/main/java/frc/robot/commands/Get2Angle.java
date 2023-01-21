package frc.robot.commands;

import java.io.Console;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis.Chassis;
import frc.robot.utils.Utils;

public class Get2Angle extends CommandBase{

    private Chassis chassis;
    private SwerveModuleState[] swerveModulesStates;
    private double wantedDegree;

    public Get2Angle(Chassis chassis, double wantedDegree) {
        this.chassis = chassis;
        this.wantedDegree = wantedDegree;
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        chassis.setNeutralModeSteer(true);
        chassis.setNeutralModeMove(true);
    }

    @Override
    public void execute() {
        //wantedRad = Utils.getXBoxControllerAngle(RobotContainer.xBoxController, false);
        swerveModulesStates = Utils.getSwerveStates(0, 0, wantedDegree);
        swerveModulesStates = chassis.getModulesOptimize(swerveModulesStates);
        chassis.setModules(swerveModulesStates);   
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Utils.radianFromDegrees(wantedDegree)-Utils.getGyroPosition(RobotContainer.gyro))<
        Constants.ChassisConst.DEADBAND_AUTONOMOUS_RAD/180*Math.PI;
    }

    @Override
    public void end(boolean interrupted) {
        chassis.setNeutralModeSteer(false);
        chassis.setNeutralModeMove(false);
        chassis.setPowerSteer(0);
        chassis.setPowerSteer(0);
    }

    
    
}

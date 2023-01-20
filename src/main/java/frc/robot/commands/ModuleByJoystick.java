package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.utils.SwerveModule;
import frc.robot.utils.Utils;

public class ModuleByJoystick extends CommandBase {

    private SwerveModule module;
    private Joystick joystick;

    public ModuleByJoystick(SwerveModule module) {
        this.module = module;
        joystick = RobotContainer.joystickXY;
    }

    @Override
    public void execute() {
        double powerSpeed = Utils.getJoystickX(joystick);
        double powerAngle = Utils.getJoystickY(joystick);

        module.setPowerVelocity(powerSpeed);
        module.setPowerSteerMotor(powerAngle);
    }

    @Override
    public void end(boolean interrupted) {
        module.setPowerVelocity(0);
        module.setPowerSteerMotor(0);
    }

}

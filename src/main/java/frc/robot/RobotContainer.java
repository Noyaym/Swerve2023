// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ModuleOne;
import frc.robot.subsystems.Chassis.Chassis;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  //  new Drive2simple(this.chassis, targetPose).andThen(new InstantCommand(()-> {chassis.setPowerAngle(0);
  //   chassis.setPowerVelocity(0);}))
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //Pose2d targetPose = new Pose2d(1, 2, new Rotation2d());
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }

  


//Command tryYCommand = new InstantCommand(()-> chassis.setNeutralModeAngle(true), chassis);



  public ModuleOne moduleOne;
  public static PigeonIMU gyro;
  public static Joystick joystickXY;
  public static Joystick joystickDirection;
  public Chassis chassis;
  public static JoystickButton button;
  public static XboxController xBoxController;

  public RobotContainer() {
    gyro = new WPI_PigeonIMU(Constants.ChassisConst.GYRO_PORT_NUM);
    joystickXY = new Joystick(Constants.Buttons.JOYSTICK_XY_PORT_NUM);
    joystickDirection = new Joystick(Constants.Buttons.JOYSTICK_DIRECTION_PORT_NUM);
    moduleOne = new ModuleOne();
    chassis = new Chassis(gyro);
    button = new JoystickButton(joystickXY, 1);
    xBoxController = new XboxController(Constants.Buttons.XBOX_PORT_NUM);

  }


  public Joystick getJoystickXY() {
    return joystickXY;
  }

  public Joystick getJoystickDirection() {
    return joystickDirection;
  }

  // Command findFeedForward = new InstantCommand(() -> chassis.setNeutralModeAngle(true), chassis)
      // .andThen(new InstantCommand(() -> chassis.setAngle(0), chassis),
        //  new InstantCommand(() -> chassis.setPowerVelocity(0.2), chassis),
          // new WaitCommand(2),
          //  new InstantCommand(() -> SmartDashboard.putNumber("Velocity power 1",
              //  chassis.getThisSwerveModules()[0].getVel()), chassis),
          //  new InstantCommand(() -> chassis.setPowerVelocity(0), chassis),
          // new WaitCommand(2),
          // new InstantCommand(() -> chassis.setPowerVelocity(0.6), chassis),
          // new WaitCommand(2),
          // new InstantCommand(() -> SmartDashboard.putNumber("Velocity power 2",
          // chassis.getThisSwerveModules()[0].getVel()), chassis),
          // new InstantCommand(() -> chassis.setNeutralModeAngle(false), chassis));

}

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SwerveModule;

public class ModuleOne extends SubsystemBase {

    private SwerveModule module;
    public boolean hasSetVel = false;
    // private NetworkTableEntry velocityEntry = Shuffleboard.getTab("Module 1 data").add("Velocity", 0).getEntry();
    // private NetworkTableEntry angleEntry = Shuffleboard.getTab("Module 1 data").add("Angle", 0).getEntry();
    // private NetworkTableEntry offsetEntry = Shuffleboard.getTab("Module 1 data").add("offset", 0).getEntry();
    // private NetworkTableEntry motorPosEntry = Shuffleboard.getTab("Module 1 data").add("motorPosition", 0).getEntry();
    // private NetworkTableEntry targetVelocityEntry = Shuffleboard.getTab("Module 1 data").add("targetVelocity", 0).getEntry();
    // private NetworkTableEntry targetAngleEntry = Shuffleboard.getTab("Module 1 data").add("targetAngle", 0).getEntry();

    public ModuleOne() {

        module = new SwerveModule(Constants.Offsets.BACK_RIGHT_OFFSEST, Constants.ModuleConst.BACK_RIGHT_MOVE_MOTOR_ID,
                Constants.ModuleConst.BACK_RIGHT_TURN_MOTOR_ID,
                Constants.ModuleConst.BACK_RIGHT_CANCODER_ID, Constants.ModuleConst.BACK_RIGHT_SET_INVERT_TYPE);
        // module.getMoveMotor().setInverted(Constants.ModuleConst.BACK_RIGHT_SET_INVERT_TYPE);
        // Command setVelocityCommand = new RunCommand(() -> module.setVel(targetVelocityEntry.getDouble(0)), this)
        //        .andThen(new InstantCommand(() -> module.setVel(0), this));
        // Shuffleboard.getTab("Module 1 data").add((Sendable) setVelocityCommand);
        // SmartDashboard.putNumber("target vel", 0);
        // SmartDashboard.putNumber("target angle", 0);

        SmartDashboard.putData("set vel",
                new RunCommand(() -> module.setVel(SmartDashboard.getNumber("target velocity", 0)), this)
                        .andThen(new InstantCommand(() -> module.setVel(0), this)));

        SmartDashboard.putData("set angle",
                new RunCommand(() -> module.setAngle(SmartDashboard.getNumber("target angle", 0)), this)
                .andThen(new InstantCommand(() -> module.setPowerAngle(0), this)));

        SmartDashboard.putData("Calibrate", new InstantCommand(() ->
        module.calibrate()));
    }

    public double getVelocity() {
        return module.getVel();
    }

    public double getAngle() {
        return module.getAngle();
    }

    public double getSelectedSensorPosition() {
        return module.getSelectedSensorPosition();
    }

    public double getOffset() {
        return module.getOffset();
    }

    public double getError() {
        return module.getSteerMotor().getClosedLoopError();
    }

    public boolean isErrorNegative() {
        return module.getSteerMotor().getClosedLoopError()<0;
    }

    public double getTarget() {
        if (hasSetVel)
            return module.getSteerMotor().getClosedLoopTarget();
        return 0;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        SmartDashboard.putNumber("target velocity", 0);
        SmartDashboard.putNumber("target angle", 0);
        builder.addDoubleProperty("velocity", this::getVelocity, null);
        builder.addDoubleProperty("angle", this::getAngle, null);

        builder.addDoubleProperty("offset", this::getOffset, null);
        builder.addDoubleProperty("pulses", this::getSelectedSensorPosition, null);
        builder.addDoubleProperty("error", this::getError, null);
        builder.addDoubleProperty("target", this::getTarget, null);
        builder.addBooleanProperty("is negative", this::isErrorNegative, null);

    }

// Command setAngleCommand = new RunCommand(() -> module.setAngle(targetAngleEntry.getDouble(0)), this)
//            .andThen(new InstantCommand(() -> module.setPowerAngle(0), this));

    // Command calibrateCommand = new InstantCommand(() -> module.calibrate(), this);

    @Override
    public void periodic() {
        //velocityEntry.setDouble(getVelocity());
        //angleEntry.setDouble(getAngle());
        //offsetEntry.setDouble(getOffset());
       // motorPosEntry.setDouble(getSelectedSensorPosition());

        // Shuffleboard.getTab("Module 1 data").add("set angle command", setAngleCommand);
        // Shuffleboard.getTab("Module 1 data").add("calibrate command", calibrateCommand);

    }

}

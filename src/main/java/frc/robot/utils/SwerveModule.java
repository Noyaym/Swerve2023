package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConst;

public class SwerveModule {

    private final TalonFX mVel;
    private final TalonFX mAngle;
    private final CANCoder encoder;
    private final SimpleMotorFeedforward ff;
    private double offset;

    public SwerveModule(double offset, int vel, int angle, int CAN, boolean setInverted) {
        this.mVel = new WPI_TalonFX(vel);
        this.mAngle = new WPI_TalonFX(angle);
        this.encoder = new WPI_CANCoder(CAN);
        this.offset = offset;

        mVel.configFactoryDefault();
        mAngle.configFactoryDefault();

        mAngle.setInverted(setInverted);

        mVel.config_kP(0, ModuleConst.mVel_Kp);
        mVel.config_kI(0, ModuleConst.mVel_Ki);
        mVel.config_kD(0, ModuleConst.mVel_Kd);

        mAngle.config_kP(0, ModuleConst.mAngle_Kp);
        mAngle.config_kI(0, ModuleConst.mAngle_Ki);
        mAngle.config_kD(0, ModuleConst.mAngle_Kd);

        //mAngle.configAllowableClosedloopError(0, 10); TODO: check if works

        this.ff = new SimpleMotorFeedforward(ModuleConst.Ks, ModuleConst.Kv);

        // if (isC) {
        //     calibrate();
        // }

    }

    public double getAngle() {
        double value  = encoder.getAbsolutePosition() - offset;
        if (value<0) value = 360 + value;
        return value;
    }

    

    public double getAngleNotWithin360() {
        return encoder.getAbsolutePosition() - offset;
    }

    public Rotation2d getAngleRotation2d() {
        return Rotation2d.fromDegrees(encoder.getAbsolutePosition() - offset);
    }



    public double getVel() {
        return mVel.getSelectedSensorVelocity() / ModuleConst.PULSE_PER_METER * 10;
    }

    public double getDistance() {
        return mVel.getSelectedSensorPosition() / ModuleConst.PULSE_PER_METER;
    }

    public double getOffset() {
        return offset;
    }

    public void setVel(double velocity) {
        mVel.set(ControlMode.Velocity, velocity * ModuleConst.PULSE_PER_METER / 10,
                DemandType.ArbitraryFeedForward, ff.calculate(velocity));
    }



    public double convertAngle2Pulse(double angle) {
        return angle * ModuleConst.PULSE_PER_ANGLE;
    }

    public double convertOffset2Pulse() {
        return offset * ModuleConst.PULSE_PER_ANGLE;
    }

    public double calcFF(double angle) {
        return Math.signum(angle-getAngle())*(10+getAngle())*ModuleConst.mAngle_Kp; // still not sure about that
    }

    public double FeedForward(double difference) {
        if (Math.abs(difference)<Constants.ModuleConst.TOLERANCE) {
            return 0;
        }
        return Math.signum(difference)*ModuleConst.mAngle_Ks;
    }

    public void setAngle(double angle) {
        double dif = angle - getAngle();
        double difference = Utils.optimizeAngleDemacia(dif);
        mAngle.set(ControlMode.Position,
                mAngle.getSelectedSensorPosition()+convertAngle2Pulse(difference),
                DemandType.ArbitraryFeedForward, 
                FeedForward(difference));
        SmartDashboard.putNumber("difference", difference);
        SmartDashboard.putNumber("angle", angle);
        SmartDashboard.putNumber("selctedSensorPosition", 
        mAngle.getSelectedSensorPosition()/ModuleConst.PULSE_PER_ANGLE);
    }

    public void setPowerAngle(double power) {
        mAngle.set(ControlMode.PercentOutput, power);
    }

    public void setPowerVelocity(double power) {
        mVel.set(ControlMode.PercentOutput, power);
    }

    public void setNeutraleModeSteerMotor(boolean isBrake) {
        mAngle.setNeutralMode(isBrake? NeutralMode.Brake:NeutralMode.Coast);
    }

    public void setNeutraleModeMoveMotor(boolean isBrake) {
        mVel.setNeutralMode(isBrake? NeutralMode.Brake:NeutralMode.Coast);
    }

    public TalonFX getMoveMotor() {
        return mVel;
    }

    public SwerveModuleState getState() { // gets the state of a module
        double velocity = getVel();
        Rotation2d angle = Rotation2d.fromDegrees(getAngle());

        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition() { // gets the position of a module.
        double distanceDriven = getDistance();
        return new SwerveModulePosition(distanceDriven, getAngleRotation2d());
    }

    public TalonFX getSteerMotor() {
        return mAngle;
    }

    public double getSelectedSensorPosition() {
        return mAngle.getSelectedSensorPosition();
    }

    // @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Module vel", this::getVel, null);
        builder.addDoubleProperty("Module angle", this::getAngle, null);

    }

    public void calibrate() {
        offset = encoder.getAbsolutePosition();
        mAngle.setSelectedSensorPosition(0);
    }

}

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public class Constants {
    public static final double intakeSpinMotorSupplyCurrentLimit = 0;
    public static final double intakeSpinMotorClosedLoopRampPeriod = 0;
    public static final double intakeSpinMotorPeakForwardVoltage = 0;
    public static final double intakeSpinMotorPeakReverseVoltage = 0;
    public static final InvertedValue intakeSpinMotorDirection = InvertedValue.Clockwise_Positive;
    public static final double intakeSpinMotorProportional = 0;
    public static final double intakeSpinMotorIntegral = 0;
    public static final double intakeSpinMotorDerivative = 0;
    public static final double intakeSpinMotorGravityFeedForward = 0;
    public static final double intakeSpinMotorVelocityFeedForward = 0;
    public static final double intakeSpinMotorStaticFeedForward = 0;

    public static final InvertedValue intakeElectricSlideMotorDirection = InvertedValue.Clockwise_Positive;
    public static final double intakeElectricSlideMotorSupplyCurrentLimit = 0;
    public static final double intakeElectricSlideMotorVoltageClosedLoopRampPeriod = 0;
    public static final double intakeElectricSlideMotorMaxForwardVoltage = 0;
    public static final double intakeElectricSlideMotorMaxReverseVoltage = 0;
    public static final double intakeElectricSlideMotorProportional = 0;
    public static final double intakeElectricSlideMotorIntegral = 0;
    public static final double intakeElectricSlideMotorDerivative = 0;
    public static final double intakeElectricSlideMotorVelocityFeedForward = 0;
    public static final double intakeElectricSlideMotorGravityFeedForward = 0;
    public static final double intakeElectricSlideMotorStaticFeedForward = 0;
}

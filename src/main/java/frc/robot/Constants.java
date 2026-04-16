package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;


public class Constants {
    public static final CANBus kCanivore = new CANBus("Canivore");
    public static final double[] kBlueHubCoord = {4.627, 4.024};
    public static final double[] kRedHubCoord = {11.939, 4.024};
    public static final double[] kBluePassCoords = {1.905, 6.222, 2.059};

    // key - distance    value - speed
    public static final InterpolatingDoubleTreeMap kSpeedTable = new InterpolatingDoubleTreeMap();
    static {
        kSpeedTable.put(4.57, 99.0);
        kSpeedTable.put(3.6, 89.0);
        kSpeedTable.put(3.39, 83.0); 
        kSpeedTable.put(3.1, 80.0); // SET FOR GOOD
        kSpeedTable.put(2.99, 76.0);
        kSpeedTable.put(2.6, 65.0);
        /* 
        kSpeedTable.put(4.57, 83.0);
        kSpeedTable.put(3.39, 75.0);
        kSpeedTable.put(2.6, 64.0);
        */
    } 

    // This is only necessary for shooting on move
    // The value is the time from shooting to the time that the shot lands
    // key - distance    value - time till shot lands
    public static final InterpolatingDoubleTreeMap kShotTimeTable = new InterpolatingDoubleTreeMap();
    static {
        kShotTimeTable.put(3.66, 1.3);
        kShotTimeTable.put(2.74, 1.2);
    }

    // limelight forward pos according to intake position
    public static final InterpolatingDoubleTreeMap kLLForwardPos = new InterpolatingDoubleTreeMap();
    static {
        kLLForwardPos.put(0.0, 0.4699);
        kLLForwardPos.put(-2.0, 0.6199);
    }
    public static final double kAccelCompFactor = 0.1;
    public static final double kCustomBrownout = 5.0;

/*
 * ####################################################################################################################################
 * ##################################################### Positions ###################################################################
 * ####################################################################################################################################
*/

// Velocity Controls
    public static final double kIndexerMainSpeed = 85;
    public static final double kIndexerNoSpeed = 0;

    public static final double kShooterOutSpeed = 67;
    public static final double kShooterAutoSpeed = 75;

    public static final double kRollerMainSpeed = 0.55;

    public static final double kIntakeInSpeed = -60;
    public static final double kIntakeInPower = -0.55;
    public static final double kIntakeIdleSpeed = -10;
    public static final double kIntakeNoSpeed = 0;
    


// Position Controls
    public static final double kClimbRotateOutPos = 13;
    public static final double kClimbRotateDownPos = 0;

    // Intake 0 -> -15
    public static final double kIntakeSlideOutPos = -14.6;
    public static final double kIntakeMidPos = -9;
    public static final double kIntakeSlideInPos = -3;

    public static final double kHoodServoDownPos = 0.2;

    public static final double kTurretMax = 2;
    public static final double kTurretMin = -2;
    // Left turret .3 to -4
    // Right turret -.4 to 4


/*
 * ####################################################################################################################################
 * ####################################################### Intake Spin ###############################################################
 * ####################################################################################################################################
*/
    
    public static final double kIntakeSpinMotorProportional = 2;
    public static final double kIntakeSpinMotorIntegral = 0;
    public static final double kIntakeSpinMotorDerivative = 0;

    public static final double kIntakeSpinMotorVelocityFeedForward = 0.122;
    public static final double kIntakeSpinMotorStaticFeedForward = 0;
    public static final double kIntakeSpinMotorGravityFeedForward = 0;

    public static final double kIntakeSpinMotorPeakForwardVoltage = 11;
    public static final double kIntakeSpinMotorPeakReverseVoltage = -11;

    public static final InvertedValue kIntakeSpinMotorDirection = InvertedValue.Clockwise_Positive;
    public static final double kIntakeSpinMotorSupplyCurrentLimit = 40;

    public static final double kIntakeSpinMotorClosedLoopRampPeriod = 0;
    
  
/*
 * ####################################################################################################################################
 * ####################################################### Intake Slide ###############################################################
 * ####################################################################################################################################
*/

    public static final double kIntakeElectricSlideMotorProportional = 2;
    public static final double kIntakeElectricSlideMotorIntegral = 0;
    public static final double kIntakeElectricSlideMotorDerivative = 0;    

    public static final double kIntakeElectricSlideMotorVelocityFeedForward = 0.2;
    public static final double kIntakeElectricSlideMotorGravityFeedForward = 0;

    public static final double kIntakeElectricSlideMotorMaxForwardVoltage = 11;
    public static final double kIntakeElectricSlideMotorMaxReverseVoltage = -11;

    public static final double kIntakeElectricSlideMotorVoltageClosedLoopRampPeriod = 0;
    public static final InvertedValue kIntakeElectricSlideMotorDirection = InvertedValue.CounterClockwise_Positive;
    public static final double kIntakeElectricSlideMotorSupplyCurrentLimit = 20;
    
/*
 * ####################################################################################################################################
 * ####################################################### Hood ###############################################################
 * ####################################################################################################################################
*/

    public static final double kHoodMotorProportional = 4;
    public static final double kHoodMotorIntegral = 0;
    public static final double kHoodMotorDerivative = 0;    

    public static final double kHoodMotorVelocityFeedForward = 0.2;
    public static final double kHoodMotorStaticFeedForward = 0.05;
    public static final double kHoodMotorGravityFeedForward = 0;

    public static final double kHoodMotorMaxForwardVoltage = 11;
    public static final double kHoodMotorMaxReverseVoltage = -11;

    public static final double kHoodMotorVoltageClosedLoopRampPeriod = 0;
    public static final InvertedValue kHoodMotorDirection = InvertedValue.CounterClockwise_Positive;
    public static final double kHoodMotorSupplyCurrentLimit = 40;
    


/*
 * ####################################################################################################################################
 * ####################################################### Roller ###############################################################
 * ####################################################################################################################################
*/
    
    public static final double kRightRollerSpinMotorProportional = 1;
    public static final double kRightRollerSpinMotorIntegral = 0;
    public static final double kRightRollerSpinMotorDerivative = 0;

    public static final double kRightRollerSpinMotorVelocityFeedForward = 0.122;
    public static final double kRightRollerSpinMotorStaticFeedForward = 0;
    public static final double kRightRollerSpinMotorGravityFeedForward = 0;

    public static final double kRightRollerSpinMotorPeakForwardVoltage = 11;
    public static final double kRightRollerSpinMotorPeakReverseVoltage = -11;

    public static final InvertedValue kRightRollerSpinMotorDirection = InvertedValue.CounterClockwise_Positive;
    public static final double kRightRollerSpinMotorSupplyCurrentLimit = 40;

    public static final double kRightRollerSpinMotorClosedLoopRampPeriod = 0;

/*
 * ####################################################################################################################################
 * ####################################################### Shooter Left ###############################################################
 * ####################################################################################################################################
*/

    public static final double kLeftShooterMotorProportional = 15;
    public static final double kLeftShooterMotorIntegral = 0;
    public static final double kLeftShooterMotorDerivative = 0;

    public static final double kLeftShooterMotorGravityFeedForward = 0;
    public static final double kLeftShooterMotorVelocityFeedForward = 0;
    public static final double kLeftShooterMotorStaticFeedForward = 3.05;

    public static final double kLeftShooterMotorPeakForwardVoltage = 11;
    public static final double kLeftShooterMotorPeakReverseVoltage = -11;

    public static final InvertedValue kLeftShooterMotorDirection = InvertedValue.Clockwise_Positive;
    public static final double kLeftShooterMotorSupplyCurrentLimit = 40;

    public static final double kLeftShooterMotorClosedLoopRampPeriod = 0;
    
    
/*
 * ####################################################################################################################################
 * ####################################################### Shooter Right ###############################################################
 * ####################################################################################################################################
*/

    public static final double kRightShooterMotorProportional = 11;
    public static final double kRightShooterMotorIntegral = 0;
    public static final double kRightShooterMotorDerivative = 0;

    public static final double kRightShooterMotorGravityFeedForward = 0;
    public static final double kRightShooterMotorVelocityFeedForward = 0;
    public static final double kRightShooterMotorStaticFeedForward = 3.05;

    public static final double kRightShooterMotorPeakForwardVoltage = 11;
    public static final double kRightShooterMotorPeakReverseVoltage = -11;

    public static final InvertedValue kRightShooterMotorDirection = InvertedValue.CounterClockwise_Positive;
    public static final double kRightShooterMotorSupplyCurrentLimit = 40;

    public static final double kRightShooterMotorClosedLoopRampPeriod = 0;
    
    
/*
 * ####################################################################################################################################
 * ####################################################### Indexer ###############################################################
 * ####################################################################################################################################
*/


    public static final double kIndexerMotorProportional = 10;
    public static final double kIndexerMotorIntegral = 0;
    public static final double kIndexerMotorDerivative = 0;

    public static final double kIndexerMotorGravityFeedForward = 0;
    public static final double kIndexerMotorVelocityFeedForward = 0.112;
    public static final double kIndexerMotorStaticFeedForward = 3.05;


    public static final double kIndexerMotorPeakForwardVoltage = 11;
    public static final double kIndexerMotorPeakReverseVoltage = -11;

    public static final InvertedValue kIndexerMotorDirection = InvertedValue.Clockwise_Positive;
    public static final double kIndexerMotorSupplyCurrentLimit = 40;

    public static final double kIndexerMotorClosedLoopRampPeriod = 0;
    

/*
 * ####################################################################################################################################
 * ####################################################### Turret Left ###############################################################
 * ####################################################################################################################################
*/

    public static final double kLeftTurretMotorProportional = 13;
    public static final double kLeftTurretMotorIntegral = 0;
    public static final double kLeftTurretMotorDerivative = 0.1;

    public static final double kLeftTurretMotorGravityFeedForward = 0;
    public static final double kLeftTurretMotorVelocityFeedForward = 0.9;
    public static final double kLeftTurretMotorStaticFeedForward = 0.1;
    
    public static final double kLeftTurretMotorPeakForwardVoltage = 11;
    public static final double kLeftTurretMotorPeakReverseVoltage = -11;

    public static final InvertedValue kLeftTurretMotorDirection = InvertedValue.Clockwise_Positive;
    public static final double kLeftTurretMotorSupplyCurrentLimit = 40;

    public static final double kLeftTurretMotorClosedLoopRampPeriod = 0;
    
/*
 * ####################################################################################################################################
 * ####################################################### Turret Right ###############################################################
 * ####################################################################################################################################
*/

    public static final double kRightTurretMotorProportional = 13;
    public static final double kRightTurretMotorIntegral = 0;
    public static final double kRightTurretMotorDerivative = 0.1;

    public static final double kRightTurretMotorGravityFeedForward = 0;
    public static final double kRightTurretMotorVelocityFeedForward = 0.9;
    public static final double kRightTurretMotorStaticFeedForward = 0.1;
    
    public static final double kRightTurretMotorPeakForwardVoltage = 11;
    public static final double kRightTurretMotorPeakReverseVoltage = -11;

    public static final InvertedValue kRightTurretMotorDirection = InvertedValue.Clockwise_Positive;
    public static final double kRightTurretMotorSupplyCurrentLimit = 40;

    public static final double kRightTurretMotorClosedLoopRampPeriod = 0;
    
    
}

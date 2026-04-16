// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.Named;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.PASS;
import frc.robot.commands.hoodDown;
import frc.robot.commands.hoodUp;
import frc.robot.commands.hoodsOUT;
import frc.robot.commands.shootNo;
import frc.robot.commands.shootStill;
import frc.robot.commands.spittersAreQuitters;
import frc.robot.commands.turnTsAround;
import frc.robot.commands.turretZero;
import frc.robot.commands.Auton.autonIntake;
import frc.robot.commands.Auton.autonShoot;
import frc.robot.commands.Auton.autonTurret;
import frc.robot.commands.Auton.flywheel;
import frc.robot.commands.Auton.intakeMid;
import frc.robot.commands.Intake.carryIntake;
import frc.robot.commands.Intake.deployIntake;
import frc.robot.commands.Intake.retractIntake;
import frc.robot.commands.autoShoot.NshootMove;
import frc.robot.commands.autoShoot.NtargetMove;
import frc.robot.commands.autoShoot.NtargetStill;
import frc.robot.commands.autoShoot.shootSelf;
import frc.robot.commands.autoShoot.shootTargetMove;
import frc.robot.commands.autoShoot.smartPass;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooterIndexer;
import frc.robot.subsystems.intakeFuel;
import frc.robot.subsystems.hoodServo;
import frc.robot.subsystems.turretHood;
import frc.robot.subsystems.spiralRoller;;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final shooterIndexer m_shooterSubsystem = new shooterIndexer();
    private final intakeFuel m_intakeSubsystem = new intakeFuel();
    private final spiralRoller m_spiralRollerSubsystem = new spiralRoller();
    private final hoodServo m_hoodServoSubsystem = new hoodServo();
    private final turretHood m_turretSubsystem = new turretHood();

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);


    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public Command AutonomousRun;

  


    private final SendableChooser<Command> autoChooser;


    public RobotContainer() {
        regisiterAutonCommands();
       autoChooser = AutoBuilder.buildAutoChooser("Test Auto");
        SmartDashboard.putData("PLEAAAASE", autoChooser);

        configureBindings();

       FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        
     
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
     
        // driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        
        /*driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivet
        rain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        */
        
        // reset the field-centric heading on start press
        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        

        // DRIVER CONTROLS
        driverController.povUp().whileTrue(new smartPass(drivetrain, driverController, m_shooterSubsystem, m_spiralRollerSubsystem, m_intakeSubsystem, m_hoodServoSubsystem));

        driverController.leftBumper().onTrue(new retractIntake(m_intakeSubsystem));
        driverController.rightBumper().onTrue(new deployIntake(m_intakeSubsystem));

        driverController.rightTrigger().whileTrue(new shootStill(m_shooterSubsystem, m_spiralRollerSubsystem, m_intakeSubsystem))
         .onFalse(new shootNo(m_shooterSubsystem, m_spiralRollerSubsystem, m_intakeSubsystem, m_hoodServoSubsystem));
        driverController.back().onTrue(new hoodsOUT(m_hoodServoSubsystem));
        driverController.povUp().onTrue(new carryIntake(m_intakeSubsystem));
        driverController.y().onTrue(new hoodUp(m_hoodServoSubsystem));
        driverController.a().onTrue(new hoodDown(m_hoodServoSubsystem));
        driverController.x().whileTrue(new spittersAreQuitters(m_shooterSubsystem, m_spiralRollerSubsystem, m_intakeSubsystem));
        driverController.b().whileTrue(new PASS(m_shooterSubsystem, m_spiralRollerSubsystem, m_intakeSubsystem, m_hoodServoSubsystem));


        // OPERATOR CONTROLS
        operatorController.leftTrigger().whileTrue(new shootSelf(m_shooterSubsystem, m_spiralRollerSubsystem, drivetrain, m_intakeSubsystem, m_turretSubsystem))
         .onFalse(new shootNo(m_shooterSubsystem, m_spiralRollerSubsystem, m_intakeSubsystem, m_hoodServoSubsystem));
        operatorController.leftTrigger().whileTrue(new NtargetStill(drivetrain, m_turretSubsystem));
        operatorController.rightTrigger().whileTrue(new shootSelf(m_shooterSubsystem, m_spiralRollerSubsystem, drivetrain, m_intakeSubsystem, m_turretSubsystem))
         .onFalse(new shootNo(m_shooterSubsystem, m_spiralRollerSubsystem, m_intakeSubsystem, m_hoodServoSubsystem));
        operatorController.rightTrigger().whileTrue(new NtargetStill(drivetrain, m_turretSubsystem));

        operatorController.x().onTrue(new turretZero(m_turretSubsystem));
        operatorController.b().onTrue(new turnTsAround(m_turretSubsystem));
        operatorController.y().onTrue(new hoodUp(m_hoodServoSubsystem));
        operatorController.a().onTrue(new hoodDown(m_hoodServoSubsystem));
    

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void regisiterAutonCommands(){
        NamedCommands.registerCommand("intakeDeploy", new autonIntake(m_intakeSubsystem));
        NamedCommands.registerCommand("shootStill", new autonShoot(m_shooterSubsystem, m_spiralRollerSubsystem, m_intakeSubsystem));
        NamedCommands.registerCommand("intakeRetract", new retractIntake(m_intakeSubsystem));
        NamedCommands.registerCommand("hoodUp", new hoodUp(m_hoodServoSubsystem));
        NamedCommands.registerCommand("hoodDown", new hoodDown(m_hoodServoSubsystem));
        NamedCommands.registerCommand("flywheel", new flywheel(m_shooterSubsystem, m_spiralRollerSubsystem, m_intakeSubsystem));
        NamedCommands.registerCommand("turretZero", new turretZero(m_turretSubsystem));
        NamedCommands.registerCommand("turretForward", new autonTurret(m_turretSubsystem));
        NamedCommands.registerCommand("intakeMid", new intakeMid(m_intakeSubsystem));
    }
    public Command getAutonomousCommand() {
      
        return autoChooser.getSelected();
    }
}

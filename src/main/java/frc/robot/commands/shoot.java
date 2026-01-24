// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.shooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class shoot extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final shoot m_shooterSubsystem;

  private boolean m_isFinished = false;
  
    /**
     * Creates a new set-PowerCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public shoot(shoot shooterSubsystem) {
      m_shooterSubsystem = shooterSubsystem;
  
      // addRequirements(m_shooterSubsystem);
    }
   
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      // m_shooterSubsystem.setShooterSpinSpeed(.57);  
    }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  
}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}

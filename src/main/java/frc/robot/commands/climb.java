package frc.robot.commands;



import frc.robot.subsystems.climberSubsystem;
import frc.robot.subsystems.intakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class climb extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final climberSubsystem m_climberSubsystem;
  
    private boolean m_isFinished = false;
  
  
    /**
     * Creates a new set-PowerCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public climb(climberSubsystem climberSubsystem) {
      m_climberSubsystem = climberSubsystem;
  
     
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(climberSubsystem);
      
          
    }
   
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_climberSubsystem.setclimberSpinSpeed(1);
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
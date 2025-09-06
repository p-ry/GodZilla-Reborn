package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.InitLogger;
import frc.robot.subsystems.Ace;



public class WaitForCoral extends Command {
      private final Ace ace;
    
      public WaitForCoral(Ace ace) {
        this.ace = ace;
        
      }
    
      @Override
      public void initialize() {
       InitLogger.logMessage("Ace","Starting Auto Loading");
       
      }
    
      @Override
      public boolean isFinished() {
        return ace.getCurrentState() == Ace.CoralIntakeState.COMPLETE;
      }
    
      @Override
      public void end(boolean interrupted) {
        ace.setSpeed(0);
        if (interrupted) {
          InitLogger.logMessage("Ace","Interrupted");
          
          
        } else {
          InitLogger.logMessage("Ace","AutoLoading Complete");
         
          
        }
      }
    }
    



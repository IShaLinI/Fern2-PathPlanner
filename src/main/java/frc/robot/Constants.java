// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.util.Units;

public class Constants {

  public static class DriveConstants{

    public static double kTrackwidth = Units.inchesToMeters(19);
    public static double kDistancePerPulse = (1.0/2048d) * (Units.inchesToMeters(6) * Math.PI) * (1/10.71);

    public static PIDConstants mTrajConstants = new PIDConstants(0.5, 0, 0);
    public static double kTrajectoryMaxSpeed = 1;
	  public static double kTrajectoryMaxAccel = 1;
    
    public static double kMaxSpeed = 6380.0 * (1 / 10.71) * (Units.inchesToMeters(6) * Math.PI) * (1 / 60d);
    public static double kMaxTurnSpeed = (kMaxSpeed * (1 / (kTrackwidth * Math.PI))) * (2 * Math.PI); 
    
    public static double kTurboTurningSpeed = 0.8;
    public static double kNormalTurningSpeed = 0.2;
    public static double kTurboForwardSpeed = 1;
    public static double kNormalForwardSpeed = 0.8;

    public static double kSlowForwardSpeed = 0.6;
    public static double kSlowTurningSpeed = 0.2;


    public static enum FrontState {
     
      FORWARD(1),
      REVERSE(-1);

      public final double direction;

      /**
       * @param direction Motor Percentage
       */

      FrontState(double direction) {
        this.direction = direction;
      }
    }
   }

   public static enum ModState {
     
    TURBO(DriveConstants.kTurboForwardSpeed, DriveConstants.kTurboTurningSpeed),
    NORMAL(DriveConstants.kNormalForwardSpeed, DriveConstants.kNormalTurningSpeed),
    SLOW(DriveConstants.kSlowForwardSpeed, DriveConstants.kSlowTurningSpeed);

    public final double xMod;
    public final double rotMod;

    /**
     * @param modifier Turbo or Slow
     */

    ModState(double xMod, double rotMod) {
      this.xMod = xMod;
      this.rotMod = rotMod;
    }
  }

    public static class RobotConstants{
        public static double maxVoltage = 15;
        
        public static class CAN {
          public static int kFrontLeft = 1;
          public static int kBackLeft = 2;
          public static int kFrontRight = 3;
          public static int kBackRight = 4;
          public static int kPigeon = 5;
          public static int kIntake = 6;
          public static int kFrontPivot = 7;
          public static int kBackPivot = 8;

        }
    }

    public static class IntakeConstants {
    
        public static final double kInSpeed = 0.25;
        public static final double kOutSpeed = 0.40;
    
        public static final double kS = 1;
    
        public static enum State {
          
          GRAB(-kInSpeed),
          RELEASE(kOutSpeed),
          L1RELEASE(0.1),
          L2RELEASE(0.15),
          L3RELEASE(0.25),
          IDLE(-kS/12),
          STOP(0),
          STARTING(0);
    
          public final double speed;
    
          /**
           * @param speed Motor Percentage
           */
          State(double speed) {
            this.speed = speed;
          }
    
        }
    
      }

      public static class PivotConstants {

        public static double kGearing = ((1.0 / 20) * (16.0 / 60));
        public static double kThroughboreOffset = 0.5149;

        public static enum State {
          SUBSTATION(5),
          L1(45),
          L2(-20),
          L3(-30),
          STOP(0),
          STARTING(-90),
          FLOOR(110),
          CARRY(-90);
    
          public final double angle;
    
          /**
           * @param angle Pivot Angle
           */
          State(double angle) {
            this.angle = angle;
          }
    
        }
    
      }
}
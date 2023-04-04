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

    public static PIDConstants mTrajConstants = new PIDConstants(0, 0, 0);
    public static double kTrajectoryMaxSpeed = 3;
	  public static double kTrajectoryMaxAccel = 3;
    
    public static double kMaxSpeed = 6380.0 * (1 / 10.71) * (Units.inchesToMeters(6) * Math.PI) * (1 / 60d);
    public static double kMaxTurnSpeed = (kMaxSpeed * (1 / (kTrackwidth * Math.PI))) * (2 * Math.PI); 
    
    public static double kTurboForwardSpeed = 1;
    public static double kNormalForwardSpeed = 0.6;
    public static double kTurboTurningSpeed = 1;
    public static double kNormalTurningSpeed = 0.2;

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
    
        public static final double kInSpeed = 0.35;
        public static final double kOutSpeed = 0.99;
    
        public static final double kS = 0;
    
        public static enum State {
          GRAB(-kInSpeed),
          RELEASE(kOutSpeed),
          L1RELEASE(0.30),
          L2RELEASE(0.60),
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

        public static double kGearing = ((1.0 / 25) * (16.0 / 60));
        public static double kVelocityConversion = kGearing * (1 / 60.0) * 360;
        public static double kPositionConversion = kGearing * 360;
        public static double kThroughboreOffset = 0.475;

        
    

        public static enum State {
          SUBSTATION(5),
          L1(45),
          L2(-20),
          STOP(0),
          STARTING(-80),
          FLOOR(110),
          CARRY(-80);
    
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
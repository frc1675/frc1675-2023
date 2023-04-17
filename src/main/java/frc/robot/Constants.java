// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
      //balance constants
      public static final double AUTO_BALANCE_TOLERANCE_DEGREES = 4;
      public static final double AUTO_BALANCE_ENGAGE_DEGREES = 10;
      public static final double MAX_AUTO_BALANCE_TRANSLATION_METERS = 5;
      
      //auto constants
      public static final double AUTO_MAX_VELOCITY = 3;
      public static final double AUTO_MAX_ACCELERATION = 3;    

      //drivetrain constants
      public static final double DRIVE_ROTATE_TOLERANCE_DEGREES = 1.0;
      public static final double DRIVE_SPEED_SCALER = 0.85;
      public static final double SLOW_DRIVE_SCALING = 0.3;
      public static final double SLOW_ROTATION_SCALING = 0.5;
      public static final double SLEW_RATE_LIMIT = 6.0;
  
      //PID constants
      public static final double ROTATION_PROPORTIONAL_COEFFICENT = 3.0;
      public static final double ROTATION_INTEGRAL_COEFFICENT = 0.5;
      public static final double ROTATION_DERIVATIVE_COEFFICENT = 0.5;

      public static final double TRANSLATION_PROPORTIONAL_COEFFICENT = 0.050;
      public static final double TRANSLATION_INTEGRAL_COEFFICENT = 0.0017;
      public static final double TRANSLATION_DERIVATIVE_COEFFICENT = 0;
  
      //controller constants
      public static final int DRIVER_CONTROLLER = 0;
      public static final int OPERATOR_CONTROLLER = 1;
      public static final int LEFT_X_AXIS = 0;
      public static final int LEFT_Y_AXIS= 1;
      public static final int RIGHT_X_AXIS = 4;
      public static final int RIGHT_Y_AXIS = 5;
      public static final int RIGHT_TRIGGER = 3;
  
      public static final int A_BUTTON = 1;
      public static final int B_BUTTON = 2;
      public static final int X_BUTTON = 3;
      public static final int Y_BUTTON = 4;

      public static final int LEFT_BUMPER = 5;
      public static final int RIGHT_BUMPER = 6;
      public static final int BACK_BUTTON = 7;
      public static final int START_BUTTON = 8;
      public static final int LEFT_JOYSTICK_BUTTON = 9;
      public static final int RIGHT_JOYSTICK_BUTTON = 10;

      //arm constants
      public static final double ARM_POWER_SCALING = 0.1;
      public static final int ARM_MOTOR = 13;
      public static final double ARM_P_COEFF = 2.5;
      public static final double ARM_I_COEFF = 0;
      public static final double ARM_D_COEFF = 0;
      public static final double ARM_SCORE_HIGH_POSITION = 0.5522-.02;  
      public static final double ARM_SCORE_MID_POSITION = 0.6137;
      public static final double ARM_HUMAN_PLAYER_POSITION = 0.5522- .02;
      public static final double ARM_INSIDE_ROBOT_POSITION = 0.8168;
      public static final double ARM_ENCODER_COUNT_ERROR = 1.0;
      public static final double MAX_ARM_VELOCITY = 0.5;
      public static final double MAX_ARM_ACCELERATION = MAX_ARM_VELOCITY * 2;
      public static final double ARM_ENCODER_COUNT_MAX_DIFF = 0.1;

      //intake constants 
      public static final int INTAKE_CURRENT_LIMIT = 20;
      public static final double INTAKE_SPEED = 1;
      public static final double INTAKE_SPEED_SLOW = 0.25;
      public static final int INTAKE_MOTOR = 15;

      //floor arm constants
      public static final double FLOOR_ARM_POWER_SCALING = 0.1;
      public static final int FLOOR_ARM_MOTOR = 14;
      public static final double FLOOR_ARM_P_COEFF = 2;
      public static final double FLOOR_ARM_I_COEFF = 0;
      public static final double FLOOR_ARM_D_COEFF = 0;
      public static final double FLOOR_ARM_GROUND_POSITION = 0.21 - 0.03;
      public static final double FLOOR_ARM_AUTO_GROUND_POSITION = 0.20;
      public static final double FLOOR_ARM_INSIDE_ROBOT_POSITION = 0.85;
      public static final double FLOOR_ARM_SHOOTING_POSITION = 0.9564;

      //floor intake constants
      public static final double FLOOR_INTAKE_NORMAL_SPEED = 0.5;
      public static final double FLOOR_INTAKE_FAST_SPEED = 1;
      public static final int FLOOR_INTAKE_MOTOR = 16;
      public static final int FLOOR_INTAKE_CURRENT_LIMIT = 40;
}

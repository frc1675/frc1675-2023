// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
      //auto constants
      public static final double AUTO_MAX_VELOCITY = 3;
      public static final double AUTO_MAX_ACCELERATION = 3;

      //sim constants
      public static final double RED_ORIGIN_POS_X_METERS = 16.541748984;
      public static final double RED_ORIGIN_POS_Y_METERS = 8.01367968;
      public static final double RED_ORIGIN_ROTATION_DEG = 180.0;      

      //drivetrain constants
      public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5969; //Measured on robot
      public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5969;
      public static final double DRIVE_ROTATE_TOLERANCE_DEGREES = 1.0;
      public static final double DRIVE_SPEED_SCALER = 0.4;
  
      //PID constants
      public static final double PROPORTIONAL_COEFFICENT = 3.0;
      public static final double INTEGRAL_COEFFICENT = 0.5;
      public static final double DERIVATIVE_COEFFICENT = 0.5;
  
      //swerve module constants
      public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3;
      public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4;
      public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 12; 
      public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(39.81);
  
      public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
      public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2; 
      public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 10; 
      public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(209.18);
  
      public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7; 
      public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8; 
      public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 11; 
      public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(14.15);
  
      public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5; 
      public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6; 
      public static final int BACK_LEFT_MODULE_STEER_ENCODER = 9; 
      public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(289.59);
  
      //controller constants
      public static final int DRIVER_CONTROLLER = 0;
      public static final int OPERATOR_CONTROLLER = 1;
      public static final int LEFT_X_AXIS = 0;
      public static final int LEFT_Y_AXIS= 1;
      public static final int RIGHT_X_AXIS = 4;
      public static final int RIGHT_Y_AXIS = 5;
      public static final int A_BUTTON = 1;
      public static final int B_BUTTON = 2;
      public static final int X_BUTTON = 3;
      public static final int Y_BUTTON = 4;
      public static final int LEFT_BUMPER = 5;
      public static final int RIGHT_BUMPER = 6;
      public static final int BACK_BUTTON = 7;
      public static final int START_BUTTON = 8;
      public static final int LEFT_JOYSTICK_BUTTON = 9;

      //arm constants
      public static final double ARM_POWER_SCALING = 0.1;
      public static final int ARM_MOTOR = 13;
      public static final double ARM_P_COEFF = 2.5;
      public static final double ARM_I_COEFF = 0;
      public static final double ARM_D_COEFF = 0;
      public static final double ARM_SCORE_HIGH_POSITION = 0.4009;
      public static final double ARM_SCORE_MID_POSITION = 0.4614;
      public static final double ARM_HUMAN_PLAYER_POSITION = 0.4038;
      public static final double ARM_INSIDE_ROBOT_POSITION = 0.6527;
      public static final double ARM_MAX_POSITION = 0.0;//fully within robot
      public static final double ARM_MIN_POSITION = 0.0;//completely outside robot
      public static final double ARM_ENCODER_COUNT_ERROR = 1.0;

      //intake constants 
      public static final double INTAKE_SPEED = 1;
      public static final int INTAKE_MOTOR = 15;

      //floor arm constants
      public static final double FLOOR_ARM_POWER_SCALING = 0.1;
      public static final int FLOOR_ARM_MOTOR = 14;
      public static final double FLOOR_ARM_P_COEFF = 2;
      public static final double FLOOR_ARM_I_COEFF = 0;
      public static final double FLOOR_ARM_D_COEFF = 0;
      public static final double FLOOR_ARM_GROUND_POSITION = 0.0652;
      public static final double FLOOR_ARM_INSIDE_ROBOT_POSITION = 0.7043;
      public static final double FLOOR_ARM_SHOOTING_POSITION = 0.8302;
      public static final double FLOOR_ARM_MAX_POSITION = 0;
      public static final double FLOOR_ARM_MIN_POSITION = 0;

      //floor intake constants
      public static final double FLOOR_INTAKE_NORMAL_SPEED = 0.5;
      public static final double FLOOR_INTAKE_FAST_SPEED = 1;
      public static final int FLOOR_INTAKE_MOTOR = 16;
}

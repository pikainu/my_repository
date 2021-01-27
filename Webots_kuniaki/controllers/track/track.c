/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:   An example of a controller using the Track node to move
 *                tracked robot.
 *
 *                The Coulomb friction value set in the WorldInfo contactProperties
 *                has a big influence on the behvaviour of the robot and especially
 *                on the turning motion.
 */

#include <webots/accelerometer.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <stdio.h>
#include <stdlib.h>

#define NB_SENSORS 10

#define TIME_STEP 100



int main(int argc, char **argv) {
  //int timeStep = wb_robot_get_basic_time_step();
  FILE *outputfile;         // 出力ストリーム
  wb_robot_init();
  int i,j;
  int flag = 5;
  int dir = 1;
  WbDeviceTag ps[NB_SENSORS];
  char name[10][16] = {"ds_right", "ds_left", "ds_lowR", "ds_lowL", "ds_ac1", "ds_ac2", "ds_center", "ds_bk" , "ds_bkR", "ds_bkL"};
  double sensor_value[NB_SENSORS];
  
  
  
  for (i = 0; i < NB_SENSORS; i++) {
    ps[i] = wb_robot_get_device(name[i]);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  };
  // Get the accelerometer and enable it.
  WbDeviceTag accelerometer = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accelerometer, TIME_STEP);
  WbDeviceTag leftMotor = wb_robot_get_device("left motor");
  WbDeviceTag rightMotor = wb_robot_get_device("right motor");
  wb_motor_set_position(leftMotor, INFINITY);
  wb_motor_set_position(rightMotor, INFINITY);
  WbDeviceTag actuater_1 = wb_robot_get_device("actuater 1");
  WbDeviceTag actuater_2 = wb_robot_get_device("actuater 2");
  //WbDeviceTag actuater_3 = wb_robot_get_device("actuater 3");
  //WbDeviceTag actuater_4 = wb_robot_get_device("actuater 4");

  // go straight
    wb_motor_set_velocity(leftMotor, -0.04);
    wb_motor_set_velocity(rightMotor, -0.04);
    j = 0;
    
  while (wb_robot_step(TIME_STEP) != -1) {
    const double *acceleration = wb_accelerometer_get_values(accelerometer);
    printf("time is %f \n",wb_robot_get_time());
    printf("acceleration0 is %f \n",acceleration[0]);
    printf("acceleration1 is %f \n",acceleration[1]);
    printf("acceleration2 is %f \n",acceleration[2]);
    
    outputfile = fopen("ACdatadown.txt", "a");  // ファイルを書き込み用にオープン(開く)
    if (outputfile == NULL) {          // オープンに失敗した場合
      printf("cannot open\n");         // エラーメッセージを出して
      exit(1);                         // 異常終了
    }
    fprintf(outputfile, "%f %f\n",acceleration[0] ,acceleration[1]); // ファイルに書く
    
    fclose(outputfile);          // ファイルをクローズ(閉じる)
    
    for (i = 0; i < NB_SENSORS; i++)
      sensor_value[i] = wb_distance_sensor_get_value(ps[i]);
      
    if (flag == 1) {
      wb_motor_set_position(actuater_1, wb_motor_get_max_position(actuater_1));
      wb_motor_set_position(actuater_2, wb_motor_get_max_position(actuater_2));
      
      if (sensor_value[4] < 250.0 || sensor_value[5] < 250.0)
        flag = 2;
    }else if(flag == 2){
      wb_motor_set_position(actuater_1, wb_motor_get_min_position(actuater_1));
      wb_motor_set_position(actuater_2, wb_motor_get_min_position(actuater_2));
      if (sensor_value[6] > 770.0){
        j = wb_robot_get_time();
        flag = 3;
      }
    }else if(flag == 3){
      wb_motor_set_position(actuater_1, 0.055);
      wb_motor_set_position(actuater_2, 0.055);
      
      if (sensor_value[7] < 200.0){
        wb_motor_set_position(actuater_1, wb_motor_get_min_position(actuater_1));
        wb_motor_set_position(actuater_2, wb_motor_get_min_position(actuater_2));
        if (wb_robot_get_time() - j > 5){
          flag = 0;
        }
      }
      
    }else if(flag == 4){
      wb_motor_set_position(actuater_1, 0.06);
      wb_motor_set_position(actuater_2, 0.06);
        if (sensor_value[7] < 500.0){
          wb_motor_set_position(actuater_1, wb_motor_get_min_position(actuater_1));
          wb_motor_set_position(actuater_2, wb_motor_get_min_position(actuater_2));
          flag = 5;
        }
    }else if(flag == 5){
      if (sensor_value[2] < 950.0 && sensor_value[3] < 950.0){
        j = wb_robot_get_time();
        flag = 6;
      }
    }else if(flag == 6){
      //wb_motor_set_position(actuater_1, 0.06);
      //wb_motor_set_position(actuater_2, 0.06);
      
      if (wb_robot_get_time() - j > 5){
        wb_motor_set_position(actuater_1, wb_motor_get_min_position(actuater_1));
        wb_motor_set_position(actuater_2, wb_motor_get_min_position(actuater_2));
        j = 0;
        flag = 0;
      }
      
    }else if (sensor_value[2] < 600.0 || sensor_value[3] <600.0){
      wb_motor_set_velocity(leftMotor, 0.04);
      wb_motor_set_velocity(rightMotor, 0.04);
      flag = 1;
      
      
    }else if (sensor_value[7] > 700.0){
        // go left
        wb_motor_set_velocity(leftMotor, -0.04);
        wb_motor_set_velocity(rightMotor, -0.04);
        flag = 4;
    }else {
      
      if ((sensor_value[0] < 950.0 && sensor_value[1] < 950.0) || dir == 1){
        // go back
        wb_motor_set_velocity(leftMotor, -0.2);
        wb_motor_set_velocity(rightMotor, -0.2);
        dir = 1;
      }
      if ((sensor_value[8] < 950.0 && sensor_value[9] < 950.0) || dir == 0){
        // go straight
        wb_motor_set_velocity(leftMotor, 0.2);
        wb_motor_set_velocity(rightMotor, 0.2);
        dir = 0;
      }
    }
  }
  
/*
  // go straight
  wb_motor_set_velocity(leftMotor, 0.1);
  wb_motor_set_velocity(rightMotor, 0.1);
  
  i = 500;
  while (wb_robot_step(timeStep) != -1 && i > 0) {
    --i;
  };
  
  // turn left
  wb_motor_set_velocity(leftMotor, 0.02);
  wb_motor_set_velocity(rightMotor, 0.02);
  wb_motor_set_position(actuater_1, wb_motor_get_max_position(actuater_1));
  wb_motor_set_position(actuater_2, wb_motor_get_max_position(actuater_2));
  

  i = 500;
  while (wb_robot_step(timeStep) != -1 && i > 0) {
    --i;
  };
  
  // turn right
  wb_motor_set_position(actuater_1, wb_motor_get_min_position(actuater_1));
  wb_motor_set_position(actuater_2, wb_motor_get_min_position(actuater_2));
  wb_motor_set_velocity(leftMotor, 0.05);
  wb_motor_set_velocity(rightMotor, 0.05);

  while (wb_robot_step(timeStep) != -1) {
  };
*/
  wb_robot_cleanup();

  return 0;
};

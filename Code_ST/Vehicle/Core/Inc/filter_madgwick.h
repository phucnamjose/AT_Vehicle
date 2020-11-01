/*
 * Madgwick.h
 *
 *  Created on: 5 thg 5, 2020
 *      Author: PC
 */

#ifndef IMU_MADGWICK_MADGWICK_H_
#define IMU_MADGWICK_MADGWICK_H_


//---------------------------------------------------------------------------------------------------
// Function declarations

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
           // vector to hold integral error for Mahony method
//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
#endif /* IMU_MADGWICK_MADGWICK_H_ */

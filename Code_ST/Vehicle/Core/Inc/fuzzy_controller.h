/*
 * fuzzy_controller.h
 *
 *  Created on: Nov 2, 2020
 *      Author: Dang Nam
 */

#ifndef INC_FUZZY_CONTROLLER_H_
#define INC_FUZZY_CONTROLLER_H_

typedef struct trimf{
	double a1;
	double a2;
	double a3;
} trimf;

/* Trapezoid function */
typedef struct trapf {
	double h1;
	double h2;
	double h3;
	double h4;
} trapf;


typedef struct IMU {
	/* Current Angle and Set Angle */
	double      Angle;
	double      Set_Angle;
	double      Pre_Angle;
	/* Fuzzy input and output */
	double      Fuzzy_Out;
	double      Pre_Fuzzy_Out;
	double      Fuzzy_Error;
	double      Fuzzy_Error_dot;
	/* Variables Ke, Kedot and Ku */
	double      Ke;
	double      Kedot;
	double      Ku;
} IMU;

/*--------Fuzzy control-------------------*/
#define Min(a, b)		((a) < (b)) ? (a) : (b)
#define Max(a, b)		((a) < (b)) ? (b) : (a)
#define Prod(a, b)		((a) * (b))

void                    Fuzzy_ParametersInit(void);
double                  Trapf(trapf *ptrapf, double x);
double                  Trimf(trimf *ptrimf, double x);
void                    Trimf_Update(trimf *ptrimf, double a1, double a2, double a3);
void                    Trapf_Update(trapf *ptrapf, double a1, double a2, double a3, double a4);
double                  Defuzzification_Max_Min(double e, double edot);
//double                  Defuzzification2_Max_Min(double e, double edot);

/*--------IMU functions ---------*/
void                    IMU_UpdateFuzzyInput(IMU *pimu);
void                    IMU_UpdateFuzzyCoefficients(IMU *pimu, double Ke, double Kedot, double Ku);

#endif /* INC_FUZZY_CONTROLLER_H_ */

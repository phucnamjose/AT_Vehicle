/*
 * fuzzy_controller.c
 *
 *  Created on: Nov 2, 2020
 *      Author: Dang Nam
 */

#include "fuzzy_controller.h"
#include "system_params.h"
#include "def_myself.h"

IMU             Mag;
/* Internal variables */
double          NB, NM, NS, ZE, PS, PM, PB; // Sugeno output
trimf           In1_NS, In1_ZE, In1_PS, In2_ZE;
trapf           In1_NB, In1_PB, In2_NE, In2_PO;



/* Implementation */

/*-------------------- Fuzzy control -----------------------*/
/** @brief  : Find maximum number
**  @agr    : 2 input values
**  @retval : Output maximum value
**/
double Fuzzy_Max(double *input,int len)
{
	double max;
	max = input[0];
	for(int i = 1; i < len; i++)
	{
		if(max < input[i])
			max = input[i];
	}
	return max;
}

/** @brief  : Trapf function
**  @agr    : 4 parameters (left to right)
**  @retval : Value relates to x
**/
double Trapf(trapf *ptrapf, double x)
{
	double result;
	if(x < ptrapf->h1) result = 0;
	else if(x < ptrapf->h2) result = (x - ptrapf->h1) / (ptrapf->h2 - ptrapf->h1);
	else if(x < ptrapf->h3) result = 1;
	else if(x < ptrapf->h4) result = (ptrapf->h4 - x) / (ptrapf->h4 - ptrapf->h3);
	else result = 0;
	return result;
}

/** @brief  : Trimf function
**  @agr    : 3 parameters (left to right)
**  @retval : Value relates to x
**/
double Trimf(trimf *ptrimf, double x)
{
	double result;
	if(x < ptrimf->a1) result = 0;
	else if(x < ptrimf->a2) result = (x - ptrimf->a1) / (ptrimf->a2 - ptrimf->a1);
	else if(x < ptrimf->a3) result = (ptrimf->a3 - x) / (ptrimf->a3 - ptrimf->a2);
	else result = 0;
	return result;
}

/** @brief  : Update Paramter for Trimf function
**  @agr    : 3 parameters (left to right)
**  @retval : Value relates to x
**/
void	Trimf_Update(trimf *ptrimf, double a1, double a2, double a3)
{
	ptrimf->a1 = a1;
	ptrimf->a2 = a2;
	ptrimf->a3 = a3;
}

/** @brief  : Update Paramerters for trapf function
**  @agr    : 3 parameters (left to right)
**  @retval : Value relates to x
**/
void	Trapf_Update(trapf *ptrapf, double a1, double a2, double a3, double a4)
{
	ptrapf->h1 = a1;
	ptrapf->h2 = a2;
	ptrapf->h3 = a3;
	ptrapf->h4 = a4;
}

/** @brief  : Fuzzy init parameter procedure
**  @agr    : 3 parameters (left to right)
**  @retval : Value relates to x
**/
void	Fuzzy_ParametersInit(void)
{
	/*   Input 1 (e = Set_theta - theta)  */
	// NB : -2 - -0.17
	Trapf_Update(&In1_NB,-2,-1,-0.22,-0.17);
	// NS : 0.15 - 0.45
	Trimf_Update(&In1_NS, -0.22, -0.11, 0.001);
	// ZE : 0 - 0.2
	Trimf_Update(&In1_ZE, -0.11, 0, 0.11);
	// PS : 0.15 - 0.45
	Trimf_Update(&In1_PS, 0.001, 0.11, 0.22);
	// PB : 0.4 - 1
	Trapf_Update(&In1_PB,0.17,0.22,1,2);

	/* Input 2 (edot = Set_thetadot - thetadot) */
	// NE : 0.3 - 1
	Trapf_Update(&In2_NE,-2,-1,-0.4,-0.003);
	// ZE : 0 - 0.4
	Trimf_Update(&In2_ZE, -0.4, 0, 0.4);
	// PO : 0.3 - 1
	Trapf_Update(&In2_PO, 0.003, 0.4, 1, 2);
	/* Output value */
	NB = -0.95;
	NM = -0.8;
	NS = -0.4;
	ZE = 0;
	PS = 0.4;
	PM = 0.8;
	PB = 0.95;
}

void SelectFuzzyOutput(double vel)
{
	/*----------Fuzzy parameter init ------------------*/
	if (vel < MPS2RPM(0.3))
	{
		/*   Input 1 (e = Set_theta - theta)  */
		// NB :
		Trapf_Update(&In1_NB, -2,-1, -0.22, -0.17);
		// NS :
		Trimf_Update(&In1_NS, -0.22, -0.11, -0.044);
		// ZE :
		Trimf_Update(&In1_ZE, -0.056, 0, 0.056);
		// PS :
		Trimf_Update(&In1_PS, 0.044, 0.11, 0.22);
		// PB :
		Trapf_Update(&In1_PB, 0.17, 0.22, 1, 2);

		/* Input 2 (edot = Set_thetadot - thetadot) */
		Trapf_Update(&In2_NE, -2, -1, -0.4, -0.05);
		Trimf_Update(&In2_ZE, -0.4, 0, 0.4);
		Trapf_Update(&In2_PO, 0.05, 0.4, 1, 2);
		/* Output value */
		NB = -0.95;
		NM = -0.8;
		NS = -0.4;
		ZE = 0;
		PS = 0.4;
		PM = 0.8;
		PB = 0.95;
	}
	else
	{
		/*   Input 1 (e = Set_theta - theta)  */
		// NB :
		Trapf_Update(&In1_NB, -2, -1, -0.45, -0.4);
		// NS :
		Trimf_Update(&In1_NS, -0.45, -0.2, -0.15);
		// ZE :
		Trimf_Update(&In1_ZE, -0.2, 0, 0.2);
		// PS :
		Trimf_Update(&In1_PS, 0.15, 0.2, 0.45);
		// PB :
		Trapf_Update(&In1_PB, 0.4, 0.45, 1, 2);
		/* Input 2 (edot = Set_thetadot - thetadot) */
		// NE :
		Trapf_Update(&In2_NE, -2, -1, -0.4, -0.1);
		// ZE :
		Trimf_Update(&In2_ZE, -0.4, 0, 0.4);
		// PO :
		Trapf_Update(&In2_PO, 0.1, 0.4, 1, 2);
		/* Output value */
		NB = -0.75;
		NM = -0.4;
		NS = -0.175;
		ZE = 0;
		PS = 0.175;
		PM = 0.4;
		PB = 0.75;
	}
}

/** @brief  : Defuzzification Max Min sugeno
**  @agr    : 2 input value
**  @retval : Output value
**/
double Defuzzification_Max_Min(double e, double edot)
{
	double pBeta[3], num, den, temp;
	double e_NB, e_NS, e_ZE, e_PS, e_PB, edot_NE, edot_ZE, edot_PO;
	e_NB = Trapf(&In1_NB, e);
	e_NS = Trimf(&In1_NS, e);
	e_ZE = Trimf(&In1_ZE, e);
	e_PS = Trimf(&In1_PS, e);
	e_PB = Trapf(&In1_PB, e);
	edot_NE = Trapf(&In2_NE, edot);
	edot_ZE = Trimf(&In2_ZE, edot);
	edot_PO = Trapf(&In2_PO, edot);
	//NB and NE is NB
	pBeta[0] = Min(e_NB, edot_NE);
	num = NB * pBeta[0];
	den = pBeta[0];
	//NS and NE is NM
	//NB and ZE is NM
	pBeta[0] = Min(e_NS, edot_NE);
	pBeta[1] = Min(e_NB, edot_ZE);
	temp = Fuzzy_Max(pBeta, 2);
	num += NM * temp;
	den += temp;
	//ZE and NE is NS
	//NS and ZE is NS
	//NB and PO is NS
	pBeta[0] = Min(e_ZE, edot_NE);
	pBeta[1] = Min(e_NS, edot_ZE);
	pBeta[2] = Min(e_NB, edot_PO);
	temp = Fuzzy_Max(pBeta, 3);
	num += NS * temp;
	den += temp;
	//PS and NE is ZE
	//ZE and ZE is ZE
	//NS and PO is ZE
	pBeta[0] = Min(e_PS, edot_NE);
	pBeta[1] = Min(e_ZE, edot_ZE);
	pBeta[2] = Min(e_NS, edot_PO);
	temp = Fuzzy_Max(pBeta, 3);
	num += ZE * temp;
	den += temp;
	//PB and NE is PS
	//PS and ZE is PS
	//ZE and PO is PS
	pBeta[0] = Min(e_PB, edot_NE);
	pBeta[1] = Min(e_PS, edot_ZE);
	pBeta[2] = Min(e_ZE, edot_PO);
	temp = Fuzzy_Max(pBeta,3);
	num += PS * temp;
	den += temp;
	//PB and ZE is PM
	//PS and PO is PM
	pBeta[0] = Min(e_PB, edot_ZE);
	pBeta[1] = Min(e_PS, edot_PO);
	temp = Fuzzy_Max(pBeta,2);
	num += PM * temp;
	den += temp;
	//PB and PO is PB
	pBeta[0] = Min(e_PB, edot_PO);
	num += PB * pBeta[0];
	den += pBeta[0];

	return (den == 0) ? 0 : (num/den);
}

//double Defuzzification2_Max_Min(double e, double edot)
//{
//	double pBeta[10], num, den;
//	double e_NB, e_NS, e_ZE, e_PS, e_PB, edot_NE, edot_ZE, edot_PO;
//	e_NB = Trapf(&In1_NB, e);
//	e_NS = Trimf(&In1_NS, e);
//	e_ZE = Trimf(&In1_ZE, e);
//	e_PS = Trimf(&In1_PS, e);
//	e_PB = Trapf(&In1_PB, e);
//	edot_NE = Trapf(&In2_NE, edot);
//	edot_ZE = Trimf(&In2_ZE, edot);
//	edot_PO = Trapf(&In2_PO, edot);
//	if(edot <= -0.4) // u_NE(edot) = 1
//	{
//		pBeta[0] = e_NB; // u_NB(e) -> output is NB
//		pBeta[1] = e_NS; // u_NS(e) -> output is NM
//		pBeta[2] = e_ZE; // u_ZE(e) -> output is NS
//		pBeta[3] = e_PS; // u_PS(e) -> output is ZE
//		pBeta[4] = e_PB; // u_PB(e) -> output is PS
//		num = pBeta[0]*NB + pBeta[1]*NM + pBeta[2]*NS + pBeta[3]*ZE + pBeta[4]*PS;
//		den = pBeta[0] + pBeta[1] + pBeta[2] + pBeta[3] + pBeta[4];
//	}
//	else if (edot < 0) // u_NE(edot) = trapf_NE(edot), u_ZE(edot) = trimf_ZE(edot)
//	{
//		/* u_NE(edot) = trapf_NE(edot) */
//		pBeta[0] = Min(e_NB, edot_NE); // output is NB
//		pBeta[1] = Min(e_NS, edot_NE); // output is NM
//		pBeta[2] = Min(e_ZE, edot_NE); // output is NS
//		pBeta[3] = Min(e_PS, edot_NE); // output is ZE
//		pBeta[4] = Min(e_PB, edot_NE); // output is PS
//		/* u_ZE(edot) = trimf_ZE(edot) */
//		pBeta[5] = Min(e_NB, edot_ZE); // output is NM
//		pBeta[6] = Min(e_NS, edot_ZE); // output is NS
//		pBeta[7] = Min(e_ZE, edot_ZE); // output is ZE
//		pBeta[8] = Min(e_PS, edot_ZE); // output is PS
//		pBeta[9] = Min(e_PB, edot_ZE); // output is PM
//		num = pBeta[0]*NB + Max(pBeta[1], pBeta[5])*NM + Max(pBeta[2], pBeta[6])*NS +
//				Max(pBeta[3], pBeta[7])*ZE + Max(pBeta[4], pBeta[8])*PS + pBeta[9]*PM;
//		den = pBeta[0] + pBeta[1] + pBeta[2] + pBeta[3] + pBeta[4] +
//				pBeta[5] + pBeta[6] + pBeta[7] + pBeta[8] + pBeta[9];
//	}
//	else if (edot < 0.4) // u_ZE(edot) = trimf_ZE(edot), u_PO(edot) = trapf_PO(edot)
//	{
//		/* u_ZE(edot) = trimf_ZE(edot) */
//		pBeta[0] = Min(e_NB, edot_ZE); // output is NM
//		pBeta[1] = Min(e_NS, edot_ZE); // output is NS
//		pBeta[2] = Min(e_ZE, edot_ZE); // output is ZE
//		pBeta[3] = Min(e_PS, edot_ZE); // output is PS
//		pBeta[4] = Min(e_PB, edot_ZE); // output is PM
//		/* u_PO(edot) = trapf_PO(edot) */
//		pBeta[5] = Min(e_NB, edot_PO); // output is NS
//		pBeta[6] = Min(e_NS, edot_PO); // output is ZE
//		pBeta[7] = Min(e_ZE, edot_PO); // output is PS
//		pBeta[8] = Min(e_PS, edot_PO); // output is PM
//		pBeta[9] = Min(e_PB, edot_PO); // output is PB
//		num = pBeta[0]*NM + Max(pBeta[1], pBeta[5])*NS + Max(pBeta[2], pBeta[6])*ZE +
//				Max(pBeta[3], pBeta[7])*PS + Max(pBeta[4], pBeta[8])*PM + pBeta[9]*PB;
//		den = pBeta[0] + pBeta[1] + pBeta[2] + pBeta[3] + pBeta[4] +
//				pBeta[5] + pBeta[6] + pBeta[7] + pBeta[8] + pBeta[9];
//	}
//	else // u_PO(edot) = 1
//	{
//		pBeta[0] = e_NB; // u_NB(e) -> output is NS
//		pBeta[1] = e_NS; // u_NS(e) -> output is ZE
//		pBeta[2] = e_ZE; // u_ZE(e) -> output is PS
//		pBeta[3] = e_PS; // u_PS(e) -> output is PM
//		pBeta[4] = e_PB; // u_PB(e) -> output is PB
//		num = pBeta[0]*NS + pBeta[1]*ZE + pBeta[2]*PS + pBeta[3]*PM + pBeta[4]*PB;
//		den = pBeta[0] + pBeta[1] + pBeta[2] + pBeta[3] + pBeta[4];
//	}
//	return (den == 0) ? 0 : (num/den);
//}


void	IMU_UpdateFuzzyInput(IMU *pimu)
{
	pimu->Fuzzy_Error = pimu->Set_Angle - pimu->Angle;
	pimu->Fuzzy_Error_dot = -(pimu->Angle - pimu->Pre_Angle) / (5*BASIC_PERIOD);
	pimu->Pre_Angle = pimu->Angle;
	pimu->Pre_Fuzzy_Out = pimu->Fuzzy_Out;

	if(pimu->Fuzzy_Error > 180)
		pimu->Fuzzy_Error -= 360;
	else if(pimu->Fuzzy_Error < -180)
		pimu->Fuzzy_Error += 360;

	pimu->Fuzzy_Error *= pimu->Ke;
	pimu->Fuzzy_Error_dot *= pimu->Kedot;
}

/** @brief  : Update fuzzy coefficients
**  @agr    : imu and Ke,kedot,ku
**  @retval : none
**/
void	IMU_UpdateFuzzyCoefficients(IMU *pimu, double Ke, double Kedot, double Ku)
{
	pimu->Ke	= Ke;
	pimu->Kedot = Kedot;
	pimu->Ku 	= Ku;
}


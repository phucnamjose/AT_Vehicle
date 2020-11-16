/*
 * fuzzy_controller.c
 *
 *  Created on: Nov 2, 2020
 *      Author: Dang Nam
 */

#include "fuzzy_controller.h"
#include "def_myself.h"
#include "system_params.h"

Fuzzy_t	myFuzzy;

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
void	Fuzzy_Init(void) {
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

	// Init factor
	myFuzzy.Ke 		= FUZZY_KE;
	myFuzzy.Kedot 	= FUZZY_KEDOT;
	myFuzzy.Ku 		= FUZZY_KU;
	myFuzzy.Fuzzy_Out 	= 0;
	myFuzzy.Fuzzy_Error = 0;
	myFuzzy.Fuzzy_Error_dot = 0;
}

void Fuzzy_SelectFuzzyOutput(double vel)
{
	/*----------Fuzzy parameter init ------------------*/
	if (vel < MPS2RPS(0.3))
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
double Fuzzy_Defuzzification_Max_Min(double e, double edot) {
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


void	Fuzzy_UpdateInput(Fuzzy_t *fuzzy) {
	fuzzy->Fuzzy_Error = fuzzy->Set_Angle - fuzzy->Angle;
	fuzzy->Fuzzy_Error_dot = -(fuzzy->Angle - fuzzy->Pre_Angle) / (5*BASIC_PERIOD);
	fuzzy->Pre_Angle = fuzzy->Angle;
	fuzzy->Pre_Fuzzy_Out = fuzzy->Fuzzy_Out;

	fuzzy->Fuzzy_Error = Pi_To_Pi(fuzzy->Fuzzy_Error);

	fuzzy->Fuzzy_Error *= fuzzy->Ke;
	fuzzy->Fuzzy_Error_dot *= fuzzy->Kedot;
}

/** @brief  : Update fuzzy coefficients
**  @agr    : imu and Ke, kedot, ku
**  @retval : none
**/
void	Fuzzy_UpdateCoefficients(Fuzzy_t *fuzzy, double Ke, double Kedot, double Ku) {
	fuzzy->Ke	= Ke;
	fuzzy->Kedot = Kedot;
	fuzzy->Ku 	= Ku;
}


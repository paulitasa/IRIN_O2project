#include "irifitnessfunction.h"
#include "collisionmanager.h"

/******************************************************************************/
/******************************************************************************/

CIriFitnessFunction::CIriFitnessFunction(const char* pch_name, 
                                                                 CSimulator* pc_simulator, 
                                                                 unsigned int un_collisions_allowed_per_epuck)
    :
    CFitnessFunction(pch_name, pc_simulator)
{

	/* Check number of robots */
	m_pcSimulator = pc_simulator;
	TEpuckVector* pvecEpucks=m_pcSimulator->GetEpucks();
	
	if ( pvecEpucks->size() == 0 )
	{
		printf("No Robot, so fitness function can not be computed.\n Exiting...\n");
		fflush(stdout);
		exit(0);
	}
	else if  (pvecEpucks->size()>1)
	{
		printf("More than 1 robot, and fitness is not prepared for it.\n Exiting...\n");
	}
    
	m_pcEpuck=(*pvecEpucks)[0];

	m_unNumberOfSteps = 0;
	m_fComputedFitness = 0.0;
	contador;
	contador_bateria;
	
}

/******************************************************************************/
/******************************************************************************/

CIriFitnessFunction::~CIriFitnessFunction(){
}
/******************************************************************************/
/******************************************************************************/

double CIriFitnessFunction::GetFitness()
{    

	/* If you need to check the collisions of the robot, here are the total number of 
	 * collisions done by the robot in the simulations */
	int coll = (CCollisionManager::GetInstance()->GetTotalNumberOfCollisions());

	/* Get the fitness divided by the number of steps */
//COMENTAR PARA CONTADORES
	//double fit = ( m_fComputedFitness / (double) m_unNumberOfSteps ) * (1 - ((double) (fmin(coll,10.0)/10.0)));
//COMENTAR PARA CONTADORES

//DESCOMENTAR PARA CONTADORES
	double fit = 0.1+ ( m_fComputedFitness / (double) m_unNumberOfSteps ) * ( (double) (fmin(contador, 8.0)/ 8.0 )) * ( (double) (fmin(contador_bateria, 8.0)/ 8.0 )) * (1 - ((double) (fmin(coll,10.0)/10.0)));
//DESCOMENTAR PARA CONTADORES

	/* If fitness less than 0, put it to 0 */
	if ( fit < 0.0 ) fit = 0.0;

	return fit;
}

/******************************************************************************/
/******************************************************************************/
void CIriFitnessFunction::SimulationStep(unsigned int n_simulation_step, double f_time, double f_step_interval)
{
	/* See Evolutionary Robotics Book */
	/* This is the function to be implemented */
	/* f = V * ( 1 - sqrt(Delta(v)) ) * (1 - i)
	 * V relates to the maximum speed
	 * Delta(v) relates to the movement on the same direction
	 * i relates to the maximum sensor value
	 */

	/* Get actual SPEED of the left and right wheel */
	double leftSpeed = 0.0;
	double rightSpeed = 0.0;
	m_pcEpuck->GetWheelSpeed(&leftSpeed,&rightSpeed);
	leftSpeed = 0.5 + ( leftSpeed / ( 2.0 *  m_pcEpuck->GetMaxWheelSpeed()) );
	rightSpeed = 0.5 + ( rightSpeed / ( 2.0 *  m_pcEpuck->GetMaxWheelSpeed()) );

	/* Eval maximum speed partial fitness */
	double maxSpeedEval = (fabs(leftSpeed - 0.5) + fabs(rightSpeed - 0.5));

	/* Eval same direction partial fitness */
	double sameDirectionEval = 1 - sqrt(fabs(leftSpeed - rightSpeed));
	
	/* Eval SENSORS */

	/* Where the Max PROXIMITY sensor will be stored*/
	double maxProxSensorEval 		= 0.0;
	/* Where the Max LIGHT sensor will be stored*/
	double maxLightSensorEval 	= 0.0;
	/* Where the Max BLUE LIGHT sensor will be stored*/
	double maxBlueLightSensorEval 	= 0.0;
	/* Where the Max RED LIGHT sensor will be stored*/
	double maxRedLightSensorEval 	= 0.0;
	/* Where the Max CONTACT sensor will be stored*/
	double maxContactSensorEval = 0.0;

	/* Where the GROUND MEMORY will be stored */
	double* groundMemory;
	/* Where the GROUND will be stored */
	double* ground;
	/* whre the BATTERY will be sotored */
	double *battery;
	/* whre the BLUE BATTERY will be sotored */
	double *blueBattery;
	/* whre the RED BATTERY will be sotored */
	double *redBattery;

	double blueLightS0=0;
	double blueLightS7=0;
	double lightS0=0;
	double lightS7=0;
	double redLightS0=0;
	double redLightS7=0;

	/* Auxiluar variables */
	unsigned int unThisSensorsNumberOfInputs; 
	double* pfThisSensorInputs; 
	
	/* Go in all the sensors */
	TSensorVector vecSensors = m_pcEpuck->GetSensors();
	for (TSensorIterator i = vecSensors.begin(); i != vecSensors.end(); i++)
	{
		/* Check type of sensor */
		switch ( (*i)->GetType() )
		{
			/* If sensor is PROXIMITY */
			case SENSOR_PROXIMITY:
				/* Get the number of inputs */
				unThisSensorsNumberOfInputs = (*i)->GetNumberOfInputs();
				/* Get the actual values */
				pfThisSensorInputs = (*i)->GetComputedSensorReadings();

				/* For every input */
				for (int j = 0; j < unThisSensorsNumberOfInputs; j++)
				{
					/* If reading bigger than maximum */
					if ( pfThisSensorInputs[j] > maxProxSensorEval )
					{	
						/* Store maximum value */
						maxProxSensorEval = pfThisSensorInputs[j];
					}
				}
				break;

			/* If sensor is GROUND_MEMORY */
			case SENSOR_GROUND_MEMORY:
				/* Get the actual value */
				groundMemory = (*i)->GetComputedSensorReadings();
				break;
	
			/* If sensor is GROUND */
			case SENSOR_GROUND:
				/* Get actual values */
				ground = (*i)->GetComputedSensorReadings();
				break;	
			/* If sensor is LIGHT */
			case SENSOR_REAL_LIGHT:
				/* Get number of inputs */
				unThisSensorsNumberOfInputs = (*i)->GetNumberOfInputs();
				/* Get the actual values */
				pfThisSensorInputs = (*i)->GetComputedSensorReadings();

				/* For every input */
				for (int j = 0; j < unThisSensorsNumberOfInputs; j++)
				{
					/* If reading bigger than maximum */
					if ( pfThisSensorInputs[j] > maxLightSensorEval )
					{	
						/* Store maximum value */
						maxLightSensorEval = pfThisSensorInputs[j];
					}
					if (j==0)
						lightS0 = pfThisSensorInputs[j];
					else if (j==7)
						lightS7 = pfThisSensorInputs[j];
				}
				break;
			case SENSOR_REAL_BLUE_LIGHT:
				unThisSensorsNumberOfInputs = (*i)->GetNumberOfInputs();
				pfThisSensorInputs = (*i)->GetComputedSensorReadings();

				for (int j = 0; j < unThisSensorsNumberOfInputs; j++)
				{
					if ( pfThisSensorInputs[j] > maxBlueLightSensorEval )
					{	
						maxBlueLightSensorEval = pfThisSensorInputs[j];
					}
					if (j==0)
						blueLightS0 = pfThisSensorInputs[j];
					else if (j==7)
						blueLightS7 = pfThisSensorInputs[j];
				}
				break;
			
			case SENSOR_REAL_RED_LIGHT:
				unThisSensorsNumberOfInputs = (*i)->GetNumberOfInputs();
				pfThisSensorInputs = (*i)->GetComputedSensorReadings();

				for (int j = 0; j < unThisSensorsNumberOfInputs; j++)
				{
					if ( pfThisSensorInputs[j] > maxRedLightSensorEval )
					{	
						maxRedLightSensorEval = pfThisSensorInputs[j];
					}
					if (j==0)
						redLightS0 = pfThisSensorInputs[j];
					else if (j==7)
						redLightS7 = pfThisSensorInputs[j];
				}
				break;

			/* If sensor is BATTERY */
			case SENSOR_BATTERY:
         			 battery = (*i)->GetComputedSensorReadings();
				 break;
			
			case SENSOR_BLUE_BATTERY:
				blueBattery = (*i)->GetComputedSensorReadings();

				break;
			
			case SENSOR_RED_BATTERY:
				redBattery = (*i)->GetComputedSensorReadings();
				break;
			
			/* If sensor is CONTACT */
			case SENSOR_CONTACT:
				/* Get number of inputs */
				unThisSensorsNumberOfInputs = (*i)->GetNumberOfInputs();
				/* Get actual values */
				pfThisSensorInputs = (*i)->GetComputedSensorReadings();

				/* For every input */
				for (int j = 0; j < unThisSensorsNumberOfInputs; j++)
				{
					/* If reading bigger than maximum */
					if ( pfThisSensorInputs[j] > maxContactSensorEval )
					{
						/* Store maximum value */
						maxContactSensorEval = pfThisSensorInputs[j];
					}
				}
				break;
		}
	}
	
	/* FROM HERE YOU NEED TO CREATE YOU FITNESS */	

	double coef1 = 0.25;
  	double coef2 = 0.4;
	double coef3 = 0.65;
	double coef4 = 0.7;
	int flag=0;
	int flag2=0;
	
  	double fitness= coef1 * maxSpeedEval * sameDirectionEval;

//COMENTAR PARA PENALIZACIONES Y PARA CONTADORES   	
	//if ( blueBattery[0] < 0.7){
    	//	fitness = fitness + (coef3 * ( blueLightS0 + blueLightS7) );
	//	flag2=1;
	//}
 	// else {
    	//	fitness = fitness + (coef4 * ( lightS0 + lightS7));
	//}

	
	//if(maxLightSensorEval > 0.9){
	//flag =1;	
	//}

//COMENTAR PARA PENALIZACIONES Y PARA CONTADORES   	
	

//DESCOMENTAR PARA PENALIZACIONES Y PARA CONTADORES
	if ( blueBattery[0] < 0.45){
    		fitness = fitness + (coef3 * ( blueLightS0 + blueLightS7)) - (coef2 *(maxLightSensorEval)) - (coef1 * (0.45 -blueBattery[0]));
		flag2=1;
	}
 	 if ( blueBattery[0] > 0.75){
    		fitness = fitness + (coef4 * ( lightS0 + lightS7)) - (coef2 *(maxBlueLightSensorEval));
	}

	
	if(maxLightSensorEval > 0.9){
	flag =1;	
	}
//DESCOMENTAR PARA PENALIZACIONES Y PARA CONTADORES
	
//DESCOMENTAR PARA CONTADORES
	if((flag ==1) && (maxBlueLightSensorEval>0.9)){
	flag=0;
	contador++;
	}
	
	if((flag2==1) && (blueBattery[0] > 0.9)){
	flag2=0;
	contador_bateria++;
	}
//DESCOMENTAR PARA CONTADORES

	

	/* TO HERE YOU NEED TO CREATE YOU FITNESS */	

	m_unNumberOfSteps++;
	m_fComputedFitness += fitness;
}

/******************************************************************************/
/******************************************************************************/

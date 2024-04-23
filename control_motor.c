/*                                                                  
 * POSIX Real Time Example
 * using a single pthread as RT thread
 */
 
#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>
#include <pigpio.h>

#define NSEC_PER_SEC 1000000000		// número de ns en un seg
#define TS 100000000			// periodo de muestreo (100 ms, en ns)

#define V_DC 6.0			// tensión de alimentación del puente en H

#define SP 60.0				// consigna en rpm

#define TO_RPM 60.0/159.0/0.1		// factor para calcular rpm a partir de cuentas del decoder

/* Definición de los parámetros del PI discreto */
#define B0 0.005455
#define B1 0.003455
#define A1 -1.0

/* Definición de los GPIO utilizados */
#define GPIO_PWM 18			// GPIO para sacar PWM, va al pin EN del puente en H
#define GPIO_DIR 17			// GPIO para sacar dirección de giro, va al pin DIR del puente en H
#define GPIO_SA 22			// entrada GPIO generada por el encoder SA
#define GPIO_SB 23			// entrada GPIO generada por el encoder SB

/* Declaración de variables globales */
int encoder_cont;			// número de vueltas del encoder
int ret;

/* using clock_nanosleep of librt */
extern int clock_nanosleep(clockid_t __clock_id, int __flags,
      __const struct timespec *__req,
      struct timespec *__rem);

/* the struct timespec consists of nanoseconds and seconds. if the nanoseconds are getting
 * bigger than 1000000000 (= 1 second) the variable containing seconds has to be
 * incremented and the nanoseconds decremented by 1000000000.
 */
static inline void tsnorm(struct timespec *ts)
{
   while (ts->tv_nsec >= NSEC_PER_SEC) {
      ts->tv_nsec -= NSEC_PER_SEC;
      ts->tv_sec++;
   }
}

/* Función que se ejecuta cuando se produce un evento en el GPIO_SA */
void edgeDetected(int gpio, int level, uint32_t tick)
{
	int a;
	a = gpioRead(GPIO_SB);
	if (level == 1) {
		if (a == 0) {
			encoder_cont++;
		}
		else {
			encoder_cont--;
		}
		printf("%d\n", a);
	}
}

/* Hilo1: cálculo de la señal de control */
void *thread1_control(void *data)
{		
		struct timespec t;
		
		/* declaración de variable con copia de la cuenta del decoder */
		int encoder_cont_k;
		
		/* declaración de variable con la velocidad del motor en rpm */
		float rpm;

		/* declaración de las variables del controlador discreto */
		float ek, ek1;		/* error en k y en k-1 */
		float uk, uk1;		/* señal de control en k y en k-1 */
		float uk_sat_dz;	/* señal de control saturada y con zona muerta */
		
		/* inicialización de ek1 y uk1 con condiciones iniciales nulas */
		ek1 = 0.0;
		uk1 = 0.0;
		
		/* inicialización de las cuentas del decoder a 0 */
		encoder_cont = 0;
		
		/* declaración de las variables con el ciclo de trabajo de la PWM */
		float dc_float;		/* ciclo de trabajo en flotante, entre 0 y 1 */
		int dc;			/* ciclo de trabajo en entero, tanto por millón */
		int dir = 1;		/* 0 si uk positivo, 1 si uk negativo */
		
		/* get current time */
		clock_gettime(0,&t);
		
		/* start after one second for synchronization */
		t.tv_sec++;
		
		while(1){
			/* wait until next shot */
			clock_nanosleep(0, TIMER_ABSTIME, &t, NULL);
			
			/* calcular velocidad a partir del contador del decoder */
			//printf("%d \n", encoder_cont);
			encoder_cont_k = encoder_cont;		// copiar cuenta del decoder
			encoder_cont = 0;			// poner a cero cuenta del decoder
			rpm = encoder_cont_k*TO_RPM;		// cálculo de RPMs
			
			//printf("%d\n", encoder_cont_k);
			//printf("%f\n", rpm);
		
			
			/* calcular señal de error */
			ek = SP - rpm;
			
			/* calcular señal de control (PID discreto) */
			uk = B0*ek+B1*ek1-A1*uk1;
			
			/* aplicar zona muerta al pasar de uk positivo a negativo */
			/* saturar si abs(uk) > 6.0 */
			if (uk > 0.0 && uk1 < 0.0) {
				uk_sat_dz = 0.0;
			}
			else if (uk < 0.0 && uk1 > 0.0) {
				uk_sat_dz = 0.0;
			}
			else if (uk >= 6.0) {
				uk_sat_dz = 6.0;
				dir = 1;
			}
			else if (uk <= -6.0) {
				uk_sat_dz = 6.0;
				dir = 0;
			}
			else if (uk >= 0.0) {		// uk positivo
				uk_sat_dz = uk;
				dir = 1;
			}
			else {				// uk negativo
				uk_sat_dz = -uk;
				dir = 0;
			}
			
			
			/* saturar uk y aplicar zona muerta en torno a 0.0 */
			//if (uk >= 6.0) {
				//uk_sat_dz = 6.0;
				//dir = 1;
			//}
			//else if (uk >= 0.0) {
				//if (uk1 >= 0.0) {		// si uk generada en el periodo anterior es positiva
					//uk_sat_dz = uk;		// zona lineal, dir se deja a 1
					//dir = 1;
				//}
				//else {				// si uk aplicada en el periodo anterior es negativa
					//uk_sat_dz = 0.0;	// un periodo de tiempo muerto para evitar cortos
				//}
			//}
			//else if (uk > -6.0) {
				//if (uk1 <= 0.0) {		// si uk generada en el periodo anterior es negativa
					//uk_sat_dz = -uk;	// zona lineal, dir se pone a 0
					//dir = 0;
				//}
				//else {
					//uk_sat_dz = 0.0;	// un periodo de tiempo muerto para evitar cortos
				//}
			//}
			//else {					// uk <= -6.0
				//uk_sat_dz = 6.0;
				//dir = 0;
			//}
			
			/* actualizar señal de dirección de PWM */
			gpioWrite(GPIO_DIR, dir);
			
			/* actualizar ciclo de trabajo de la PWM */
			dc_float = uk_sat_dz / V_DC;		// ciclo de trabajo en tanto por 1
			dc = (int)(dc_float*1000000.0);		// ciclo de trabajo en tanto por millón
			gpioHardwarePWM(GPIO_PWM, 1000, dc);	// PWM de 10 kHz en el GPIO18 con ciclo de trabajo dc
			
			// printf("%d \n", dc);
			
			
			/* registrar señal de control y error */
			uk1 = uk;
			ek1 = ek;
			
			/* actualizar ciclo de trabajo de PWM según señal de control */
			//dc_float = uk/max_v;
			//if (dc_float <= 0.0) {
				//dc_float = -dc_float;
				//signo = 1;
			//}
			//else {
				//signo = 0;
			//}
			
			/* pasar a tanto por millón, saturar a ciclo de trabajo 100 % */
			//if (dc_float >= 1.0) {
				//dc = 1000000;
			//else { 
				//dc = (int)(dc_float*1000000.0);
			//}

			/* calculate next shot */
			t.tv_nsec+=TS;
			tsnorm(&t);
		}
        return NULL;
}

/* Hilo 2: adquisición de señal del enconder */
void *thread2_encoder(void *data)
{
	encoder_cont = 0;	// poner a 0 contador de vueltas
	/* Set up a callback for GPIO events in GPIO_SA */
	ret = gpioSetAlertFunc(GPIO_SA, edgeDetected);
	// GESTIONAR ERROR DEL HILO
	if (ret) {
		printf("Failed to set callback function.\n");
		gpioTerminate();
	}
	
}
 
int main(int argc, char* argv[])
{
        struct sched_param param;			// crear una estructura de tipo sched_param para configurar las propiedades relacionadas con el scheduler
        pthread_attr_t attr;				// crear un objeto de tipo pthread_attr_t para configurar los atributos del hilo
        pthread_t thread1, thread2;
	
	/* Inicializar GPIO */
	ret = gpioInitialise();
	if (ret < 0) {
		printf("Failed to initialize GPIO.\n");
		gpioTerminate();
		goto out;
	}
	
	/* Configurar GPIO_SA como entrada */
	ret = gpioSetMode(GPIO_SA, PI_INPUT);
	if (ret) {
		printf("Failed to set GPIO pin mode.\n");
		gpioTerminate();
		goto out;
	}
	
	/* Configurar GPIO_SB como entrada */
	ret = gpioSetMode(GPIO_SB, PI_INPUT);
	if (ret) {
		printf("Failed to set GPIO pin mode.\n");
		gpioTerminate();
		goto out;
	}
	
	/* Configurar GPIO_DIR como salida */
	ret = gpioSetMode(GPIO_DIR, PI_OUTPUT);
	if (ret) {
		printf("Failed to set GPIO pin mode.\n");
		gpioTerminate();
		goto out;
	}
 
        /* Lock memory */
        /*if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
                printf("mlockall failed: %m\n");
                exit(-2);
        }*/
 
        /* Initialize pthread attributes (default values) */
        ret = pthread_attr_init(&attr);		// initializes the thread attributes object pointed to by attr with default attribute values
        if (ret) {
                printf("init pthread attributes failed\n");
                goto out;
        }
 
        /* Set a specific stack size  */
        ret = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
        if (ret) {
            printf("pthread setstacksize failed\n");
            goto out;
        }
 
        /* Set scheduler policy and priority of pthread */
        ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
        if (ret) {
                printf("pthread setschedpolicy failed\n");
                goto out;
        }
        param.sched_priority = 90;
        ret = pthread_attr_setschedparam(&attr, &param);
        if (ret) {
                printf("pthread setschedparam failed\n");
                goto out;
        }
        ret = pthread_attr_getschedparam(&attr, &param);
		printf("Param %d", param.sched_priority);
        if (ret) {
                printf("pthread getschedparam failed\n");
                goto out;
        }
        /* Use scheduling parameters of attr */
        ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
        if (ret) {
                printf("pthread setinheritsched failed\n");
                goto out;
        }
 
        /* Create a pthread with specified attributes */
        ret = pthread_create(&thread1, &attr, thread1_control, NULL);
        if (ret) {
                printf("create pthread failed\n");
                goto out;
        }
	
	/* Create a pthread with specified attributes */
        ret = pthread_create(&thread2, &attr, thread2_encoder, NULL);
        if (ret) {
                printf("create pthread failed\n");
                goto out;
        }
 
        /* Join the thread and wait until it is done */
        ret = pthread_join(thread1, NULL);
        if (ret)
                printf("join pthread failed: %m\n");
		
	/* Join the thread and wait until it is done */
        ret = pthread_join(thread2, NULL);
        if (ret)
                printf("join pthread failed: %m\n");
 
out:
        return ret;
}

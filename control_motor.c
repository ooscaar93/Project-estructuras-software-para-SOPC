/*                                                                  
 * DC motor control
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
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/ioctl.h>

/* Definición de constantes */
#define NSEC_PER_SEC 1000000000		// número de ns en un seg
#define TS 100000000			// periodo de muestreo (100 ms, en ns)

#define V_DC 6.0			// tensión de alimentación del puente en H

#define SP 80.0				// consigna en rpm

#define TO_RPM 60.0/(159.0*0.1*4.0)	// factor para calcular rpm a partir de cuentas del decoder

/* Definición de los parámetros del PI discreto: uk = B0*ek+B1*ek1-A1*uk1 */
#define B0 0.005455
#define B1 0.003455
#define A1 -1.0

/* Definición de los GPIO utilizados */
#define GPIO_PWM 18			// GPIO para sacar PWM, va al pin EN del puente en H
#define GPIO_DIR 17			// GPIO para sacar dirección de giro, va al pin DIR del puente en H
#define GPIO_SA 22			// entrada GPIO generada por el encoder SA
#define GPIO_SB 23			// entrada GPIO generada por el encoder SB

/* Declaración de variables globales */
int encoder_cont;			// número de vueltas del encoder ¿NECESARIO MUTEX?

int server_socket;			// server socket descriptor
int client_socket;			// client socket descriptor
struct sockaddr_un server_addr;		// server address
struct sockaddr_un client_addr;		// client address

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

/* 
 * Función que se ejecuta cuando se produce un evento en el GPIO_SA o GPIO_SB.
 * Máquina de estados del decoder en cuadratura.
 */
void edgeDetected(int gpio, int level, uint32_t tick)
{	
	// declaración de variables estáticas
	static int a = 0;		// valor de la señal GPIO_SA (disco A del encoder)
	static int b = 0;		// valor de la señal GPIO_SB (disco B del encoder, en cuadratura con A)
	static int estado = 1;		// estado actual de la máquina de estados del decoder
	
	// actualización del valor de las variables a y b
	// (asociadas a las formas de onda en SA y SB)
	if (gpio == GPIO_SA) {		// flanco en SA
		if (level == 1) {	// flanco ascendente en SA
			a = 1;
		}
		else if (level == 0) {	// flanco descendente en SA
			a = 0;
		}
	}
	else if (gpio == GPIO_SB) {	// flanco en SB				
		if (level == 1) {	// flanco ascendente en SB
			b = 1;
		}
		else if (level == 0) {	// flanco descendente en SB
			b = 0;
		}
	}
	
	// máquina de estados para incremento/decremento de encoder_cont
	// estado (a,b): 1 (0,0), 2 (1,0), 3 (1,1), 4 (0,1)
	switch (estado) {
		case 1:					// a == 0, b == 0
			if (a == 1 && b == 0) {		// si flanco ascendente en a
				estado = 2;		// --> a == 1, b == 0 (estado 2)
				encoder_cont--;
			}
			else if (a == 0 && b == 1) {	// si flanco ascendente en b
				estado = 4;		// --> a == 0, b == 1 (estado 4)
				encoder_cont++;
			}
			break;
		case 2:					// a == 1, b == 0
			if (a == 1 && b == 1) {		// si flanco ascendente en b
				estado = 3;		// --> a == 1, b == 1 (estado 3)
				encoder_cont--;
			}
			else if (a == 0 && b == 0) {	// si flanco descendente en a
				estado = 1;		// --> a == 0, b == 0 (estado 1)
				encoder_cont++;
			}
			break;
		case 3:					// a == 1, b == 1
			if (a == 0 && b == 1) {		// si flanco descendente en a
				estado = 4;		// --> a == 0, b == 1 (estado 4)
				encoder_cont--;
			}
			else if (a == 1 && b == 0) {	// si flanco descendente en b
				estado = 2;		// --> a == 1, b == 0 (estado 2)
				encoder_cont++;
			}
			break;
		case 4:					// a == 0, b == 1
			if (a == 0 && b == 0) {		// si flanco descendente en b
				estado = 1;		// --> a == 0, b == 0 (estado 1)
				encoder_cont--;
			}
			else if (a == 1 && b == 1) {	// si flanco ascendente en a
				estado = 3;		// --> a == 1, b == 1 (estado 3)
				encoder_cont++;
			}
			break;

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
	float ek, ek1;			// error en k y en k-1
	float uk, uk1;			// señal de control en k y en k-1
	float uk_sat_dz;		// señal de control saturada y con zona muerta
	
	/* declaración de variables con el número de bytes de algunas variables */
	int tsec_bytes = sizeof t.tv_sec;	// tamaño en bytes de t.tv_sec
	int tnsec_bytes = sizeof t.tv_nsec;	// tamaño en bytes de t.tv_nsec
	int rpm_bytes = sizeof rpm;		// tamaño en bytes de rpm
	int uk_bytes = sizeof uk;		// tamaño en bytes de uk
	
	/* inicialización de ek1 y uk1 con condiciones iniciales nulas */
	ek1 = 0.0;
	uk1 = 0.0;
	
	/* inicialización de las cuentas del decoder a 0 */
	encoder_cont = 0;
	
	/* declaración de las variables con el ciclo de trabajo de la PWM */
	float dc_float;		// ciclo de trabajo en flotante, entre 0 y 1
	int dc;			// ciclo de trabajo en entero, tanto por millón
	int dir = 1;		// 0 si uk positivo, 1 si uk negativo
	
	/* get current time */
	clock_gettime(0,&t);
	
	/* start after one second for synchronization */
	t.tv_sec++;
	
	while(1){
		/* wait until next shot */
		clock_nanosleep(0, TIMER_ABSTIME, &t, NULL);
		
		/* calcular velocidad a partir del contador del decoder */
		encoder_cont_k = encoder_cont;		// copiar cuenta del decoder
		encoder_cont = 0;			// poner a cero cuenta del decoder
		rpm = encoder_cont_k*TO_RPM;		// cálculo de RPMs
		
		/* calcular señal de error */
		ek = SP - rpm;
		
		/* calcular señal de control (PID discreto) */
		uk = B0*ek+B1*ek1-A1*uk1;
		
		/* aplicar saturación */
		if (uk > 6.0) {				// si uk > 6 voltios, 6 voltios
			uk = 6.0;
		}
		else if (uk < -6.0) {			// si uk < -6 voltios, -6 voltios
			uk = -6.0;
		}
		
		/* aplicar zona muerta y configurar dirección de la tensión */
		if (uk > 0.0 && uk1 < 0.0) {		// zona muerta si uk pasa de negativo a positivo
			uk_sat_dz = 0.0;
		}
		else if (uk < 0.0 && uk1 > 0.0) {	// zona muerta si uk pasa de positivo a negativo
			uk_sat_dz = 0.0;
		}
		else if (uk >= 0.0) {		// uk positivo
			uk_sat_dz = uk;
			dir = 1;
		}
		else {				// uk negativo
			uk_sat_dz = -uk;
			dir = 0;
		}
		
		/* actualizar señal de dirección de PWM */
		gpioWrite(GPIO_DIR, dir);
		
		/* actualizar ciclo de trabajo de la PWM */
		dc_float = uk_sat_dz / V_DC;		// ciclo de trabajo en tanto por 1
		dc = (int)(dc_float*1000000.0);		// ciclo de trabajo en tanto por millón
		gpioHardwarePWM(GPIO_PWM, 2000, dc);	// PWM de 10 kHz en el GPIO_PWM con ciclo de trabajo dc
		
		/* registrar señal de control y error */
		uk1 = uk;
		ek1 = ek;
		
		// Accept an incoming connection from the client
		int clen = sizeof(client_addr);	// Get number of bytes of client_addr
		// accept() takes the server socket as an argument and retrieves the first socket connection in the queue,
		// creates a new socket for communication with client and returns the file descriptor for that socket
		client_socket = accept(server_socket, (struct sockaddr *) &client_addr, &clen);
		if (client_socket >= 0) {	// If connection with client has been established
			// Write to client
			write(client_socket, &t.tv_sec, tsec_bytes);	// use client socket file descriptor to send t.tv_sec
			write(client_socket, &t.tv_nsec, tnsec_bytes);	// use client socket file descriptor to send t.tv_nsec
			write(client_socket, &rpm, rpm_bytes);	// use client socket file descriptor to send rpm
			write(client_socket, &uk, uk_bytes);	// use client socket file descriptor to send uk
			// Close socket connection to client
			close(client_socket);			// use client socket file descriptor
		}
		// If connection from client not available, doesn't send data
		
		/* calculate next shot */
		t.tv_nsec+=TS;
		tsnorm(&t);
	}
	return NULL;
}

/* Hilo 2: adquisición de señal del enconder */
void *thread2_encoder(void *data)
{	
	int ret;
	encoder_cont = 0;	// poner a 0 contador de vueltas
	/* Set up a callback for GPIO events in GPIO_SA */
	ret = gpioSetAlertFunc(GPIO_SA, edgeDetected);
	if (ret) {
		printf("Failed to set callback function.\n");
		gpioTerminate();
		return NULL;
	}
	ret = gpioSetAlertFunc(GPIO_SB, edgeDetected);
	// GESTIONAR ERROR DEL HILO
	if (ret) {
		printf("Failed to set callback function.\n");
		gpioTerminate();
		return NULL;
	}
	
}

/* Función que realiza la iniciación y configuración de los GPIO para el control del motor */
int gpio_init()
{
	int ret;					// variable para almacenar retorno de funciones
	
	/* Inicializar GPIO */
	ret = gpioInitialise();
	if (ret < 0) {
		printf("Failed to initialize GPIO.\n");
		gpioTerminate();
		return ret;
	}
	
	/* Configurar GPIO_SA como entrada */
	ret = gpioSetMode(GPIO_SA, PI_INPUT);
	if (ret) {
		printf("Failed to set GPIO pin mode.\n");
		gpioTerminate();
		return ret;
	}
	
	/* Configurar GPIO_SB como entrada */
	ret = gpioSetMode(GPIO_SB, PI_INPUT);
	if (ret) {
		printf("Failed to set GPIO pin mode.\n");
		gpioTerminate();
		return ret;
	}
	
	/* Configurar GPIO_DIR como salida */
	ret = gpioSetMode(GPIO_DIR, PI_OUTPUT);
	if (ret) {
		printf("Failed to set GPIO pin mode.\n");
		gpioTerminate();
		return ret;
	}
	
	/* Si todo ha ido correctamente, se retorna 0 */
	return 0;
}
 
int main(int argc, char* argv[])
{	
	int ret;					// variable para almacenar retorno de funciones
    struct sched_param param;			// crear una estructura de tipo sched_param para configurar las propiedades relacionadas con el scheduler
    pthread_attr_t attr;				// crear un objeto de tipo pthread_attr_t para configurar los atributos del hilo
    pthread_t thread1, thread2;			// crear dos hilos de tipo pthread_t
	
	/* Inicializar y configurar GPIOs */
	ret = gpio_init();
	if (ret) {
		printf("Error when initialising and configuring GPIOs\n");
		return ret;
	}
	
	/* Create socket on server side */
	
	// Delete socket file before creating socket
	remove("./control_socket");
	
	// Create socket descriptor for server:
	// - Socket family AF_UNIX (UNIX Address Family)
	// - Socket type SOCK_STREAM (end-to-end connection, data is recieved in send order)
	server_socket = socket(AF_UNIX, SOCK_STREAM, 0);
	if (server_socket < 0)
	{
		perror("socket() failed");
		return server_socket;
	}
	
	// Set socket to be non-blocking
	int on = 1;
	ret = ioctl(server_socket, FIONBIO, (char *)&on);
	if (ret < 0)
	{
		perror("ioctl() failed");
	    close(server_socket);
	    return ret;
	}
	
	// Bind socket to a local address on the machine
	server_addr.sun_family = AF_UNIX;				// Set socket family of server sockaddr structure
	strcpy(server_addr.sun_path, "./control_socket");	// Set path of server sockaddr structure
	int slen = sizeof(server_addr);					// Get number of bytes of server_addr
	ret = bind(server_socket, (struct sockaddr *) &server_addr, slen);	// Bind server_socket to server_addr
	if (ret < 0)
	{
		perror("bind() failed");
		close(server_socket);
		return ret;
	}
	
	// Listen for incoming connections to server_socket, with 1 max connection queue
	ret = listen(server_socket, 1);
	if (ret < 0)
	{
		perror("listen() failed");
		close(server_socket);
		return ret;
	}
 
	/* Initialize pthread attributes (default values) */
	ret = pthread_attr_init(&attr);		// initializes the thread attributes object pointed to by attr with default attribute values
	if (ret) {
		printf("init pthread attributes failed\n");
		return ret;
	}

	/* Set a specific stack size  */
	ret = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
	if (ret) {
		printf("pthread setstacksize failed\n");
		return ret;
	}
 
	/* Set scheduler policy and priority of pthread */
	ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
	if (ret) {
		printf("pthread setschedpolicy failed\n");
		return ret;
	}
	param.sched_priority = 90;
	ret = pthread_attr_setschedparam(&attr, &param);
	if (ret) {
		printf("pthread setschedparam failed\n");
		return ret;
	}
	ret = pthread_attr_getschedparam(&attr, &param);
	printf("Param %d", param.sched_priority);
	if (ret) {
		printf("pthread getschedparam failed\n");
		return ret;
	}
    
	/* Use scheduling parameters of attr */
	ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	if (ret) {
		printf("pthread setinheritsched failed\n");
		return ret;
	}
 
	/* Create a pthread with specified attributes */
	ret = pthread_create(&thread1, &attr, thread1_control, NULL);
	if (ret) {
		printf("create pthread failed\n");
		return ret;
	}
	
	/* Create a pthread with specified attributes */
	ret = pthread_create(&thread2, &attr, thread2_encoder, NULL);
	if (ret) {
		printf("create pthread failed\n");
		return ret;
    }
 
	/* Join the thread and wait until it is done */
	ret = pthread_join(thread1, NULL);
	if (ret) {
		printf("join pthread failed: %m\n");
		return ret;
	}
		
	/* Join the thread and wait until it is done */
	ret = pthread_join(thread2, NULL);
	if (ret) {
		printf("join pthread failed: %m\n");
	}
 
        return 0;
}

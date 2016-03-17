#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "make_msg.h"
#include "RCommands.h"
#include "actions.h"

char f_local_addr[32]="192.168.1.1";
char f_remote_addr[32]="192.168.1.2";
int f_remote_port=2000;
int f_quit = 0;
int rgb[3] = { 127, 127, 127};
int f_verbose = 0;

int args(int argc, char **argv);
extern char * optarg;

int main(int argc, char **argv)
{
	if (args(argc, argv)) return -1;
	
	/* Establish connection to render */

	if (f_verbose) 
	  printf("init tcpip connection....\n");
	init_tcpip(f_local_addr, f_remote_addr, f_remote_port, 1);

	if (!f_quit)
	{	
		// reset
		if (f_verbose)
		  printf("render_reset()...\n");
		render_reset();
		
		// background
		if (f_verbose)
		  printf("render_bgcolor(%d, %d, %d)\n", rgb[0], rgb[1], rgb[2]);
		render_bgcolor(rgb[0], rgb[1], rgb[2]);
		
		// frame
		if (f_verbose)
		  printf("render_frame(1)\n");
		render_frame(1);
	}
	else
	{
		if (f_verbose)
		  printf("render_quit()\n");
		render_quit();
	}
	return 0;
}

int args(int argc, char **argv)
{
	int c;
	int errflag = 0;
	int i;
    while((c = getopt( argc, argv, "qb:cp:r:v")) != -1) 
    {
    	switch(c)
    	{
    	case 'q':
   			f_quit = 1;
   			break;
		case 'b':
			i = atoi(optarg);
			if (i >= 0 && i<=255) 
			  rgb[0] = rgb[1] = rgb[2] = i;
			break;
		case 'c':
			break;
		case 'p':
			i = atoi(optarg);
			if (i>1024 && i<65536)
			  f_remote_port = i;
			break;
		case 'r':
			if (strlen(optarg) < 32)
			  strcpy(f_local_addr, optarg);
			else
			  printf("local addr too long\n");
			break;
		case 'v':
			f_verbose = 1;
			break;
   		default:
  			printf("Unknown option: %c\n", c);
   			errflag++;	
   			break;
    	}
    }    		
	return errflag;
}

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define INVALID     -1
#define INIT_DRIVER 	0
#define BLINK_PERIOD	20
#define ARRAY_SIZE 	5

int main(int argc, char **argv){
	
	char *app_name = argv[0];
	char *dev_name = "/dev/sample";
	int fd = -1;
	int x, c;
	int blk_hperiod;
	int W = 0, D = 0, period = 500, array_index = 0;
	int array_val[ARRAY_SIZE];
	
	if ((fd = open(dev_name, O_RDWR)) < 0) {
		fprintf(stderr, "%s: unable to open %s: %s\n", 
			app_name, dev_name, strerror(errno));
		return( 1 );
	}
	
	printf( "App: Welcome\n" );
	ioctl(fd, INIT_DRIVER, NULL);
	write(fd, &period, sizeof(period));

	while(1){
				
		do{
			x = read(fd, &c, sizeof(c));
		} while (c == 0);

		W = c;
		
		D = (int) ((((float)(1.0*W)) / 58) + 0.5);
		
		if (D >= 2 && D <= 400){
		
				if (array_index < ARRAY_SIZE){
					array_val[array_index++] = D;
				}
		
				else{
			
					array_index = 0;
					array_val[array_index++] = D;
					
					D = find_mostfrq(array_val);
			
					printf ("Distance = %d cm\n\n", D);

					period = compute_blink(D);

					ioctl(fd, BLINK_PERIOD, NULL);
					write(fd, &period, sizeof(period));

				}
		}
					
	}
}

int compute_blink (int D){

	int blk_hperiod;

	switch (D/5){
		
		case 0: case 1:									/* from 0 to 10) */
		blk_hperiod = 0;
		break;
		
		case 2: case 3: case 4:							/* from 10 to 25) */
		blk_hperiod = 200;
		break;
		
		case 5: case 6: case 7: case 8: case 9:	/* from 25 to 50) */
		blk_hperiod = 300;
		break;
		
		case 10: case 11: case 12: case 13: case 14:	/* from 50 to 75) */
		blk_hperiod = 400;
		break;
		
		case 15: case 16: case 17: case 18: case 19:	/* from 75 to 100) */
		blk_hperiod = 500;
		break;
		
		default:
		blk_hperiod = 1000;
		break;
	}
	
	return blk_hperiod;
}

int find_mostfrq(int array_val[]){
		
	int count[ARRAY_SIZE] = { 0, 0, 0, 0, 0 };
	int i = 0, j = 0, value = 0, max = 0, index = 0;
	
	for(i = 0; i < ARRAY_SIZE; i++){
	
		value = array_val[i]; 
		
		for (j = 0; j < ARRAY_SIZE; j++){
		
			if (array_val[j] == value){
				
				count[i]++;
				if (count[i] > max){
				
					max = count[i];
					index = i;
				}
			}
		}
	}
	return array_val[index];
}

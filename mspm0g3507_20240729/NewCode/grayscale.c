#include "grayscale.h"



int8_t track_assignment[12];
int8_t track_sum;

void Track_follow(void)
{
	track_assignment[0] = H12 ? 0 : 6;
	track_assignment[1] = H11 ? 0 : 5;
	track_assignment[2] = H10 ? 0 : 4;
	track_assignment[3] = H9 ? 0 : 3;
	track_assignment[4] = H8 ? 0 : 2;
	track_assignment[5] = H7 ? 0 : 1;
	track_assignment[6] = H6 ? 0 : -1;
	track_assignment[7] = H5 ? 0 : -2;
	track_assignment[8] = H4 ? 0 : -3;
	track_assignment[9] = H3 ? 0 : -4;
	track_assignment[10] = H2 ? 0 : -5;
	track_assignment[11] = H1 ? 0 : -6;
	
	track_sum=track_assignment[0]+track_assignment[1]+track_assignment[2]+track_assignment[3]+track_assignment[4]
	          +track_assignment[5]+track_assignment[6]+track_assignment[7]+track_assignment[8]+track_assignment[9]+track_assignment[10]+track_assignment[11];				
	
}


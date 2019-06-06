//#include <iostream.h>

#define GRAY_ARR 40
#define GRAY_MAX 2
#define GRAY_MIN 2

//int X[GRAY_ARR] = {0};

void Store_Value(int i, int*x);
int Check_Gray(int* x);

int gray(int i, int *x){
		int cnt;
		Store_Value(i, x);
		cnt = Check_Gray(x);
		if(cnt == GRAY_ARR){
			return 1;
		}else{
			return 0;
		}
}

void Store_Value(int i, int* x)
{
	for(int j = GRAY_ARR-1; j > 0; j--){
			x[j] = x[j-1];
	}
	x[0] = i;
}

int Check_Gray(int* x)
{
	int cnt = 0;
	for(int j = GRAY_ARR-1; j >= 0; j--){
		if(x[j] >= GRAY_MIN && x[j] <= GRAY_MAX){
			cnt++;
		}
	}
	return cnt;
}

#include <stdio.h>
#include <stdlib.h>

int main()
{
    int n ,i , sum =0 ;
    printf ( " Enter an integer :" );
    scanf ("%d",&n);
    for (i = 1; i <= n ; i++)
    {
        sum += i;
    }
    printf ( "sum = %d ", sum );
    return 0;
}

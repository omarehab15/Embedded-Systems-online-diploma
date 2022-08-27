#include <stdio.h>
#include <stdlib.h>

int main()
{
    float x;
    printf ( "Enter a number : ");
    scanf ("%f",&x);
    if ( x > 0 )
        printf ( "%f is positive.\n",x);
    else if ( x == 0 )
        printf ( "You entered zero\n");
    else
        printf ( "%f is negative.\n",x);
    return 0;
}

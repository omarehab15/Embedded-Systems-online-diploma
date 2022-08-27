#include <stdio.h>
#include <stdlib.h>

int main()
{
    float a ,b ,c ;
    printf ("Enter three numbers : ");
    scanf ("%f %f %f", &a , &b , &c);
    if ( a > b)
    {
        if ( a > c )
        {
            printf ( "Largest number = %f",a);
        }
        else
        {
           printf ( "Largest number = %f",c);
        }
    }
    else
    {
        if ( b > c)
        {
            printf ( "Largest number = %f",b);
        }
        else
        {
            printf ( "Largest number = %f",c);
        }
    }
    return 0;
}

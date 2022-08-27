#include <stdio.h>
#include <stdlib.h>

int main()
{
    float x , y ;
    char a ;
    printf ( "Enter operator either + or - or * or /:");
    scanf ("%c",&a);
    printf ("Enter two operands : ");
    scanf ("%f %f",&x,&y);
    switch (a)
    {
        case '+' :
             printf ("%f + %f = %f\n",x,y,x+y);
             break ;
        case '-' :
             printf ("%f - %f = %f\n",x,y,x-y);
             break;
        case '*' :
            printf ("%f * %f = %f\n",x,y,x*y);
            break;
        case '/' :
             printf ("%f / %f = %f\n",x,y,x/y);
             break;
    }
    return 0;
}

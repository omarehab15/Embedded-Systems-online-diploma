#include <stdio.h>
#include <stdlib.h>

int main()
{
    int x,i,f=1;
    printf ( "Enter an integer : ");
    scanf ("%d",&x);
    if (x > 0)
    {
        for (i = 0; i < x ;i++)
        {
            f*=(i+1);
        }
        printf ("Factorial = %d\n",f);
    }
    else if (x == 0)
    {
        printf ("Factorial = 1\n");
    }
    else
    {
        printf ("Error!!!Factorial of negative number does not exist.\n");
    }

    return 0;
}

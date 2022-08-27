#include <stdio.h>
#include <stdlib.h>

int main()
{
    char x;
    printf ( " Enter alphabet : ");
    scanf ("%c",&x);
    if( x == 'a' || x == 'o' || x == 'e' || x == 'i' || x == 'u')
    {
       printf("%c is vowel.\n", x);
    }
    else
    {
        printf("%c is constant.\n", x);
    }
    return 0;
}

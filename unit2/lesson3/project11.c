#include <stdio.h>
#include <stdlib.h>

int main()
{
    char x ;
    printf("Enter a character : ");
    scanf ("%c",&x);
    if ((64 < x && x <=90) || (96 < x && x<=122))
    {
        printf("%c is an alphabet.",x);
    }
    else
    {
        printf("%c is not an alphabet.",x);
    }
    return 0;
}

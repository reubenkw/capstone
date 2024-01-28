#include "log.h"

#include <stdio.h>

void c_log(const char *  message) {
    FILE *fp;
    fp = fopen( "log.txt", "a");
    fputs(message, fp);
    fclose(fp);
}
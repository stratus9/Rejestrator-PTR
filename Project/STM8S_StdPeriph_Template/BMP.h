#ifndef __BMP_H
#define __BMP_H

void BMP_init(bmp_t * BMP);
void BMP_read(bmp_t * BMP);
float BMP_altitude(float startPress, float currPress);


#endif
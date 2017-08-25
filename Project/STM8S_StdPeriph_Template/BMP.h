#ifndef __BMP_H
#define __BMP_H

void BMP_init(bmp_t * BMP);
void BMP_read(bmp_t * BMP);
int32_t BMP_altitude(int32_t startPress, int32_t currPress);


#endif
/*
 * si512.h
 *
 *  Created on: 2022年6月14日
 *      Author: LIUBING
 */

#ifndef DRIVERS_DRIVERS_SI5XX_SI512_H_
#define DRIVERS_DRIVERS_SI5XX_SI512_H_

#include "nrf_log.h"
#include "si522aDef.h"
#include "si522a.h"
#include "stdio.h"

int8_t si512_p2p_send_data(si522_dev *dev, uint8_t * const w_data, uint8_t len);
int8_t si512_p2p_rseceive_data(si522_dev *dev, uint8_t *r_data, uint8_t len);
#endif /* DRIVERS_DRIVERS_SI5XX_SI512_H_ */

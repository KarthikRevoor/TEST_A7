/*
 * utilities.h
 *
 *  Created on: Jul 21, 2023
 *      Author: lpandit
 */

#ifndef UTILITIES_H_
#define UTILITIES_H_
#include <stdint.h>

#define MODIFY_FIELD(reg, field, value) \
   ((reg) = ((reg & ~(field ## _Msk)) | \
	           (((uint32_t) (value)  << field ## _Pos) & field ## _Msk)))


#define ESF_GPIO_MODER_INPUT     (0)
#define ESF_GPIO_MODER_OUTPUT    (1)
#define ESF_GPIO_MODER_ALT_FUNC  (2)
#define ESF_GPIO_MODER_ANALOG    (3)

#define UNUSED(X)  (void)X


#endif /* UTILITIES_H_ */

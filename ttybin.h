/*
 * ttybin.h
 *
 *  Created on: Oct 21, 2016
 *      Author: epenate
 */

#ifndef TTYBIN_H_
#define TTYBIN_H_

#ifdef __cplusplus
extern "C"  {
#endif

#ifdef	__TTYBIN_GLOBAL
	#define	__TTYBIN_EXT
#else
	#define __TTYBIN_EXT	extern
#endif


__TTYBIN_EXT void ttybin_init(void);


#ifdef __cplusplus
}
#endif





#ifdef __cplusplus


#endif



#endif /* TTYBIN_H_ */

/* 
 * File:   CO_main_PIC32.h
 * Author: Enric Fernandez
 *
 * Created on 30 de enero de 2025, 13:08
 * 
 * This file is required by CanOpenManager C++ file so it calls the methods in the source file.
 * Thus, this is the C-wrap entry point.
 * However, it should not be included by its own source file, as it doesn't needs it, at least yet.
 */

#ifndef CO_MAIN_PIC32_H
#define	CO_MAIN_PIC32_H

#ifdef	__cplusplus
extern "C" {
#endif

    
int CO_Init();

int CO_Config();

int CO_Update();


// interrupts and similar methods...
void CO_RT_THREAD_ISR();
    
void CO_RxInterrupt();


#ifdef	__cplusplus
}
#endif

#endif	/* CO_MAIN_PIC32_H */


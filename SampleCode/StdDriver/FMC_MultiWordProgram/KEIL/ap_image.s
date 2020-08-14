;/*---------------------------------------------------------------------------------------------------------*/
;/*                                                                                                         */
;/* SPDX-License-Identifier: Apache-2.0                                                                     */
;/* Copyright(c) 2020 Nuvoton Technology Corp. All rights reserved.                                         */
;/*                                                                                                         */
;/*---------------------------------------------------------------------------------------------------------*/


    AREA _image, DATA, READONLY

    EXPORT  loaderImage1Base
    EXPORT  loaderImage1Limit
    
    ALIGN   4
        
loaderImage1Base
    INCBIN ./multi_word_prog.bin
loaderImage1Limit

    END
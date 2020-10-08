#pragma once
#ifndef _MACROS_H
#define _MACROS_H

/*MACRO PARA VARIABLES BOLEANAS*/

#define BOOL1 0x01
#define BOOL2 0x02
#define BOOL3 0x04
#define BOOL4 0x08
#define BOOL5 0x10
#define BOOL6 0x20
#define BOOL7 0x40
#define BOOL8 0x80
/*
Para comprobar un estado:
if ( "nombre de variable" & BOOLX ){
// Ejecuta alguna instrucción
}
Establece el estado 1
"nombre de variable" |= BOOLX;
Limpia el estado BOOL1
"nombre de variable" &=~ BOOLX;
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif
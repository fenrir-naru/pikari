#include "f99x_cs0.h"

void cs0_turn_on(){
  CS0EN = 1;
  /*
   * CS0SMEN = 0 (CS0SCAN0/1 are ignored)
   * CS0CM[2:0] = 0 (start conversion with CS0BUSY)
   * CS0MCEN = 0 (No multiple channel feature)
   * CS0ACU[2:0] = 0 (Accumulate 1 sample)
   */
  //CS0CF = 0x00;

  /*
   * CS0CG[2:0] = 7 (Gain = 8)
   */
  //CS0MD1 = 0x07;

  /*
   * CS0CR[1:0] = 01 (13 bit conversion)
   * CS0DT[2:0] = 0 (Discharge time is 0.75us)
   * CS0IA[2:0] = 0 (Full Current)
   */
  //CS0MD2 = 0x40;
}

void cs0_turn_off(){
  CS0EN = 0;
}


u16 cs0_data(){
  WORD_t res;
  res.c[0] = CS0DL;
  res.c[1] = CS0DH;
  return res.i;
}

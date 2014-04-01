/******************************************
*  My 4x4 RED LED cube
*  using 2 595 shift registers
*
*  animations and base from Kevin Darrah's
*  8x8x8 RGB LED cube.
*
*******************************************/

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

#define SHIFT_REGISTER DDRB
#define SHIFT_PORT PORTB
#define DATA (1<<PB3)		//MOSI (SI)
#define LATCH (1<<PB2)		//SS   (RCK)
#define CLOCK (1<<PB5)		//SCK  (SCK)
#define BLANK (1<<PB1)		//GPIO PB1 as blank

#define CONTROL_REGISTER DDRD
#define CONTROL_PORT PORTD
#define CONTROL_LED (1<<PD4)

/* VARIABLES ********************************************************************************/

uint8_t shift_out;//used in the code a lot in for(i= type loops

//This is how the brightness for every LED is stored,
//Each LED only needs a 'bit' to know if it should be ON or OFF, so 8 Bytes gives you 64 bits= 64 LEDs
//Since we are modulating the LEDs, using 4 bit resolution, each color has 4 arrays containing 64 bits each
uint8_t red0[8], red1[8], red2[8], red3[8];

uint8_t level=0;//keeps track of which level we are shifting data to
uint8_t mByte = 0x02; //0x02 because I have one shift register designated for the anodes where pin0 is not used
uint8_t BAM_Bit, BAM_Counter=0; // Bit Angle Modulation variables to keep track of things

 uint8_t start;//for a seconds timer to cycle through the animations

/* PROTOTYPES ********************************************************************************/
void test();
void wipe_out();
void rainVersionTwo();
void folder();
void bouncyvTwo();
void sinwaveTwo();
void color_wheelTWO();
void testRandomizer();
/* *******************************************************************************************/

void init_Timer_Animations(void){
	/* We are using this 16 bit timer to get 1 seconds interrupts, updating char start for the duration
	 * of the animations.
	 * WGM12 is set to get CTC mode, OCIE1A is set to enable OCR1A interrupts and CS10+12 sets the
	 * prescaler to 1024. With OCR1A to 15625 it should be about one second interrupts. */
	TCCR1A = 0x00;
	TCCR1B |= 1<<WGM12;
	TIMSK1 |= 1<<OCIE1A;
	OCR1A=15625;
	TCCR1B |= (1<<CS10 | 1<<CS12);
}

void init_Timer_BAM(void){
	/* Using this 8-bit timer to get the BAM modulation/multiplexing working on a software basis.
	 * WGM01 is set to work in CTC mode, OCIE0A to enable OCR0A interrupts. CS01 and CS00 gives a
	 * prescaler of 64, meaning an effective clock frequency of 16M/64 = 250kHz. With OCR0A set to
	 * 30 this means an interrupt every 1/250k = 40 us --> (30 + 1)*40 = 1,24 ms so a multiplex frequency
	 * of 1/1,24 ms = 806 Hz. */
	TCCR0A = 1<<WGM01;
	TIMSK0 |= 1<<OCIE0A;
	OCR0A=30;
	TCCR0B |= (1<<CS01 | 1<<CS00);
}

void init_IO(void){
	SHIFT_REGISTER |= (DATA | LATCH | CLOCK | BLANK);	//Set control pins as outputs
	SHIFT_PORT &= ~(DATA | LATCH | CLOCK);		//Set control pins low
	SHIFT_PORT |= BLANK; //Set blank pin high (Low is Output Enabled)

	CONTROL_REGISTER |= CONTROL_LED; //Set control led as output
	CONTROL_PORT &= ~CONTROL_LED; //and set it low
}

void init_SPI(void){
	SPCR = (1<<SPE) | (1<<MSTR);	//Start SPI as Master
	SHIFT_PORT &= ~LATCH; //Pull LATCH low (Important: this is necessary to start the SPI transfer!)
}

void spi_send(uint8_t byte){
	SPDR = byte;			//Shift in some data
	while(!(SPSR & (1<<SPIF)));	//Wait for SPI process to finish
}

int main(void)
{
	cli();
	init_IO();
	init_SPI();
	init_Timer_Animations();
	init_Timer_BAM();
	sei();
	while(1)
	{
		//test();
		//testRandomizer();
		wipe_out();
		rainVersionTwo();
		folder();
		bouncyvTwo();
		sinwaveTwo();
		color_wheelTWO();
	}
}

void LED(int8_t z, int8_t y, int8_t x, int8_t bright){
	if(z<0){
		z=0;
		//return;
	}
	if(z>3){
		z=3;
		//return;
	}
	if(y<0){
		y=0;
		//return;
	}
	if(y>3){
		y=3;
		//return;
	}
	if(x<0){
		x=0;
		//return;
	}
	if(x>3){
		x=3;
		//return;
	}
	if(bright<0){
		bright=0;
		//return;
	}
	if(bright>15){
		bright=15;
		//return;
	}

	int byte_number = (z*2)+ (int) (y/2);  //figuring out which byte the LED is located. 2 bytes per layer, 2 rows per byte.
	int shift_in = x + y*4-( ((int) y/2))*8; //figuring out the place of the LED within the byte

	red0[byte_number] &= ~(0x01<<shift_in); //Setting them to 0 first
	red1[byte_number] &= ~(0x01<<shift_in);
	red2[byte_number] &= ~(0x01<<shift_in);
	red3[byte_number] &= ~(0x01<<shift_in);

	red0[byte_number] |= ((bright & 0x01)>>0)<<shift_in; //bringing the 1 if set in byte BRIGHT representing 1 to the correct location
	red1[byte_number] |= ((bright & 0x02)>>1)<<shift_in; //bringing the 1 if set in byte BRIGHT representing 2 to the correct location
	red2[byte_number] |= ((bright & 0x04)>>2)<<shift_in; //bringing the 1 if set in byte BRIGHT representing 4 to the correct location
	red3[byte_number] |= ((bright & 0x08)>>3)<<shift_in; //bringing the 1 if set in byte BRIGHT representing 8 to the correct location
}

ISR(TIMER1_COMPA_vect){
	/* This interrupt increases char start every 1 seconds, used for the duration of the animations */
	CONTROL_PORT ^= CONTROL_LED; // toggle the green control led
	start++;
}
ISR(TIMER0_COMPA_vect)
{
	SHIFT_PORT |= BLANK;	//Disable outputs, blank high

	if(BAM_Counter==8)
	BAM_Bit++;
	else if(BAM_Counter==24)
	BAM_Bit++;
	else if(BAM_Counter==56)
	BAM_Bit++;

	BAM_Counter++;

	switch (BAM_Bit){//The BAM bit will be a value from 0-3, and only shift out the arrays corresponding to that bit, 0-3
		//Here's how this works, each case is the bit in the Bit angle modulation from 0-4,
		//Next, it depends on which level we're on, so the byte in the array to be written depends on which level, but since each level contains 64 LED,
		//we only shift out 8 bytes for each color
		case 0:
			for(shift_out=level; shift_out<level+2; shift_out++)
			spi_send(red0[shift_out]);
			break;
		case 1:
			for(shift_out=level; shift_out<level+2; shift_out++)
			spi_send(red1[shift_out]);
			break;
		case 2:
			for(shift_out=level; shift_out<level+2; shift_out++)
			spi_send(red2[shift_out]);
			break;
		case 3:
			for(shift_out=level; shift_out<level+2; shift_out++)
			spi_send(red3[shift_out]);
			/*	Here is where the BAM_Counter is reset back to 0, it's only 4 bit,
			*	but since each cycle takes 8 counts.
			*	It goes 0 8 16 32, and when BAM_counter hits 64 we reset the BAM. */
			if(BAM_Counter==120){
				BAM_Counter=0;
				BAM_Bit=0;
			}
			break;
	}//switch_case

	spi_send(mByte);//finally, send out the anode level byte
	//spi_send((uint8_t)binary_counter);

	SHIFT_PORT |= LATCH;	//Toggle latch to copy data to the storage register
	SHIFT_PORT &= ~LATCH;	//Toggle latch to copy data to the storage register
	SHIFT_PORT &= ~BLANK;	//Blank low, outputs enabled

	mByte = mByte<<1;
	level = level+2;//increment the level variable by 8, which is used to shift out data, since the next level would be the next 8 bytes in the arrays

	if(level==8){//if you hit 64 on level, this means you just sent out all 63 bytes, so go back
		level=0;
		mByte = 0x02;
	}
}

int8_t mRandom(int8_t min, int8_t max){
	return (rand() % max) + min;
}

void wipe_out(){//*****wipe_out*****wipe_out*****wipe_out*****wipe_out*****wipe_out*****wipe_out*****wipe_out*****wipe_out
	int8_t xxx=0, yyy=0, zzz=0;
	int8_t fx=mRandom(0,4), fy=mRandom(0,4), fz=mRandom(0,4), direct, fxm=1, fym=1, fzm=1, fxo=0, fyo=0, fzo=0;
	int8_t ftx=mRandom(0,4), fty=mRandom(0,4), ftz=mRandom(0,4), ftxm=1, ftym=1, ftzm=1, ftxo=0, ftyo=0, ftzo=0;
	int8_t rr, rrt;
	for(xxx=0; xxx<4; xxx++){
		for(yyy=0; yyy<4; yyy++){
			for(zzz=0; zzz<4; zzz++){
				LED(xxx, yyy, zzz, 0);
			}
		}
	}

	rr=mRandom(6,16);
	rrt=mRandom(1,5);

	start = 0;
	while(start < 10){

		LED(fxo, fyo, fzo, 0);
		LED(fxo, fyo, fzo+1, 0);
		LED(fxo, fyo, fzo-1, 0);
		LED(fxo+1, fyo, fzo, 0);
		LED(fxo-1, fyo, fzo, 0);
		LED(fxo, fyo+1, fzo, 0);
		LED(fxo, fyo-1, fzo, 0);

		LED(ftxo, ftyo, ftzo, 0);
		LED(ftxo, ftyo, ftzo+1, 0);
		LED(ftxo, ftyo, ftzo-1, 0);
		LED(ftxo+1, ftyo, ftzo, 0);
		LED(ftxo-1, ftyo, ftzo, 0);
		LED(ftxo, ftyo+1, ftzo, 0);
		LED(ftxo, ftyo-1, ftzo, 0);

		LED(ftx, fty, ftz, rr);
		LED(ftx, fty, ftz+1, rr);
		LED(ftx, fty, ftz-1,  rr);
		LED(ftx+1, fty, ftz, rr);
		LED(ftx-1, fty, ftz, rr);
		LED(ftx, fty+1, ftz, rr);
		LED(ftx, fty-1, ftz, rr);

		LED(fx, fy, fz, rrt);
		LED(fx, fy, fz+1, rrt);
		LED(fx, fy, fz-1, rrt);
		LED(fx+1, fy, fz, rrt);
		LED(fx-1, fy, fz, rrt);
		LED(fx, fy+1, fz, rrt);
		LED(fx, fy-1, fz, rrt);

		_delay_ms(75);

		fxo=fx;
		fyo=fy;
		fzo=fz;

		ftxo=ftx;
		ftyo=fty;
		ftzo=ftz;

		direct=mRandom(0,3);
		if(direct==0)
			fx= fx+fxm;
		if(direct==1)
			fy= fy+fym;
		if(direct==2)
			fz= fz+fzm;
		if(fx<0){
			fx=0;
			fxm=1;
		}
		if(fx>3){
			fx=3;
			fxm=-1;
		}
		if(fy<0){
			fy=0;
			fym=1;
		}
		if(fy>3){
			fy=3;
			fym=-1;
		}
		if(fz<0){
			fz=0;
			fzm=1;
		}
		if(fz>3){
			fz=3;
			fzm=-1;
		}

		direct=mRandom(0,3);
		if(direct==0)
			ftx= ftx+ftxm;
		if(direct==1)
			fty= fty+ftym;
		if(direct==2)
			ftz= ftz+ftzm;

		if(ftx<0){
			ftx=0;
			ftxm=1;
		}
		if(ftx>3){
			ftx=3;
			ftxm=-1;
		}
		if(fty<0){
			fty=0;
			ftym=1;
		}
		if(fty>3){
			fty=3;
			ftym=-1;
		}
		if(ftz<0){
			ftz=0;
			ftzm=1;
		}
		if(ftz>3){
			ftz=3;
			ftzm=-1;
		}
	}//while
	for(xxx=0; xxx<4; xxx++){
		for(yyy=0; yyy<4; yyy++){
			for(zzz=0; zzz<4; zzz++){
				LED(xxx, yyy, zzz, 0);
			}
		}
	}
}//wipeout


void rainVersionTwo(){//****rainVersionTwo****rainVersionTwo****rainVersionTwo****rainVersionTwo****rainVersionTwo
	int8_t x[16], y[16], z[16], addr, leds=16;
	int8_t xold[16], yold[16], zold[16];

	for(addr=0; addr<16; addr++){
		x[addr]=mRandom(0,4);
		y[addr]=mRandom(0,4);
		z[addr]=mRandom(0,4);
	}

	start = 0;
	while(start < 15){
		for(addr=0; addr<leds; addr++){
			LED(zold[addr], xold[addr], yold[addr], 0);
			if(z[addr]>=3)
				LED(z[addr], x[addr], y[addr], 15);
			if(z[addr]==2)
				LED(z[addr], x[addr], y[addr], 10);
			if(z[addr]==1)
				LED(z[addr], x[addr], y[addr], 5);
			if(z[addr]<=0)
				LED(0, x[addr], y[addr], 1);
		}

		for(addr=0; addr<leds; addr++){
			xold[addr]=x[addr];
			yold[addr]=y[addr];
			zold[addr]=z[addr];
		}

		_delay_ms(80);

		for(addr=0; addr<leds; addr++){

			z[addr] = z[addr]-1;

			if(z[addr]<mRandom(0,4)-5){
				LED(0,x[addr], y[addr], 0);
				x[addr]=mRandom(0,4);
				y[addr]=mRandom(0,4);
				z[addr]=3;
			}//-check
		}//add
	}//while
}//rainv2


void folder(){//****folder****folder****folder****folder****folder****folder****folder****folder****folder
	int8_t xx, yy, zz, pullback[16];
	int8_t folderaddr[16], LED_Old[16], oldpullback[16], ranx=15;
	int8_t bot=0, top=1, right=0, left=0, back=0, front=0, side=0, side_select;

	folderaddr[0]=-3;
	folderaddr[1]=-2;
	folderaddr[2]=-1;
	folderaddr[3]=-0;

	for(xx=0; xx<4; xx++){
	oldpullback[xx]=0;
	pullback[xx]=0;
	}



	start=0;
	while(start < 40){
		if(top==1){
			if(side==0){
				//top to left-side
				for(yy=0; yy<4; yy++){
					for(xx=0; xx<4; xx++){
						LED(3-LED_Old[yy], yy-oldpullback[yy],xx , 0);
						LED(3-folderaddr[yy], yy-pullback[yy],xx , ranx);
					}
				}
			}
			if(side==2){
				//top to back-side
				for(yy=0; yy<4; yy++){
					for(xx=0; xx<4; xx++){
						LED(3-LED_Old[yy], xx, yy-oldpullback[yy], 0);
						LED(3-folderaddr[yy], xx, yy-pullback[yy], ranx);
					}
				}
			}
			if(side==3){
				//top-side to front-side
				for(yy=0; yy<4; yy++){
					for(xx=0; xx<4; xx++){
						LED(3-LED_Old[3-yy], xx, yy+oldpullback[yy], 0);
						LED(3-folderaddr[3-yy], xx, yy+pullback[yy], ranx);
					}
				}
			}
			if(side==1){
				//top-side to right
				for(yy=0; yy<4; yy++){
					for(xx=0; xx<4; xx++){
						LED(3-LED_Old[3-yy], yy+oldpullback[yy],xx , 0);
						LED(3-folderaddr[3-yy], yy+pullback[yy],xx , ranx);
					}
				}
			}
		}//top

		if(right==1){
			if(side==4){
				//right-side to top
				for(yy=0; yy<4; yy++){
					for(xx=0; xx<4; xx++){
						LED(yy+oldpullback[3-yy],3-LED_Old[3-yy],xx , 0);
						LED( yy+pullback[3-yy],3-folderaddr[3-yy],xx , ranx);
					}
				}
			}
			if(side==3){
				//right-side to front-side
				for(yy=0; yy<4; yy++){
					for(xx=0; xx<4; xx++){
						LED(xx, 3-LED_Old[3-yy],yy+oldpullback[yy], 0);
						LED(xx,3-folderaddr[3-yy], yy+pullback[yy], ranx);
					}
				}
			}
			if(side==2){
				//right-side to back-side
				for(yy=0; yy<4; yy++){
					for(xx=0; xx<4; xx++){
						LED(xx, 3-LED_Old[yy],yy-oldpullback[yy], 0);
						LED(xx,3-folderaddr[yy], yy-pullback[yy], ranx);
					}
				}
			}
			if(side==5){
				//right-side to bottom
				for(yy=0; yy<4; yy++){
					for(xx=0; xx<4; xx++){
						LED(yy-oldpullback[yy],3-LED_Old[yy],xx , 0);
						LED( yy-pullback[yy],3-folderaddr[yy],xx , ranx);
					}
				}
			}
		}//right

		if(left==1){
			if(side==4){
				//left-side to top
				for(yy=0; yy<4; yy++){
					for(xx=0; xx<4; xx++){
						LED(yy+oldpullback[yy],LED_Old[3-yy],xx , 0);
						LED( yy+pullback[yy],folderaddr[3-yy],xx , ranx);
					}
				}
			}
			if(side==3){
				//left-side to front-side
				for(yy=0; yy<4; yy++){
					for(xx=0; xx<4; xx++){
						LED(xx, LED_Old[3-yy],yy+oldpullback[yy], 0);
						LED(xx,folderaddr[3-yy], yy+pullback[yy], ranx);
					}
				}
			}
			if(side==2){
				//left-side to back-side
				for(yy=0; yy<4; yy++){
					for(xx=0; xx<4; xx++){
						LED(xx, LED_Old[yy],yy-oldpullback[yy], 0);
						LED(xx,folderaddr[yy], yy-pullback[yy], ranx);
					}
				}
			}
			if(side==5){
				//left-side to bottom
				for(yy=0; yy<4; yy++){
					for(xx=0; xx<4; xx++){
						LED(yy-oldpullback[yy],LED_Old[yy],xx , 0);
						LED( yy-pullback[yy],folderaddr[yy],xx , ranx);
					}
				}
			}
		}//left


		if(back==1){
			if(side==1){
				//back-side to right-side
				for(yy=0; yy<4; yy++){
					for(xx=0; xx<4; xx++){
						LED(xx, yy+oldpullback[yy],LED_Old[3-yy], 0);
						LED(xx, yy+pullback[yy],folderaddr[3-yy], ranx);
					}
				}
			}
			if(side==4){
				// back-side to top-side
				for(yy=0; yy<4; yy++){
					for(xx=0; xx<4; xx++){
						LED(yy+oldpullback[yy],xx,LED_Old[3-yy] , 0);
						LED( yy+pullback[yy],xx,folderaddr[3-yy] , ranx);
					}
				}
			}
			if(side==5){
				// back-side to bottom
				for(yy=0; yy<4; yy++){
					for(xx=0; xx<4; xx++){
						LED(yy-oldpullback[yy],xx,LED_Old[yy] , 0);
						LED( yy-pullback[yy],xx,folderaddr[yy] , ranx);
					}}}//state1
					if(side==0){
						//back-side to left-side
						for(yy=0; yy<4; yy++){
							for(xx=0; xx<4; xx++){
								LED(xx, yy-oldpullback[yy],LED_Old[yy], 0);
								LED(xx, yy-pullback[yy],folderaddr[yy], ranx);
							}
						}
					}
				}//back
				if(bot==1){
					if(side==1){
						// bottom-side to right-side
						for(yy=0; yy<4; yy++){
							for(xx=0; xx<4; xx++){
								LED(LED_Old[3-yy], yy+oldpullback[yy],xx , 0);
								LED(folderaddr[3-yy], yy+pullback[yy],xx , ranx);
							}
						}
					}
					if(side==3){
						//bottom to front-side
						for(yy=0; yy<4; yy++){
							for(xx=0; xx<4; xx++){
								LED(LED_Old[3-yy], xx, yy+oldpullback[yy], 0);
								LED(folderaddr[3-yy], xx, yy+pullback[yy], ranx);
							}
						}
					}
					if(side==2){
						//bottom to back-side
						for(yy=0; yy<4; yy++){
							for(xx=0; xx<4; xx++){
								LED(LED_Old[yy], xx, yy-oldpullback[yy], 0);
								LED(folderaddr[yy], xx, yy-pullback[yy], ranx);
							}
						}
					}
					if(side==0){
						//bottom to left-side
						for(yy=0; yy<4; yy++){
							for(xx=0; xx<4; xx++){
								LED(LED_Old[yy], yy-oldpullback[yy],xx , 0);
								LED(folderaddr[yy], yy-pullback[yy],xx , ranx);
							}
						}
					}
				}//bot

				if(front==1){
					if(side==0){
						//front-side to left-side
						for(yy=0; yy<4; yy++){
							for(xx=0; xx<4; xx++){
								LED(xx, yy-oldpullback[yy],3-LED_Old[yy], 0);
								LED(xx, yy-pullback[yy],3-folderaddr[yy], ranx);
							}
						}
					}
					if(side==5){
						// front-side to bottom
						for(yy=0; yy<4; yy++){
							for(xx=0; xx<4; xx++){
								LED(yy-oldpullback[yy],xx,3-LED_Old[yy] , 0);
								LED( yy-pullback[yy],xx,3-folderaddr[yy] , ranx);
							}
						}
					}
					if(side==4){
						// front-side to top-side
						for(yy=0; yy<4; yy++){
							for(xx=0; xx<4; xx++){
								LED(yy+oldpullback[yy],xx,3-LED_Old[3-yy] , 0);
								LED( yy+pullback[yy],xx,3-folderaddr[3-yy] , ranx);
							}
						}
					}
					if(side==1){
						//front-side to right-side
						for(yy=0; yy<4; yy++){
							for(xx=0; xx<4; xx++){
								LED(xx, yy+oldpullback[yy],3-LED_Old[3-yy], 0);
								LED(xx, yy+pullback[yy],3-folderaddr[3-yy], ranx);
							}
						}
					}
				}//front

				_delay_ms(60);//               DELAY   DELAY  DELAY

				for(xx=0; xx<4; xx++){
					LED_Old[xx]=folderaddr[xx];
					oldpullback[xx]=pullback[xx];
				}

				if(folderaddr[3]==3){
					// pullback=8;
					for(zz=0; zz<4; zz++)
					pullback[zz] = pullback[zz]+1;

					if(pullback[3]==4){//finished with fold
						_delay_ms(20);

						ranx=mRandom(2,15);
						side_select=mRandom(0,2);
						//side_select=3;

						if(top==1){//                 TOP
							top=0;
							if(side==0){//top to left
								left=1;
								if(side_select==0) side=2;
								if(side_select==1) side=3;
								//if(side_select==2) side=4;
								if(side_select==2) side=5;
							}
							else if(side==1){//top to right
								right=1;
								if(side_select==0) side=5;
								if(side_select==1) side=2;
								if(side_select==2) side=3;
								//if(side_select==3) side=4;
							}
							else if(side==2){//top to back
								back=1;
								if(side_select==0) side=0;
								if(side_select==1) side=1;
								if(side_select==2) side=5;
								//if(side_select==3) side=4;
							}
							else if(side==3){//top to front
								front=1;
								if(side_select==0) side=0;
								if(side_select==1) side=1;
								if(side_select==2) side=5;
								//if(side_select==3) side=4;
							}
						}
						else if(bot==1){//                 BOTTOM
							bot=0;
							if(side==0){//bot to left
								left=1;
								if(side_select==0) side=2;
								if(side_select==1) side=3;
								if(side_select==2) side=4;
								//if(side_select==3) side=5;
							}
							else if(side==1){//bot to right
								right=1;
								//if(side_select==0) side=5;
								if(side_select==0) side=2;
								if(side_select==1) side=3;
								if(side_select==2) side=4;
							}
							else if(side==2){//bot to back
								back=1;
								if(side_select==0) side=0;
								if(side_select==1) side=1;
								//if(side_select==2) side=5;
								if(side_select==2) side=4;
							}
							else if(side==3){//bot to front
								front=1;
								if(side_select==0) side=0;
								if(side_select==1) side=1;
								//if(side_select==2) side=5;
								if(side_select==2) side=4;
							}
						}
						else if(right==1){//                 RIGHT
							right=0;
							if(side==4){//right to top
								top=1;
								if(side_select==0) side=2;
								if(side_select==1) side=3;
								if(side_select==2) side=0;
								//if(side_select==3) side=1;
							}
							else if(side==5){//right to bot
								bot=1;
								if(side_select==0) side=0;
								if(side_select==1) side=2;
								if(side_select==2) side=3;
								//if(side_select==3) side=1;
							}
							else if(side==2){//right to back
								back=1;
								if(side_select==0) side=0;
								//if(side_select==1) side=1;
								if(side_select==1) side=5;
								if(side_select==2) side=4;
							}
							else if(side==3){//right to front
								front=1;
								if(side_select==0) side=0;
								//if(side_select==1) side=1;
								if(side_select==1) side=5;
								if(side_select==2) side=4;
							}
						}
						else if(left==1){//                 LEFT
							left=0;
							if(side==4){//left to top
								top=1;
								//if(side_select==0) side=2;
								if(side_select==0) side=3;
								if(side_select==1) side=2;
								if(side_select==2) side=1;
							}
							else if(side==5){//left to bot
								bot=1;
								//if(side_select==0) side=0;
								if(side_select==0) side=2;
								if(side_select==1) side=3;
								if(side_select==2) side=1;
							}
							else if(side==2){//left to back
								back=1;
								//if(side_select==0) side=0;
								if(side_select==0) side=1;
								if(side_select==1) side=5;
								if(side_select==2) side=4;
							}
							else if(side==3){//left to front
								front=1;
								//if(side_select==0) side=0;
								if(side_select==0) side=1;
								if(side_select==1) side=5;
								if(side_select==2) side=4;
							}
						}
						else if(front==1){//                 front
							front=0;
							if(side==4){//front to top
								top=1;
								if(side_select==0) side=2;
								//if(side_select==1) side=3;
								if(side_select==1) side=0;
								if(side_select==2) side=1;
							}
							else if(side==5){//front to bot
								bot=1;
								if(side_select==0) side=0;
								if(side_select==1) side=2;
								//if(side_select==2) side=3;
								if(side_select==2) side=1;
							}
							else if(side==0){//front to left
								left=1;
								if(side_select==0) side=2;
								// if(side_select==1) side=3;
								if(side_select==1) side=5;
								if(side_select==2) side=4;
							}
							else if(side==1){//front to right
								right=1;
								if(side_select==0) side=2;
								// if(side_select==1) side=3;
								if(side_select==1) side=5;
								if(side_select==2) side=4;
							}
						}
						else if(back==1){//                 back
							back=0;
							if(side==4){//back to top
								top=1;
								//if(side_select==0) side=2;
								if(side_select==0) side=3;
								if(side_select==1) side=0;
								if(side_select==2) side=1;
							}
							else if(side==5){//back to bot
								bot=1;
								if(side_select==0) side=0;
								//if(side_select==1) side=2;
								if(side_select==1) side=3;
							if(side_select==2) side=1;} else
							if(side==0){//back to left
								left=1;
								//if(side_select==0) side=2;
								if(side_select==0) side=3;
								if(side_select==1) side=5;
								if(side_select==2) side=4;
							}
							else if(side==1){//back to right
								right=1;
								if(side_select==0) side=3;
								if(side_select==1) side=5;
								if(side_select==2) side=4;
							}
						} //bot


						// for(yy=0; yy<8; yy++)
						//for(xx=0; xx<8; xx++)
						//LED(LED_Old[yy], xx, yy-oldpullback[yy], 0, 0, 0);
						for(xx=0; xx<4; xx++){
							oldpullback[xx]=0;
							pullback[xx]=0;
						}
						folderaddr[0]=-4;
						folderaddr[1]=-3;
						folderaddr[2]=-2;
						folderaddr[3]=-1;
					}//pullback==7
				}//folderaddr==7

				if(folderaddr[3]!=3)
				for(zz=0; zz<4; zz++)
				folderaddr[zz] = folderaddr[zz]+1;

			}//while

}//folder


void bouncyvTwo(){//****bouncyTwo****bouncyTwo****bouncyTwo****bouncyTwo****bouncyTwo****bouncyTwo****bouncyTwo
	int8_t oldx[50], oldy[50], oldz[50];
	int8_t x[50], y[50], z[50], addr, ledcount=20, direct;
	int8_t xx[50], yy[50], zz[50];
	int8_t xbit=1, ybit=1, zbit=1;

	for(addr=0; addr<ledcount+1; addr++){
		oldx[addr]=0;
		oldy[addr]=0;
		oldz[addr]=0;
		x[addr]=0;
		y[addr]=0;
		z[addr]=0;
		xx[addr]=0;
		yy[addr]=0;
		zz[addr]=0;
	}

	start=0;
	while(start < 10){
		direct = rand() % 3;

		for(addr=1; addr<ledcount+1; addr++){
			LED(oldx[addr], oldy[addr],oldz[addr], 0);
			LED(x[addr], y[addr], z[addr], xx[addr]);
		}

		for(addr=1; addr<ledcount+1; addr++){
			oldx[addr]=x[addr];
			oldy[addr]=y[addr];
			oldz[addr]=z[addr];
		}
		_delay_ms(20);

		if(direct==0)
		x[0]= x[0]+xbit;
		if(direct==1)
		y[0]= y[0]+ybit;
		if(direct==2)
		z[0]= z[0]+zbit;

		if(direct==3)
		x[0]= x[0]-xbit;
		if(direct==4)
		y[0]= y[0]-ybit;
		if(direct==5)
		z[0]= z[0]-zbit;

		if(x[0]>3){
			xbit=-1;
			x[0]=3;
			xx[0]=rand() % 16;
			yy[0]=rand() % 16;
			zz[0]=0;
			//wipe_out();
		}
		if(x[0]<0){
			xbit=1;
			x[0]=0;
			xx[0]=rand() % 16;
			yy[0]=0;
			zz[0]=rand() % 16;
			//wipe_out();
		}
		if(y[0]>3){
			ybit=-1;
			y[0]=3;
			xx[0]=0;
			yy[0]=rand() % 16;
			zz[0]=rand() % 16;
			//wipe_out();
		}
		if(y[0]<0){
			ybit=1;
			y[0]=0;
			xx[0]=0;
			yy[0]=rand() % 16;
			zz[0]=rand() % 16;
			//wipe_out();
		}
		if(z[0]>3){
			zbit=-1;
			z[0]=3;
			xx[0]=rand() % 16;
			yy[0]=0;
			zz[0]=rand() % 16;
			//wipe_out();
		}
		if(z[0]<0){
			zbit=1;
			z[0]=0;
			xx[0]=rand() % 16;
			yy[0]=rand() % 16;
			zz[0]=0;
		}

		for(addr=ledcount; addr>0; addr--){
			x[addr]=x[addr-1];
			y[addr]=y[addr-1];
			z[addr]=z[addr-1];
			xx[addr]=xx[addr-1];
			yy[addr]=yy[addr-1];
			zz[addr]=zz[addr-1];
		}
	}//while
}//bouncyv2


void sinwaveTwo(){//*****sinewaveTwo*****sinewaveTwo*****sinewaveTwo*****sinewaveTwo*****sinewaveTwo*****sinewaveTwo
	int8_t sinewavearray[4], addr, sinemult[4], rr=0;
	int8_t sinewavearrayOLD[4], subT=3;//random(-1, 2);

	sinewavearray[0]=0;
	sinemult[0]=1;
	sinewavearray[1]=1;
	sinemult[1]=1;
	sinewavearray[2]=2;
	sinemult[2]=1;
	sinewavearray[3]=3;
	sinemult[3]=1;

	start=0;
	while(start < 10){
		for(addr=0; addr<4; addr++){
			if(sinewavearray[addr]==3){
				sinemult[addr]=-1;
			}
			if(sinewavearray[addr]==0){
				sinemult[addr]=1;
			}
			sinewavearray[addr] = sinewavearray[addr] + sinemult[addr];
		}//addr
		if(sinewavearray[0]==3){
			rr=(rand() % 6) + 10;
		}

		for(addr=0; addr<4; addr++){
			LED(sinewavearrayOLD[addr], addr, 0, 0);
			LED(sinewavearrayOLD[addr], 0, addr, 0);
			LED(sinewavearrayOLD[addr], subT-addr, 3, 0);
			LED(sinewavearrayOLD[addr], 3, subT-addr, 0);
			LED(sinewavearray[addr], addr, 0, rr);
			LED(sinewavearray[addr], 0, addr, rr);
			LED(sinewavearray[addr], subT-addr,3, rr);
			LED(sinewavearray[addr], 3, subT-addr, rr);
		}

		for(addr=1; addr<3; addr++){
			LED(sinewavearrayOLD[addr], addr, 1, 0);
			LED(sinewavearrayOLD[addr], 1, addr, 0);
			LED(sinewavearrayOLD[addr], subT-addr, 2, 0);
			LED(sinewavearrayOLD[addr], 2, subT-addr, 0);
			LED(sinewavearray[addr], addr, 1, addr*3);
			LED(sinewavearray[addr], 1, addr, addr*3);
			LED(sinewavearray[addr], subT-addr,2, addr*3);
			LED(sinewavearray[addr], 2, subT-addr, addr*3);
		}
		for(addr=0; addr<4; addr++)
			sinewavearrayOLD[addr]=sinewavearray[addr];

		_delay_ms(100);
	}//while
}//SinewaveTwo



void color_wheelTWO(){//*****colorWheelTwo*****colorWheelTwo*****colorWheelTwo*****colorWheelTwo*****colorWheelTwo
	int8_t xx, yy, zz, ranx, swiper;

	start=0;
	while(start < 10){
		swiper = rand() % 6;
		ranx=(rand() % 12) + 4;

		if(swiper==0){
			for(yy=0;yy<4;yy++){//left to right
				for(xx=0;xx<4;xx++){
					for(zz=0;zz<4;zz++){
						LED(xx, yy, zz,  ranx);
					}
				}
				_delay_ms(30);
			}
		}
		if(swiper==1){//bot to top
			for(xx=0;xx<4;xx++){
				for(yy=0;yy<4;yy++){
					for(zz=0;zz<4;zz++){
						LED(xx, yy, zz,  ranx);
					}
				}
				_delay_ms(30);
			}
		}
		if(swiper==2){//back to front
			for(zz=0;zz<4;zz++){
				for(xx=0;xx<4;xx++){
					for(yy=0;yy<4;yy++){
						LED(xx, yy, zz,  ranx);
					}
				}
				_delay_ms(30);
			}
		}
		if(swiper==3){
			for(yy=3;yy>=0;yy--){//right to left
				for(xx=0;xx<4;xx++){
					for(zz=0;zz<4;zz++){
						LED(xx, yy, zz,  ranx);
					}
				}
				_delay_ms(30);
			}
		}
		if(swiper==4){//top to bot
			for(xx=3;xx>=0;xx--){
				for(yy=0;yy<4;yy++){
					for(zz=0;zz<4;zz++){
						LED(xx, yy, zz,  ranx);
					}
				}
				_delay_ms(30);
			}
		}
		if(swiper==5){//front to back
			for(zz=3;zz>=0;zz--){
				for(xx=0;xx<4;xx++){
					for(yy=0;yy<4;yy++){
						LED(xx, yy, zz,  ranx);
					}
				}
				_delay_ms(30);
			}
		}
	}//while
}//color wheel


void clean(){
	int8_t ii, jj, kk;
	for(ii=0; ii<4; ii++)
		for(jj=0; jj<4; jj++)
			for(kk=0; kk<4; kk++)
				LED(ii,jj,kk,0);
}

void testRandomizer(){
	char x = mRandom(0,15);
	LED(4,4,4,x);
	_delay_ms(5);
	//LED(4,4,4,0);
}

void test(){
	int8_t ii, jj, kk;
	for(ii=0; ii<4; ii++)
		for(jj=0; jj<4; jj++)
			for(kk=0; kk<4; kk++){
				LED(ii,jj,kk,15);
				_delay_ms(150);
				LED(ii,jj,kk,0);
			}
}

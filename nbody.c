// gcc -o nbody nbody.c `sdl-config --cflags --libs` -lm
/*
nbody #1 #2 #3 #4 #5 #6 (#7)
#1: number of bodies (int)
#2: number of dimensions (int)
#3: timestep (double)
#4: softening factor (double)
#5: window width in pixels (int)
#6: window height in pixels (int)
#7: optional seed to PRNG (int)

example:
./nbody 3 2 30 30 500 500

controls:
ESC: quit
r:   re-initialize
s:   save seed and screenshot
b:   blank screen (deletes trails)
*/
#include "SDL.h" 
#include <stdio.h> 
#include <stdlib.h>
#include <math.h>
#include <time.h>

void putpixels(SDL_Surface *surface, double **x, int n, int w, int h);
void events( int *q, SDL_Surface *surface, double **x, double **xp, 
             double **xn, double **v, int n, int d, int w, int h,
             double *a, double dt, double s, int *seed );
void init( double **x, double **v, int n, int d, int w, int h, int *seed);
void accelerate( double *x1, double *x2, double *a, int d, double s );
void dxi( double*x, double *xp, double *v, double *a, double dt, int d );
void dx( double *xn, double *x, double *xp, double *a, double dt, int d );
void firstStep( double **x, double **xp, double **xn,
                double **v, int n, int d, int w, int h,
                double *a, double dt, double s, int *seed );

int main(int argc, char *argv[]){

    int n = atoi( argv[1] );    // number of bodies
    int d = atoi( argv[2] );    // number of spatial dimensions
    double dt = atof( argv[3] ); // time step
    double s = atof( argv[4] ); //softening radius
    int w = atoi( argv[5] ); //window width
    int h = atoi( argv[6] ); //window height
    int *seed = malloc(sizeof(int)); //random seed
    if(argc<7){
        printf("Error. Nbody takes 6 or 7 arguments.\n");
        return -1;
    }
    else if(argc==7){ // if optional seed is omitted,
        *seed = (int)time(NULL); // generate one
    }
    else if(argc==8){ // else, use provided seed
        *seed = atoi( argv[7] );
    }
    else{
        printf("Error. Nbody takes 6 or 7 arguments.\n");
        return -1;
    }
    SDL_Surface *screen; 
    int *q = malloc(sizeof(int)); // quit flag 
    *q=0;
    int i,j;

    double **x  = malloc(sizeof(double)*n); //init n rows 
    double **xp = malloc(sizeof(double)*n);
    double **xn = malloc(sizeof(double)*n);
    double **v  = malloc(sizeof(double)*n);
    for( i=0; i<n; i++){
        x[i]  = malloc(sizeof(double)*d); //each row is a d-length array
        xp[i] = malloc(sizeof(double)*d); 
        xn[i] = malloc(sizeof(double)*d); 
        v[i]  = malloc(sizeof(double)*d);
    }

    double *a = malloc(sizeof(double)*d); //d-length array to hold accel.

    if((SDL_Init(SDL_INIT_VIDEO)==-1)){
        printf("Couldn't initialize SDL: %s.\n", SDL_GetError()); 
        return -1; 
    } 

    screen = SDL_SetVideoMode(w, h, 8, SDL_SWSURFACE); 
    if ( screen == NULL ){
        fprintf(stderr, "Couldn't set video mode: %s\n", SDL_GetError()); 
        return -2; 
    }

    // initialize and do first step
    firstStep( x, xp, xn, v, n, d, w, h, a, dt, s, seed );

    // here is the main loop,
    // it continues until q=1
    while( !*q ){
        events(q, screen, x, xp, xn, v, n, d, w, h, a, dt, s, seed );


        for( i=0; i<n; i++ ){    //finding acceleration of this body
            for( j=0; j<n; j++ ) //due to these bodies,
                accelerate( x[i], x[j], a, d, s );   //summing it up,
            dx( xn[i], x[i], xp[i], a, dt, d ); //update xn
            for( j=0; j<d; j++ )
                a[j] = 0.0;
        }

        putpixels(screen, x, n, w, h);

        //shift the prev, current, and next pointers for next timestep
        xp = x;  //for next timestep
        x  = xn;
        xn = xp; //overwriting xp

    } 

    SDL_Quit(); 
  
    return 0; 
} 

void putpixels(SDL_Surface *surface, double **x, int n, int w, int h ){
    int i, xi, yi;
    for(i=0; i<n; i++){
        xi = (int)x[i][0];
        yi = (int)x[i][1];
        // p is pointer to location in screen buffer 
        if( xi<w && xi>0 && yi<h && yi>0){
            Uint8 *p = (Uint8 *)surface->pixels + yi * surface->pitch + xi; 
            p[0] = 0xff&((1+i)*55); 
            SDL_UpdateRect(surface, xi, yi, 1, 1);
        }
    }
} 

void events( int *q, SDL_Surface *surface, double **x, double **xp, 
             double **xn, double **v, int n, int d, int w, int h,
             double *a, double dt, double s, int *seed ){

    SDL_Event event;
    char fname[50];
    while( SDL_PollEvent( &event ) ){ 
        if(event.type == SDL_QUIT){
            *q=1;
            break;
        }
        else if(event.type == SDL_KEYUP){
            switch( event.key.keysym.sym ){ // that's the ASCII code
                case 27: // escape
                    *q=1;
                    break;
                case 98: // b
                    SDL_FillRect( surface, NULL, 0 ); //black screen
                    SDL_UpdateRect( surface,0,0,0,0); //and update
                    break;
                case 115: // s
                    printf("%d saved\n", *seed);
                    FILE *fp;
                    fp=fopen("saved.txt","a");
                    fprintf(fp,"./nbody %d %d %f %f %d %d %d\n", n, d, 
                                        dt, s, w, h, *seed);
                    fclose(fp);
                    sprintf(fname, "%d-%f-%f.bmp",*seed,dt,s);
                    SDL_SaveBMP(surface, fname);
                    break;
                case 114: // r
                    *seed = time(NULL);
                    firstStep(x, xp, xp, v, n, d, w, h, a, dt, s, seed);
                    SDL_FillRect( surface, NULL, 0 ); //black screen
                    SDL_UpdateRect( surface,0,0,0,0); //and update
                    break;
                default:
                    break;
            }
        }
    }
}

// x1 and x2 are d-length arrays of position coords
// a is the d-length acceleration vector
// d is number of spatial dimensions
// s is the softening radius
void accelerate( double *x1, double *x2, double *a, int d, double s ){
    double dSqr = 0.0, r2, aPrt;
    int i;

    for( i=0; i<d; i++ ) // d^2 = x1^2 + x2^2 + ... + xd^2
        dSqr += (x1[i] - x2[i])*(x1[i] - x2[i]);

    r2 = dSqr + s*s;
    aPrt = 1.0/sqrt(r2*r2*r2);
    
    for( i=0; i<d; i++ )
        a[i] += aPrt*(x2[i] - x1[i]);
}

// need to find x1 from x0 and v0
void dxi( double *x, double *xp, double *v, double *a, double dt, int d){
    int i;
    for( i=0; i<d; i++ )
        x[i] = xp[i] + v[i]*dt + a[i]*dt*dt/2;
    //at this point, xp is initial conds (x0),
    //and x is first step (x1).
}

// verlet algorithm
void dx( double *xn, double *x, double *xp, double *a, double dt, int d ){
    int i;
    for( i=0; i<d; i++ ){ 
        xn[i] = 2*x[i] - xp[i] + a[i]*dt*dt;
    }
}

// initializes the position and velocity matrices,
// then finds position and velocity of CoM and subtracts
// these coords from each particle.
void init(double **x, double **v, int n, int d, int w, int h, int *seed){
    int i,j;
    double scale;
    double *comx = malloc(sizeof(double)*d);
    double *comv = malloc(sizeof(double)*d);
    for( i=0; i<d; i++ ){
        comx[i] = 0.0;
        comv[i] = 0.0;
    }
    double vmult; // multiplier to get vel. from pos.
                  // I think the way I calculate an
                  // init. vel. helps give a bit of
                  // "swirl" to the initial conds.

    if(h<w)
        scale=(double)h;
    else
        scale=(double)w;

    srand(*seed);
    for( i=0; i<n; i++ ){
        for( j=0; j<d; j++ ){
            vmult = (rand()%10)/10000.0;
            vmult = vmult* pow(-1,j); // swirliness?
            x[i][j] = rand()%(int)(scale/2) +(scale/4);
            v[i][j] = vmult * x[i][j];
            comx[j] += x[i][j]/n;
            comv[j] += v[i][j]/n;
        }
    }
    // now need to subtract CoM position and velocity
    // from each of the bodies.
    for( i=0; i<n; i++ ){
        for( j=0; j<d; j++ ){
            x[i][j] = x[i][j] - comx[j] + (scale/2);
            v[i][j] -= comv[j];
        }
    }
}

// initializes the coordinates and does first integ. step.
// called at beginning of program and on reset.
void firstStep( double **x, double **xp, double **xn,
                double **v, int n, int d, int w, int h,
                double *a, double dt, double s, int *seed){
    int i,j;
    // assign initial conditions (to xp and v)
    init( xp, v, n, d, w, h, seed ); 

    // we += on this array, so it needs to be zeroed
    for( i=0; i<d; i++ )
        a[i] = 0.0;

    // need to do this for the first integ. step
    // so that we have xp and x both defined
    for( i=0; i<n; i++ ){
        for( j=0; j<n; j++ )
            accelerate ( xp[i], xp[j], a, d, s );
        dxi( x[i], xp[i], v[i], a, dt, d ); // first integ. step is special
        for( j=0; j<d; j++ )
            a[j] = 0.0;
    }
}

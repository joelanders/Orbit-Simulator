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

void putpixels(SDL_Surface *surface, double **x, int w, int h);
void events( SDL_Surface *surface, double **x, double **xp, 
             double **xn, double **v, int w, int h,
             double *a, double dt, double s );
void init( double **x, double **v, int w, int h );
void accelerate( double *x1, double *x2, double *a, double s );
void dxi( double*x, double *xp, double *v, double *a, double dt );
void dx( double *xn, double *x, double *xp, double *a, double dt );
void firstStep( double **x, double **xp, double **xn,
                double **v, int w, int h,
                double *a, double dt, double s );

int seed;
int quit;
int num;
int dim;

int main(int argc, char *argv[]){

    if(argc<7 || argc>8){
        printf("Error. Nbody takes 6 or 7 arguments.\n");
        return -1;
    }
    num = atoi( argv[1] );    // number of bodies
    dim = atoi( argv[2] );    // number of spatial dimensions
    double dt = atof( argv[3] ); // time step
    double s = atof( argv[4] ); //softening radius
    int w = atoi( argv[5] ); //window width
    int h = atoi( argv[6] ); //window height
    if(argc==7){ // if optional seed is omitted,
        seed = time(NULL); // generate one
    }
    else if(argc==8){ // else, use provided seed
        seed = atoi( argv[7] );
    }
    SDL_Surface *screen; 
    int i,j;

    double **x  = malloc(sizeof(double)*num); //init n rows 
    double **xp = malloc(sizeof(double)*num);
    double **xn = malloc(sizeof(double)*num);
    double **v  = malloc(sizeof(double)*num);
    for( i=0; i<num; i++){
        x[i]  = malloc(sizeof(double)*dim); //each row is a dim-length array
        xp[i] = malloc(sizeof(double)*dim); 
        xn[i] = malloc(sizeof(double)*dim); 
        v[i]  = malloc(sizeof(double)*dim);
    }

    double *a = malloc(sizeof(double)*dim); //dim-length array to hold accel.

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
    firstStep( x, xp, xn, v, w, h, a, dt, s );

    // here is the main loop,
    // it continues until quit=1
    while( !quit ){
        events(screen, x, xp, xn, v, w, h, a, dt, s );


        for( i=0; i<num; i++ ){    //finding acceleration of this body
            for( j=0; j<num; j++ ) //due to these bodies,
                accelerate( x[i], x[j], a, s );   //summing it up,
            dx( xn[i], x[i], xp[i], a, dt ); //update xn
            for( j=0; j<dim; j++ )
                a[j] = 0.0;
        }

        putpixels(screen, x, w, h);

        //shift the prev, current, and next pointers for next timestep
        xp = x;  //for next timestep
        x  = xn;
        xn = xp; //overwriting xp

    } 

    SDL_Quit(); 
  
    return 0; 
} 

void putpixels(SDL_Surface *surface, double **x, int w, int h ){
    int i, xi, yi;
    for(i=0; i<num; i++){
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

void events( SDL_Surface *surface, double **x, double **xp, 
             double **xn, double **v, int w, int h,
             double *a, double dt, double s ){

    SDL_Event event;
    char fname[50];
    while( SDL_PollEvent( &event ) ){ 
        if(event.type == SDL_QUIT){
            quit=1;
            break;
        }
        else if(event.type == SDL_KEYUP){
            switch( event.key.keysym.sym ){ // that's the ASCII code
                case 27: // escape
                    quit=1;
                    break;
                case 98: // b
                    SDL_FillRect( surface, NULL, 0 ); //black screen
                    SDL_UpdateRect( surface,0,0,0,0); //and update
                    break;
                case 115: // s
                    printf("%d saved\n", seed);
                    FILE *fp;
                    fp=fopen("saved.txt","a");
                    fprintf(fp,"./nbody %d %d %f %f %d %d %d\n", num, dim, 
                                        dt, s, w, h, seed);
                    fclose(fp);
                    sprintf(fname, "%d-%f-%f.bmp",seed,dt,s);
                    SDL_SaveBMP(surface, fname);
                    break;
                case 114: // r
                    seed = time(NULL);
                    firstStep(x, xp, xp, v, w, h, a, dt, s);
                    SDL_FillRect( surface, NULL, 0 ); //black screen
                    SDL_UpdateRect( surface,0,0,0,0); //and update
                    break;
                default:
                    break;
            }
        }
    }
}

// x1 and x2 are dim-length arrays of position coords
// a is the dim-length acceleration vector
// dim is number of spatial dimensions
// s is the softening radius
void accelerate( double *x1, double *x2, double *a, double s ){
    double dSqr = 0.0, r2, aPrt;
    int i;

    for( i=0; i<dim; i++ ) // d^2 = x1^2 + x2^2 + ... + xd^2
        dSqr += (x1[i] - x2[i])*(x1[i] - x2[i]);

    r2 = dSqr + s*s;
    aPrt = 1.0/sqrt(r2*r2*r2);
    
    for( i=0; i<dim; i++ )
        a[i] += aPrt*(x2[i] - x1[i]);
}

// need to find x1 from x0 and v0
void dxi( double *x, double *xp, double *v, double *a, double dt){
    int i;
    for( i=0; i<dim; i++ )
        x[i] = xp[i] + v[i]*dt + a[i]*dt*dt/2;
    //at this point, xp is initial conds (x0),
    //and x is first step (x1).
}

// verlet algorithm
void dx( double *xn, double *x, double *xp, double *a, double dt ){
    int i;
    for( i=0; i<dim; i++ ){ 
        xn[i] = 2*x[i] - xp[i] + a[i]*dt*dt;
    }
}

// initializes the position and velocity matrices,
// then finds position and velocity of CoM and subtracts
// these coords from each particle.
void init(double **x, double **v, int w, int h){
    int i,j;
    double scale;
    double *comx = malloc(sizeof(double)*dim);
    double *comv = malloc(sizeof(double)*dim);
    for( i=0; i<dim; i++ ){
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

    srand(seed);
    for( i=0; i<num; i++ ){
        for( j=0; j<dim; j++ ){
            vmult = (rand()%10)/10000.0;
            vmult = vmult* pow(-1,j); // swirliness?
            x[i][j] = rand()%(int)(scale/2) +(scale/4);
            v[i][j] = vmult * x[i][j];
            comx[j] += x[i][j]/num;
            comv[j] += v[i][j]/num;
        }
    }
    // now need to subtract CoM position and velocity
    // from each of the bodies.
    for( i=0; i<num; i++ ){
        for( j=0; j<dim; j++ ){
            x[i][j] = x[i][j] - comx[j] + (scale/2);
            v[i][j] -= comv[j];
        }
    }
}

// initializes the coordinates and does first integ. step.
// called at beginning of program and on reset.
void firstStep( double **x, double **xp, double **xn,
                double **v, int w, int h,
                double *a, double dt, double s){
    int i,j;
    // assign initial conditions (to xp and v)
    init( xp, v, w, h );

    // we += on this array, so it needs to be zeroed
    for( i=0; i<dim; i++ )
        a[i] = 0.0;

    // need to do this for the first integ. step
    // so that we have xp and x both defined
    for( i=0; i<num; i++ ){
        for( j=0; j<num; j++ )
            accelerate ( xp[i], xp[j], a, s );
        dxi( x[i], xp[i], v[i], a, dt ); // first integ. step is special
        for( j=0; j<dim; j++ )
            a[j] = 0.0;
    }
}

//-----------------------------------//
//  This file is part of MuJoCo.     //
//  Copyright (C) 2016 Roboti LLC.   //
//-----------------------------------//


#include "mujoco.h"
#include "glfw3.h"
#include "stdlib.h"
#include "string.h"
#include <mutex>

//--------------FOR FILE OPENING
#include <array>
#include <fstream>
#include <string>
#include <sstream>

//---- string printing cout
#include <iostream>




//--------------------------------------------------------------------------------------





//---------------------------------------------------------------------------------------

int number_of_samples = 300;
const int n_rows_initial = 1000;//number of video frames -will be modified later by nrows_list
int n_rows = 1000;
const int n_quats = 20; //number of quaternion elements = dof * 4 = number of joints * 4
const int n_cols_out = 24; // number of columns of output file


std::array<std::array<float, n_quats>, n_rows_initial> qposdata;//i rows, j cols
std::array<std::array<float, n_cols_out>, n_rows_initial> outputdata;//i rows, j cols
std::array<std::array<float, 11>, 1> inputdata_qpos;//i rows, j cols
std::array<std::array<float, 4>, 1000> inputdata_inforces;//3 input forces, 1000 rows
std::ifstream datfile_qpos("qposInit.dat");
std::ifstream datfile_inforces("inforces.dat");

//std::ifstream datfile("KM04_HOP_12_QUATS.dat");
//----FOR IK
double ikoutput[] = {0, 0};
double hip_xzInit[] = {0, 0};
double knee_xzInit[] = {0, 0};
double ankle_xzInit[] = {0, 0};
double snout_xzInit[] = {0, 0};
double qk_targ = 0;
double jump_dist = 0;
double time_landing = 0;
double jump_pos[] = {0, 0, 0};
double jump_posInit[] = {0, 0, 0};
bool is_takeoff = true;
double qa_takeoff = 0;
std::array<double, 500> qbuffer;

//-------------------------------- global variables -------------------------------------

// model
char error[1000];
mjModel* m = mj_loadXML("3SegRMdevel01.xml", NULL, error, 1000);
mjData* d = 0;
char lastfile[1000] = "";


// IO data files

char const *outfile= "XYZdata.txt";
std::string const quatsfilelist_L[] = {"FrogKinematicsDemo_QUATS.dat"};


//----ADJUST THESE PARAMETERS depending on data you are loading in
int n_trials = 1; // number of trials to load
int const nrows_list[] = {58};// number of rows of time points for each trial


int counter = 0;

// user state
bool paused = false;
bool showoption = false;
bool showinfo = true;
bool showfullscreen = false;
bool slowmotion = false;
bool showdepth = false;
int showhelp = 1;                   // 0: none; 1: brief; 2: full

// abstract visualization
mjvScene scn;
mjvCamera cam;
mjvOption vopt;
mjvPerturb pert;
char status[1000] = "";

// OpenGL rendering
int refreshrate;
const int fontscale = mjFONTSCALE_150;
mjrContext con;
float depth_buffer[5120*2880];        // big enough for 5K screen
unsigned char depth_rgb[1280*720*3];  // 1/4th of screen

// selection and perturbation
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;
int needselect = 0;                 // 0: none, 1: select, 2: center, 3: center and track 
double window2buffer = 1;           // framebuffersize / windowsize (for scaled video modes)

// help strings
const char help_title[] = 
"Help\n"
"Option\n"
"Info\n"
"Depth\n"
"Full screen\n"
"Stereo\n"
"Slow motion\n"
"Pause\n"
"Reset\n"
"Forward\n"
"Back\n"
"Forward 100\n"
"Back 100\n"
"Autoscale\n"
"Reload\n"
"Geoms\n"
"Sites\n"
"Select\n"
"Center\n"
"Track\n"
"Zoom\n"
"Translate\n"
"Rotate\n"
"Perturb\n"
"Free Camera\n"
"Camera\n"
"Frame\n"
"Label";


const char help_content[] = 
"F1\n"
"F2\n"
"F3\n"
"F4\n"
"F5\n"
"F6\n"
"Enter\n"
"Space\n"
"BackSpace\n"
"Right arrow\n"
"Left arrow\n"
"Down arrow\n"
"Up arrow\n"
"Ctrl A\n"
"Ctrl L\n"
"0 - 4\n"
"Shift 0 - 4\n"
"L dblclick\n"
"R dblclick\n"
"Ctrl R dblclick\n"
"Scroll or M drag\n"
"[Shift] R drag\n"
"L drag\n"
"Ctrl [Shift] L/R drag\n"
"Esc\n"
"[ ]\n"
"; '\n"
". /";

char opt_title[1000] = "";
char opt_content[1000];
//-------------------------------------------------------------------------------------
void ik2dof(double *output, double length1, double length2, double targetX, double targetY, bool whichangle)
{

    double angle1;   // Angle of bone 1
    double angle2;   // Angle of bone 2
    bool solvePosAngle2 = whichangle; // Solve for positive angle 2 instead of negative angle 2
    //double length1;      // Length of bone 1. Assumed to be >= zero
    //double length2;      // Length of bone 2. Assumed to be >= zero
    //double targetX;     // Target x position for the bones to reach
    //double targetY;      // Target y position for the bones to reach


    //Debug.Assert(length1 >= 0);
    //Debug.Assert(length2 >= 0);
  
    const double epsilon = 0.0001; // used to prevent division by small numbers
  
    bool foundValidSolution = true;
  
    double targetDistSqr = (targetX*targetX + targetY*targetY);
  
    //===
    // Compute a new value for angle2 along with its cosine
    double sinAngle2;
    double cosAngle2;
  
    double cosAngle2_denom = 2*length1*length2;
    if ( cosAngle2_denom > epsilon ) 
    {
        cosAngle2 = (targetDistSqr - length1*length1 - length2*length2) / (cosAngle2_denom);
  
        // if our result is not in the legal cosine range, we can not find a
        // legal solution for the target
        if( (cosAngle2 < -1.0) || (cosAngle2 > 1.0) )
            foundValidSolution = false;
  
        // clamp our value into range so we can calculate the best
        // solution when there are no valid ones
        cosAngle2 = fmax(-1, fmin( 1, cosAngle2 ) );//CHECK!!!!!
  
        // compute a new value for angle2
        angle2 = acos( cosAngle2 );
  
        // adjust for the desired bend direction
        if( !solvePosAngle2 )
            angle2 = -angle2;
  
        // compute the sine of our angle
        sinAngle2 = sin( angle2 );
    }
    else
    {
        // At least one of the bones had a zero length. This means our
        // solvable domain is a circle around the origin with a radius
        // equal to the sum of our bone lengths.
        double totalLenSqr = (length1 + length2) * (length1 + length2);
        if(    targetDistSqr < (totalLenSqr-epsilon)
            || targetDistSqr > (totalLenSqr+epsilon) )
        {
            foundValidSolution = false;
        }
  
        // Only the value of angle1 matters at this point. We can just
        // set angle2 to zero. 
        angle2    = 0.0;
        cosAngle2 = 1.0;
        sinAngle2 = 0.0;
    }
  
    //===
    // Compute the value of angle1 based on the sine and cosine of angle2
    double triAdjacent = length1 + length2*cosAngle2;
    double triOpposite = length2*sinAngle2;
  
    double tanY = targetY*triAdjacent - targetX*triOpposite;
    double tanX = targetX*triAdjacent + targetY*triOpposite;
  
    // Note that it is safe to call Atan2(0,0) which will happen if targetX and
    // targetY are zero
    angle1 = atan2( tanY, tanX );
    //double output[] = {angle1, angle2};
    output[0] = angle1;
    output[1] = angle2;
  
    //return output;

}




//-------------------------------- utility functions ------------------------------------

// center and scale view
void autoscale(GLFWwindow* window)
{

    // autoscale
    cam.lookat[0] = m->stat.center[0];
    cam.lookat[1] = m->stat.center[1];
    cam.lookat[2] = m->stat.center[2];
    cam.distance = 1 * m->stat.extent;


    // set to free camera
    cam.type = mjCAMERA_FREE;
}

// load mjb or xml model
void loadmodel(GLFWwindow* window, const char* filename, const char* xmlstring)
{
    // make sure one source is given
    if( !filename && !xmlstring )
        return;
    filename = "3SegRMdevel01.xml";
    // load and compile
    //char error[1000] = "could not load binary model, you silly person";
    mjModel* mnew = 0;
    //mnew = mj_loadXML("hello.xml", 0, error, 1000);
    if( xmlstring )
       mnew = mj_loadXML(0, xmlstring, error, 1000);
    else if( strlen(filename)>4 && !strcmp(filename+strlen(filename)-4, ".mjb") )
       mnew = mj_loadModel(filename, 0, 0);
    else
      mnew = mj_loadXML(filename, 0, error, 1000);
    if( !mnew )
    
    {
        printf("%s\n", error);
        return;
    }

    // delete old model, assign new
    mj_deleteData(d);
    mj_deleteModel(m);
    m = mnew;
    //m = mj_loadXML("hello.xml", NULL, error, 1000);
    d = mj_makeData(m);
    //mj_forward(m, d);

    // save filename for reload
    if( !xmlstring )
        strcpy(lastfile, filename);
    else
        lastfile[0] = 0;

    // re-create custom context
    mjr_makeContext(m, &con, fontscale);

    // clear perturbation state
    pert.active = 0;
    pert.select = 0;
    needselect = 0;

    // center and scale view, update scene
    autoscale(window);
    mjv_updateScene(m, d, &vopt, &pert, &cam, mjCAT_ALL, &scn);

    // set window title to mode name
    if( window && m->names )
        glfwSetWindowTitle(window, m->names);
}

//--------------------------------- GLFW callbacks --------------------------------------

// keyboard
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    int n;

    // require model
    if( !m )
        return;

    // do not act on release
    if( act==GLFW_RELEASE )
        return;

    switch( key )
    {
    case GLFW_KEY_F1:                   // help
        showhelp++;
        if( showhelp>2 )
            showhelp = 0;
        break;

    case GLFW_KEY_F2:                   // option
        showoption = !showoption;
        break;

    case GLFW_KEY_F3:                   // info
        showinfo = !showinfo;
        break;

    case GLFW_KEY_F4:                   // depth
        showdepth = !showdepth;
        break;

    case GLFW_KEY_F5:                   // toggle full screen
        showfullscreen = !showfullscreen;
        if( showfullscreen )
            glfwMaximizeWindow(window);
        else
            glfwRestoreWindow(window);
        break;

    case GLFW_KEY_F6:                   // stereo
        scn.stereo = (scn.stereo==mjSTEREO_NONE ? mjSTEREO_QUADBUFFERED : mjSTEREO_NONE);
        break;

    case GLFW_KEY_ENTER:                // slow motion
        slowmotion = !slowmotion;
        break;

    case GLFW_KEY_SPACE:                // pause
        paused = !paused;
        break;

    case GLFW_KEY_BACKSPACE:            // reset
        mj_resetData(m, d);
        mj_forward(m, d);
        break;

    case GLFW_KEY_RIGHT:                // step forward
        if( paused )
            mj_step(m, d);
        break;

    case GLFW_KEY_LEFT:                 // step back
        if( paused )
        {
            m->opt.timestep = -m->opt.timestep;
            mj_step(m, d);
            m->opt.timestep = -m->opt.timestep;
        }
        break;

    case GLFW_KEY_DOWN:                 // step forward 100
        if( paused )
            for( n=0; n<100; n++ )
                mj_step(m,d);
        break;

    case GLFW_KEY_UP:                   // step back 100
        if( paused )
        {
            m->opt.timestep = -m->opt.timestep;
            for( n=0; n<100; n++ )
                mj_step(m,d);
            m->opt.timestep = -m->opt.timestep;
        }
        break;

    case GLFW_KEY_ESCAPE:               // free camera
        cam.type = mjCAMERA_FREE;
        break;

    case '[':                           // previous fixed camera or free
        if( m->ncam && cam.type==mjCAMERA_FIXED )
        {
            if( cam.fixedcamid>0 )
                cam.fixedcamid--;
            else
                cam.type = mjCAMERA_FREE;
        }
        break;

    case ']':                           // next fixed camera
        if( m->ncam )
        {
            if( cam.type!=mjCAMERA_FIXED )
            {
                cam.type = mjCAMERA_FIXED;
                cam.fixedcamid = 0;
            }
            else if( cam.fixedcamid<m->ncam-1 )
                cam.fixedcamid++;
        }
        break;

    case ';':                           // cycle over frame rendering modes
        vopt.frame = mjMAX(0, vopt.frame-1);
        break;

    case '\'':                          // cycle over frame rendering modes
        vopt.frame = mjMIN(mjNFRAME-1, vopt.frame+1);
        break;

    case '.':                           // cycle over label rendering modes
        vopt.label = mjMAX(0, vopt.label-1);
        break;

    case '/':                           // cycle over label rendering modes
        vopt.label = mjMIN(mjNLABEL-1, vopt.label+1);
        break;

    default:                            // toggle flag
        // control keys
        if( mods & GLFW_MOD_CONTROL )
        {
            if( key==GLFW_KEY_A )
                autoscale(window);
            else if( key==GLFW_KEY_L && lastfile[0] )
                loadmodel(window, lastfile, 0);

            break;
        }

        // toggle visualization flag
        for( int i=0; i<mjNVISFLAG; i++ )
            if( key==mjVISSTRING[i][2][0] )
                vopt.flags[i] = !vopt.flags[i];

        // toggle rendering flag
        for( int i=0; i<mjNRNDFLAG; i++ )
            if( key==mjRNDSTRING[i][2][0] )
                scn.flags[i] = !scn.flags[i];

        // toggle geom/site group
        for( int i=0; i<mjNGROUP; i++ )
            if( key==i+'0')
            {
                if( mods & GLFW_MOD_SHIFT )
                    vopt.sitegroup[i] = !vopt.sitegroup[i];
                else
                    vopt.geomgroup[i] = !vopt.geomgroup[i];
            }
    }
}


// mouse button
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // past data for double-click detection
    static int lastbutton = 0;
    static double lastclicktm = 0;

    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);

    // require model
    if( !m )
        return;

    // set perturbation
    int newperturb = 0;
    if( act==GLFW_PRESS && (mods & GLFW_MOD_CONTROL) && pert.select>0 )
    {
        // right: translate;  left: rotate
        if( button_right )
            newperturb = mjPERT_TRANSLATE;
        else if( button_left )
            newperturb = mjPERT_ROTATE;

        // perturbation onset: reset reference
        if( newperturb && !pert.active )
            mjv_initPerturb(m, d, &scn, &pert);
    }
    pert.active = newperturb;

    // detect double-click (250 msec)
    if( act==GLFW_PRESS && glfwGetTime()-lastclicktm<0.25 && button==lastbutton )
    {
        if( button==GLFW_MOUSE_BUTTON_LEFT )
            needselect = 1;
        else if( mods & GLFW_MOD_CONTROL )
            needselect = 3;
        else
            needselect = 2;

        // stop perturbation on select
        pert.active = 0;
    }

    // save info
    if( act==GLFW_PRESS )
    {
        lastbutton = button;
        lastclicktm = glfwGetTime();
    }
}


// mouse move
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // require model
    if( !m )
        return;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move perturb or camera
    if( pert.active )
        mjv_movePerturb(m, d, action, dx/height, dy/height, &scn, &pert);
    else
        mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // require model
    if( !m )
        return;

    // scroll: emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


// drop
void drop(GLFWwindow* window, int count, const char** paths)
{
    // make sure list is non-empty
    if( count>0 )
        loadmodel(window, paths[0], 0);
}

//-------------------------------- simulation and rendering -----------------------------

// make option string
void makeoptionstring(const char* name, char key, char* buf)
{
    int i=0, cnt=0;

    // copy non-& characters
    while( name[i] && i<50 )
    {
        if( name[i]!='&' )
            buf[cnt++] = name[i];

        i++;
    }

    // finish
    buf[cnt] = ' ';
    buf[cnt+1] = '(';
    buf[cnt+2] = key;
    buf[cnt+3] = ')';
    buf[cnt+4] = 0;
}




//  INPUT  mocap KINEMATICS
void mopos(void)
{

    int fr = (int)(counter / 1);
    float tt = d->time;
    float mocapXYZ[3] = {-0.003, 0, 0.008277};
    mocapXYZ[0] = mocapXYZ[0] + 0.01*tt;
    mocapXYZ[2] = mocapXYZ[2] + 0.01*tt;
    for (int i = 0; i < 3; i++) {
        d->mocap_pos[i] = mocapXYZ[i];
    }

}

void ik(void)
{
    double snout_xz[] = {d->geom_xpos[30], d->geom_xpos[32]};//does not exist at the moment!!!!!!!
    double hip_xz[] = {d->xpos[15], d->xpos[17]}; 
    double knee_xz[] = {d->xpos[12], d->xpos[14]}; 
    double ankle_xz[] = {d->xpos[9], d->xpos[11]}; 
    double qt = 2 * acos(d->qpos[3]);// tmt angle
    double qh = d->qpos[9] + m->qpos0[9]; //hip angle
    double qk = d->qpos[8] + m->qpos0[8]; //knee angle
    double qa = d->qpos[7] + m->qpos0[7]; //ankle angle



    if (counter == 0) {
        hip_xzInit[0] = hip_xz[0];
        hip_xzInit[1] = hip_xz[1];
        knee_xzInit[0] = knee_xz[0];
        knee_xzInit[1] = knee_xz[1];
        ankle_xzInit[0] = ankle_xz[0];
        ankle_xzInit[1] = ankle_xz[1];
        snout_xzInit[0] = snout_xz[0];
        snout_xzInit[1] = snout_xz[1];

    }
    //double targ[] = {0.014769 - ankle_xz[0], 0.020092 - ankle_xz[1]}; // target x z position
    double targ[] = {hip_xzInit[0], hip_xzInit[1] + 0.01*(d->time)}; // target x z position
    //double targ[] = {0.0012, 0.0278};

    double targ1[] = {targ[0] * cos(-qt) - targ[1] * sin(-qt), targ[1] * cos(-qt) + targ[0] * sin(-qt)};
    double targ2[] = {targ1[0] * cos(-qa) - targ1[1] * sin(-qa), targ1[1] * cos(-qa) + targ1[0] * sin(-qa)};
    d->mocap_pos[0] = hip_xz[0];
    d->mocap_pos[2] = hip_xz[1];


    
    ik2dof(ikoutput, 0.01779120, 0.0197439, targ2[0], targ2[1], true);
    ikoutput[0] = 3.14 - ikoutput[0] + 0*m->qpos0[8];
    ikoutput[1] = 3.14 - ikoutput[1] + 0*m->qpos0[9];

    //---- make local axis from tibia bone ---
    double localAx[] = {0, 0, 0};

    localAx[0] = knee_xz[0] - ankle_xz[0];
    localAx[2] = knee_xz[1] - ankle_xz[1];
    mju_normalize3(localAx);

    double targVec[] = {0, 0, 0}; // vector between knee and target point

    targVec[0] = targ[0] - knee_xz[0];
    targVec[2] = targ[1] - knee_xz[1];
    mju_normalize3(targVec);

    qk_targ = 3.141 - acos(mju_dot3(localAx, targVec)); //target knee angle
    


    if (counter == 1) {
       //printf("target x y %f   %f\n", snout_xz[0], snout_xz[1]);
        int ibody = 2;
        printf("z axis orientation %f  %f  %f\n", d->xmat[9 * ibody + 6], d->xmat[9 * ibody + 7], d->xmat[9 * ibody + 8]);
        printf("body coordinates %f  %f  %f\n", d->xpos[3 * (ibody + 1) + 0], d->xpos[3 * (ibody + 1) + 1], d->xpos[3 * (ibody + 1) + 2]);
        //printf("vec math %f  %f  %f\n", targVec[0], targVec[1], targVec[2]);
        //printf("target knee angle %f, knee agnle %f\n", qk_targ, d->qpos[8]);

    }
    

    

    
}

void setcontrol(mjtNum time, mjtNum* ctrl, int nu)
{
    if (counter > 0) {
    int switches[3] = {0, 0, 0};
    //printf("%f   %f\n", 3.14 - ikoutput[0], 3.14 - ikoutput[1]);
    if (is_takeoff) {
        ctrl[0] = inputdata_inforces[counter][0];//ankle extensor force
        //qbuffer[counter] = d->qpos[8];// store previous ankle angle values
        //qa_takeoff = qbuffer[counter - 1];
    }
    else {
            d->ctrl[0] = 0;
        
    }
    //ctrl[0] = inputdata_inforces[counter][0];//ankle extensor force
    ctrl[1] = inputdata_inforces[counter][1];//knee servo position
    ctrl[2] = inputdata_inforces[counter][2];//hip extensor force
    
    // --- apply force to COM from forelimbs ---
    //d->xfrc_applied[6*6 + 2] = inputdata_inforces[counter][3]; // body 6 (from 0) is com
    }
    //printf("%f\n", d->qpos[8]);

}

// simulation
void advance(void)
{
    //mj_forward(m, d);
    // quatpos();// function to input kinematics data into qpos, qvel, qacc
    //mopos();
    ik();
    setcontrol(d->time, d->ctrl, m->nu);
    mj_step(m, d);
    // mj_inverse(m, d);
    //printf("%f\n", d->xpos[6]);
    counter ++;
    usleep(10000);
    double camdx = 0;
    double camdy = -0.1;
    if (counter == 1) {
        mjv_moveCamera(m, 2, camdx, camdy, &scn, &cam);
        cam.distance = 1.5 * m->stat.extent;
    }
    

    
    if (counter == 1) {
        jump_posInit[0] = d->mocap_pos[0];
        jump_posInit[2] = d->mocap_pos[2];

    }
    jump_pos[0] = d->mocap_pos[0];
    jump_pos[2] = d->mocap_pos[2];
    jump_dist = mju_dist3(jump_posInit, jump_pos);

    // Landing detect

    double grf_z = d->qfrc_constraint[2]; // z component of ground reaction force
    //printf("%f\n", grf_z);
    if (grf_z == 0) {
        //printf("%f\n", d->time);
        jump_dist = mju_dist3(jump_posInit, jump_pos);
        time_landing = d->time;
        is_takeoff = false;
        
    }
    else {
        jump_dist = jump_dist;
        time_landing = time_landing;
    }
    if (counter == number_of_samples) {
        printf("total jump distance = %f at landing time %f \n", jump_dist, time_landing);
        
    }
    for (int i = 0; i < (m->nbody)*3; i++) {
        outputdata[counter][i] = d->xpos[i];
    }
    
    //printf("%f\n", d->ten_moment[7]);


    mj_tendon(m, d);

    mj_transmission(m, d);
    // if (counter == 2) {
    //     //std::string input = "";
    //     //std::cout << "Please press <ENTER> to start simulation:\n>";
    //     //getline(std::cin, input);
    //     usleep(800000);
    // }
    // if (counter == number_of_samples) {
    //     printf("total jump distance = %f in %i iterations\n", jump_dist, number_of_samples);
    // }



}

// render
void render(GLFWwindow* window)
{
    // past data for FPS calculation
    static double lastrendertm = 0;

    // get current framebuffer rectangle
    mjrRect rect = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &rect.width, &rect.height);

    // no model: empty screen
    if( !m )
    {
        mjr_rectangle(rect, 0.2f, 0.3f, 0.4f, 1);
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, "Drag-and-drop model file here", 0, &con);

        // swap buffers
        glfwSwapBuffers(window); 
        return;
    }

    // advance simulation
    advance();

    // update simulation statistics
    if( !paused )
    {
        // camera string
        char camstr[20];
        if( cam.type==mjCAMERA_FREE )
            strcpy(camstr, "Free");
        else if( cam.type==mjCAMERA_TRACKING )
            strcpy(camstr, "Tracking");
        else
            sprintf(camstr, "Fixed %d", cam.fixedcamid);

        // status
        sprintf(status, "%f\n%f (%d)\n%.0f\n%.2f\n%.2f (%02d it)\n%.1f %.1f\n%s\n%s\n%s",
                d->time, 
                jump_dist,
                //d->nefc, 
                d->ncon, 
                1.0/(glfwGetTime()-lastrendertm),
                d->energy[0]+d->energy[1],
                mju_log10(mju_max(mjMINVAL, d->solver_trace[mjMAX(0,mjMIN(d->solver_iter-1,mjNTRACE-1))])),
                d->solver_iter, 
                mju_log10(mju_max(mjMINVAL,d->solver_fwdinv[0])),
                mju_log10(mju_max(mjMINVAL,d->solver_fwdinv[1])),
                camstr, 
                mjFRAMESTRING[vopt.frame], 
                mjLABELSTRING[vopt.label] );
    }

    // timing satistics
    lastrendertm = glfwGetTime();

    // update scene
    mjv_updateScene(m, d, &vopt, &pert, &cam, mjCAT_ALL, &scn);

    // selection
    if( needselect )
    {
        // find selected geom
        mjtNum pos[3];
        int selgeom = mjr_select(rect, &scn, &con, 
            (int)(window2buffer*lastx), (int)(rect.height-window2buffer*lasty), pos, NULL);

        // find corresponding body if any
        int selbody = 0;
        if( selgeom>=0 && selgeom<scn.ngeom && scn.geoms[selgeom].objtype==mjOBJ_GEOM )
        {
            selbody = m->geom_bodyid[scn.geoms[selgeom].objid];
            if( selbody<0 || selbody>=m->nbody )
                selbody = 0;
        }

        // set lookat point, start tracking is requested
        if( needselect==2 || needselect==3 )
        {
            if( selgeom>=0 )
                mju_copy3(cam.lookat, pos);

            // switch to tracking camera
            if( needselect==3 && selbody )
            {
                cam.type = mjCAMERA_TRACKING;
                cam.trackbodyid = selbody;
                cam.fixedcamid = -1;
            }
        }

        // set body selection
        else
        {
            if( selbody )
            {
                // record selection
                pert.select = selbody;

                // compute localpos
                mjtNum tmp[3];
                mju_sub3(tmp, pos, d->xpos+3*pert.select);
                mju_mulMatTVec(pert.localpos, d->xmat+9*pert.select, tmp, 3, 3);
            }
            else
                pert.select = 0;
        }

        needselect = 0;
    }

    // render
    mjr_render(rect, &scn, &con);

    // show depth map
    if( showdepth )
    {
        // get the depth buffer
        mjr_readPixels(NULL, depth_buffer, rect, &con);

        // convert to RGB, subsample by 4
        for( int r=0; r<rect.height; r+=4 )
            for( int c=0; c<rect.width; c+=4 )
            {
                // get subsampled address
                int adr = (r/4)*(rect.width/4) + c/4;

                // assign rgb
                depth_rgb[3*adr] = depth_rgb[3*adr+1] = depth_rgb[3*adr+2] = 
                    (unsigned char)((1.0f-depth_buffer[r*rect.width+c])*255.0f);
            }

        // show in bottom-right corner
        mjrRect bottomright = {(3*rect.width)/4, 0,  rect.width/4, rect.height/4};
        mjr_drawPixels(depth_rgb, NULL, bottomright, &con);
    }

    // show overlays
    if( showhelp==1 )
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, "Help  ", "F1  ", &con);
    else if( showhelp==2 )
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, help_title, help_content, &con);

    // show info
    if( showinfo )
    {
        if( paused )
            mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, rect, "PAUSED", 0, &con);
        else
            mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, rect, 
                "Time\nJump Dist\nFPS\nEnergy\nSolver\nFwdInv\nCamera\nFrame\nLabel", status, &con);
    }

    // show options
    if( showoption )
    {
        int i;
        char buf[100];

        // fill titles on first pass
        if( !opt_title[0] )
        {
            for( i=0; i<mjNRNDFLAG; i++)
            {
                makeoptionstring(mjRNDSTRING[i][0], mjRNDSTRING[i][2][0], buf);
                strcat(opt_title, buf);
                strcat(opt_title, "\n");
            }
            for( i=0; i<mjNVISFLAG; i++)
            {
                makeoptionstring(mjVISSTRING[i][0], mjVISSTRING[i][2][0], buf);
                strcat(opt_title, buf);
                if( i<mjNVISFLAG-1 )
                    strcat(opt_title, "\n");
            }
        }

        // fill content
        opt_content[0] = 0;
        for( i=0; i<mjNRNDFLAG; i++)
        {
            strcat(opt_content, scn.flags[i] ? " + " : "   ");
            strcat(opt_content, "\n");
        }
        for( i=0; i<mjNVISFLAG; i++)
        {
            strcat(opt_content, vopt.flags[i] ? " + " : "   ");
            if( i<mjNVISFLAG-1 )
                strcat(opt_content, "\n");
        }

        // show
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, rect, opt_title, opt_content, &con);
    }

    // swap buffers
    glfwSwapBuffers(window); 
}

//-------------------------------- main function ----------------------------------------

int main(int argc, const char** argv)
{
  //printf("output data size = %lu\n", sizeof(outputdata)/sizeof(outputdata[0]));
  // print version, check compatibility

    // Get number of time samples
    number_of_samples = 0;
    std::string line;
    while (std::getline(datfile_inforces, line)) {
        ++number_of_samples;
    }
    datfile_inforces.close ();
    std::ifstream datfile_inforces("inforces.dat");

    for (int i = 0; i <= 1; i++) {
        for (int j = 0; j < 11; j++) {
            datfile_qpos >> inputdata_qpos[i][j];
        }
    }

    // inforces is ankle actuator force, knee servo position, hip servo position, forelimb force
    for (int i = 0; i < number_of_samples; i++) {
        for (int j = 0; j < 4; j++) {
            datfile_inforces >> inputdata_inforces[i][j];
        }
    }

    datfile_inforces.close ();



        





  printf("MuJoCo Pro library version %.2lf\n\n", 0.01*mj_version());
  if( mjVERSION_HEADER!=mj_version() )
      mju_error("Headers and library have different versions");

  // activate MuJoCo license
  mj_activate("mjkey.txt");
  printf("atan test %f\n", 2.0);
  
  // install control callback
  //mjcb_control = mycontroller;

  // init GLFW, set multisampling
  if (!glfwInit())
      return 1;
  glfwWindowHint(GLFW_SAMPLES, 4);

  // try stereo if refresh rate is at least 100Hz
  GLFWwindow* window = 0;
  if( refreshrate>=100 )
  {
      glfwWindowHint(GLFW_STEREO, 1);
      window = glfwCreateWindow(1200, 900, "Simulate", NULL, NULL);
  }

  // no stereo: try mono
  if( !window )
  {
      glfwWindowHint(GLFW_STEREO, 0);
      window = glfwCreateWindow(1200, 900, "Simulate", NULL, NULL);
  }
  if( !window )
  {
      glfwTerminate();
      return 1;
  }

  // make context current, request v-sync on swapbuffers
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // save window-to-framebuffer pixel scaling (needed for OSX scaling)
  int width, width1, height;
  glfwGetWindowSize(window, &width, &height);
  glfwGetFramebufferSize(window, &width1, &height);
  window2buffer = (double)width1 / (double)width;

  // init MuJoCo rendering, get OpenGL info
  mjv_makeScene(&scn, 1000);
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&vopt);
  mjr_defaultContext(&con);
  mjr_makeContext(m, &con, fontscale);

  // set GLFW callbacks
  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);
  glfwSetDropCallback(window, drop);
  glfwSetWindowRefreshCallback(window, render);

  loadmodel(window, "nothing.nothing", 0);
  //mju_copy(d->qpos, m->key_qpos, m->nq*1);
  

    for (int i = 0; i < m->nq; i++) {
        m->qpos0[i] = inputdata_qpos[0][i];
    }   
    //now account for free joint
    for (int i = 0; i < 7; i++) {
        d->qpos[i] = inputdata_qpos[0][i];
    }
    printf("pos ref\n");
    for (int i = 0; i < m->nq; i++) {
        printf("%f\n", m->qpos0[i]);
    }

    for (int i = 0; i < m->nq; i++) {
        m->qpos_spring[i] = m->qpos0[i];
    }

    printf("spring ref\n");
    for (int i = 0; i < m->nq; i++) {
        printf("%f\n", m->qpos_spring[i]);
    }
    advance();


  // main loop
  for (int itrial = 0; itrial < n_trials; itrial++ ) {//----------INSERT LOOP HERE FOR LOOPING THROUGH MULTIPLE QUAT FILES 
      printf("nbod %i\n", m->nbody);
      printf("nq = %i\n", m->nq);
      printf("nv = %i\n", m->nv);
      printf("nu = %i\n", m->nu);
      printf("ngeom = %i\n", m->ngeom);
      printf("n iterations = %i\n", number_of_samples);


      
      n_rows = nrows_list[itrial];



      //-------LOAD DATA for left leg
      std::ifstream datfile_L(quatsfilelist_L[itrial]);
      for (int i = 0; i < n_rows; i ++) {
          for (int j = 0; j < n_quats; j ++) {
             datfile_L >> qposdata[i][j];
          }   
      }


      counter = 0;
      while( !glfwWindowShouldClose(window) && counter < number_of_samples) //10 * n_rows
          {
          // simulate and render
          // run simulation for 10 seconds
          //mj_step(m, d);
          render(window);

          // finalize
          //glfwSwapBuffers(window);
          glfwPollEvents();
      }
      


        FILE *f = fopen(outfile, "wt");//a for append, t for text mode
        //fwrite(arrayout, sizeof(char), sizeof(arrayout), f);

        for(int i = 0; i < number_of_samples; i++) {
            for (int j = 0; j < (m->nbody) * 3; j++ ) {
                if (j == (m->nbody) * 3 - 1) {
                    fprintf(f, "%f", outputdata[i][j]);//without delimiter
                }
                else {
                    fprintf(f, "%f\t", outputdata[i][j]);//with tab
                    //fprintf(f, "%f,", 1E8 * outputdata[i][j]);//with comma
                }
            }
            fprintf(f, "\n");
        }
        fclose(f);

    } //----------------------------------------------------------------itr trial loop



    
    // delete everything we allocated

    mj_deleteData(d);
    mj_deleteModel(m);
    mjr_freeContext(&con);
    mjv_freeScene(&scn);

    // terminate
    glfwTerminate();
    mj_deactivate();
    

    return 0;
    
}
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <math.h>
#include <ctime>
#include <unistd.h>
#include <sys/time.h>
#include <SDL_video.h>
#include <dirent.h>
#include <string>
#include <sys/types.h>
#include <stdlib.h>

#include "GL/glew.h"
#include "SDL2/SDL.h"

// tests
struct vertix{
    float x;
    float y;
    float z;
};

//#define TEST
#define INTERFACE
SDL_DisplayMode current;

static int interface_x = 0;
static int interface_y = 0;
static int gl_x = 0;
static int gl_y = 0;
static float mouseMoveX = 0.0f;
static float mouseMoveY = 0.0f;
static const int height = 360;
static const int width  = 360;
static const int length  = 360;
static const int inter  = 2;

static std::vector<int> robotScheme(((height - 1)/inter)*((width - 1)/inter), 0);
static std::vector<vertix> scheme;
static vertix center;
static std::vector<std::string> schemeList;
static int schemeId = 0;
static bool schemeChange = false;

static float initialPos[2] = { 0.0f, 0.0f };
static float robotPos[] = { 0.0f, 0.0f, length };
static int robot_up    = 0;
static int robot_down  = 0;
static int robot_left  = 0;
static int robot_right = 0;

static float g_inertia   = 0.5f;
static float oldCamPos[] = { -150.0f, -(height + width)/16, -4.0f*length };
static float oldCamRot[] = { 30.0f, 320.0f, 0.0f };
static float newCamPos[] = { -150.0f, -(height + width)/16, -4.0f*length };
static float newCamRot[] = { 30.0f, 320.0f, 0.0f };

static bool fullscreen      = false;
static bool simu_show       = true;
static bool interface_show  = true;
static bool simu_pause      = false;


void InitRobot() {

    center.x = height / (2*inter) - 1;
    center.y = 0.0f;
    center.z = width / (2*inter) - 1;
    std::cout << "Center X: " << center.x << " | Y: " << center.z << std::endl;

    srand((unsigned int)time(NULL));
    if ((rand() % 2) == 1) {
        std::cout << "Color: 1" << std::endl;
        initialPos[0] = 2600;
        initialPos[1] = 400;
    } else {
        std::cout << "Color: 0" << std::endl;
        initialPos[0] = 400;
        initialPos[1] = 400;
    }
    robotPos[0] = initialPos[0];
    robotPos[1] = initialPos[1];
}

void FindFiles() {

    DIR* directory;
    struct dirent* readFile = NULL;
    std::string str1 = ".", str2 = "..", name;

    directory = opendir( "../../config/schemes" );
    std::cout << "Lecture du Dossier: |" << "src/config/schemes" << "|" << std::endl;
    while ((readFile = readdir(directory)) != NULL) {
        if (str1.compare(readFile->d_name) && str2.compare(readFile->d_name)) {
            name = readFile->d_name;
            schemeList.push_back(name);
        }
    }
    closedir(directory);

    std::cout << "Scheme list size: " << schemeList.size() << std::endl;
    for (int i = 0; i < schemeList.size() ; i++) {
        std::cout << "Scheme Name " << i << ": " << schemeList[i] << std::endl;
    }
}

void CheckMatrix( int i ) {

    int cpt = 0;
    bool angle = false;
    if (robotScheme[i - ((height - 1)/inter) - 1]) {
        cpt++;
        if (!robotScheme[i + ((height - 1)/inter) + 1])
            angle = true;
    }
    if (robotScheme[i - ((height - 1)/inter)]) {
        cpt++;
        if (!robotScheme[i + ((height - 1)/inter)])
            angle = true;
    }
    if (robotScheme[i - ((height - 1)/inter) + 1]) {
        cpt++;
        if (!robotScheme[i + ((height - 1)/inter) - 1])
            angle = true;
    }
    if (robotScheme[i - 1]) {
        cpt++;
        if (!robotScheme[i + 1])
            angle = true;
    }
    if (robotScheme[i + 1]) {
        cpt++;
        if (!robotScheme[i - 1])
            angle = true;
    }
    if (robotScheme[i + ((height - 1)/inter) - 1]) {
        cpt++;
        if (!robotScheme[i - ((height - 1)/inter) + 1])
            angle = true;
    }
    if (robotScheme[i + ((height - 1)/inter)]) {
        cpt++;
        if (!robotScheme[i - ((height - 1)/inter)])
            angle = true;
    }
    if (robotScheme[i + ((height - 1)/inter) + 1]) {
        cpt++;
        if (!robotScheme[i - ((height - 1)/inter) - 1])
            angle = true;
    }

    if ((cpt == 2) && angle) {
        vertix point;
        int value = ((height - 1)/inter);
        int x = i / value;
        int z = i - (value*x);
//        std::cout << "Position: X = " << x << " | Z = " << z << " | I = " << i << std::endl;
        point.x = ((center.x + 1) - x)*inter;
        point.y = 4.0f;
        point.z = (z - center.z)*inter;
//        std::cout << "Center: X = " << center.x << " | Z = " << center.z << std::endl;
//        std::cout << "Point: X = " << point.x << " | Z = " << point.z << std::endl;
        scheme.push_back(point);
    }
}

void LoadStructure() {

    scheme.clear();
    for (int i = 0; i < robotScheme.size(); i++) {
        if (robotScheme[i]) {
            CheckMatrix(i);
        }
    }
    std::cout << "Structure points number: " << scheme.size() << std::endl;
    for (int i = 0; i < scheme.size() ; i++) {
        std::cout << "Scheme: X = " << scheme[i].x << "  \t|  Z = " << scheme[i].z  << std::endl;
        if (scheme[i].x > robot_up)
            robot_up = (int)scheme[i].x;
        else if (scheme[i].x < robot_down)
            robot_down = (int)scheme[i].x;
        if (scheme[i].z > robot_right)
            robot_right = (int)scheme[i].z;
        else if (scheme[i].z < robot_left)
            robot_left = (int)scheme[i].z;
    }
    std::cout << "UP = " << robot_up << "  \t| DOWN = " << robot_down << " \t| LEFT = " << robot_left << "  \t| RIGHT = " << robot_right << std::endl;
}

void LoadRobot( std::string str ) {

    std::string path = "../../config/schemes/" + str;
    FILE* robot = fopen(path.c_str(), "r");
    char line[width/inter];
    int nbHeight = height/inter;
    int nbWidth = width/inter;

    if (robot != NULL) {
        for (int i = 0; i < nbHeight - 1; i++) {
            if (fgets(line, nbHeight + 1, robot) != NULL)
                for (int j = 0; j < nbWidth - 1; j++) {
                    robotScheme[i*(nbWidth - 1) + j] = (int) (line[j] - '0');
                }
        }
    } else
        std::cout << "Impossible d'ouvrir robot!" << std::endl;

    fclose(robot);
}

void ShowRobot() {

    int i = 0;
    std::cout << robotScheme.size() << std::endl;
    for ( i = 0; i < robotScheme.size(); i++ ) {
        std::cout << robotScheme[i];
        if ((i + 1)%((width - 1)/inter) == 0)
            std::cout << std::endl;
    }
    std::cout << i << std::endl;
}

void DrawRobotXZ( float x, float z, float h ) {

    glBegin(GL_POLYGON);
    glColor3f( 0.2f, 0.4f, 0.8f );
    for (int i = 0; i < scheme.size() ; i += 2) {
        glVertex3f(x + scheme[i].x, scheme[i].y, z + scheme[i].z);
    }
    for (int i = (int)scheme.size() - 1; i > 0 ; i -= 2) {
        glVertex3f(x + scheme[i].x, scheme[i].y, z + scheme[i].z);
    }
    glEnd();

    glBegin(GL_POLYGON);
    glColor3f(0.2f, 0.4f, 0.8f);
    for (int i = 0; i < scheme.size() ; i += 2) {
        glVertex3f(x + scheme[i].x, h + scheme[i].y, z + scheme[i].z);
    }
    for (int i = (int)scheme.size() - 1; i > 0 ; i -= 2) {
        glVertex3f(x + scheme[i].x, h + scheme[i].y, z + scheme[i].z);
    }
    glEnd();

    glColor3f(0.8f, 0.2f, 0.2f);
    for (int i = 0; i < scheme.size() - 2 ; i += 2) {
        glBegin(GL_POLYGON);
        glVertex3f(x + scheme[i].x, scheme[i].y, z + scheme[i].z);
        glVertex3f(x + scheme[i].x, h + scheme[i].y, z + scheme[i].z);
        glVertex3f(x + scheme[i+2].x, h + scheme[i+2].y, z + scheme[i+2].z);
        glVertex3f(x + scheme[i+2].x, scheme[i+2].y, z + scheme[i+2].z);
        glEnd();
    }

    glBegin(GL_POLYGON);
    glVertex3f(x + scheme[scheme.size()-2].x, scheme[scheme.size()-2].y, z + scheme[scheme.size()-2].z);
    glVertex3f(x + scheme[scheme.size()-2].x, h + scheme[scheme.size()-2].y, z + scheme[scheme.size()-2].z);
    glVertex3f(x + scheme[scheme.size()-1].x, h + scheme[scheme.size()-1].y, z + scheme[scheme.size()-1].z);
    glVertex3f(x + scheme[scheme.size()-1].x, scheme[scheme.size()-1].y, z + scheme[scheme.size()-1].z);
    glEnd();

    for (int i = (int)scheme.size() - 1; i > 1 ; i -= 2) {
        glBegin(GL_POLYGON);
        glVertex3f(x + scheme[i].x, scheme[i].y, z + scheme[i].z);
        glVertex3f(x + scheme[i].x, h + scheme[i].y, z + scheme[i].z);
        glVertex3f(x + scheme[i-2].x, h + scheme[i-2].y, z + scheme[i-2].z);
        glVertex3f(x + scheme[i-2].x, scheme[i-2].y, z + scheme[i-2].z);
        glEnd();
    }

    glBegin(GL_POLYGON);
    glVertex3f(x + scheme[0+1].x, scheme[0+1].y, z + scheme[0+1].z);
    glVertex3f(x + scheme[0+1].x, h + scheme[0+1].y, z + scheme[0+1].z);
    glVertex3f(x + scheme[0].x, h + scheme[0].y, z + scheme[0].z);
    glVertex3f(x + scheme[0].x, scheme[0].y, z + scheme[0].z);
    glEnd();
}

void DrawGridXZ() {

    glBegin(GL_QUADS);
    glColor3f(   0.24f, 0.24f,   0.24f);
    glVertex3f(   0.0f,  0.0f,    0.0f);
    glVertex3f(   0.0f,  0.0f, 2000.0f);
    glVertex3f(3000.0f,  0.0f, 2000.0f);
    glVertex3f(3000.0f,  0.0f,    0.0f);

    glBegin(GL_QUADS);
    glColor3f(0.48f,  0.48f,   0.48f);
    glVertex3f(0.0f,   0.0f,    0.0f);
    glVertex3f(0.0f, 100.0f,    0.0f);
    glVertex3f(0.0f, 100.0f, 2000.0f);
    glVertex3f(0.0f,   0.0f, 2000.0f);

    glBegin(GL_QUADS);
    glColor3f(   0.48f,  0.48f,   0.48f);
    glVertex3f(   0.0f,   0.0f, 2000.0f);
    glVertex3f(   0.0f, 100.0f, 2000.0f);
    glVertex3f(3000.0f, 100.0f, 2000.0f);
    glVertex3f(3000.0f,   0.0f, 2000.0f);

    glBegin(GL_QUADS);
    glColor3f(   0.48f,  0.48f,   0.48f);
    glVertex3f(3000.0f,   0.0f, 2000.0f);
    glVertex3f(3000.0f, 100.0f, 2000.0f);
    glVertex3f(3000.0f, 100.0f,    0.0f);
    glVertex3f(3000.0f,   0.0f,    0.0f);

    glBegin(GL_QUADS);
    glColor3f(   0.48f,  0.48f, 0.48f);
    glVertex3f(   0.0f,   0.0f,  0.0f);
    glVertex3f(   0.0f, 100.0f,  0.0f);
    glVertex3f(3000.0f, 100.0f,  0.0f);
    glVertex3f(3000.0f,   0.0f,  0.0f);

    glEnd();
}

void ShowAxes() {

    glLineWidth( 2.0f );

    glBegin( GL_LINES );

    glColor3f( 1.0f, 0.0f, 0.0f );
    glVertex3f( 0.0f, 0.0f, 0.0f );
    glVertex3f( 2.0f, 0.0f, 0.0f );

    glColor3f( 0.0f, 1.0f, 0.0f );
    glVertex3f( 0.0f, 0.0f, 0.0f );
    glVertex3f( 0.0f, 2.0f, 0.0f );

    glColor3f( 0.0f, 0.0f, 1.0f );
    glVertex3f( 0.0f, 0.0f, 0.0f );
    glVertex3f( 0.0f, 0.0f, 2.0f );

    glEnd();
}

void DrawInterface() {

    glBegin(GL_QUADS);

    glColor3f(0.24f, 0.24f, 0.24f);
    glVertex3f( height, 0.0f,  width);
    glVertex3f( height, 0.0f, -width);
    glVertex3f(-height, 0.0f, -width);
    glVertex3f(-height, 0.0f,  width);

    glEnd();
}

void InitializeCam() {

    if (interface_show) {
        oldCamPos[0] = 0.0f;
        oldCamPos[1] = -400.0f;
        oldCamPos[2] = -1200.0f;
        oldCamRot[0] = 20.0f;
        oldCamRot[1] = 140.0f;
        oldCamRot[2] = 0.0f;
    } else {
        oldCamPos[0] = -150.0f;
        oldCamPos[1] = -(height + width) / 16;
        oldCamPos[2] = -4.0f * length;
        oldCamRot[0] = 20.0f;
        oldCamRot[1] = 320.0f;
        oldCamRot[2] = 0.0f;
    }
    newCamPos[0] = oldCamPos[0];
    newCamPos[1] = oldCamPos[1];
    newCamPos[2] = oldCamPos[2];
    newCamRot[0] = oldCamRot[0];
    newCamRot[1] = oldCamRot[1];
    newCamRot[2] = oldCamRot[2];
}

int CheckPossibility( int direction ) {

    if ((robotPos[0] + robot_up > 2980) && (direction == 1)) {
        std::cout << "UP : " << robotPos[0] << std::endl;
        return 0;
    } else if ((robotPos[0] + robot_down < 20) && (direction == 2)) {
        std::cout << "DOWN : " << robotPos[0]  << std::endl;
        return 0;
    } else if ((robotPos[1] + robot_left < 10) && (direction == 3)) {
        std::cout << "LEFT : " << robotPos[1]  << std::endl;
        return 0;
    } else if ((robotPos[1] + robot_right > 1990) && (direction == 4)) {
        std::cout << "RIGHT : " << robotPos[1]  << std::endl;
        return 0;
    }

    return 1;
}

void UpdateButtons( SDL_Rect* launch, SDL_Rect* quit, SDL_Rect* prev, SDL_Rect* next, SDL_Rect* pause, SDL_Rect* reset, SDL_Rect* interface ) {

    launch->w = interface_x/16 + interface_x/128;
    launch->h = interface_y/32;
    launch->x = interface_x - interface_x/8 + interface_x/32 - launch->w/2;
    launch->y = interface_y - interface_y/4 - interface_y/16 + launch->h/2;

    quit->w = interface_x/8 + interface_x/16;
    quit->h = interface_y/32 + interface_y/128;
    quit->x = interface_x - interface_x/32 - quit->w/2;
    quit->y = quit->h/2;

    prev->w = interface_x/32 + interface_x/256;
    prev->h = interface_y/32;
    prev->x = interface_x - interface_x/8 - interface_x/32 + interface_x/256 - prev->w/2;
    prev->y = interface_y - interface_y/4 - interface_y/16 + prev->h/2;

    next->w = interface_x/32 + interface_x/256;
    next->h = interface_y/32;
    next->x = interface_x - interface_x/8 + interface_x/16 + interface_x/32 - interface_x/256 - next->w/2;
    next->y = interface_y - interface_y/4 - interface_y/16 + next->h/2;

    pause->w = interface_x/16 + interface_x/32;
    pause->h = interface_y/32;
    pause->x = interface_x - interface_x/8 - pause->w/2;
    pause->y = interface_y - interface_y/16 - pause->h/2;

    reset->w = interface_x/16 + interface_x/32;
    reset->h = interface_y/32;
    reset->x = interface_x - interface_x/8 - reset->w/2;
    reset->y = interface_y/4 - interface_y/16 - reset->h/2;

    interface->w = interface_x/16 + interface_x/32;
    interface->h = interface_y/32;
    interface->x = interface_x - interface_x/8 - interface->w/2;
    interface->y = interface_y/4 - interface_y/8 - interface->h/2;
}

int UpdateCheckMouse( SDL_Rect rectangle ) {

    int button_G = rectangle.x;
    int button_D = button_G + rectangle.w;
    int button_B = rectangle.y;
    int button_H = button_B + rectangle.h;

    if ((mouseMoveX >= button_G) && (mouseMoveX <= button_D) && (mouseMoveY >= button_B) && (mouseMoveY <= button_H))
        return 1;

    return 0;
}

int Simulation( SDL_Renderer* renderer ) {

    SDL_GetRendererOutputSize(renderer, &interface_x, &interface_y);
    SDL_Rect rectangle;

    // Title screen
    rectangle.w = interface_x/4;
    rectangle.h = interface_y/16 - interface_y/64;
    rectangle.x = -interface_x/8 + rectangle.w/2;
    rectangle.y = interface_y - rectangle.h - rectangle.h/2;
    glColor3f(0.0f, 0.6f, 1.0f);
    SDL_RenderDrawRect(renderer, &rectangle);
    glColor3f(1.0f, 1.0f, 1.0f);
    SDL_RenderFillRect(renderer, &rectangle);

    // Run-Pause Slot
    rectangle.w = interface_x/16 + interface_x/32;
    rectangle.h = interface_y/32;
    rectangle.x = interface_x - interface_x/8 - rectangle.w/2;
    rectangle.y = interface_y - interface_y/16 - rectangle.h/2;
    if (UpdateCheckMouse(rectangle)) {
        glColor3f(0.2f, 0.2f, 0.2f);
        SDL_RenderDrawRect(renderer, &rectangle);
        glColor3f(0.18f, 0.68f, 1.0f);
    } else {
        glColor3f(0.18f, 0.68f, 1.0f);
        SDL_RenderDrawRect(renderer, &rectangle);
        glColor3f(0.2f, 0.2f, 0.2f);
    }
    SDL_RenderFillRect(renderer, &rectangle);

    // Position X Slot
    rectangle.w = interface_x/8 + interface_x/8;
    rectangle.h = interface_y/32;
    rectangle.x = interface_x - interface_x/32 - rectangle.w/2;
    rectangle.y = interface_y/2 + interface_y/4 + interface_y/8 - rectangle.h/2;
    glColor3f(0.18f, 0.68f, 1.0f);
    SDL_RenderDrawRect(renderer, &rectangle);
    glColor3f(1.0f, 1.0f, 1.0f);
    SDL_RenderFillRect(renderer, &rectangle);

    // Position Y Slot
    rectangle.w = interface_x/8 + interface_x/8;
    rectangle.h = interface_y/32;
    rectangle.x = interface_x - interface_x/32 - rectangle.w/2;
    rectangle.y = interface_y/2 + interface_y/4 + interface_y/16 - rectangle.h/2;
    glColor3f(0.18f, 0.68f, 1.0f);
    SDL_RenderDrawRect(renderer, &rectangle);
    glColor3f(1.0f, 1.0f, 1.0f);
    SDL_RenderFillRect(renderer, &rectangle);

    // Speed X Slot
    rectangle.w = interface_x/8 + interface_x/8;
    rectangle.h = interface_y/32;
    rectangle.x = interface_x - interface_x/32 - rectangle.w/2;
    rectangle.y = interface_y/2 + interface_y/4 - rectangle.h/2;
    glColor3f(0.18f, 0.68f, 1.0f);
    SDL_RenderDrawRect(renderer, &rectangle);
    glColor3f(1.0f, 1.0f, 1.0f);
    SDL_RenderFillRect(renderer, &rectangle);

    // Speed Y Slot
    rectangle.w = interface_x/8 + interface_x/8;
    rectangle.h = interface_y/32;
    rectangle.x = interface_x - interface_x/32 - rectangle.w/2;
    rectangle.y = interface_y/2 + interface_y/4 - interface_y/16 - rectangle.h/2;
    glColor3f(0.18f, 0.68f, 1.0f);
    SDL_RenderDrawRect(renderer, &rectangle);
    glColor3f(1.0f, 1.0f, 1.0f);
    SDL_RenderFillRect(renderer, &rectangle);

    // Reset Slot
    rectangle.w = interface_x/16 + interface_x/32;
    rectangle.h = interface_y/32;
    rectangle.x = interface_x - interface_x/8 - rectangle.w/2;
    rectangle.y = interface_y/4 - interface_y/16 - rectangle.h/2;
    if (UpdateCheckMouse(rectangle)) {
        glColor3f(0.2f, 0.2f, 0.2f);
        SDL_RenderDrawRect(renderer, &rectangle);
        glColor3f(0.18f, 0.68f, 1.0f);
    } else {
        glColor3f(0.18f, 0.68f, 1.0f);
        SDL_RenderDrawRect(renderer, &rectangle);
        glColor3f(0.2f, 0.2f, 0.2f);
    }
    SDL_RenderFillRect(renderer, &rectangle);

    // Return Slot
    rectangle.w = interface_x/16 + interface_x/32;
    rectangle.h = interface_y/32;
    rectangle.x = interface_x - interface_x/8 - rectangle.w/2;
    rectangle.y = interface_y/4 - interface_y/8 - rectangle.h/2;
    if (UpdateCheckMouse(rectangle)) {
        glColor3f(0.2f, 0.2f, 0.2f);
        SDL_RenderDrawRect(renderer, &rectangle);
        glColor3f(0.18f, 0.68f, 1.0f);
    } else {
        glColor3f(0.18f, 0.68f, 1.0f);
        SDL_RenderDrawRect(renderer, &rectangle);
        glColor3f(0.2f, 0.2f, 0.2f);
    }
    SDL_RenderFillRect(renderer, &rectangle);

    // Quit slot
    rectangle.w = interface_x/8 + interface_x/16;
    rectangle.h = interface_y/32 + interface_y/128;
    rectangle.x = interface_x - interface_x/32 - rectangle.w/2;
    rectangle.y = rectangle.h/2;
    glColor3f(0.6f, 0.2f, 0.0f);
    SDL_RenderDrawRect(renderer, &rectangle);
    if (UpdateCheckMouse(rectangle)) {
        glColor3f(0.2f, 0.0f, 0.0f);
    } else {
        glColor3f(1.0f, 1.0f, 1.0f);
    }
    SDL_RenderFillRect(renderer, &rectangle);

    // White interface
    rectangle.w = interface_x/4;
    rectangle.h = interface_y + interface_y/4;
    rectangle.x = interface_x  + interface_x/16 - rectangle.w;
    rectangle.y = -interface_y/8;
    glColor3f(0.18f, 0.68f, 1.0f);
    SDL_RenderDrawRect(renderer, &rectangle);
    glColor4f(0.12f, 0.12f, 0.12f, 0.96f);
    SDL_RenderFillRect(renderer, &rectangle);

    return 0;
}

int Interface( SDL_Renderer* renderer ) {

    SDL_GetRendererOutputSize(renderer, &interface_x, &interface_y);
    SDL_Rect rectangle;

    // Title screen
    rectangle.w = interface_x/4;
    rectangle.h = interface_y/16 - interface_y/64;
    rectangle.x = -interface_x/8 + rectangle.w/2;
    rectangle.y = interface_y - rectangle.h - rectangle.h/2;
    glColor3f(0.0f, 0.6f, 1.0f);
    SDL_RenderDrawRect(renderer, &rectangle);
    glColor3f(1.0f, 1.0f, 1.0f);
    SDL_RenderFillRect(renderer, &rectangle);

    // Launch Slot
    rectangle.w = interface_x/16 + interface_x/128;
    rectangle.h = interface_y/32;
    rectangle.x = interface_x - interface_x/8 + interface_x/32 - rectangle.w/2;
    rectangle.y = interface_y - interface_y/4 - interface_y/16 + rectangle.h/2;
    if (UpdateCheckMouse(rectangle)) {
        glColor3f(0.2f, 0.2f, 0.2f);
        SDL_RenderDrawRect(renderer, &rectangle);
        glColor3f(0.18f, 0.68f, 1.0f);
    } else {
        glColor3f(0.18f, 0.68f, 1.0f);
        SDL_RenderDrawRect(renderer, &rectangle);
        glColor3f(0.2f, 0.2f, 0.2f);
    }
    SDL_RenderFillRect(renderer, &rectangle);

    // Previous Slot
    rectangle.w = interface_x/32 + interface_x/256;
    rectangle.h = interface_y/32;
    rectangle.x = interface_x - interface_x/8 - interface_x/32 + interface_x/256 - rectangle.w/2;
    rectangle.y = interface_y - interface_y/4 - interface_y/16 + rectangle.h/2;
    if (UpdateCheckMouse(rectangle)) {
        glColor3f(0.2f, 0.2f, 0.2f);
        SDL_RenderDrawRect(renderer, &rectangle);
        glColor3f(0.18f, 0.68f, 1.0f);
    } else {
        glColor3f(0.18f, 0.68f, 1.0f);
        SDL_RenderDrawRect(renderer, &rectangle);
        glColor3f(0.2f, 0.2f, 0.2f);
    }
    SDL_RenderFillRect(renderer, &rectangle);

    // Next Slot
    rectangle.w = interface_x/32 + interface_x/256;
    rectangle.h = interface_y/32;
    rectangle.x = interface_x - interface_x/8 + interface_x/16 + interface_x/32 - interface_x/256 - rectangle.w/2;
    rectangle.y = interface_y - interface_y/4 - interface_y/16 + rectangle.h/2;
    if (UpdateCheckMouse(rectangle)) {
        glColor3f(0.2f, 0.2f, 0.2f);
        SDL_RenderDrawRect(renderer, &rectangle);
        glColor3f(0.18f, 0.68f, 1.0f);
    } else {
        glColor3f(0.18f, 0.68f, 1.0f);
        SDL_RenderDrawRect(renderer, &rectangle);
        glColor3f(0.2f, 0.2f, 0.2f);
    }
    SDL_RenderFillRect(renderer, &rectangle);

    // Quit slot
    rectangle.w = interface_x/8 + interface_x/16;
    rectangle.h = interface_y/32 + interface_y/128;
    rectangle.x = interface_x - interface_x/32 - rectangle.w/2;
    rectangle.y = rectangle.h/2;
    glColor3f(0.6f, 0.2f, 0.0f);
    SDL_RenderDrawRect(renderer, &rectangle);
    if (UpdateCheckMouse(rectangle)) {
        glColor3f(0.2f, 0.0f, 0.0f);
    } else {
        glColor3f(1.0f, 1.0f, 1.0f);
    }
    SDL_RenderFillRect(renderer, &rectangle);

    // White interface
    rectangle.w = interface_x/4;
    rectangle.h = interface_y + interface_y/4;
    rectangle.x = interface_x  + interface_x/16 - rectangle.w;
    rectangle.y = -interface_y/8;
    glColor3f(0.18f, 0.68f, 1.0f);
    SDL_RenderDrawRect(renderer, &rectangle);
    glColor4f(0.12f, 0.12f, 0.12f, 0.96f);
    SDL_RenderFillRect(renderer, &rectangle);

    return 0;
}

int main( int argc, char ** argv ) {

#ifndef TEST
    SDL_Window* window;
    SDL_Event event;
    SDL_Rect launch, next, prev;
    SDL_Rect pause, reset, interface;
    SDL_Rect quit;

    bool done = false;
    float mouseOriginX = 0.0f;
    float mouseOriginY = 0.0f;
    float mouseDeltaX = 0.0f;
    float mouseDeltaY = 0.0f;

    struct timeval begin, end;
    float fps = 0.0;
    char sfps[40] = "FPS: ";

    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        printf("error: unable to init sdl\n");
        return -1;
    }

    if (SDL_GetDesktopDisplayMode(0, &current) ) {
        SDL_Log("error: unable to get current display mode\n");
        return -1;
    }

    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);

    window = SDL_CreateWindow("RoSim", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, current.w, current.h, SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE | SDL_WINDOW_OPENGL);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

    SDL_GLContext glWindow = SDL_GL_CreateContext(window);

    glOrtho((GLdouble) -100, (GLdouble) 100, (GLdouble) -100, (GLdouble) 100, (GLdouble) -1, (GLdouble) 1);

    GLenum status = glewInit();
    if (status != GLEW_OK) {
        SDL_Log("error: unable to init glew\n");
        return -1;
    }

    SDL_GL_SetSwapInterval(1);

    oldCamPos[2] += (2 / 10.0f)*1.0f*fabs(oldCamPos[2]);
    oldCamPos[2] = oldCamPos[2] > -5.0f ? -5.0f : oldCamPos[2];

    FindFiles();
    InitRobot();
    LoadRobot(schemeList[schemeId]);
    //ShowRobot();
    LoadStructure();

    while (!done) {

        int i;
        while (SDL_PollEvent(&event)) {

            UpdateButtons(&launch, &quit, &prev, &next, &pause, &reset, &interface);
            if (event.type == SDL_MOUSEBUTTONDOWN) {
                if (UpdateCheckMouse(quit))
                    done = true;
                if (interface_show) {
                    if (UpdateCheckMouse(launch)) {
                        InitializeCam();
                        interface_show = false;
                    } else if (UpdateCheckMouse(next)) {
                        if (schemeId == (int) schemeList.size() - 1)
                            schemeId = 0;
                        else
                            schemeId += 1;
                        schemeChange = true;
                    } else if (UpdateCheckMouse(prev)) {
                        if (schemeId == 0)
                            schemeId = (int) schemeList.size() - 1;
                        else
                            schemeId -= 1;
                        schemeChange = true;
                    }
                } else {
                    if (UpdateCheckMouse(pause)) {
                        simu_pause = !simu_pause;
                    } else if (UpdateCheckMouse(reset)) {
                        robotPos[0] = initialPos[0];
                        robotPos[1] = initialPos[1];
                    } else if (UpdateCheckMouse(interface)) {
                        robotPos[0] = initialPos[0];
                        robotPos[1] = initialPos[1];
                        InitializeCam();
                        interface_show = true;
                    }
                }
            } else if (event.type == SDL_MOUSEMOTION) {
                mouseMoveX = event.motion.x;
                mouseMoveY = interface_y - event.motion.y;
            } else if (event.type == SDL_MOUSEWHEEL) {
                if (!interface_show) {
                    oldCamPos[2] += (event.wheel.y / 10.0f)*1.0f*fabs(oldCamPos[2]);
                    oldCamPos[2] = oldCamPos[2] > -5.0f ? -5.0f : oldCamPos[2];
                }
            } else if (event.type == SDL_KEYDOWN) {
                //std::cout << event.key.keysym.sym << std::endl;
                if (event.key.keysym.sym == SDLK_F1) {
                    simu_show = !simu_show;
                } else if (event.key.keysym.sym == SDLK_F2) {
                    fullscreen = !fullscreen;
                } else if (event.key.keysym.sym == SDLK_ESCAPE) {
                    done = true;
                } else if (event.key.keysym.sym == SDLK_SPACE) {
                    simu_pause = true;
                } else if (interface_show) {
                    if (event.key.keysym.sym == SDLK_LEFT) {
                        if (schemeId == 0)
                            schemeId = (int)schemeList.size() - 1;
                        else
                            schemeId -= 1;
                        schemeChange = true;
                    } else if (event.key.keysym.sym == SDLK_RIGHT) {
                        if (schemeId == (int)schemeList.size() - 1)
                            schemeId = 0;
                        else
                            schemeId += 1;
                        schemeChange = true;
                    } else if (event.key.keysym.sym == SDLK_RETURN || event.key.keysym.sym == SDLK_KP_ENTER) {
                        InitializeCam();
                        interface_show = false;
                    }
                } else if ((!interface_show) && (!simu_pause)) {
                    if (event.key.keysym.sym == SDLK_UP || event.key.keysym.sym == SDLK_z) {
                        if (CheckPossibility(1))
                            robotPos[0] += 10.0f;
                    } else if (event.key.keysym.sym == SDLK_DOWN || event.key.keysym.sym == SDLK_s) {
                        if (CheckPossibility(2))
                            robotPos[0] -= 10.0f;
                    } else if (event.key.keysym.sym == SDLK_RIGHT || event.key.keysym.sym == SDLK_d) {
                        if (CheckPossibility(4))
                            robotPos[1] += 10.0f;
                    } else if (event.key.keysym.sym == SDLK_LEFT || event.key.keysym.sym == SDLK_q) {
                        if (CheckPossibility(3))
                            robotPos[1] -= 10.0f;
                    }
                }
            } else if (event.type == SDL_KEYUP) {
                if (event.key.keysym.sym == SDLK_SPACE) {
                    simu_pause = false;
                }
            }

            if (event.type == SDL_QUIT) {
                printf("quit\n");
                done = true;
            }
        }

        SDL_GL_GetDrawableSize(window, &gl_x, &gl_y);
        mouseDeltaX = mouseMoveX - mouseOriginX;
        mouseDeltaY = mouseMoveY - mouseOriginY;
        if (!interface_show) {
            if (SDL_GetMouseState(0, 0) & SDL_BUTTON_LMASK) {
                oldCamRot[0] -= mouseDeltaY/5.0f;
                oldCamRot[1] += mouseDeltaX/5.0f;
            }
        } else {
            if (schemeChange) {
                LoadRobot(schemeList[schemeId]);
                LoadStructure();
                schemeChange = false;
            }
            if (SDL_GetMouseState(0, 0) & SDL_BUTTON_LMASK) {
                oldCamRot[0] -= mouseDeltaY/5.0f;
                oldCamRot[1] += mouseDeltaX/5.0f;
            } else {
                oldCamRot[0] = 30.0f;
                oldCamRot[1] -= 1.0f/10.0f;
            }
        }

        mouseOriginX = mouseMoveX;
        mouseOriginY = mouseMoveY;

        glViewport(0, 0, gl_x, gl_y);
        glClearColor(0.4f, 0.4f, 0.4f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_BLEND);
        glBlendEquation(GL_FUNC_ADD);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDisable(GL_TEXTURE_2D);
        glEnable(GL_DEPTH_TEST);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(50.0f, (float)gl_x / (float)gl_y, 0.1f, 100000.0f);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();


        gettimeofday(&begin, NULL);

        for (i = 0; i < 3; ++i) {
            newCamPos[i] += (oldCamPos[i] - newCamPos[i]) * g_inertia;
            newCamRot[i] += (oldCamRot[i] - newCamRot[i]) * g_inertia;
        }

        glTranslatef(newCamPos[0], newCamPos[1], newCamPos[2]);
        glRotatef(newCamRot[0], 1.0f, 0.0f, 0.0f);
        glRotatef(newCamRot[1], 0.0f, 1.0f, 0.0f);

        if (interface_show) {
            DrawInterface();
            DrawRobotXZ(0.0f, 0.0f, length);
        } else if (simu_show) {
            DrawGridXZ();
            DrawRobotXZ(robotPos[0], robotPos[1], robotPos[2]);
        }

        if (fullscreen) {
            ShowAxes();
            SDL_SetWindowFullscreen(window, SDL_WINDOW_FULLSCREEN);
        } else
            SDL_SetWindowFullscreen(window, 0);

        gettimeofday(&end, NULL);
        /*fps = (float)1.0f / ( ( end.tv_sec - begin.tv_sec ) * 1000000.0f + end.tv_usec - begin.tv_usec) * 1000000.0f;
        sprintf( sfps, "FPS : %.4f", fps );
        std::cout << sfps << std::endl;*/

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluOrtho2D(0, gl_x, 0, gl_y);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        if (interface_show)
            Interface(renderer);
        else
            Simulation(renderer);
        SDL_GL_SwapWindow(window);
        SDL_UpdateWindowSurface(window);
    }

    SDL_DestroyRenderer(renderer);
    SDL_GL_DeleteContext(glWindow);
    SDL_DestroyWindow(window);
    SDL_Quit();

#else

#ifdef INTERFACE



#else
    SDL_Init( SDL_INIT_EVERYTHING );

    // Set postion and size for main window
    int mainSizeX = 600;
    int mainSizeY = 600;
    int mainPosX = 100;
    int mainPosY = 100;

    // Set postion and size for sub window based on those of main window
    int subSizeX = mainSizeX / 2;
    int subSizeY = mainSizeY / 2;
    int subPosX = mainPosX + mainSizeX / 4;
    int subPosY = mainPosY + mainSizeY / 4;

    // Set up main window
    SDL_Window* mainWindow = SDL_CreateWindow( "Main Window", mainPosX, mainPosY, mainSizeX, mainSizeY, SDL_WINDOW_RESIZABLE );
    SDL_Renderer* mainRenderer = SDL_CreateRenderer( mainWindow, -1, SDL_RENDERER_ACCELERATED );
    SDL_SetRenderDrawColor( mainRenderer , 255, 0, 0, 255 );

    // Set up sub window
    SDL_Window* subWindow  = SDL_CreateWindow( "Sub Window" , subPosX, subPosY, subSizeX, subSizeY, 0 ); //SDL_WINDOW_BORDERLESS );
    SDL_Renderer* subRenderer  = SDL_CreateRenderer( subWindow, -1, SDL_RENDERER_SOFTWARE );
    SDL_SetRenderDrawColor( subRenderer , 0, 255, 0, 255 );

    // Render empty ( red ) background in mainWindow
    SDL_RenderClear( mainRenderer );
    SDL_RenderPresent( mainRenderer );

    // Render empty ( green ) background in subWindow
    SDL_RenderClear( subRenderer );
    SDL_RenderPresent( subRenderer );

    SDL_Event event;
    bool done = false;
    while ( !done ) {

        while (SDL_PollEvent(&event)) {

            if (event.key.keysym.sym == SDLK_ESCAPE)
                done = true;

            if (event.type == SDL_QUIT) {
                printf("quit\n");
                done = true;
            }

        }

        SDL_RenderClear( mainRenderer );
        // Try
        SDL_SetRenderDrawColor( mainRenderer, 0, 0, 255, 255 );
        SDL_Rect rectangle;
        rectangle.x = mainSizeX/4;
        rectangle.y = mainSizeY/4;
        rectangle.w = mainSizeX/2;
        rectangle.h = mainSizeY/2;
        SDL_RenderFillRect( mainRenderer, &rectangle );
        SDL_SetRenderDrawColor( mainRenderer, 255, 0, 0, 255 );
        SDL_RenderPresent( mainRenderer );
    }
#endif
#endif

    return 0;
}
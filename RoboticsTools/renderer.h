
#ifndef RENDERER_H
#define RENDERER_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "symbolicc++.h"

#include <SDL2/SDL.h>

static double s_arm_scaling, s_z_scaling;

static const std::vector<std::vector<double>>
Tx { {1, 0, 0, 0.2},
     {0, 1, 0, 0},
     {0, 0, 1, 0},
     {0, 0, 0, 1} };

static const std::vector<std::vector<double>>
Ty { {1, 0, 0, 0},
     {0, 1, 0, 0.2},
     {0, 0, 1, 0},
     {0, 0, 0, 1} };

static const std::vector<std::vector<double>>
Tz { {1, 0, 0, 0},
     {0, 1, 0, 0},
     {0, 0, 1, 0.2},
     {0, 0, 0, 1} };

static void render_grid(SDL_Renderer* renderer,
                        double center_x, double center_y) {
    SDL_SetRenderDrawColor(renderer, 70,70,70, SDL_ALPHA_OPAQUE);
    double width = 1300;
    double height = -200;
    center_y += height;

    for (double scale = 1.0; scale <= 2.01; scale += 0.1) {
        SDL_RenderDrawLine(renderer,
                           -(width*0.45)/scale + center_x,
                           -height/scale + center_y,
                           (width*0.45)/scale + center_x,
                           -height/scale + center_y);
    }
    for (double w = -width*0.45; w <= width*0.45; w+=width*0.05) {
        SDL_RenderDrawLine(renderer,
                           w + center_x,
                           -height + center_y,
                           w/2.0 + center_x,
                           -height/2.0 + center_y);
    }
}

static void render_link(SDL_Renderer* renderer,
                        double* x0p, double* y0p, double* z0p,
                        double center_x, double center_y,
                        const std::vector<std::vector<double>>& p) {

    center_y -= 268;
    double x0 = *x0p;
    double y0 = *y0p+200;
    double z0 = *z0p;

    double x1 = -p[0][3]*s_arm_scaling;
    double y1 = -p[2][3]*s_arm_scaling+200;
    double z1 = -p[1][3]*s_z_scaling;

    double delta_x = (x1-x0)/10.0;
    double delta_y = (y1-y0)/10.0;
    double delta_z = (z1-z0)/10.0;

    double shade, scale=z0+1.0;
    for (double shading = 1; shading <= 10.0; shading+=1.0) {
        shade = 175 - (z0 + delta_z*shading)*500;
        shade = (shade < 100) ? 100 :( (shade > 255) ? 255 : shade );
        SDL_SetRenderDrawColor(renderer, shade, shade, shade, SDL_ALPHA_OPAQUE);

        SDL_RenderDrawLine(renderer,
                           (x0+(delta_x*(shading-1.0)))/scale + center_x,
                           (y0+(delta_y*(shading-1.0)))/scale + center_y,
                           (x0+(delta_x*shading))      /(scale+delta_z) + center_x,
                           (y0+(delta_y*shading))      /(scale+delta_z) + center_y);
        scale += delta_z;
    }

    // Drawing the frame orientations
    auto Px = multiply_transforms(p, Tx);
    auto Py = multiply_transforms(p, Ty);
    auto Pz = multiply_transforms(p, Tz);

    SDL_SetRenderDrawColor(renderer, shade, 0, 0, SDL_ALPHA_OPAQUE);
    SDL_RenderDrawLine(renderer,
                       x1/scale + center_x, y1/scale + center_y,
                       -Px[0][3]*s_arm_scaling/(-Px[1][3]*s_z_scaling+1.0) + center_x,
                       (-Px[2][3]*s_arm_scaling+200)/(-Px[1][3]*s_z_scaling+1.0) + center_y);
    SDL_SetRenderDrawColor(renderer, 0, shade, 0, SDL_ALPHA_OPAQUE);

    SDL_RenderDrawLine(renderer,
                       x1/scale + center_x, y1/scale + center_y,
                       -Py[0][3]*s_arm_scaling/(-Py[1][3]*s_z_scaling+1.0) + center_x,
                       (-Py[2][3]*s_arm_scaling+200)/(-Py[1][3]*s_z_scaling+1.0) + center_y);

    SDL_SetRenderDrawColor(renderer, 0, 0, shade, SDL_ALPHA_OPAQUE);
    SDL_RenderDrawLine(renderer,
                       x1/scale + center_x, y1/scale + center_y,
                       -Pz[0][3]*s_arm_scaling/(-Pz[1][3]*s_z_scaling+1.0) + center_x,
                       (-Pz[2][3]*s_arm_scaling+200)/(-Pz[1][3]*s_z_scaling+1.0) + center_y);

    *x0p = x1;
    *y0p = y1-200;
    *z0p = z1;
}

static void render_arm(Arm* arm) {
   
   
    s_arm_scaling = 0;
    s_z_scaling = 0;
    std::vector<double> joints (arm->m_actuated_joints.size(),0);
    double arm_link_lengths;
    auto arm_positions = arm->get_positions(joints);
    for (auto p : arm_positions) {
        arm_link_lengths += sqrt( p[0][3]*p[0][3] + p[2][3]*p[2][3] + p[2][3]*p[2][3] ) ;
    }
    s_arm_scaling = 450.0/arm_link_lengths;
    s_z_scaling = s_arm_scaling*0.001;

    if (SDL_Init(SDL_INIT_VIDEO) == 0) {
        SDL_Window* window = NULL;
        SDL_Renderer* renderer = NULL;

        if (SDL_CreateWindowAndRenderer(1300, 750, 0, &window, &renderer) == 0) {
            SDL_bool done = SDL_FALSE;
            bool mouse_down;
            int mouse_x, mouse_y;
            while (!done) {
                SDL_Event event;

                auto positions = arm->get_positions(joints);

                for (int j = 0; j < joints.size(); j++) {
                    joints[j] += 0.005;
                }

                // BACKGROUND
                SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
                SDL_RenderClear(renderer);

                double center_x = 1300/2;
                double center_y = 2*750/3;
                
                render_grid(renderer, center_x, center_y);

                double x0=0, y0=0, z0=0;
                for (const auto& p : positions) {
                    render_link(renderer, &x0, &y0, &z0, center_x, center_y, p);
                }

                SDL_RenderPresent(renderer);

                if (mouse_down) {
                    int mouse_x_new, mouse_y_new;
                    SDL_GetMouseState(&mouse_x_new, &mouse_y_new);
                    joints[0] += 0.01*(mouse_x_new - mouse_x);
                    joints[1] += 0.01*(mouse_y_new - mouse_y);
                    mouse_x = mouse_x_new;
                    mouse_y = mouse_y_new;
                }

                while (SDL_PollEvent(&event)) {
                    switch (event.type) {
                        case SDL_QUIT:
                            done = SDL_TRUE;
                            break;
                        case SDL_MOUSEBUTTONDOWN:
                            SDL_GetMouseState(&mouse_x, &mouse_y);
                            mouse_down = true;
                            break;
                        case SDL_MOUSEBUTTONUP:
                            mouse_down = false;
                            break;
                        default:
                            break;
                    }
                }
            }
        }

        if (renderer) {
            SDL_DestroyRenderer(renderer);
        }
        if (window) {
            SDL_DestroyWindow(window);
        }
    }
    SDL_Quit();
}


#endif

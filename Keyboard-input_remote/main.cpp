#include <iostream>
#include <SDL2/SDL.h>
#include "comm.hpp"

int main(int argc, char* argv[]) {
    
    SerialComm serialComm;
    
    // select and open port
    std::string portName = serialComm.selectPort();
    if (portName.empty()) {
        std::cerr << "Invalid port selection." << std::endl;
        std::cerr << "Press Enter to exit." << std::endl;
        std::cin.ignore();
        std::cin.get();
        return 1;
    }
    
    if (!serialComm.open(portName)) {
        std::cerr << "Press Enter to exit." << std::endl;
        std::cin.ignore();
        std::cin.get();
        return 1;
    }

    std::cout << "Press W, A, S, D, Q, E, R, F, C, V, TAB, or SPACE." << std::endl;
    std::cout << "Press 0 - 9 to change mac_index_dest." << std::endl;
    std::cout << "Press 'ESC' to quit." << std::endl;
    std::cout << "--- (Click the new 'Key Sender' window to give it focus) ---" << std::endl;

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cerr << "Error: Failed to initialize SDL: " << SDL_GetError() << std::endl;
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "Key Sender (Press ESC to quit)",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        400, 
        200,
        0    
    );

    if (!window) {
        std::cerr << "Error: Failed to create SDL window: " << SDL_GetError() << std::endl;
        return 1;
    }

    bool running = true;
    SDL_Event event;

    while (running) {
        while (SDL_PollEvent(&event)) {
            
            if (event.type == SDL_QUIT) {
                running = false;
            }

            if (event.type == SDL_KEYDOWN) {
                char dataToSend = 0;

                switch (event.key.keysym.sym) {
                    case SDLK_w:      dataToSend = 'W'; break;
                    case SDLK_a:      dataToSend = 'A'; break;
                    case SDLK_s:      dataToSend = 'S'; break;
                    case SDLK_d:      dataToSend = 'D'; break;
                    case SDLK_q:      dataToSend = 'Q'; break;
                    case SDLK_e:      dataToSend = 'E'; break;
                    case SDLK_r:      dataToSend = 'R'; break;
                    case SDLK_f:      dataToSend = 'F'; break;
                    case SDLK_c:      dataToSend = 'C'; break;
                    case SDLK_v:      dataToSend = 'V'; break;
                    case SDLK_SPACE:  dataToSend = ' '; break;
                    case SDLK_TAB:    dataToSend = '\t'; break;
                    
                    case SDLK_0:
                        mac_index_dest = 0;
                        std::cout << "Change mac_index_dest to " << static_cast<int>(mac_index_dest) 
                                  << " (" << mac_names[mac_index_dest] << ")" << std::endl;
                        break;

                    case SDLK_1:
                        mac_index_dest = 1;
                        std::cout << "Change mac_index_dest to " << static_cast<int>(mac_index_dest) 
                                  << " (" << mac_names[mac_index_dest] << ")" << std::endl;
                        break;

                    case SDLK_2:
                        mac_index_dest = 2;
                        std::cout << "Change mac_index_dest to " << static_cast<int>(mac_index_dest) 
                                  << " (" << mac_names[mac_index_dest] << ")" << std::endl;
                        break;

                    case SDLK_3:
                        mac_index_dest = 3;
                        std::cout << "Change mac_index_dest to " << static_cast<int>(mac_index_dest) 
                                  << " (" << mac_names[mac_index_dest] << ")" << std::endl;
                        break;

                    case SDLK_4:
                        mac_index_dest = 4;
                        std::cout << "Change mac_index_dest to " << static_cast<int>(mac_index_dest) 
                                  << " (" << mac_names[mac_index_dest] << ")" << std::endl;
                        break;

                    case SDLK_5:
                        mac_index_dest = 5;
                        std::cout << "Change mac_index_dest to " << static_cast<int>(mac_index_dest) 
                                  << " (" << mac_names[mac_index_dest] << ")" << std::endl;
                        break;

                    case SDLK_6:
                        mac_index_dest = 6;
                        std::cout << "Change mac_index_dest to " << static_cast<int>(mac_index_dest) 
                                  << " (" << mac_names[mac_index_dest] << ")" << std::endl;
                        break;

                    case SDLK_7:
                        mac_index_dest = 7;
                        std::cout << "Change mac_index_dest to " << static_cast<int>(mac_index_dest) 
                                  << " (" << mac_names[mac_index_dest] << ")" << std::endl;
                        break;

                    case SDLK_8:
                        mac_index_dest = 8;
                        std::cout << "Change mac_index_dest to " << static_cast<int>(mac_index_dest) 
                                  << " (" << mac_names[mac_index_dest] << ")" << std::endl;
                        break;
                    
                    case SDLK_9:
                        mac_index_dest = 9;
                        std::cout << "Change mac_index_dest to " << static_cast<int>(mac_index_dest) 
                                  << " (" << mac_names[mac_index_dest] << ")" << std::endl;
                        break;

                    case SDLK_ESCAPE:
                        running = false;
                        break;
                    default:
                        break;
                }

                if (dataToSend != 0) {
                    if (!serialComm.send(dataToSend)) {
                        running = false;
                    }
                }
            }
        }
    }

    std::cout << "Exiting..." << std::endl;
    serialComm.close();
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
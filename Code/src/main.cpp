/// @file     main.cpp
/// @author   Erik ZORZIN
/// @date     30JUL2020
/// @brief    Single 1/2 spin spinor, simulated as a tangle of a 3D continuum body.

// OPENGL:
#define GUI_SIZE_X    800                                                                           // Window x-size [px].
#define GUI_SIZE_Y    600                                                                           // Window y-size [px].
#define GUI_NAME      "SpinorCodes - Spinor"                                                        // Window name.

// OPENCL:
#define QUEUE_NUM     1                                                                             // Number of OpenCL queues [#].
#define KERNEL_NUM    1                                                                             // Number of OpenCL kernel [#].
#define KERNEL_FILE   "spinor_kernel.cl"                                                            // OpenCL kernel.

#ifdef __linux__
  #define SHADER_HOME "../Code/shader"                                                              // Linux OpenGL shaders directory.
  #define KERNEL_HOME "../Code/kernel"                                                              // Linux OpenCL kernels directory.
#endif

#ifdef __APPLE__
  #define SHADER_HOME "../Code/shader"                                                              // Mac OpenGL shaders directory.
  #define KERNEL_HOME "../Code/kernel"                                                              // Mac OpenCL kernels directory.
#endif

#ifdef WIN32
  #define SHADER_HOME "..\\..\\Code\\shader"                                                        // Windows OpenGL shaders directory.
  #define KERNEL_HOME "..\\..\\Code\\kernel"                                                        // Windows OpenCL kernels directory.
#endif

#define SHADER_VERT   "cross.vert"                                                                  // OpenGL vertex shader.
#define SHADER_GEOM   "cross.geom"                                                                  // OpenGL geometry shader.
#define SHADER_FRAG   "cross.frag"                                                                  // OpenGL fragment shader.

// INCLUDES:
#include "nu.hpp"                                                                                   // Neutrino header file.


int main ()
{

}
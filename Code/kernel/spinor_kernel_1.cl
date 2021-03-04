/// @file     spinor_kernel_1.cl
/// @author   Erik ZORZIN
/// @date     16JAN2021
/// @brief    Does nothing.
/// @details  The nothing:
__kernel void thekernel(__global float4*    color,                              // Color.
                        __global float4*    position,                           // Position.
                        __global float4*    position_int,                       // Position (intermediate).
                        __global float4*    velocity,                           // Velocity.
                        __global float4*    velocity_int,                       // Velocity (intermediate).
                        __global float4*    acceleration,                       // Acceleration.
                        __global float*     stiffness,                          // Stiffness.
                        __global float*     resting,                            // Resting distance.
                        __global float*     friction,                           // Friction.
                        __global float*     central,                            // Central.
                        __global int*       nearest,                            // Neighbour.
                        __global int*       offset,                             // Offset.
                        __global int*       freedom,                            // Freedom flag.
                        __global float*     dt_simulation)                      // Simulation time step.
{

}
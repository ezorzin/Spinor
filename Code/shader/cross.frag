/// @file   cross.frag
/// @author Erik ZORZIN
/// @date   30JUL2020
/// @brief  This fragment file creates a cross out of a 4D point coordinates.

#version 410 core

in  vec4 voxel_color;                                                           // Voxel color.
out vec4 fragment_color;                                                        // Fragment color.

void main(void)
{
  fragment_color = voxel_color;                                                 // Setting fragment color...
}

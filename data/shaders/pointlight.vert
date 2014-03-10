uniform mat4 ViewMatrix;
uniform mat4 ProjectionMatrix;

in vec3 Position;
in float Energy;
in vec3 Color;

in vec2 Corner;

flat out vec3 center;
flat out float energy;
flat out vec3 col;

void main(void)
{
    // Beyond that value, light is too attenuated
    float r = 40 * Energy;
    center = Position;
    energy = Energy;
    vec4 Center = ViewMatrix * vec4(Position, 1.);
    float zDepth = Center.z;
//    if (zDepth - r < 1. && zDepth + r > 1.)
//        Center.z = Center.w + 0.001;
    col = Color;
    gl_Position = ProjectionMatrix * (Center + r * vec4(Corner, 0., 0.));
	gl_Position.z -= gl_Position.w * r;
}

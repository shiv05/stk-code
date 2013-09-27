uniform sampler2D tex;
uniform vec3 ambient;

void main()
{
	vec2 texc = gl_TexCoord[0].xy;

	vec4 col = texture2D(tex, texc);

	col.xyz += ambient;
	float spec = col.a - 0.05;
	col.xyz += spec * col.xyz;
	col.a = 1.0;

	gl_FragColor = col;
}

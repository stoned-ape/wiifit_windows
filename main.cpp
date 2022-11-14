// #define DICK
#ifndef __APPLE__
//#define KINECT
#include <windows.h>
#include <gl/gl.h>
#else 
#define GL_SILENCE_DEPRECATION
#include <OpenGL/gl.h>
#endif
#include <GLFW/glfw3.h>

#include <math.h>
#include <assert.h>
#include <time.h>
#include <sys/timeb.h>
#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "kinect_struct.h"



#pragma comment(lib,"opengl32.lib")
#pragma comment(lib,"user32.lib")
#pragma comment(lib,"gdi32.lib")
#pragma comment(lib,"shell32.lib")
#pragma comment(lib,"glfw-3.3.7.bin.WIN64/lib-vc2019/glfw3_mt.lib")

#pragma comment(lib,"kinect20.lib")
#pragma comment(lib,"kinect20.fusion.lib")
#pragma comment(lib,"kinect20.face.lib")
#pragma comment(lib,"kinect20.visualgesturebuilder.lib")


const float pi = 3.141592653589793;


float4x4 rotx(float theta) {
	float _rot[4 * 4] = {
		1,0,0,0,
		0,+cos(theta),sin(theta),0,
		0,-sin(theta),cos(theta),0,
		0,0,0,1,
	};
	return *(float4x4 *)&_rot;
}

float4x4 roty(float theta) {
	float _rot[4 * 4] = {
		+cos(theta),0,sin(theta),0,
		0,1,0,0,
		-sin(theta),0,cos(theta),0,
		0,0,0,1,
	};
	return *(float4x4 *)&_rot;
}

float4x4 rotz(float theta) {
	float _rot[4 * 4] = {
		+cos(theta),sin(theta),0,0,
		-sin(theta),cos(theta),0,0,
		0,0,1,0,
		0,0,0,1,
	};
	return *(float4x4 *)&_rot;
}

float4x4 trans(float4 v) {
	float _mtx[4 * 4] = {
		1. ,0. ,0. ,v.x,
		0. ,1. ,0. ,v.y,
		0. ,0. ,1. ,v.z,
		0. ,0. ,0. ,1.
	};
	return *(float4x4 *)&_mtx;
}
float4x4 trans(float x, float y, float z) {
	return trans(float4(x, y, z, 0));
}
float4x4 trans(float3 p) {
	return trans(float4(p.x, p.y, p.z, 0));
}
float4x4 scale(float4 v) {
	float _mtx[4 * 4] = {
		v.x,0.,0.,0.,
		0.,v.y,0.,0.,
		0.,0.,v.z,0.,
		0.,0.,0.,1.
	};
	return *(float4x4 *)&_mtx;
}
float4x4 scale(float x, float y, float z) {
	return scale(float4(x, y, z, 0));
}
float4x4 scale(float s) {
	return scale(float4(s, s, s, 0));
}

struct _quat {
	float s;
	float3 v;
	_quat(float _s, float3 _v) :s(_s), v(_v) {}
	_quat operator*(_quat q) {
		return _quat(s * q.s - dot(v, q.v), s * q.v + q.s * v + cross(v, q.v));
	}
	_quat conj() {
		return _quat(s, -v);
	}
	float4x4 to_mtx() {
		auto i = float3(1, 0, 0);
		auto j = float3(0, 1, 0);
		auto k = float3(0, 0, 1);
		i = (*this * _quat(0, i) * this->conj()).v;
		j = (*this * _quat(0, j) * this->conj()).v;
		k = (*this * _quat(0, k) * this->conj()).v;
		float _mtx[4 * 4] = {
			i.x,i.y,i.z,0. ,
			j.x,j.y,j.z,0. ,
			k.x,k.y,k.z,0. ,
			0. ,0. ,0. ,1. ,
		};
		return *(float4x4 *)&_mtx;
	}
};

float3 rotate(float3 v, _quat q) {
	return (q * _quat(0, v) * q.conj()).v;
}

_quat angle_axis(float theta, float3 n) {
	return _quat(cos(theta / 2), sin(theta / 2) * normalize(n));
}
_quat from_to(float3 a, float3 b) {
	return angle_axis(acos(dot(normalize(a), normalize(b))), normalize(cross(a, b)));
}

struct _vertex {
	float4 v;
	float4 col;
	_vertex() {}
	_vertex(float4 _v, float4 c) :v(_v), col(c) {}
};

inline float pos(bool p) { return p ? 1.0f : -1.0f; }

inline float map(float t, float t0, float t1, float s0, float s1) {
	return s0 + (s1 - s0) * (t - t0) / (t1 - t0);
}


float4 color_wheel(float t) {
	float theta = map(t, 0., 1., 0., 2. * pi);
	float modulus = 2.0f * pi * .75f;
	theta = fmodf(theta, modulus) + 2. * pi * .87;
	float2 angles = float2(cos(theta), sin(theta));
	angles.x = map(angles.x, -1., 1., 0., pi / 2.);
	angles.y = map(angles.y, -1., 1., 0., pi / 2.);
	return float4(cos(angles.x) * sin(angles.y), cos(angles.y), sin(angles.x) * sin(angles.y), 1);
}

skeleton3d get_default_skeleton() {
	skeleton3d sk;
	sk.head = float4(0, .8, 0, 0);
	sk.neck = float4(0, .6, 0, 0);
	sk.spine_chest = float4(0, .4, 0, 0);
	sk.spine_naval = float4(0, .2, -.005, 0);
	sk.pelvis = float4(0, -.05, 0, 0);
	for (int i = 0; i < 2; i++) {
		//sk.eyes[i] = float4(pos(i) * .1, .5, 0, 0);
		//sk.clavicles[i] = float4(pos(i) * .1, .6, 0, 0);
		sk.upper[i].shoulder = float4(pos(i) * .35, .6, 0, 0);
		sk.upper[i].elbow = float4(pos(i) * .45, .25, .1, 0);
		sk.upper[i].wrist = float4(pos(i) * .3, -.05, -.2, 0);
		sk.upper[i].hand = float4(pos(i) * .3, -.05, -.2, 0);
		sk.hand[i].hand_tip = float4(pos(i) * .34, -.1, -.36, 0);
		sk.hand[i].thumb = float4(pos(i) * .2, 0, -.25, 0);

		sk.lower[i].hip = float4(pos(i) * .2, 0, 0, 0);
		sk.lower[i].knee = float4(pos(i) * .17, -.5, -.1, 0);
		sk.lower[i].ankle = float4(pos(i) * .15, -.9, 0, 0);
		sk.lower[i].foot = float4(pos(i) * .15, -.9, -.2, 0);
	}
	return sk;
}




namespace vertex_buffer {
	void begin() {
		glBegin(GL_TRIANGLES);
	}
	void end() {
		glEnd();
	}
	void push(_vertex v) {
		glColor3f(v.col.r, v.col.g, v.col.b);
		glVertex3f(v.v.x, v.v.y, v.v.z);
	}
	void push_mtx(float4x4 mtx) {
		end();
		glPushMatrix();
		mtx = transpose(mtx);
		glMultMatrixf((float *)&mtx);
		begin();
	}
	void pop_mtx() {
		end();
		glPopMatrix();
		begin();
	}
	void push_test_tri() {
		push(_vertex(float4(0, 0, 0, 1), float4(0, 1, 1, 1)));
		push(_vertex(float4(+.5f, +.5f, 0, 1), 0.0f * float4(1, 1, 1, 1)));
		push(_vertex(float4(+.5f, -.5f, 0, 1), 0.0f * float4(1, 1, 1, 1)));
	}
	void push_test_square() {
		_vertex v[4];
		for (int i = 0; i < 4; i++) {
			v[i].v = vec4(pos(i & 1), pos(i & 2), 0, 1);
			v[i].col = vec4((float)(i & 1) * .3, 0, (float)(i & 2) * .3, 1);
		}
		push(v[0]);
		push(v[1]);
		push(v[2]);
		push(v[2]);
		push(v[1]);
		push(v[3]);
	}
	void push_test_image() {
		_vertex v[4];
		for (int i = 0; i < 4; i++) {
			v[i].v = vec4(pos(i & 1), pos(i & 2), 0, 1);
			v[i].col = vec4(1, 1, 1, 1); 
		}
		end();
		glEnable(GL_TEXTURE_2D);
		begin();
		glTexCoord2f(0, 0);
		push(v[0]);
		glTexCoord2f(1, 0);
		push(v[1]);
		glTexCoord2f(0, 1);
		push(v[2]);
		glTexCoord2f(0, 1);
		push(v[2]);
		glTexCoord2f(1, 0);
		push(v[1]);
		glTexCoord2f(1, 1);
		push(v[3]);
		end();
		glDisable(GL_TEXTURE_2D);
		glEnable(GL_DEPTH_TEST);
		begin();
	}
	void push_test_cube() {
		for (int k = 0; k < 3; k++) {
			float rgb[3] = { (float)(k == 0),(float)(k == 1),(float)(k == 2) };
			for (int j = 0; j < 2; j++) {
				float4 col;
				if (j == 0) col = float4(rgb[0], rgb[1], rgb[2], 1);
				else col = float4(1 - rgb[0], 1 - rgb[1], 1 - rgb[2], 1);
				float vtx[3];
				vtx[k] = .5 * pos(j);
				for (int i = 0; i < 3; i++) {
					vtx[(k + 1) % 3] = +.5 * pos(i / 2);
					vtx[(k + 2) % 3] = +.5 * pos(i % 2);
					push(_vertex(float4(vtx[0], vtx[1], vtx[2], 1), col));
				}
				for (int i = 1; i < 4; i++) {
					vtx[(k + 1) % 3] = +.5 * pos(i / 2);
					vtx[(k + 2) % 3] = +.5 * pos(i % 2);
					push(_vertex(float4(vtx[0], vtx[1], vtx[2], 1), col));
				}
			}
		}
	}
	void push_test_octa() {
		for (int i = 0; i < 8; i++) {
			float4 col(i & 1, i & 2, i & 4, 1);
			push(_vertex(float4(.5 * pos(i & 1), 0, 0, 1), col));
			push(_vertex(float4(0, .5 * pos(i & 2), 0, 1), col));
			push(_vertex(float4(0, 0, .5 * pos(i & 4), 1), col));
		}
	}
	void push_test_tetra() {
		const float a = sqrt(2.0f) / 4;
		float4 points[4] = {
			float4(+a,+.25,+0,1),
			float4(-a,+.25,+0,1),
			float4(+0,-.25,+a,1),
			float4(+0,-.25,-a,1),
		};
		for (int i = 0; i < 4; i++) {
			float4 col = color_wheel(i / 5.0 + .1);
			for (int j = 0; j < 4; j++) if (i != j) {
				push(_vertex(points[j], col));
			}
		}
	}
	void push_test_sphere(int res) {
		float inc = pi / res;
		float rho = .5;
		for (float phi = 0; phi < pi; phi += inc) {
			for (float theta = 0; theta < 2 * pi; theta += inc) {
				float4 col(
					.5 + .5 * cos(theta + inc / 2) * sin(phi + inc / 2),
					.5 + .5 * cos(phi + inc / 2),
					.5 + .5 * sin(theta + inc / 2) * sin(phi + inc / 2), 1
				);
				if (phi != 0) for (int i = 0; i < 3; i++) {
					float t = theta + inc * (i & 1);
					float p = phi + .5 * inc * (i & 2);
					float4 v(
						rho * cos(t) * sin(p),
						rho * cos(p),
						rho * sin(t) * sin(p),
						1
					);
					push(_vertex(v, col));
				}
				if (phi < pi - inc) for (int i = 1; i < 4; i++) {
					float t = theta + inc * (i & 1);
					float p = phi + .5 * inc * (i & 2);
					float4 v(
						rho * cos(t) * sin(p),
						rho * cos(p),
						rho * sin(t) * sin(p),
						1
					);
					push(_vertex(v, col));
				}
			}
		}
	}
	void push_test_ico() {
		float a = 2 * sin(pi / 5);
		float b = a * sin(acos(1 / a));
		float d = 2 * sin(pi / 10);
		float c = sqrt(a * a - d * d) / 2;
		float s = .5;
		float4 ud[2] = { s * float4(0,b + c,0,1),s * float4(0,-b - c,0,1) };
		float4 mid[2][5];
		for (int i = 0; i < 2; i++) {
			for (int j = 0; j < 5; j++) {
				float theta = 2 * pi * j / 5.0 + pi / 5.0 * i;
				mid[i][j] = s * float4(cos(theta), -pos(i) * c, sin(theta), 1);
			}
		}
		for (int i = 0; i < 2; i++) {
			for (int j = 0; j < 5; j++) {
				float4 col = color_wheel(j / 6.0 + i * 1 / 10.0);
				push(_vertex(mid[i][j], col));
				push(_vertex(mid[i][(j + 1) % 5], col));
				push(_vertex(ud[i], col));
			}
		}
		for (int j = 0; j < 5; j++) {
			for (int i = 0; i < 2; i++) {
				float4 col = color_wheel(j / 6.0 + 1 / 20.0 + 1 / 40.0 * i);
				push(_vertex(mid[i][j], col));
				push(_vertex(mid[i][(j + (i ? 4 : 1)) % 5], col));
				push(_vertex(mid[!i][j], col));
			}
		}
	}
	void push_clip_test(float a) {
		push(_vertex(a * float4(0, 0, 0, 1), float4(1, 0, 0, 1)));
		push(_vertex(a * float4(-2, .5, 0, 1), float4(1, 0, 0, 1)));
		push(_vertex(a * float4(-2, -.5, 0, 1), float4(1, 0, 0, 1)));

		push(_vertex(a * float4(2, 0, 0, 1), float4(1, 0, 0, 1)));
		push(_vertex(a * float4(0, .5, 0, 1), float4(1, 0, 0, 1)));
		push(_vertex(a * float4(0, -.5, 0, 1), float4(1, 0, 0, 1)));
	}
	void push_mtx_test() {
		push_mtx(scale(1.0 / 4));
		//        push_mtx(trans(1,0,0));
		push_test_cube();
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				float4 a(0);
				a[i] = 2 * pos(j);
				push_mtx(trans(a));
				push_test_cube();
				pop_mtx();
			}
		}
		pop_mtx();
	}
	void push_line(float3 a, float3 b) {
		float3 c = (a + b) / 2.0f;
		push_mtx(trans(c));
		float d = length(a - b);
		if (abs(1 - abs(dot(normalize(a - b), float3(0, 1, 0)))) > .01) {
			push_mtx(from_to(a - b, float3(0, 1, 0)).to_mtx());
			push_mtx(scale(.05, d, .05));
			push_test_cube();
			pop_mtx();
			pop_mtx();
		}
		else {
			push_mtx(scale(.05, d, .05));
			push_test_cube();
			pop_mtx();
		}
		pop_mtx();
	}
	void push_line(float4 a, float4 b) {
		float3 _a(a.x, a.y, a.z), _b(b.x, b.y, b.z);
		push_line(_a, _b);
	}
	void push_line_test() {
		push_mtx(scale(.5));
		vec4 points[8];
		for (int i = 0; i < 8; i++) {
			points[i] = vec4(pos(i & 1), pos(i & 2), pos(i & 4), 1);
		}
		for (int i = 0; i < 7; i++) {
			for (int j = i + 1; j < 8; j++) {
				int matches = 0;
				for (int k = 0; k < 3; k++) {
					matches += points[i][k] == points[j][k];
				}
				if (matches == 2) push_line(points[i], points[j]);
			}
		}
		pop_mtx();
	}
	void push_model(skeleton3d sk) {
		push_mtx(trans(sk.head));
		push_mtx(scale(.3));
		push_test_cube();
		pop_mtx();
		pop_mtx();

		push_line(sk.head, sk.neck);
		push_line(sk.neck, sk.spine_chest);
		push_line(sk.spine_chest, sk.spine_naval);
		push_line(sk.spine_naval, sk.pelvis);

#ifdef DICK
		vec3 i = normalize(sk.spine_naval - sk.pelvis);
		vec3 j = normalize(sk.right_hip - sk.left_hip);
		vec3 k = normalize(cross(i, j));
		push_line(sk.pelvis, sk.pelvis + 1.0f * vec4(k, 1));
		push_mtx(trans(sk.pelvis - vec4((i - k) * .2f, 0) * .3f));
		for (int i = 0; i < 2; i++) {
			push_mtx(trans(pos(i) * j * .04f));
			push_mtx(scale(.1));
			push_test_sphere(8);
			pop_mtx();
			pop_mtx();
		}
		pop_mtx();
#endif

		for (int i = 0; i < 2; i++) {
			push_line(sk.spine_chest, sk.upper[i].shoulder);
			push_line(sk.upper[i].shoulder, sk.upper[i].elbow);
			push_line(sk.upper[i].elbow, sk.upper[i].wrist);
			push_line(sk.upper[i].wrist, sk.upper[i].hand);
			push_line(sk.upper[i].hand, sk.hand[i].hand_tip);
			push_line(sk.upper[i].hand, sk.hand[i].thumb);

			push_line(sk.pelvis, sk.lower[i].hip);
			push_line(sk.lower[i].hip, sk.lower[i].knee);
			push_line(sk.lower[i].knee, sk.lower[i].ankle);
			push_line(sk.lower[i].ankle, sk.lower[i].foot);
		}
	}
	void push_test_model() {

		push_model(get_default_skeleton());
	}
};



float itime() {
	struct timeb now;
	ftime(&now);
	return (float)(now.time % (60 * 60 * 24)) + now.millitm / 1e3;
}


float compare_skeletons(skeleton3d *sk1, skeleton3d *sk2) {
	float res = 0;
	for (int i = 0; i < JointType_Count; i++) {
		res += length(sk1->joints[i] - sk2->joints[i]);
	}
	return res / JointType_Count;
}


inline float deg(float rad) { return rad * 180 / pi; }

#ifndef __APPLE__
kinect _kinect;
#endif

struct renderer {
	float width, height;
	GLFWwindow *window;
	uint32_t tex_id;
	static const int img_w = 1920, img_h = 1080;
	bgra8 img[img_h][img_w];
	void init() {
		puts(__func__);
		assert(glfwInit());
		width = 640;
		height = 480;
		window = glfwCreateWindow(width, height, "._____.", NULL, NULL);
		assert(window);
		glfwMakeContextCurrent(window);

		glfwSetMouseButtonCallback(window, [](GLFWwindow *window, int button, int action, int mods) {
			if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
				puts("click");
#ifdef KINECT
				if (_kinect.recording) _kinect.stop_recording();
				else _kinect.start_recording("record.bin");
#endif
			}
		});

		glEnable(GL_DEPTH_TEST);


		for (int j = 0; j < img_h; j++) {
			for (int i = 0; i < img_w; i++) {
				img[j][i].r = i *(255.0 / img_w);
				img[j][i].g = 0;
				img[j][i].b = j *(255.0 / img_h);
				img[j][i].a = 1;
			}
		}

		glGenTextures(1, &tex_id);
		assert(tex_id>0);
		glBindTexture(GL_TEXTURE_2D, tex_id);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, img_w, img_h, 0, GL_RGBA, GL_UNSIGNED_BYTE,img);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	}
	void update_frame() {
		glTexSubImage2D(
			GL_TEXTURE_2D, 0, 0, 0,
			img_w,
			img_h,
			GL_RGBA,
			GL_UNSIGNED_BYTE,
			img
		);
	}
	bool update(uint32_t n,const skeleton3d *sk) {
		if (glfwWindowShouldClose(window)) return true;
		double xpos, ypos;
		glfwGetCursorPos(window, &xpos, &ypos);
		float theta = map(xpos, 0, width, pi, -pi);
		float phi = map(ypos, 0, height, pi / 2, -pi / 2);
		int iwidth, iheight;
		glfwGetFramebufferSize(window, &iwidth, &iheight);
		width = iwidth;
		height = iheight;

		glViewport(0, 0, iwidth, iheight);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glDepthMask(GL_TRUE);

		vertex_buffer::begin();

		vertex_buffer::push_mtx(trans(0, 0, .9));
		//vertex_buffer::push_test_square();
		vertex_buffer::push_test_image();
		vertex_buffer::pop_mtx();

		vertex_buffer::push_mtx(scale(height / width, 1, .25));
		vertex_buffer::push_mtx(rotx(-phi));
		vertex_buffer::push_mtx(roty(theta));

		//vertex_buffer::push_test_square();
		//vertex_buffer::push_test_cube();
		//vertex_buffer::push_test_tetra();
		//vertex_buffer::push_test_octa();
		//vertex_buffer::push_test_ico();
		//vertex_buffer::push_mtx_test();
		//vertex_buffer::push_line_test();
		//vertex_buffer::push_test_model();
		//vertex_buffer::push_test_sphere(15);

		if (sk == NULL ) vertex_buffer::push_test_model();
		else for (int i = 0; i < n; i++) {
			vertex_buffer::push_model(sk[i]);
		}

		vertex_buffer::pop_mtx();
		vertex_buffer::pop_mtx();
		vertex_buffer::pop_mtx();

		vertex_buffer::end();

		glfwSwapBuffers(window);
		glfwPollEvents();
		return false;
	}
};


struct record {
	FILE *file;
	record(const char *fname) {
		file = fopen(fname, "rb");
		assert(file);
	}
	void get_next_frame(skeleton3d *sk) {
		int ret=fread(sk, 1, sizeof(skeleton3d), file);
		if (ret == 0) {
			fseek(file, 0, SEEK_SET);
			ret=fread(sk, 1, sizeof(skeleton3d), file);
			assert(ret);
		}
	}
};



int main() {
	puts(__func__);
	const uint32_t n = 3;
	skeleton3d sk[n];
	for (int i = 0; i < n; i++) {
		sk[i] = get_default_skeleton();
		if (n > 1) {
			for (int j = 0; j < JointType_Count; j++) {
				sk[i].joints[j].x += map(i, 0, n - 1, -.7, .7);
			}
		}
	}
	static renderer ren;
	ren.init();
#ifdef KINECT
	_kinect.init();
#endif
	bool done;
	record r("record.bin");
	do {
#ifdef KINECT
		_kinect.update(n-1,sk+1);
		//_kinect.update_frame(&ren.img[0][0]);
#endif	
		r.get_next_frame(sk);

		done = ren.update(n,sk);
	} while (!done);

	return 0;

}

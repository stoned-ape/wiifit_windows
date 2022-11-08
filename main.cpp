//#define DICK
//#define KINECT
#include <math.h>
#include <windows.h>
#include <gl/gl.h>
#include <GLFW/glfw3.h>
#include <assert.h>
#include <time.h>
#include <sys/timeb.h>
#include <math.h>
#include <stdio.h>
#include <stdbool.h>

#include <kinect.h>

#define GLM_FORCE_RADIANS
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/mat2x2.hpp>
#include <glm/mat3x3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/geometric.hpp>
#include <glm/ext.hpp>
using glm::vec2;
using glm::vec3;
using glm::vec4;
using glm::transpose;
using glm::cross;
using glm::length;
using glm::normalize;

typedef glm::vec2 float2;
typedef glm::vec3 float3;
typedef glm::vec4 float4;

typedef glm::ivec2 int2;
typedef glm::ivec3 int3;
typedef glm::ivec4 int4;

typedef glm::mat2 float2x2;
typedef glm::mat3 float3x3;
typedef glm::mat4 float4x4;

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
	return angle_axis(acos(dot(normalize(a), normalize(b))),
		normalize(cross(a, b)));
}

struct _vertex {
	float4 v;
	float4 col;
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


/*
JointType_SpineBase = 0,
JointType_SpineMid = 1,
JointType_Neck = 2,
JointType_Head = 3,
JointType_ShoulderLeft = 4,
JointType_ElbowLeft = 5,
JointType_WristLeft = 6,
JointType_HandLeft = 7,
JointType_ShoulderRight = 8,
JointType_ElbowRight = 9,
JointType_WristRight = 10,
JointType_HandRight = 11,
JointType_HipLeft = 12,
JointType_KneeLeft = 13,
JointType_AnkleLeft = 14,
JointType_FootLeft = 15,
JointType_HipRight = 16,
JointType_KneeRight = 17,
JointType_AnkleRight = 18,
JointType_FootRight = 19,
JointType_SpineShoulder = 20,
JointType_HandTipLeft = 21,
JointType_ThumbLeft = 22,
JointType_HandTipRight = 23,
JointType_ThumbRight = 24,
*/

union skeleton3d {
	struct {
		float4 joints[32];
	};
	struct {
		float4 pelvis;
		float4 spine_naval;
		float4 neck;
		float4 head;
		float4 left_shoulder;
		float4 left_elbow;
		float4 left_wrist;
		float4 left_hand;
		float4 right_shoulder;
		float4 right_elbow;
		float4 right_wrist;
		float4 right_hand;
		float4 left_hip;
		float4 left_knee;
		float4 left_ankle;
		float4 left_foot;
		float4 right_hip;
		float4 right_knee;
		float4 right_ankle;
		float4 right_foot;
		float4 spine_chest;
		float4 left_hand_tip;
		float4 left_thumb;
		float4 right_hand_tip;
		float4 right_thumb;

		float4 left_eye;
		float4 right_eye;
		float4 left_ear;
		float4 right_ear;
		float4 left_clavicle;
		float4 right_clavicle;
		float4 nose;
	};
	struct {
		float4 _pelvis;
		float4 _spine_naval;
		float4 _neck;
		float4 _head;
		struct {
			float4 shoulder;
			float4 elbow;
			float4 wrist;
			float4 hand;
		}upper[2];
		struct {
			float4 hip;
			float4 knee;
			float4 ankle;
			float4 foot;
		}lower[2];
		float4 _spine_chest;
		struct {
			float4 hand_tip;
			float4 thumb;
		}hand[2];
	};
};

static_assert(sizeof(skeleton3d) == 32 * sizeof(vec4), "");

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
		push(_vertex(float4(-.5, -.5, 0, 1), float4(1, 0, 1, 1)));
		push(_vertex(float4(-.5, +.5, 0, 1), float4(1, 0, 1, 1)));
		push(_vertex(float4(+.5, -.5, 0, 1), float4(1, 0, 1, 1)));
		push(_vertex(float4(-.5, +.5, 0, 1), float4(1, 0, 1, 1)));
		push(_vertex(float4(+.5, -.5, 0, 1), float4(1, 0, 1, 1)));
		push(_vertex(float4(+.5, +.5, 0, 1), float4(1, 0, 1, 1)));
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
		vec3 i = sk.spine_naval - sk.pelvis;
		vec3 j = sk.right_hip - sk.pelvis;
		vec3 k = normalize(cross(i, j));
		push_line(sk.pelvis, sk.pelvis + 1.0f * vec4(k, 1));
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




inline float deg(float rad) { return rad * 180 / pi; }

struct renderer {
	float width, height;
	GLFWwindow *window;
	void init() {
		puts(__func__);
		bool b = glfwInit();
		assert(b);
		width = 640;
		height = 480;
		window = glfwCreateWindow(width, height, "._____.", NULL, NULL);
		assert(window);
		glfwMakeContextCurrent(window);
		glEnable(GL_DEPTH_TEST);
	}
	bool update(const skeleton3d *sk) {
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

		vertex_buffer::push_mtx(scale(height / width, 1, 1));
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

		if (sk == NULL) vertex_buffer::push_test_model();
		else vertex_buffer::push_model(*sk);

		vertex_buffer::pop_mtx();
		vertex_buffer::pop_mtx();
		vertex_buffer::pop_mtx();

		vertex_buffer::end();

		glfwSwapBuffers(window);
		glfwPollEvents();
		return false;
	}
};

template<class T>
inline void safe_release(T *&obj) {
	if (obj != NULL) {
		obj->Release();
		obj = NULL;
	}
}

struct kinect {
	IKinectSensor *kinect_sensor;
	ICoordinateMapper *coordinate_mapper;
	IBodyFrameSource *body_frame_source;
	IBodyFrameReader *body_frame_reader;
	void init() {
		HRESULT hr;
		hr = GetDefaultKinectSensor(&kinect_sensor);
		assert(!FAILED(hr));
		assert(kinect_sensor);
		hr = kinect_sensor->Open();
		assert(!FAILED(hr));
		hr = kinect_sensor->get_CoordinateMapper(&coordinate_mapper);
		assert(!FAILED(hr));
		assert(coordinate_mapper);
		hr = kinect_sensor->get_BodyFrameSource(&body_frame_source);
		assert(!FAILED(hr));
		assert(body_frame_source);
		hr = body_frame_source->OpenReader(&body_frame_reader);
		assert(!FAILED(hr));
		assert(body_frame_reader);
	}
	void update(skeleton3d *sk) {
		HRESULT hr;
		if (!body_frame_reader) return;
		IBodyFrame *body_frame = NULL;
		hr = body_frame_reader->AcquireLatestFrame(&body_frame);
		if (FAILED(hr)) goto l1;

		IBody *bodies[6] = { 0 };
		long long ntime = 0;
		hr = body_frame->get_RelativeTime(&ntime);
		//if (FAILED(hr)) return;
		hr = body_frame->GetAndRefreshBodyData(_countof(bodies), bodies);
		if (FAILED(hr)) goto l2;
		for (int i = 0; i < _countof(bodies); ++i) {
			puts("bruh");
			if (!bodies[i]) continue;
			BOOLEAN istracked = false;
			hr = bodies[i]->get_IsTracked(&istracked);

			if (FAILED(hr) || !istracked) continue;

			Joint joints[JointType_Count];
			HandState left_hand_state = HandState_Unknown;
			HandState right_hand_state = HandState_Unknown;

			bodies[i]->get_HandLeftState(&left_hand_state);
			bodies[i]->get_HandRightState(&right_hand_state);

			hr = bodies[i]->GetJoints(_countof(joints), joints);
			if (FAILED(hr)) continue;
			puts("bruh2");

			auto joint2vec4 = [](Joint j)->vec4 {
				auto csp = j.Position;
				return vec4(csp.X, csp.Y, csp.Z - 2, 1);
			};

			for (int j = 0; j < 24; j++) {
				sk->joints[j] = joint2vec4(joints[j]);
			}

			//sk->nose=joint2vec4(joints[JointType_]);
			//sk->left_eye=joint2vec4(joints[JointType_]);
			//sk->right_eye=joint2vec4(joints[JointType_]);
			//sk->left_ear=joint2vec4(joints[JointType_]);
			//sk->right_ear=joint2vec4(joints[JointType_]);
			//sk->left_clavicle=joint2vec4(joints[JointType_]);
			//sk->right_clavicle=joint2vec4(joints[JointType_]);
		}
	l2:
		for (int i = 0; i < _countof(bodies); ++i) {
			safe_release(bodies[i]);
		}
	l1:
		safe_release(body_frame);
	}
};



int main() {
	puts(__func__);
	skeleton3d sk = get_default_skeleton();
	renderer ren;
	kinect k;
#ifdef KINECT
	k.init();
#endif
	ren.init();
	bool done;
	do {
#ifdef KINECT
		k.update(&sk);
#endif
		done = ren.update(&sk);
	} while (!done);

	return 0;

}

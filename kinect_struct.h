#pragma once
#ifndef __APPLE__
#include <kinect.h>
#else
#define JointType_Count 25
#endif
#include <stdio.h>
#include <stdint.h>

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

#ifndef __APPLE__
struct kinect {
	IKinectSensor *kinect_sensor;
	ICoordinateMapper *coordinate_mapper;
	IBodyFrameReader *body_frame_reader;
	bool recording;
	FILE *file;
	kinect();
	~kinect();
	void start_recording(const char *fname);
	void stop_recording();
	void init();
	void update(uint32_t n,skeleton3d *sk);
};
#endif

#include "kinect_struct.h"

template<class T>
inline void safe_release(T *&obj) {
	if (obj != NULL) {
		obj->Release();
		obj = NULL;
	}
}

#ifndef __APPLE__
kinect::kinect() {
	kinect_sensor = NULL;
	coordinate_mapper = NULL;
	body_frame_reader = NULL;
	color_frame_reader = NULL;
	recording = false;
	file = NULL;
}
kinect::~kinect() {
	safe_release(coordinate_mapper);
	safe_release(body_frame_reader);
	if (kinect_sensor) kinect_sensor->Close();
	safe_release(kinect_sensor);
}
void kinect::start_recording(const char *fname) {
	assert(!recording);
	file = fopen(fname, "wb");
	recording = true;
}
void kinect::stop_recording() {
	assert(recording);
	fclose(file);
	file = NULL;
	recording = false;
}
void kinect::init() {
	HRESULT hr;
	IBodyFrameSource *body_frame_source;
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
	IColorFrameSource *color_frame_source = NULL;
	hr = kinect_sensor->get_ColorFrameSource(&color_frame_source);
	assert(!FAILED(hr));
	assert(color_frame_source);
	hr = color_frame_source->OpenReader(&color_frame_reader);
	assert(!FAILED(hr));
	assert(color_frame_reader);
	
	safe_release(color_frame_source);
	safe_release(body_frame_source);
}
void kinect::update_frame(bgra8 *data) {
	HRESULT hr;
	IColorFrame *color_frame = NULL;
	hr = color_frame_reader->AcquireLatestFrame(&color_frame);
	if (FAILED(hr)) goto l0;
	BYTE *ptr;
	uint32_t size;
	hr = color_frame->AccessRawUnderlyingBuffer(&size, &ptr);
	if (FAILED(hr)) goto l0;
	memcpy(data, ptr, size);
l0:
	safe_release(color_frame);
}
void kinect::update(uint32_t n,skeleton3d *sk) {
	HRESULT hr;
	IBodyFrame *body_frame = NULL;
	hr = body_frame_reader->AcquireLatestFrame(&body_frame);
	if (FAILED(hr)) goto l1;
	IBody *bodies[6];
	memset(bodies, 0, sizeof(bodies));
	hr = body_frame->GetAndRefreshBodyData(_countof(bodies), bodies);
	if (FAILED(hr)) goto l2;
	for (int i = 0; i < _countof(bodies); ++i) {
		puts("bruh");
		if (i >= n) break;
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
		for (int j = 0; j < JointType_Count; j++) {
			sk[i].joints[j] = joint2vec4(joints[j]);
		}

		if (recording && i == 0) {
			fwrite(sk, 1, sizeof(skeleton3d), file);
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
#endif

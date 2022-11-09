#include "kinect_struct.h"

template<class T>
inline void safe_release(T *&obj) {
	if (obj != NULL) {
		obj->Release();
		obj = NULL;
	}
}

kinect::kinect() {
	kinect_sensor = NULL;
	coordinate_mapper = NULL;
	body_frame_reader = NULL;
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
	safe_release(body_frame_source);
}
void kinect::update(uint32_t n,skeleton3d *sk) {
	HRESULT hr;
	if (!body_frame_reader) return;
	IBodyFrame *body_frame = NULL;
	hr = body_frame_reader->AcquireLatestFrame(&body_frame);
	if (FAILED(hr)) goto l1;
	IBody *bodies[6] = { 0 };
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
		for (int j = 0; j < 24; j++) {
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


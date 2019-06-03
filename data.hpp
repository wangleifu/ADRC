#ifndef DATA_H
#define DATA_H
namespace DATA
{	
	typedef float  float32_t;

	typedef struct Positon{
		float32_t x;  // position set-point in x axis of ground frame (m) 
		float32_t y;  // position set-point in y axis of ground frame (m)
		float32_t z;  // position set-point in z axis of ground frame (m), Limit: 0 to 120 m
	}Positon;

	typedef struct Velocity{
		float32_t Vx;  // velocity set-point in x axis of ground frame (m/s), Limit: 30 m/s
		float32_t Vy;  // velocity set-point in y axis of ground frame (m/s), Limit: 30 m/s
		float32_t Vz;  // velocity set-point in z axis of ground frame (m/s), Limit: -5 to 5 m/s 
	}Velocity;

	typedef struct Attitude{
		float32_t roll;      // attitude set-point in x axis of body frame (FRU) (deg), Limit: 35 degree
		float32_t pitch;     // attitude set-point in y axis of body frame (FRU) (deg), Limit: 35 degree
		float32_t yaw;       // attitude set-point in z axis of ground frame (NED) (deg) 
	}Attitude;

	typedef struct AngularRate{
		float32_t rollRate;  // attitude rate set-point in x axis of body frame (FRU) (deg/s)
		float32_t pitchRate; // attitude rate set-point in y axis of body frame (FRU) (deg/s)
		float32_t yawRate;   //attitude rate set-point in z axis of body frame (FRU) (deg/s), Limite: 150 deg/s
	} AngularRate;

	typedef struct FlightData{
		Positon      pos;
		Velocity     vel;
		Attitude     att;
		AngularRate  ang;
	}FlightData;
}

#endif

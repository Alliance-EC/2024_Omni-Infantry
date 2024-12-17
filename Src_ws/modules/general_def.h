#ifndef GENERAL_DEF_H
#define GENERAL_DEF_H

#define bool  _Bool
#define true  1
#define false 0

#ifndef PI
#define PI 3.1415926535f
#endif
#define PI2                 (PI * 2.0f) // 2 pi

#define RAD_2_DEGREE        57.2957795f    // 180/pi
#define DEGREE_2_RAD        0.01745329252f // pi/180

#define RPM_2_ANGLE_PER_SEC 6.0f         // ×360°/60sec
#define RPM_2_RAD_PER_SEC   0.104719755f // ×2pi/60sec

#endif // !GENERAL_DEF_H
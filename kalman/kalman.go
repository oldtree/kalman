package kalman

/*
在写代码的时候也深刻地发现没有什么状态估计问题是一个Kalman Filter解决不了的，如果有，那就用两个。
有人说20世纪人类最伟大的发现是相对论和量子力学，然而我觉得20世纪人类最伟大的发现是1944年克劳德-香农提出的信息论，1948年诺伯特-维纳提出的控制论，
以及1960年鲁道夫-卡尔曼提出的线性系统论，我把他们叫做系统控制技术三大理论。在 @吴军 老师的《硅谷之谜》里，信息论和控制论还有管理学中的系统论构成了硅谷发展的奥秘。
而对于飞机火箭和各种自动控制设备中，则是R.E. Kalman的线性系统论，连接了信息论和控制论。

作者：YY硕
链接：https://www.zhihu.com/question/48145674/answer/109668492
来源：知乎
著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。
*/

//系统、控制与滤波哲学的数学原理》
//香农的信息论是《通信、数据压缩等哲学的数学原理》
type Kalman struct {
	/* Kalman filter variables */
	Qangle   float64 // Process noise variance for the accelerometer
	GyroBias float64 // Process noise variance for the gyro bias
	Measure  float64 // Measurement noise variance - this is actually the variance of the measurement noise

	Angle float64 // The angle calculated by the Kalman filter - part of the 2x1 state vector
	bias  float64 // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
	rate  float64 // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

	P [2][2]float64 // Error covariance matrix - This is a 2x2 matrix

}

func (k *Kalman) Init() {
	k.P = [2][2]float64{{0.0, 0.0}, {0.0, 0.0}}
	k.Qangle = 0.001
	k.GyroBias = 0.003
	k.Measure = 0.03
	k.Angle = 0.0
	k.bias = 0.0

}

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
func (k *Kalman) GetAngle(newAngle, newRate, deltaTime float64) float64 {
	// KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
	// Modified by Kristian Lauszus
	// See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

	// Discrete Kalman filter time update equations - Time Update ("Predict")
	// Update xhat - Project the state ahead
	/* Step 1 */
	k.rate = newRate - k.bias
	k.Angle = k.Angle + deltaTime*k.rate

	// Update estimation error covariance - Project the error covariance ahead
	/* Step 2 */
	k.P[0][0] += deltaTime * (deltaTime*k.P[1][1] - k.P[0][1] - k.P[1][0] + k.Qangle)
	k.P[0][1] -= deltaTime * k.P[1][1]
	k.P[1][0] -= deltaTime * k.P[1][1]
	k.P[1][1] += k.GyroBias * deltaTime

	// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
	// Calculate Kalman gain - Compute the Kalman gain
	/* Step 4 */
	S := k.P[0][0] + k.Measure // Estimate error
	/* Step 5 */
	K := [2]float64{} // Kalman gain - This is a 2x1 vector
	K[0] = k.P[0][0] / S
	K[1] = k.P[1][0] / S

	// Calculate angle and bias - Update estimate with measurement zk (newAngle)
	/* Step 3 */
	y := newAngle - k.Angle // Angle difference
	/* Step 6 */
	k.Angle += K[0] * y
	k.bias += K[1] * y

	// Calculate estimation error covariance - Update the error covariance
	/* Step 7 */
	P00_temp := k.P[0][0]
	P01_temp := k.P[0][1]

	k.P[0][0] = k.P[0][0] - K[0]*P00_temp
	k.P[0][1] = k.P[0][1] - K[0]*P01_temp
	k.P[1][0] = k.P[1][0] - K[1]*P00_temp
	k.P[1][1] = k.P[1][1] - K[1]*P01_temp

	return k.Angle
}

// Used to set angle, this should be set as the starting angle
func (k *Kalman) SetAngle(angle float64) {
	k.Angle = angle
}

// Return the unbiased rate
func (k *Kalman) GetRate() float64 {
	return k.rate
}

/* These are used to tune the Kalman filter */
func (k *Kalman) SetQangle(Q_angle float64) {
	k.Qangle = Q_angle
}

/**
 * setQbias(float Q_bias)
 * Default value (0.003f) is in Kalman.cpp.
 * Raise this to follow input more closely,
 * lower this to smooth result of kalman filter.
 */
func (k *Kalman) SetQbias(bias float64) {
	k.GyroBias = bias
}

func (k *Kalman) SetRmeasure(rmeasure float64) {
	k.Measure = rmeasure
}

func (k *Kalman) GetQangle() float64 {
	return k.Qangle
}

func (k *Kalman) GetQbias() float64 {
	return k.GyroBias
}

func (k *Kalman) GetRmeasure() float64 {
	return k.Measure
}
